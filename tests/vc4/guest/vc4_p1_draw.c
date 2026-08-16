/*
 * Phase-1 exercise for the QEMU bcm2835-v3d control-list interpreter.
 *
 * Submits a complete, realistic single-triangle job through the mainline vc4
 * SUBMIT_CL ioctl: a bin control list with a full state block, a GL shader
 * record with one vertex attribute pointing at a real vertex BO, and either a
 * GL_ARRAY_PRIMITIVE or a GL_INDEXED_PRIMITIVE draw.  The kernel generates the
 * matching render control list (tile rendering mode config, clear colours,
 * per-tile sub-list branches and the EOF store).
 *
 * Nothing here is a bug trigger -- it is the workload the emulated V3D must
 * decode.  Run it under QEMU with
 *     -d trace:bcm2835_v3d_*
 * and check the trace against the numbers this program prints.
 *
 *   usage: vc4_p1_draw [array|indexed]      (default: array)
 */
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/vc4_drm.h>

/* Packet opcodes and sizes from drivers/gpu/drm/vc4/vc4_packet.h, kept local
 * so this builds against the uapi headers alone. */
enum {
    P_FLUSH                  = 4,   /* size 1  */
    P_START_TILE_BINNING     = 6,   /* size 1  */
    P_INCREMENT_SEMAPHORE    = 7,   /* size 1  */
    P_GL_INDEXED_PRIMITIVE   = 32,  /* size 14 */
    P_GL_ARRAY_PRIMITIVE     = 33,  /* size 10 */
    P_PRIMITIVE_LIST_FORMAT  = 56,  /* size 2  */
    P_GL_SHADER_STATE        = 64,  /* size 5  */
    P_CONFIGURATION_BITS     = 96,  /* size 4  */
    P_CLIP_WINDOW            = 102, /* size 9  */
    P_VIEWPORT_OFFSET        = 103, /* size 5  */
    P_CLIPPER_XY_SCALING     = 105, /* size 9  */
    P_CLIPPER_Z_SCALING      = 106, /* size 9  */
    P_TILE_BINNING_CONFIG    = 112, /* size 16 */
    P_GEM_HANDLES            = 254, /* size 9, consumed by the validator */
};

/* Primitive modes are the GL ones; bit 4 of the indexed mode byte selects
 * 16-bit indices. */
#define PRIM_TRIANGLES        4
#define INDEX_BUFFER_U16      (1 << 4)

#define FB_WIDTH   16
#define FB_HEIGHT  16
#define BO_SIZE    (FB_WIDTH * FB_HEIGHT * 4)

#define VERTEX_COUNT   3
#define ATTR_SIZE      12       /* 3 floats per vertex */
#define ATTR_STRIDE    12

/* Minimal QPU encoder: the kernel validates every shader BO, so the programs
 * have to be well-formed.  A PROG_END signal plus its two delay slots is the
 * shortest valid shader. */
enum {
    QPU_W_NOP = 39,
    QPU_R_NOP = 39,
    QPU_SIG_NONE = 1,
    QPU_SIG_PROG_END = 3,
    QPU_COND_ALWAYS = 1,
    QPU_PACK_A_NOP = 0,
    QPU_A_NOP = 0,
    QPU_M_NOP = 0,
};

static uint64_t qpu_nop(uint32_t sig)
{
    uint64_t inst = 0;

    inst |= (uint64_t)sig << 60;
    inst |= (uint64_t)QPU_PACK_A_NOP << 52;
    inst |= (uint64_t)QPU_COND_ALWAYS << 49;
    inst |= (uint64_t)QPU_COND_ALWAYS << 46;
    inst |= (uint64_t)QPU_W_NOP << 38;
    inst |= (uint64_t)QPU_W_NOP << 32;
    inst |= (uint64_t)QPU_R_NOP << 18;
    inst |= (uint64_t)QPU_R_NOP << 12;
    inst |= (uint64_t)QPU_A_NOP << 9;
    inst |= (uint64_t)QPU_A_NOP << 6;
    inst |= (uint64_t)QPU_M_NOP << 3;
    inst |= (uint64_t)QPU_M_NOP;
    return inst;
}

static int open_dri(void)
{
    /* The primary node first: the sole opener becomes DRM master, and
     * SUBMIT_CL is render-allowed there too. */
    static const char *paths[] = { "/dev/dri/card0", "/dev/dri/card1",
                                   "/dev/dri/renderD128" };
    for (unsigned i = 0; i < 3; i++) {
        int fd = open(paths[i], O_RDWR | O_CLOEXEC);
        if (fd >= 0)
            return fd;
    }
    return -1;
}

static int create_shader_bo(int fd, const uint64_t *code, uint32_t size,
                            uint32_t *handle)
{
    struct drm_vc4_create_shader_bo args;

    memset(&args, 0, sizeof(args));
    args.size = size;
    args.data = (uintptr_t)code;
    if (ioctl(fd, DRM_IOCTL_VC4_CREATE_SHADER_BO, &args) != 0)
        return -1;
    *handle = args.handle;
    return 0;
}

static int create_bo(int fd, uint32_t size, uint32_t *handle)
{
    struct drm_vc4_create_bo args;

    memset(&args, 0, sizeof(args));
    args.size = size;
    if (ioctl(fd, DRM_IOCTL_VC4_CREATE_BO, &args) != 0)
        return -1;
    *handle = args.handle;
    return 0;
}

/* Put a recognisable triangle in the vertex BO.  Purely cosmetic: the model
 * fetches the bytes, it does not shade them yet. */
static void fill_vertices(int fd, uint32_t handle)
{
    struct drm_vc4_mmap_bo map;
    static const float verts[VERTEX_COUNT * 3] = {
        -1.0f, -1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         0.0f,  1.0f, 0.0f,
    };
    void *p;

    memset(&map, 0, sizeof(map));
    map.handle = handle;
    if (ioctl(fd, DRM_IOCTL_VC4_MMAP_BO, &map) != 0) {
        printf("P1: note: MMAP_BO failed (%s), vertex data left zeroed\n",
               strerror(errno));
        return;
    }
    p = mmap(NULL, BO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
             (off_t)map.offset);
    if (p == MAP_FAILED) {
        printf("P1: note: mmap failed (%s), vertex data left zeroed\n",
               strerror(errno));
        return;
    }
    memcpy(p, verts, sizeof(verts));
    munmap(p, BO_SIZE);
}

static void fill_hindex_none(struct drm_vc4_submit_cl *s)
{
    s->color_read.hindex = ~0u;
    s->color_write.hindex = ~0u;
    s->zs_read.hindex = ~0u;
    s->zs_write.hindex = ~0u;
    s->msaa_color_write.hindex = ~0u;
    s->msaa_zs_write.hindex = ~0u;
}

/* Little-endian emitters for the bin control list. */
static uint8_t bin[128];
static unsigned bin_len;

static void e8(uint8_t v)  { bin[bin_len++] = v; }
static void e16(uint16_t v) { e8(v & 0xff); e8(v >> 8); }
static void e32(uint32_t v) { e16(v & 0xffff); e16(v >> 16); }
static void ef(float v)     { uint32_t u; memcpy(&u, &v, 4); e32(u); }

int main(int argc, char **argv)
{
    const char *mode = argc > 1 ? argv[1] : "array";
    bool indexed = !strcmp(mode, "indexed");
    uint64_t shader[3];
    uint32_t fs, vs, cs, vbo, fbo, ibo = 0;
    uint32_t bo_handles[6];
    uint32_t bo_count;
    uint8_t rec[16 + 44];
    uint32_t *rec_handles = (uint32_t *)rec;
    uint8_t *pkt = rec + 16;
    uint32_t uniforms[4];
    struct drm_vc4_submit_cl s;
    unsigned prim_off;
    int fd, r;

    if (strcmp(mode, "array") && strcmp(mode, "indexed")) {
        printf("usage: vc4_p1_draw [array|indexed]\n");
        return 2;
    }

    fd = open_dri();
    if (fd < 0) {
        printf("P1: FAIL: cannot open a DRI node (%s)\n", strerror(errno));
        return 2;
    }

    shader[0] = qpu_nop(QPU_SIG_PROG_END);
    shader[1] = qpu_nop(QPU_SIG_NONE);
    shader[2] = qpu_nop(QPU_SIG_NONE);
    if (create_shader_bo(fd, shader, sizeof(shader), &fs) != 0 ||
        create_shader_bo(fd, shader, sizeof(shader), &vs) != 0 ||
        create_shader_bo(fd, shader, sizeof(shader), &cs) != 0) {
        printf("P1: FAIL: CREATE_SHADER_BO: %s\n", strerror(errno));
        return 2;
    }
    if (create_bo(fd, BO_SIZE, &vbo) != 0 ||
        create_bo(fd, BO_SIZE, &fbo) != 0 ||
        (indexed && create_bo(fd, BO_SIZE, &ibo) != 0)) {
        printf("P1: FAIL: CREATE_BO: %s\n", strerror(errno));
        return 2;
    }
    fill_vertices(fd, vbo);

    bo_handles[0] = fs;
    bo_handles[1] = vs;
    bo_handles[2] = cs;
    bo_handles[3] = vbo;
    bo_handles[4] = fbo;
    bo_handles[5] = ibo;
    bo_count = indexed ? 6 : 5;

    /*
     * Bin control list.  The validator rewrites the binning config addresses
     * and the shader-record and index-buffer pointers into absolute DMA
     * addresses before the GPU sees them.
     */
    e8(P_TILE_BINNING_CONFIG);
    e32(0);                       /* tile alloc address  (validator fills in) */
    e32(0);                       /* tile alloc size     (validator fills in) */
    e32(0);                       /* tile state address  (validator fills in) */
    e8(1);                        /* tiles in x */
    e8(1);                        /* tiles in y */
    e8(0);                        /* flags */

    e8(P_START_TILE_BINNING);

    e8(P_CONFIGURATION_BITS);
    e8(0x03);                     /* rasterize front and back faces */
    e8(0x00);
    e8(0x00);

    e8(P_CLIP_WINDOW);
    e16(0); e16(0); e16(FB_WIDTH); e16(FB_HEIGHT);

    e8(P_VIEWPORT_OFFSET);
    e16(0); e16(0);

    e8(P_CLIPPER_XY_SCALING);
    ef(FB_WIDTH * 16 / 2.0f); ef(-(FB_HEIGHT * 16 / 2.0f));

    e8(P_CLIPPER_Z_SCALING);
    ef(0.5f); ef(0.5f);

    e8(P_PRIMITIVE_LIST_FORMAT);
    e8((1 << 4) | 2);             /* 16-bit indices, triangles */

    if (indexed) {
        /* The indexed primitive takes its index BO from bo_index[0], which
         * this packet loads; it is consumed by the validator and never
         * reaches the GPU. */
        e8(P_GEM_HANDLES);
        e32(5);                   /* bo_handles[] index of the index buffer */
        e32(0);
    }

    /* One attribute, non-extended record: the low bits of the shader record
     * reference carry the attribute count. */
    e8(P_GL_SHADER_STATE);
    e32(1);

    prim_off = bin_len;
    if (indexed) {
        e8(P_GL_INDEXED_PRIMITIVE);
        e8(INDEX_BUFFER_U16 | PRIM_TRIANGLES);
        e32(VERTEX_COUNT);        /* number of indices */
        e32(0);                   /* offset into the index BO */
        e32(VERTEX_COUNT - 1);    /* maximum index */
    } else {
        e8(P_GL_ARRAY_PRIMITIVE);
        e8(PRIM_TRIANGLES);
        e32(VERTEX_COUNT);        /* number of vertices */
        e32(0);                   /* base index */
    }

    /* The kernel requires the bin list to end with exactly these two. */
    e8(P_INCREMENT_SEMAPHORE);
    e8(P_FLUSH);

    /*
     * Shader record: four BO-list indices (fs, vs, cs, then one per
     * attribute) followed by the 36 + 8 byte record itself.
     */
    memset(rec, 0, sizeof(rec));
    rec_handles[0] = 0;           /* fs */
    rec_handles[1] = 1;           /* vs */
    rec_handles[2] = 2;           /* cs */
    rec_handles[3] = 3;           /* vbo for attribute 0 */
    pkt[0] = 1;                   /* flags: FS is single-threaded */
    /* +4/+16/+28 are the fs/vs/cs code offsets and must be 0; +8/+20/+32 are
     * the uniform pointers, which the kernel fills in. */
    pkt[36 + 4] = ATTR_SIZE - 1;  /* attribute 0: bytes per element - 1 */
    pkt[36 + 5] = ATTR_STRIDE;    /* attribute 0: stride */

    memset(uniforms, 0, sizeof(uniforms));
    memset(&s, 0, sizeof(s));
    s.bin_cl = (uintptr_t)bin;
    s.bin_cl_size = bin_len;
    s.shader_rec = (uintptr_t)rec;
    s.shader_rec_size = sizeof(rec);
    s.shader_rec_count = 1;
    s.uniforms = (uintptr_t)uniforms;
    s.uniforms_size = sizeof(uniforms);
    s.bo_handles = (uintptr_t)bo_handles;
    s.bo_handle_count = bo_count;
    s.width = FB_WIDTH;
    s.height = FB_HEIGHT;
    fill_hindex_none(&s);
    s.color_write.hindex = 4;
    s.color_write.bits = 1 << 2;  /* VC4_RENDER_CONFIG_FORMAT_RGBA8888 */
    s.flags = VC4_SUBMIT_CL_USE_CLEAR_COLOR;
    s.clear_color[0] = 0xff00ff00;
    s.clear_color[1] = 0xff00ff00;
    s.clear_z = 0xffffff;
    s.clear_s = 0;

    printf("P1: submitting %s draw: bin_cl %u bytes, primitive at +%u,\n"
           "P1:   %u vertices x 1 attribute of %u bytes (stride %u),"
           " %ux%u framebuffer\n",
           mode, bin_len, prim_off, VERTEX_COUNT, ATTR_SIZE, ATTR_STRIDE,
           FB_WIDTH, FB_HEIGHT);
    if (indexed) {
        printf("P1:   index buffer: %u 16-bit indices (%u bytes)\n",
               VERTEX_COUNT, VERTEX_COUNT * 2);
    }
    printf("P1: expect the model to fetch %u vertices = %u attribute bytes\n",
           VERTEX_COUNT, VERTEX_COUNT * ATTR_SIZE);
    fflush(stdout);

    errno = 0;
    r = ioctl(fd, DRM_IOCTL_VC4_SUBMIT_CL, &s);
    if (r != 0) {
        printf("P1: FAIL: SUBMIT_CL: %s\n", strerror(errno));
        close(fd);
        return 1;
    }
    printf("P1: SUBMIT_CL ok, seqno %llu\n", (unsigned long long)s.seqno);

    if (s.seqno) {
        struct drm_vc4_wait_seqno wait;

        memset(&wait, 0, sizeof(wait));
        wait.seqno = s.seqno;
        wait.timeout_ns = 2ull * 1000 * 1000 * 1000;
        errno = 0;
        if (ioctl(fd, DRM_IOCTL_VC4_WAIT_SEQNO, &wait) != 0) {
            printf("P1: FAIL: WAIT_SEQNO: %s\n", strerror(errno));
            close(fd);
            return 1;
        }
        printf("P1: job %llu completed\n", (unsigned long long)s.seqno);
    }

    printf("P1: PASS (%s)\n", mode);
    close(fd);
    return 0;
}
