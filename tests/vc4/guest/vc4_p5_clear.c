/*
 * Pixel test for the QEMU bcm2835-v3d tile buffer / TLB store path.
 *
 * Submits a render-only job (no bin control list) that clears a framebuffer to
 * a known colour, then maps the framebuffer BO and checks every pixel.  This
 * is the first test that verifies the model produces *pixels* rather than a
 * trace: a clear needs the tile buffer and the tile store, but no shading and
 * no rasterizer, so it can pass before either exists.
 *
 *   usage: vc4_p5_clear [width height]      (default: 16 16)
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/vc4_drm.h>

#define CLEAR_COLOR 0xdeadbeefu

/* VC4_RENDER_CONFIG_FORMAT_RGBA8888 in bits 3:2, linear memory layout. */
#define RENDER_BITS_RGBA8888_LINEAR (1 << 2)

static int open_dri(void)
{
    static const char *paths[] = { "/dev/dri/card0", "/dev/dri/card1",
                                   "/dev/dri/renderD128" };
    for (unsigned i = 0; i < 3; i++) {
        int fd = open(paths[i], O_RDWR | O_CLOEXEC);
        if (fd >= 0)
            return fd;
    }
    return -1;
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

static void *map_bo(int fd, uint32_t handle, uint32_t size)
{
    struct drm_vc4_mmap_bo map;
    void *p;

    memset(&map, 0, sizeof(map));
    map.handle = handle;
    if (ioctl(fd, DRM_IOCTL_VC4_MMAP_BO, &map) != 0) {
        printf("P5: FAIL: MMAP_BO: %s\n", strerror(errno));
        return NULL;
    }
    p = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
             (off_t)map.offset);
    if (p == MAP_FAILED) {
        printf("P5: FAIL: mmap: %s\n", strerror(errno));
        return NULL;
    }
    return p;
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

int main(int argc, char **argv)
{
    unsigned width = argc > 2 ? (unsigned)atoi(argv[1]) : 16;
    unsigned height = argc > 2 ? (unsigned)atoi(argv[2]) : 16;
    /* The surface is padded to whole 4-pixel microtiles, which is what the
     * kernel's own bounds check assumes for a linear 32-bit surface. */
    unsigned stride_px = (width + 3) & ~3u;
    uint32_t size = stride_px * height * 4;
    uint32_t handle;
    uint32_t bo_handles[1];
    struct drm_vc4_submit_cl s;
    uint32_t *fb;
    unsigned x, y, bad = 0;
    int fd, r;

    if (width == 0 || height == 0 || width > 64 || height > 64) {
        printf("P5: FAIL: size must be 1..64 in each dimension\n");
        return 2;
    }
    if (size < 4096) {
        size = 4096;    /* BOs are page granular anyway */
    }

    fd = open_dri();
    if (fd < 0) {
        printf("P5: FAIL: cannot open a DRI node (%s)\n", strerror(errno));
        return 2;
    }
    if (create_bo(fd, size, &handle) != 0) {
        printf("P5: FAIL: CREATE_BO: %s\n", strerror(errno));
        return 2;
    }

    fb = map_bo(fd, handle, size);
    if (!fb) {
        return 2;
    }
    /* Poison the buffer so a store that never happens cannot pass. */
    memset(fb, 0x5a, size);

    bo_handles[0] = handle;
    memset(&s, 0, sizeof(s));
    /* No bin control list: this job is a clear, so there is no geometry and
     * the kernel emits a render list with no sub-list branches. */
    s.bo_handles = (uintptr_t)bo_handles;
    s.bo_handle_count = 1;
    s.width = width;
    s.height = height;
    fill_hindex_none(&s);
    s.color_write.hindex = 0;
    s.color_write.bits = RENDER_BITS_RGBA8888_LINEAR;
    s.flags = VC4_SUBMIT_CL_USE_CLEAR_COLOR;
    s.clear_color[0] = CLEAR_COLOR;
    s.clear_color[1] = CLEAR_COLOR;
    s.clear_z = 0xffffff;
    s.clear_s = 0;

    printf("P5: clearing a %ux%u RGBA8888 surface (stride %u px) to 0x%08x\n",
           width, height, stride_px, CLEAR_COLOR);
    fflush(stdout);

    errno = 0;
    r = ioctl(fd, DRM_IOCTL_VC4_SUBMIT_CL, &s);
    if (r != 0) {
        printf("P5: FAIL: SUBMIT_CL: %s\n", strerror(errno));
        return 1;
    }
    if (s.seqno) {
        struct drm_vc4_wait_seqno wait;

        memset(&wait, 0, sizeof(wait));
        wait.seqno = s.seqno;
        wait.timeout_ns = 2ull * 1000 * 1000 * 1000;
        errno = 0;
        if (ioctl(fd, DRM_IOCTL_VC4_WAIT_SEQNO, &wait) != 0) {
            printf("P5: FAIL: WAIT_SEQNO: %s\n", strerror(errno));
            return 1;
        }
    }

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            uint32_t got = fb[y * stride_px + x];

            if (got != CLEAR_COLOR) {
                if (bad < 4) {
                    printf("P5: pixel (%u,%u) = 0x%08x, expected 0x%08x\n",
                           x, y, got, CLEAR_COLOR);
                }
                bad++;
            }
        }
    }

    if (bad) {
        printf("P5: FAIL: %u of %u pixels wrong\n", bad, width * height);
        return 1;
    }
    printf("P5: PASS: all %u pixels are 0x%08x\n", width * height,
           CLEAR_COLOR);
    close(fd);
    return 0;
}
