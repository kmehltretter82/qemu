/*
 * Nintendo 3DS ARM9 SHA engine (0x1000A000): SHA-256/224/1 over a
 * 64-byte input FIFO window, hash readable as 8 words with selectable
 * endianness (3dbrew "SHA Registers"). No DMA.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "hw/misc/n3ds.h"
#include "trace.h"

#define TYPE_N3DS_SHA "n3ds-sha"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSShaState, N3DS_SHA)

#define REG_CNT     0x00
#define REG_BLKCNT  0x04
#define REG_HASH    0x40
#define REG_FIFO    0x80

#define CNT_START    (1u << 0)
#define CNT_FINAL    (1u << 1)
#define CNT_OUT_BE   (1u << 3)
#define CNT_MODE(c)  (((c) >> 4) & 3)   /* 0 SHA256, 1 SHA224, 2/3 SHA1 */
#define CNT_STORED   0x000007FCu

struct N3DSShaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t cnt;
    uint32_t h[8];
    uint8_t buf[64];
    int buf_len;
    uint64_t total;      /* bytes hashed so far */
    /*
     * Read-back FIFO: every completed 64-byte input block can be read back
     * through the FIFO window (CNT bit 9 reads 1 while data is there).
     * boot9 streams NAND data SD -> AES -> SHA FIFO -> memory this way.
     */
    uint8_t rfifo[64];
    int rfifo_pos;
    int rfifo_len;
};

/* ---- SHA-256 / SHA-1 compression ---- */

static const uint32_t k256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
    0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
    0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
    0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
    0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static void sha256_block(uint32_t *h, const uint8_t *p)
{
    uint32_t w[64], a, b, c, d, e, f, g, hh, t1, t2;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (; i < 64; i++) {
        uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = h[0]; b = h[1]; c = h[2]; d = h[3];
    e = h[4]; f = h[5]; g = h[6]; hh = h[7];
    for (i = 0; i < 64; i++) {
        t1 = hh + (ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25)) +
             ((e & f) ^ (~e & g)) + k256[i] + w[i];
        t2 = (ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22)) +
             ((a & b) ^ (a & c) ^ (b & c));
        hh = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

static void sha1_block(uint32_t *h, const uint8_t *p)
{
    uint32_t w[80], a, b, c, d, e, t;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = ldl_be_p(p + 4 * i);
    }
    for (; i < 80; i++) {
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    a = h[0]; b = h[1]; c = h[2]; d = h[3]; e = h[4];
    for (i = 0; i < 80; i++) {
        uint32_t f, k;

        if (i < 20) {
            f = (b & c) | (~b & d); k = 0x5a827999;
        } else if (i < 40) {
            f = b ^ c ^ d; k = 0x6ed9eba1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d); k = 0x8f1bbcdc;
        } else {
            f = b ^ c ^ d; k = 0xca62c1d6;
        }
        t = rol32(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol32(b, 30); b = a; a = t;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

static void sha_init_state(N3DSShaState *s)
{
    static const uint32_t iv256[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    };
    static const uint32_t iv224[8] = {
        0xc1059ed8, 0x367cd507, 0x3070dd17, 0xf70e5939,
        0xffc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4,
    };
    static const uint32_t iv1[8] = {
        0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0, 0, 0, 0,
    };

    switch (CNT_MODE(s->cnt)) {
    case 0:
        memcpy(s->h, iv256, sizeof(s->h));
        break;
    case 1:
        memcpy(s->h, iv224, sizeof(s->h));
        break;
    default:
        memcpy(s->h, iv1, sizeof(s->h));
        break;
    }
    s->buf_len = 0;
    s->total = 0;
    s->rfifo_len = s->rfifo_pos = 0;
}

static void sha_compress(N3DSShaState *s, const uint8_t *block)
{
    if (CNT_MODE(s->cnt) >= 2) {
        sha1_block(s->h, block);
    } else {
        sha256_block(s->h, block);
    }
}

static void sha_push_bytes(N3DSShaState *s, const uint8_t *p, int n)
{
    while (n--) {
        s->buf[s->buf_len++] = *p++;
        s->total++;
        if (s->buf_len == 64) {
            sha_compress(s, s->buf);
            memcpy(s->rfifo, s->buf, 64);
            s->rfifo_len = 64;
            s->rfifo_pos = 0;
            s->buf_len = 0;
        }
    }
}

static void sha_final(N3DSShaState *s)
{
    uint64_t bits = s->total * 8;
    uint8_t pad[72] = { 0x80 };
    int padlen = (s->buf_len < 56) ? 56 - s->buf_len : 120 - s->buf_len;

    stq_be_p(pad + padlen, bits);
    /* push without counting towards the message length */
    {
        uint64_t saved = s->total;
        sha_push_bytes(s, pad, padlen + 8);
        s->total = saved;
    }
}

static uint64_t n3ds_sha_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSShaState *s = opaque;
    uint32_t v = 0;

    trace_n3ds_sha_read((uint32_t)off, size);

    if (off >= REG_HASH && off < REG_HASH + 32) {
        v = s->h[(off - REG_HASH) / 4];
        if (s->cnt & CNT_OUT_BE) {
            v = bswap32(v);
        }
    } else if (off >= REG_FIFO && off < REG_FIFO + 64) {
        unsigned i;

        for (i = 0; i < size && s->rfifo_pos < s->rfifo_len; i++) {
            v |= (uint32_t)s->rfifo[s->rfifo_pos++] << (8 * i);
        }
        if (s->rfifo_pos >= s->rfifo_len) {
            s->rfifo_len = s->rfifo_pos = 0;
        }
        return v;
    } else {
        switch (off & ~3) {
        case REG_CNT:
            /* start/final read as done; bit 9: read-back data available */
            v = (s->cnt & ~(1u << 9)) | (s->rfifo_len ? 1u << 9 : 0);
            break;
        case REG_BLKCNT:
            v = s->total;
            break;
        default:
            break;
        }
    }
    return (v >> (8 * (off & 3))) & (size == 4 ? 0xFFFFFFFF :
                                     size == 2 ? 0xFFFF : 0xFF);
}

static void n3ds_sha_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    N3DSShaState *s = opaque;
    uint32_t v = val;

    trace_n3ds_sha_write((uint32_t)off, v, size);

    if (off >= REG_FIFO && off < REG_FIFO + 64) {
        uint8_t b[4];

        stl_le_p(b, v);
        sha_push_bytes(s, b, size);
        return;
    }
    if (off >= REG_HASH && off < REG_HASH + 32) {
        s->h[(off - REG_HASH) / 4] = (s->cnt & CNT_OUT_BE) ? bswap32(v) : v;
        return;
    }
    switch (off) {
    case REG_CNT:
        s->cnt = v & CNT_STORED;
        if (v & CNT_START) {
            sha_init_state(s);
        }
        if (v & CNT_FINAL) {
            sha_final(s);
            trace_n3ds_sha_final(CNT_MODE(s->cnt), (uint32_t)s->total, s->h[0]);
        }
        break;
    case REG_BLKCNT:
        s->total = v;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "n3ds-sha: unimplemented write 0x%02x = 0x%x\n",
                      (unsigned)off, v);
        break;
    }
}

static const MemoryRegionOps n3ds_sha_ops = {
    .read = n3ds_sha_read,
    .write = n3ds_sha_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void n3ds_sha_reset(DeviceState *dev)
{
    N3DSShaState *s = N3DS_SHA(dev);

    s->cnt = 0;
    sha_init_state(s);
}

static void n3ds_sha_init(Object *obj)
{
    N3DSShaState *s = N3DS_SHA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_sha_ops, s, "n3ds-sha", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void n3ds_sha_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_sha_reset);
}

static const TypeInfo n3ds_sha_info = {
    .name = TYPE_N3DS_SHA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSShaState),
    .instance_init = n3ds_sha_init,
    .class_init = n3ds_sha_class_init,
};

static void n3ds_sha_register_types(void)
{
    type_register_static(&n3ds_sha_info);
}

type_init(n3ds_sha_register_types)
