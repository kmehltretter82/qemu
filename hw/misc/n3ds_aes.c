/*
 * Nintendo 3DS ARM9 AES engine (0x10009000).
 *
 * Register-level model after 3dbrew "AES Registers": word FIFOs with
 * selectable byte/word order, 0x40 keyslots fed through the KEY/KEYX/KEYY
 * FIFOs (keyY completion runs the 3DS or DSi key scrambler), CTR/CBC/ECB
 * on AES-128 via QEMU's crypto/aes. CCM/MAC is not implemented (data is
 * passed through and the MAC reported as valid). No DMA requests.
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
#include "crypto/aes.h"
#include "hw/misc/n3ds.h"
#include "trace.h"

#define TYPE_N3DS_AES "n3ds-aes"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSAesState, N3DS_AES)

#define REG_CNT        0x000
#define REG_BLKCNT     0x004   /* [15:0] MAC blocks, [31:16] data blocks */
#define REG_WRFIFO     0x008
#define REG_RDFIFO     0x00C
#define REG_KEYSEL     0x010
#define REG_KEYCNT     0x011
#define REG_CTR        0x020
#define REG_MAC        0x030
#define REG_KEY0       0x040   /* 4 x {normal, x, y} DSi-style key registers */
#define REG_KEYFIFO    0x100
#define REG_KEYXFIFO   0x104
#define REG_KEYYFIFO   0x108

#define CNT_FLUSH_WR   (1u << 10)
#define CNT_FLUSH_RD   (1u << 11)
#define CNT_MAC_STATUS (1u << 21)
#define CNT_OUT_BE     (1u << 22)
#define CNT_IN_BE      (1u << 23)
#define CNT_OUT_NORMAL (1u << 24)
#define CNT_IN_NORMAL  (1u << 25)
#define CNT_KEY_UPDATE (1u << 26)
#define CNT_MODE_SHIFT 27
#define CNT_IRQ_EN     (1u << 30)
#define CNT_START      (1u << 31)
#define CNT_STORED_MASK 0xFFFFF000u   /* bits kept from a write (minus 26) */

#define FIFO_WORDS     16

struct N3DSAesState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t cnt;
    uint16_t mac_blkcnt;
    uint16_t blkcnt;
    uint8_t keysel;
    uint8_t keycnt;
    uint8_t ctr[16];
    uint8_t mac[16];
    struct {
        uint8_t normal[16];
        uint8_t x[16];
        uint8_t y[16];
    } key[0x40];
    AES_KEY enc;
    AES_KEY dec;

    /* input: words are assembled into 16-byte blocks, then queued */
    uint8_t in_block[16];
    int in_word;
    uint8_t in_fifo[FIFO_WORDS * 4];
    int in_words;
    /* output: transformed words */
    uint32_t out_fifo[FIFO_WORDS];
    int out_words;
    uint32_t last_out;

    uint8_t keyfifo[3][16];
    int keyfifo_ctr[3];
};

/* ---- 128-bit helpers on big-endian byte arrays ---- */

static void be128_get(const uint8_t *b, uint64_t *hi, uint64_t *lo)
{
    *hi = ldq_be_p(b);
    *lo = ldq_be_p(b + 8);
}

static void be128_put(uint8_t *b, uint64_t hi, uint64_t lo)
{
    stq_be_p(b, hi);
    stq_be_p(b + 8, lo);
}

static void u128_rol(uint64_t *hi, uint64_t *lo, unsigned n)
{
    uint64_t h = *hi, l = *lo;

    n &= 127;
    if (n >= 64) {
        uint64_t t = h; h = l; l = t; n -= 64;
    }
    if (n) {
        uint64_t nh = (h << n) | (l >> (64 - n));
        uint64_t nl = (l << n) | (h >> (64 - n));
        h = nh; l = nl;
    }
    *hi = h; *lo = l;
}

static void u128_add(uint64_t *hi, uint64_t *lo, uint64_t chi, uint64_t clo)
{
    uint64_t l = *lo + clo;

    *hi += chi + (l < clo);
    *lo = l;
}

/* 3DS: normal = ror((rol(X, 2) ^ Y) + C, 41) */
static void n3ds_aes_keygen_ctr(uint8_t *out, const uint8_t *x, const uint8_t *y)
{
    uint64_t xh, xl, yh, yl;

    be128_get(x, &xh, &xl);
    be128_get(y, &yh, &yl);
    u128_rol(&xh, &xl, 2);
    xh ^= yh; xl ^= yl;
    u128_add(&xh, &xl, 0x1FF9E9AAC5FE0408ull, 0x024591DC5D52768Aull);
    u128_rol(&xh, &xl, 128 - 41);
    be128_put(out, xh, xl);
}

/* DSi: normal = rol((X ^ Y) + C, 42) */
static void n3ds_aes_keygen_twl(uint8_t *out, const uint8_t *x, const uint8_t *y)
{
    uint64_t xh, xl, yh, yl;

    be128_get(x, &xh, &xl);
    be128_get(y, &yh, &yl);
    xh ^= yh; xl ^= yl;
    u128_add(&xh, &xl, 0xFFFEFB4E29590258ull, 0x2A680F5F1A4F3E79ull);
    u128_rol(&xh, &xl, 42);
    be128_put(out, xh, xl);
}

/* ---- word <-> byte array conversions per CNT endian/order bits ---- */

static void aes_input_word(N3DSAesState *s, uint8_t *vec, int index,
                           uint32_t value, bool force_order)
{
    if (!(s->cnt & CNT_IN_NORMAL) && !force_order) {
        index = 3 - index;
    }
    if (!(s->cnt & CNT_IN_BE)) {
        value = bswap32(value);
    }
    stl_le_p(vec + index * 4, value);
}

static uint32_t aes_output_word(N3DSAesState *s, const uint8_t *vec, int index)
{
    uint32_t value;

    if (!(s->cnt & CNT_OUT_NORMAL)) {
        index = 3 - index;
    }
    value = ldl_le_p(vec + index * 4);
    if (!(s->cnt & CNT_OUT_BE)) {
        value = bswap32(value);
    }
    return value;
}

static void aes_select_key(N3DSAesState *s)
{
    const uint8_t *k = s->key[s->keysel & 0x3F].normal;

    AES_set_encrypt_key(k, 128, &s->enc);
    AES_set_decrypt_key(k, 128, &s->dec);
}

static void aes_ctr_increment(uint8_t *ctr)
{
    int i;

    for (i = 15; i >= 0; i--) {
        if (++ctr[i]) {
            break;
        }
    }
}

static void aes_process(N3DSAesState *s)
{
    while ((s->cnt & CNT_START) && s->in_words >= 4 &&
           s->out_words <= FIFO_WORDS - 4) {
        uint8_t in[16], out[16], tmp[16];
        int mode = (s->cnt >> CNT_MODE_SHIFT) & 7;
        int i;

        memcpy(in, s->in_fifo, 16);
        memmove(s->in_fifo, s->in_fifo + 16, (s->in_words - 4) * 4);
        s->in_words -= 4;

        switch (mode) {
        case 0: /* CCM decrypt */
        case 1: /* CCM encrypt */
            qemu_log_mask(LOG_UNIMP, "n3ds-aes: CCM mode not implemented\n");
            memcpy(out, in, 16);
            s->cnt |= CNT_MAC_STATUS;
            break;
        case 2:
        case 3: /* CTR */
            AES_encrypt(s->ctr, tmp, &s->enc);
            for (i = 0; i < 16; i++) {
                out[i] = in[i] ^ tmp[i];
            }
            aes_ctr_increment(s->ctr);
            break;
        case 4: /* CBC decrypt */
            AES_decrypt(in, tmp, &s->dec);
            for (i = 0; i < 16; i++) {
                out[i] = tmp[i] ^ s->ctr[i];
            }
            memcpy(s->ctr, in, 16);
            break;
        case 5: /* CBC encrypt */
            for (i = 0; i < 16; i++) {
                tmp[i] = in[i] ^ s->ctr[i];
            }
            AES_encrypt(tmp, out, &s->enc);
            memcpy(s->ctr, out, 16);
            break;
        case 6: /* ECB decrypt */
            AES_decrypt(in, out, &s->dec);
            break;
        default: /* 7: ECB encrypt */
            AES_encrypt(in, out, &s->enc);
            break;
        }

        for (i = 0; i < 4; i++) {
            s->out_fifo[s->out_words++] = aes_output_word(s, out, i);
        }
        if (s->blkcnt) {
            s->blkcnt--;
        }
        if (s->blkcnt == 0) {
            s->cnt &= ~CNT_START;
            if (s->cnt & CNT_IRQ_EN) {
                qemu_irq_pulse(s->irq);
            }
        }
    }
}

static void aes_write_fifo(N3DSAesState *s, uint32_t value)
{
    aes_input_word(s, s->in_block, s->in_word, value, false);
    if (++s->in_word == 4) {
        s->in_word = 0;
        if (s->in_words <= FIFO_WORDS - 4) {
            memcpy(s->in_fifo + s->in_words * 4, s->in_block, 16);
            s->in_words += 4;
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "n3ds-aes: write FIFO overflow\n");
        }
    }
    aes_process(s);
}

static uint32_t aes_read_fifo(N3DSAesState *s)
{
    if (s->out_words) {
        s->last_out = s->out_fifo[0];
        memmove(s->out_fifo, s->out_fifo + 1, (s->out_words - 1) * 4);
        s->out_words--;
    }
    aes_process(s);
    return s->last_out;
}

static void aes_write_keyfifo(N3DSAesState *s, int which, uint32_t value)
{
    int slot = s->keycnt & 0x3F;

    aes_input_word(s, s->keyfifo[which], s->keyfifo_ctr[which], value, false);
    if (++s->keyfifo_ctr[which] < 4) {
        return;
    }
    s->keyfifo_ctr[which] = 0;
    trace_n3ds_aes_keyfifo(which, slot, s->keycnt);
    switch (which) {
    case 0:
        memcpy(s->key[slot].normal, s->keyfifo[0], 16);
        break;
    case 1:
        memcpy(s->key[slot].x, s->keyfifo[1], 16);
        break;
    default:
        memcpy(s->key[slot].y, s->keyfifo[2], 16);
        if (s->keycnt & 0x40) {
            n3ds_aes_keygen_twl(s->key[slot].normal, s->key[slot].x,
                                s->key[slot].y);
        } else {
            n3ds_aes_keygen_ctr(s->key[slot].normal, s->key[slot].x,
                                s->key[slot].y);
        }
        break;
    }
}

static uint64_t n3ds_aes_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSAesState *s = opaque;
    uint32_t v = 0;

    switch (off & ~3) {
    case REG_CNT:
        v = s->cnt | s->in_words | (s->out_words << 5);
        break;
    case REG_BLKCNT:
        v = s->mac_blkcnt | ((uint32_t)s->blkcnt << 16);
        break;
    case REG_RDFIFO: {
        uint32_t r = aes_read_fifo(s);
        trace_n3ds_aes_rdfifo(r);
        return r;
    }
    case REG_KEYSEL:
        v = s->keysel | ((uint32_t)s->keycnt << 8);
        break;
    default:
        break;
    }
    return (v >> (8 * (off & 3))) & (size == 4 ? 0xFFFFFFFF :
                                     size == 2 ? 0xFFFF : 0xFF);
}

static void n3ds_aes_write(void *opaque, hwaddr off, uint64_t val,
                           unsigned size)
{
    N3DSAesState *s = opaque;
    uint32_t v = val;

    trace_n3ds_aes_write((uint32_t)off, v, size);

    if (off >= REG_CTR && off < REG_CTR + 16) {
        aes_input_word(s, s->ctr, 3 - ((off - REG_CTR) / 4), v, true);
        return;
    }
    if (off >= REG_MAC && off < REG_MAC + 16) {
        aes_input_word(s, s->mac, 3 - ((off - REG_MAC) / 4), v, true);
        return;
    }
    if (off >= REG_KEY0 && off < REG_KEY0 + 4 * 0x30) {
        int slot = (off - REG_KEY0) / 0x30;
        int part = ((off - REG_KEY0) % 0x30) / 0x10;
        int idx = 3 - (((off - REG_KEY0) % 0x10) / 4);
        uint8_t *dst = part == 0 ? s->key[slot].normal :
                       part == 1 ? s->key[slot].x : s->key[slot].y;

        aes_input_word(s, dst, idx, v, true);
        if (part == 2 && idx == 0) {
            n3ds_aes_keygen_twl(s->key[slot].normal, s->key[slot].x,
                                s->key[slot].y);
        }
        return;
    }

    switch (off) {
    case REG_CNT:
        if (size != 4) {
            break;
        }
        if ((s->cnt ^ v) & CNT_IN_NORMAL) {
            memset(s->keyfifo_ctr, 0, sizeof(s->keyfifo_ctr));
        }
        if (v & CNT_FLUSH_WR) {
            s->in_words = 0;
            s->in_word = 0;
        }
        if (v & CNT_FLUSH_RD) {
            s->out_words = 0;
        }
        s->cnt = (v & CNT_STORED_MASK & ~CNT_KEY_UPDATE) |
                 (s->cnt & CNT_MAC_STATUS);
        if (v & CNT_KEY_UPDATE) {
            aes_select_key(s);
        }
        if (v & CNT_START) {
            trace_n3ds_aes_start((v >> CNT_MODE_SHIFT) & 7, s->keysel, s->blkcnt, v);
        }
        aes_process(s);
        break;
    case REG_BLKCNT:
        if (size == 4) {
            s->mac_blkcnt = v & 0xFFFF;
            s->blkcnt = v >> 16;
        } else {
            s->mac_blkcnt = v & 0xFFFF;
        }
        break;
    case REG_BLKCNT + 2:
        s->blkcnt = v & 0xFFFF;
        break;
    case REG_WRFIFO:
        trace_n3ds_aes_wrfifo(v);
        aes_write_fifo(s, v);
        break;
    case REG_KEYSEL:
        s->keysel = v & 0xFF;
        if (size >= 2) {
            s->keycnt = (v >> 8) & 0xFF;
        }
        break;
    case REG_KEYCNT:
        s->keycnt = v & 0xFF;
        if (s->keycnt & 0x80) {
            memset(s->keyfifo_ctr, 0, sizeof(s->keyfifo_ctr));
        }
        break;
    case REG_KEYFIFO:
        aes_write_keyfifo(s, 0, v);
        break;
    case REG_KEYXFIFO:
        aes_write_keyfifo(s, 1, v);
        break;
    case REG_KEYYFIFO:
        aes_write_keyfifo(s, 2, v);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "n3ds-aes: unimplemented write 0x%03x = 0x%x\n",
                      (unsigned)off, v);
        break;
    }
}

static const MemoryRegionOps n3ds_aes_ops = {
    .read = n3ds_aes_read,
    .write = n3ds_aes_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void n3ds_aes_reset(DeviceState *dev)
{
    N3DSAesState *s = N3DS_AES(dev);

    /* AES_CNT as the chainloader leaves it (n3ds-probes io_crypto) */
    s->cnt = 0x33c03000;
    s->mac_blkcnt = s->blkcnt = 0;
    s->keysel = s->keycnt = 0;
    memset(s->ctr, 0, sizeof(s->ctr));
    memset(s->mac, 0, sizeof(s->mac));
    memset(s->key, 0, sizeof(s->key));
    s->in_word = s->in_words = s->out_words = 0;
    s->last_out = 0;
    memset(s->keyfifo_ctr, 0, sizeof(s->keyfifo_ctr));
    aes_select_key(s);
}

static void n3ds_aes_init(Object *obj)
{
    N3DSAesState *s = N3DS_AES(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_aes_ops, s, "n3ds-aes", 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void n3ds_aes_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_aes_reset);
}

static const TypeInfo n3ds_aes_info = {
    .name = TYPE_N3DS_AES,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSAesState),
    .instance_init = n3ds_aes_init,
    .class_init = n3ds_aes_class_init,
};

static void n3ds_aes_register_types(void)
{
    type_register_static(&n3ds_aes_info);
}

type_init(n3ds_aes_register_types)
