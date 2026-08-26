/*
 * Nintendo 3DS ARM9 RSA engine (0x1000B000): raw 2048-bit modular
 * exponentiation (3dbrew "RSA Registers"). Four keyslots hold exponent and
 * modulus; TXT ^ EXP mod MOD is written back to TXT. Montgomery
 * multiplication with 64-bit limbs; an even modulus yields 0 (as observed
 * on hardware). Endianness/word-order options as for the AES engine.
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

#define TYPE_N3DS_RSA "n3ds-rsa"
OBJECT_DECLARE_SIMPLE_TYPE(N3DSRsaState, N3DS_RSA)

#define REG_CNT       0x000
#define REG_UNK_F0    0x0F0
#define REG_SLOT      0x100   /* 4 x 0x10: CNT, SIZE, ?, ? */
#define REG_EXPFIFO   0x200   /* 0x100 */
#define REG_MOD       0x400   /* 0x100 */
#define REG_TXT       0x800   /* 0x100 */

#define CNT_START     (1u << 0)
#define CNT_IRQ_EN    (1u << 1)
#define CNT_SLOT(c)   (((c) >> 4) & 3)
#define CNT_LE        (1u << 8)
#define CNT_NORMAL    (1u << 9)

#define SLOTCNT_KEYSET  (1u << 0)
#define SLOTCNT_WPROT   (1u << 1)
#define SLOTCNT_RPROT   (1u << 2)
#define SLOTCNT_PROT    (1u << 31)

#define NLIMBS 32   /* 2048 bits */

struct N3DSRsaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t cnt;
    struct {
        uint32_t cnt;
        uint32_t size;      /* words */
        uint8_t exp[0x100]; /* big-endian, right-aligned */
        uint8_t mod[0x100];
        int mod_words;
        int exp_pos;        /* EXPFIFO: a real FIFO, the address is ignored */
    } slot[4];
    uint8_t txt[0x100];
};

typedef struct { uint64_t v[NLIMBS]; } bn_t;   /* little-endian limbs */

static void bn_from_be(bn_t *r, const uint8_t *be)
{
    int i;
    for (i = 0; i < NLIMBS; i++) {
        r->v[i] = ldq_be_p(be + 0x100 - 8 * (i + 1));
    }
}

static void bn_to_be(uint8_t *be, const bn_t *a)
{
    int i;
    for (i = 0; i < NLIMBS; i++) {
        stq_be_p(be + 0x100 - 8 * (i + 1), a->v[i]);
    }
}

static int bn_cmp(const bn_t *a, const bn_t *b)
{
    int i;
    for (i = NLIMBS - 1; i >= 0; i--) {
        if (a->v[i] != b->v[i]) {
            return a->v[i] > b->v[i] ? 1 : -1;
        }
    }
    return 0;
}

static void bn_sub(bn_t *r, const bn_t *a, const bn_t *b)
{
    uint64_t borrow = 0;
    int i;
    for (i = 0; i < NLIMBS; i++) {
        uint64_t d = a->v[i] - b->v[i] - borrow;
        borrow = (a->v[i] < b->v[i]) || (a->v[i] == b->v[i] && borrow);
        r->v[i] = d;
    }
}

static bool bn_is_zero(const bn_t *a)
{
    int i;
    for (i = 0; i < NLIMBS; i++) {
        if (a->v[i]) {
            return false;
        }
    }
    return true;
}

/* r = a * b * 2^-2048 mod n  (CIOS Montgomery; a < 2^2048, b < n) */
static void mont_mul(bn_t *r, const bn_t *a, const bn_t *b, const bn_t *n,
                     uint64_t n0inv)
{
    uint64_t t[NLIMBS + 2] = { 0 };
    int i, j;

    for (i = 0; i < NLIMBS; i++) {
        unsigned __int128 c = 0;
        uint64_t m;

        for (j = 0; j < NLIMBS; j++) {
            c += (unsigned __int128)a->v[j] * b->v[i] + t[j];
            t[j] = (uint64_t)c;
            c >>= 64;
        }
        c += t[NLIMBS];
        t[NLIMBS] = (uint64_t)c;
        t[NLIMBS + 1] = (uint64_t)(c >> 64);

        m = t[0] * n0inv;
        c = (unsigned __int128)m * n->v[0] + t[0];
        c >>= 64;
        for (j = 1; j < NLIMBS; j++) {
            c += (unsigned __int128)m * n->v[j] + t[j];
            t[j - 1] = (uint64_t)c;
            c >>= 64;
        }
        c += t[NLIMBS];
        t[NLIMBS - 1] = (uint64_t)c;
        c >>= 64;
        t[NLIMBS] = t[NLIMBS + 1] + (uint64_t)c;
        t[NLIMBS + 1] = 0;
    }
    memcpy(r->v, t, sizeof(r->v));
    if (t[NLIMBS] || bn_cmp(r, n) >= 0) {
        bn_sub(r, r, n);
    }
}

/* out = base ^ exp mod n; all 256-byte big-endian; 0 if n is even/zero */
static void rsa_modexp(uint8_t *out, const uint8_t *base_be,
                       const uint8_t *exp_be, const uint8_t *mod_be)
{
    bn_t n, x, r2, res, one, tmp;
    uint64_t inv;
    int i, k;

    bn_from_be(&n, mod_be);
    if (!(n.v[0] & 1)) {
        memset(out, 0, 0x100);
        return;
    }
    /* n0inv = -n^-1 mod 2^64 (Newton iteration) */
    inv = n.v[0];
    for (k = 0; k < 6; k++) {
        inv *= 2 - n.v[0] * inv;
    }
    inv = -inv;

    /* r2 = 2^4096 mod n, by 4096 doublings */
    memset(&r2, 0, sizeof(r2));
    r2.v[0] = 1;
    for (i = 0; i < 4096; i++) {
        uint64_t carry = r2.v[NLIMBS - 1] >> 63;
        for (k = NLIMBS - 1; k > 0; k--) {
            r2.v[k] = (r2.v[k] << 1) | (r2.v[k - 1] >> 63);
        }
        r2.v[0] <<= 1;
        if (carry || bn_cmp(&r2, &n) >= 0) {
            bn_sub(&r2, &r2, &n);
        }
    }
    bn_from_be(&x, base_be);
    mont_mul(&x, &x, &r2, &n, inv);          /* x in Montgomery form */
    memset(&one, 0, sizeof(one));
    one.v[0] = 1;
    mont_mul(&res, &r2, &one, &n, inv);      /* res = R mod n = mont(1) */

    bn_from_be(&tmp, exp_be);
    if (bn_is_zero(&tmp)) {
        /* x^0 = 1 */
        mont_mul(&res, &res, &one, &n, inv);
        bn_to_be(out, &res);
        return;
    }
    for (i = NLIMBS * 64 - 1; i >= 0; i--) {
        if ((tmp.v[i / 64] >> (i % 64)) & 1) {
            break;
        }
    }
    for (; i >= 0; i--) {
        mont_mul(&res, &res, &res, &n, inv);
        if ((tmp.v[i / 64] >> (i % 64)) & 1) {
            mont_mul(&res, &res, &x, &n, inv);
        }
    }
    mont_mul(&res, &res, &one, &n, inv);
    bn_to_be(out, &res);
}

/*
 * Register areas (EXPFIFO/MOD/TXT) are 0x100-byte buffers kept in
 * big-endian word form. A bus write lands its bytes in memory order; in the
 * "big endian" setting (CNT bit 8 clear) the byte lanes are swapped, so a
 * 32-bit write stores the value big-endian and a byte write to offset k
 * lands at k ^ 3 (boot9 writes the modulus and text byte by byte, least
 * significant byte first). The number for MOD/TXT is the area with the
 * word order reversed unless CNT bit 9 (normal order) is set; the exponent
 * FIFO is never reversed.
 */
static void rsa_area_write(N3DSRsaState *s, uint8_t *t, unsigned off,
                           uint32_t v, unsigned size)
{
    unsigned b;

    for (b = 0; b < size && off + b < 0x100; b++) {
        unsigned p = off + b;
        t[(s->cnt & CNT_LE) ? p : p ^ 3] = v >> (8 * b);
    }
}

static uint32_t rsa_area_read(N3DSRsaState *s, const uint8_t *t, unsigned off,
                              unsigned size)
{
    uint32_t v = 0;
    unsigned b;

    for (b = 0; b < size && off + b < 0x100; b++) {
        unsigned p = off + b;
        v |= (uint32_t)t[(s->cnt & CNT_LE) ? p : p ^ 3] << (8 * b);
    }
    return v;
}

static void rsa_number_from_area(N3DSRsaState *s, uint8_t *n, const uint8_t *t)
{
    int i;

    for (i = 0; i < 0x100; i++) {
        n[i] = (s->cnt & CNT_NORMAL) ? t[i] : t[4 * (63 - i / 4) + (i & 3)];
    }
}

static void rsa_area_from_number(N3DSRsaState *s, uint8_t *t, const uint8_t *n)
{
    int i;

    for (i = 0; i < 0x100; i++) {
        if (s->cnt & CNT_NORMAL) {
            t[i] = n[i];
        } else {
            t[4 * (63 - i / 4) + (i & 3)] = n[i];
        }
    }
}

static void rsa_start(N3DSRsaState *s)
{
    int slot = CNT_SLOT(s->cnt);
    uint8_t base[0x100], mod[0x100], out[0x100];

    if (!(s->slot[slot].cnt & SLOTCNT_KEYSET)) {
        qemu_log_mask(LOG_GUEST_ERROR, "n3ds-rsa: start with keyslot %d unset\n", slot);
    }
    rsa_number_from_area(s, base, s->txt);
    rsa_number_from_area(s, mod, s->slot[slot].mod);
    trace_n3ds_rsa_start(slot, s->cnt, ldl_be_p(base), ldl_be_p(base + 0xFC),
                         ldl_be_p(mod), ldl_be_p(s->slot[slot].exp + 0xFC));
    rsa_modexp(out, base, s->slot[slot].exp, mod);
    trace_n3ds_rsa_done(ldl_be_p(out), ldl_be_p(out + 0xFC));
    rsa_area_from_number(s, s->txt, out);
    s->cnt &= ~CNT_START;
    if (s->cnt & CNT_IRQ_EN) {
        qemu_irq_pulse(s->irq);
    }
}

static uint64_t n3ds_rsa_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSRsaState *s = opaque;
    uint32_t v = 0;

    if (off >= REG_TXT && off < REG_TXT + 0x100) {
        return rsa_area_read(s, s->txt, off - REG_TXT, size);
    } else if (off >= REG_SLOT && off < REG_SLOT + 0x40) {
        int slot = (off - REG_SLOT) / 0x10;
        switch ((off - REG_SLOT) & 0xC) {
        case 0: v = s->slot[slot].cnt; break;
        case 4: v = s->slot[slot].size; break;
        default: v = 0; break;
        }
    } else if (off >= REG_MOD && off < REG_MOD + 0x100) {
        return rsa_area_read(s, s->slot[CNT_SLOT(s->cnt)].mod, off - REG_MOD, size);
    } else if ((off & ~3) == REG_CNT) {
        v = s->cnt;
    } else if ((off & ~3) == REG_UNK_F0) {
        v = 0x20000000;      /* fixed value read on a New 2DS XL */
    }
    return (v >> (8 * (off & 3))) & (size == 4 ? 0xFFFFFFFF : size == 2 ? 0xFFFF : 0xFF);
}

static void n3ds_rsa_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    N3DSRsaState *s = opaque;
    uint32_t v = val;
    int slot = CNT_SLOT(s->cnt);

    trace_n3ds_rsa_write((uint32_t)off, v);

    if (off >= REG_TXT && off < REG_TXT + 0x100) {
        rsa_area_write(s, s->txt, off - REG_TXT, v, size);
        return;
    }
    if (off >= REG_EXPFIFO && off < REG_EXPFIFO + 0x100) {
        if (s->slot[slot].cnt & SLOTCNT_WPROT) {
            return;
        }
        rsa_area_write(s, s->slot[slot].exp, 4 * s->slot[slot].exp_pos, v, 4);
        s->slot[slot].exp_pos = (s->slot[slot].exp_pos + 1) & 63;
        return;
    }
    if (off >= REG_MOD && off < REG_MOD + 0x100) {
        if (s->slot[slot].cnt & SLOTCNT_WPROT) {
            return;
        }
        rsa_area_write(s, s->slot[slot].mod, off - REG_MOD, v, size);
        s->slot[slot].mod_words += size;
        if (s->slot[slot].mod_words >= 0x100) {
            s->slot[slot].mod_words = 0;
            s->slot[slot].cnt |= SLOTCNT_KEYSET;
        }
        return;
    }
    if (off >= REG_SLOT && off < REG_SLOT + 0x40) {
        int sl = (off - REG_SLOT) / 0x10;
        switch ((off - REG_SLOT) & 0xC) {
        case 0:
            if (!(s->slot[sl].cnt & SLOTCNT_PROT)) {
                s->slot[sl].cnt = v & (SLOTCNT_KEYSET | SLOTCNT_WPROT | SLOTCNT_RPROT | SLOTCNT_PROT);
                if (!(v & SLOTCNT_KEYSET)) {
                    s->slot[sl].mod_words = 0;
                    s->slot[sl].exp_pos = 0;
                }
            }
            break;
        case 4:
            s->slot[sl].size = v;
            break;
        default:
            break;
        }
        return;
    }
    switch (off) {
    case REG_CNT:
        s->cnt = v & (CNT_IRQ_EN | 0xF0 | CNT_LE | CNT_NORMAL);
        if (v & CNT_START) {
            s->cnt |= CNT_START;
            rsa_start(s);
        }
        break;
    case REG_UNK_F0:
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "n3ds-rsa: unimplemented write 0x%03x = 0x%x\n",
                      (unsigned)off, v);
        break;
    }
}

static const MemoryRegionOps n3ds_rsa_ops = {
    .read = n3ds_rsa_read,
    .write = n3ds_rsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static void n3ds_rsa_reset(DeviceState *dev)
{
    N3DSRsaState *s = N3DS_RSA(dev);
    int i;

    /* RSA_CNT as the chainloader leaves it (n3ds-probes io_crypto) */
    s->cnt = 0x00000030;
    memset(s->txt, 0, sizeof(s->txt));
    for (i = 0; i < 4; i++) {
        memset(&s->slot[i], 0, sizeof(s->slot[i]));
        s->slot[i].size = 0x40;
        /* the boot ROM leaves a key in every slot (n3ds-probes rsa_regs) */
        s->slot[i].cnt = SLOTCNT_KEYSET;
    }
}

static void n3ds_rsa_init(Object *obj)
{
    N3DSRsaState *s = N3DS_RSA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &n3ds_rsa_ops, s, "n3ds-rsa", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void n3ds_rsa_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_rsa_reset);
}

static const TypeInfo n3ds_rsa_info = {
    .name = TYPE_N3DS_RSA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSRsaState),
    .instance_init = n3ds_rsa_init,
    .class_init = n3ds_rsa_class_init,
};

static void n3ds_rsa_register_types(void)
{
    type_register_static(&n3ds_rsa_info);
}

type_init(n3ds_rsa_register_types)
