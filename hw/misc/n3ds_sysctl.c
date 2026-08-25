/*
 * Nintendo 3DS system control blocks: CONFIG9, CONFIG, PRNG (ARM9 side),
 * CONFIG11 and PDN (shared), including the New 3DS ("LGR") core 2/3
 * release protocol and the SoC clock-mode acknowledge interrupt.
 *
 * Documentation: 3dbrew CONFIG9_Registers, CONFIG11_Registers,
 * PDN_Registers; observed usage in Luma3DS and firm_linux_loader.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/guest-random.h"
#include "system/memory.h"
#include "target/arm/arm-powerctl.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSSysctlState, N3DS_SYSCTL)

#define CFG9_EXTMEMCNT9     0x200
#define CFG11_BOOTROM_OVERLAY_CNT 0x420
#define CFG11_BOOTROM_OVERLAY_VAL 0x424
#define CFG11_SOCINFO       0xFFC
#define PDN_LGR_SOCMODE     0x300
#define PDN_LGR_CNT         0x304
#define PDN_LGR_CPU_CNT     0x310

struct N3DSSysctlState {
    SysBusDevice parent_obj;

    MemoryRegion cfg9;
    MemoryRegion cfg;
    MemoryRegion prng;
    MemoryRegion cfg11;
    MemoryRegion pdn;
    MemoryRegion *extwram;
    qemu_irq socmode_irq;

    uint8_t cfg9_regs[0x1000];
    uint8_t cfg_regs[0x1000];
    uint8_t cfg11_regs[0x1000];
    uint8_t pdn_regs[0x1000];
    uint32_t socinfo;
};

static uint64_t regs_read(const uint8_t *regs, hwaddr off, unsigned size)
{
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v |= (uint64_t)regs[off + i] << (8 * i);
    }
    return v;
}

static void regs_write(uint8_t *regs, hwaddr off, uint64_t v, unsigned size)
{
    unsigned i;

    for (i = 0; i < size; i++) {
        regs[off + i] = v >> (8 * i);
    }
}

/* ---- CONFIG9 (0x10000000, ARM9 only) ---- */

static uint64_t cfg9_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSSysctlState *s = opaque;
    return regs_read(s->cfg9_regs, off, size);
}

static void cfg9_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSSysctlState *s = opaque;

    regs_write(s->cfg9_regs, off, v, size);
    if (off <= CFG9_EXTMEMCNT9 && off + size > CFG9_EXTMEMCNT9 && s->extwram) {
        memory_region_set_enabled(s->extwram, s->cfg9_regs[CFG9_EXTMEMCNT9] & 1);
    }
}

static const MemoryRegionOps cfg9_ops = {
    .read = cfg9_read,
    .write = cfg9_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ---- CONFIG (0x10010000, ARM9 only): unit info etc. ---- */

static uint64_t cfg_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSSysctlState *s = opaque;
    return regs_read(s->cfg_regs, off, size);
}

static void cfg_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSSysctlState *s = opaque;
    regs_write(s->cfg_regs, off, v, size);
}

static const MemoryRegionOps cfg_ops = {
    .read = cfg_read,
    .write = cfg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ---- PRNG (0x10011000, ARM9 only) ---- */

static uint64_t prng_read(void *opaque, hwaddr off, unsigned size)
{
    uint64_t v = 0;
    qemu_guest_getrandom_nofail(&v, size);
    return v;
}

static void prng_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
}

static const MemoryRegionOps prng_ops = {
    .read = prng_read,
    .write = prng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- CONFIG11 (0x10140000, shared) ---- */

static uint64_t cfg11_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSSysctlState *s = opaque;

    if (off == CFG11_SOCINFO) {
        return s->socinfo;
    }
    return regs_read(s->cfg11_regs, off, size);
}

static void cfg11_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSSysctlState *s = opaque;

    if (off == CFG11_SOCINFO) {
        return;
    }
    regs_write(s->cfg11_regs, off, v, size);
}

static const MemoryRegionOps cfg11_ops = {
    .read = cfg11_read,
    .write = cfg11_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ---- PDN (0x10141000, shared) ---- */

static uint32_t pdn_core_entry(N3DSSysctlState *s)
{
    uint32_t cnt = regs_read(s->cfg11_regs, CFG11_BOOTROM_OVERLAY_CNT, 4);

    if (cnt & 1) {
        return regs_read(s->cfg11_regs, CFG11_BOOTROM_OVERLAY_VAL, 4);
    }
    return N3DS_BOOTROM_BASE;
}

static void pdn_cpu_cnt_write(N3DSSysctlState *s, int core, uint8_t v)
{
    uint8_t *reg = &s->pdn_regs[PDN_LGR_CPU_CNT + core];

    if (core < 2) {
        /* Cores 0/1 are always powered on this SoC model. */
        *reg = 0x10 | (v & 1);
        return;
    }
    if ((v & 1) && !(*reg & 0x10)) {
        uint32_t entry = pdn_core_entry(s);
        int ret = arm_set_cpu_on(core, entry, 0, 1, false);

        if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS &&
            ret != QEMU_ARM_POWERCTL_ALREADY_ON) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "n3ds-sysctl: cannot power on core %d: %d\n",
                          core, ret);
            return;
        }
        *reg = 0x11;
        return;
    }
    *reg = (*reg & 0x10) | (v & 1);
}

static uint64_t pdn_read(void *opaque, hwaddr off, unsigned size)
{
    N3DSSysctlState *s = opaque;
    return regs_read(s->pdn_regs, off, size);
}

static void pdn_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    N3DSSysctlState *s = opaque;
    unsigned i;

    if (off >= PDN_LGR_CPU_CNT && off < PDN_LGR_CPU_CNT + 4) {
        for (i = 0; i < size && off + i < PDN_LGR_CPU_CNT + 4; i++) {
            pdn_cpu_cnt_write(s, off + i - PDN_LGR_CPU_CNT, v >> (8 * i));
        }
        return;
    }
    if (off == PDN_LGR_SOCMODE) {
        uint16_t old = regs_read(s->pdn_regs, PDN_LGR_SOCMODE, 2);
        uint16_t new = (old & 0x8000) | (v & 0x7FFF);

        if ((old & 7) != (v & 7)) {
            /*
             * Clock mode change: set the acknowledge bit and hold the
             * interrupt until the guest writes the bit back.
             */
            new |= 0x8000;
            regs_write(s->pdn_regs, PDN_LGR_SOCMODE, new, 2);
            qemu_set_irq(s->socmode_irq, 1);
            return;
        }
        if (v & 0x8000) {
            new &= ~0x8000;
            qemu_set_irq(s->socmode_irq, 0);
        }
        regs_write(s->pdn_regs, PDN_LGR_SOCMODE, new, 2);
        return;
    }
    regs_write(s->pdn_regs, off, v, size);
}

static const MemoryRegionOps pdn_ops = {
    .read = pdn_read,
    .write = pdn_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void n3ds_sysctl_reset(DeviceState *dev)
{
    N3DSSysctlState *s = N3DS_SYSCTL(dev);

    memset(s->cfg9_regs, 0, sizeof(s->cfg9_regs));
    memset(s->cfg_regs, 0, sizeof(s->cfg_regs));
    memset(s->cfg11_regs, 0, sizeof(s->cfg11_regs));
    memset(s->pdn_regs, 0, sizeof(s->pdn_regs));
    /*
     * Values read on a New 2DS XL right after the Luma3DS chainloader
     * (n3ds-probes io_cfg9 / io_cfg11 / io_pdn snapshots).
     */
    regs_write(s->cfg9_regs, 0x000, 0x00000101, 4);   /* SYSPROT9/11 */
    regs_write(s->cfg9_regs, 0x008, 0x0000000c, 4);
    regs_write(s->cfg9_regs, 0x010, 0xffff0001, 4);
    regs_write(s->cfg9_regs, 0x014, 0x0000ffff, 4);
    regs_write(s->cfg11_regs, 0x000, 0x8c888480, 4); /* shared WRAM bank map */
    regs_write(s->cfg11_regs, 0x004, 0x9c989490, 4);
    regs_write(s->cfg11_regs, 0x008, 0x8c888480, 4);
    regs_write(s->cfg11_regs, 0x00c, 0x9c989490, 4);
    regs_write(s->pdn_regs, 0x200, 0x0001007f, 4);   /* PDN_GPU_CNT (Luma) */
    /* PDN_LGR_CPU_CNT and PDN_LGR_SOCMODE read 0 at hand-off */
    if (s->extwram) {
        memory_region_set_enabled(s->extwram, false);
    }
}

static void n3ds_sysctl_init(Object *obj)
{
    N3DSSysctlState *s = N3DS_SYSCTL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->cfg9, obj, &cfg9_ops, s, "n3ds.cfg9", 0x1000);
    memory_region_init_io(&s->cfg, obj, &cfg_ops, s, "n3ds.cfg", 0x1000);
    memory_region_init_io(&s->prng, obj, &prng_ops, s, "n3ds.prng", 0x1000);
    memory_region_init_io(&s->cfg11, obj, &cfg11_ops, s, "n3ds.cfg11", 0x1000);
    memory_region_init_io(&s->pdn, obj, &pdn_ops, s, "n3ds.pdn", 0x1000);
    sysbus_init_mmio(sbd, &s->cfg9);
    sysbus_init_mmio(sbd, &s->cfg);
    sysbus_init_mmio(sbd, &s->prng);
    sysbus_init_mmio(sbd, &s->cfg11);
    sysbus_init_mmio(sbd, &s->pdn);
    sysbus_init_irq(sbd, &s->socmode_irq);
}

static const Property n3ds_sysctl_properties[] = {
    DEFINE_PROP_LINK("extwram", N3DSSysctlState, extwram,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    /* CFG11_SOCINFO: 1 = CTR, 3 = LGR1 (New 3DS proto), 7 = LGR2 (New 3DS) */
    DEFINE_PROP_UINT32("socinfo", N3DSSysctlState, socinfo, 7),
};

static void n3ds_sysctl_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, n3ds_sysctl_reset);
    device_class_set_props(dc, n3ds_sysctl_properties);
}

static const TypeInfo n3ds_sysctl_info = {
    .name          = TYPE_N3DS_SYSCTL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(N3DSSysctlState),
    .instance_init = n3ds_sysctl_init,
    .class_init    = n3ds_sysctl_class_init,
};

static void n3ds_sysctl_register_types(void)
{
    type_register_static(&n3ds_sysctl_info);
}

type_init(n3ds_sysctl_register_types)
