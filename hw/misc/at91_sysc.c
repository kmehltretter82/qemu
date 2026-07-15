/*
 * Atmel/Microchip AT91 system-controller reset path: RSTC / SHDWC / WDT.
 *
 * RSTC_CR (key 0xa5 + PROCRST/PERRST) requests a system reset; SHDW_CR (key +
 * SHDW) requests a shutdown; the WDT is a benign write-once stub that probes
 * but never fires.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "system/runstate.h"
#include "hw/misc/at91_sysc.h"

/*  RSTC / SHDWC / WDT (datasheet sections 11, 16, 15)                        */
/*                                                                           */
/*  Reset and shutdown are wired to the QEMU run-state so that guest reboot  */
/*  and poweroff behave correctly.  The watchdog is a benign stub: it        */
/*  accepts its (write-once) mode register at the datasheet reset value so   */
/*  the kernel driver probes cleanly, but never actually fires.              */
/* ======================================================================== */

/* KEY password (bits [31:24]) shared by RSTC_CR, SHDW_CR and WDT_CR. */
#define AT91_WPKEY  0xA5
static inline bool at91_key_ok(uint32_t val)
{
    return (val >> 24) == AT91_WPKEY;
}

/* --- RSTC (0xFFFFFD00) --- */
#define RSTC_CR   0x00
#define RSTC_SR   0x04
#define RSTC_MR   0x08
#define RSTC_CR_PROCRST  (1u << 0)
#define RSTC_CR_PERRST   (1u << 2)
#define RSTC_CR_EXTRST   (1u << 3)
#define RSTC_SR_RESET    0x00000001   /* general reset, NRST high, no cmd busy */

struct AT91RstcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
};

static uint64_t rstc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91RstcState *s = AT91_RSTC(opaque);

    switch (offset) {
    case RSTC_SR: return RSTC_SR_RESET;
    case RSTC_MR: return s->mr;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-rstc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void rstc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AT91RstcState *s = AT91_RSTC(opaque);
    uint32_t val = value;

    switch (offset) {
    case RSTC_CR:
        if (at91_key_ok(val) &&
            (val & (RSTC_CR_PROCRST | RSTC_CR_PERRST | RSTC_CR_EXTRST))) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    case RSTC_MR:
        if (at91_key_ok(val)) {
            s->mr = val & 0x00000f11;   /* URSTEN, URSTIEN, ERSTL */
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-rstc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps rstc_ops = {
    .read = rstc_read,
    .write = rstc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void rstc_reset(DeviceState *dev)
{
    AT91RstcState *s = AT91_RSTC(dev);
    s->mr = 0x00000001;
}

static void rstc_dev_init(Object *obj)
{
    AT91RstcState *s = AT91_RSTC(obj);
    memory_region_init_io(&s->iomem, obj, &rstc_ops, s, "at91-rstc", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_at91_rstc = {
    .name = "at91-rstc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91RstcState),
        VMSTATE_END_OF_LIST()
    }
};

static void rstc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, rstc_reset);
    dc->vmsd = &vmstate_at91_rstc;
}

static const TypeInfo rstc_type = {
    .name = TYPE_AT91_RSTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91RstcState),
    .instance_init = rstc_dev_init,
    .class_init = rstc_class_init,
};

/* --- SHDWC (0xFFFFFD10) --- */
#define SHDW_CR   0x00
#define SHDW_MR   0x04
#define SHDW_SR   0x08
#define SHDW_CR_SHDW  (1u << 0)

struct AT91ShdwcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
};

static uint64_t shdwc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91ShdwcState *s = AT91_SHDWC(opaque);

    switch (offset) {
    case SHDW_MR: return s->mr;
    case SHDW_SR: return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-shdwc: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void shdwc_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    AT91ShdwcState *s = AT91_SHDWC(opaque);
    uint32_t val = value;

    switch (offset) {
    case SHDW_CR:
        if (at91_key_ok(val) && (val & SHDW_CR_SHDW)) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    case SHDW_MR:
        s->mr = val & 0x00070007;   /* WKMODE0, CPTWK0, RTTWKEN, RTCWKEN */
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-shdwc: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps shdwc_ops = {
    .read = shdwc_read,
    .write = shdwc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void shdwc_reset(DeviceState *dev)
{
    AT91ShdwcState *s = AT91_SHDWC(dev);
    s->mr = 0;
}

static void shdwc_dev_init(Object *obj)
{
    AT91ShdwcState *s = AT91_SHDWC(obj);
    memory_region_init_io(&s->iomem, obj, &shdwc_ops, s, "at91-shdwc", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_at91_shdwc = {
    .name = "at91-shdwc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91ShdwcState),
        VMSTATE_END_OF_LIST()
    }
};

static void shdwc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, shdwc_reset);
    dc->vmsd = &vmstate_at91_shdwc;
}

static const TypeInfo shdwc_type = {
    .name = TYPE_AT91_SHDWC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91ShdwcState),
    .instance_init = shdwc_dev_init,
    .class_init = shdwc_class_init,
};

/* --- WDT (0xFFFFFD40) - benign stub, never fires --- */
#define WDT_CR   0x00
#define WDT_MR   0x04
#define WDT_SR   0x08
#define WDT_MR_RESET  0x3FFF2FFF

struct AT91WdtState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mr;
    bool mr_written;      /* WDT_MR is write-once */
};

static uint64_t wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91WdtState *s = AT91_WDT(opaque);

    switch (offset) {
    case WDT_MR: return s->mr;
    case WDT_SR: return 0;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-wdt: read from unimplemented "
                      "offset 0x%02" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void wdt_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91WdtState *s = AT91_WDT(opaque);
    uint32_t val = value;

    switch (offset) {
    case WDT_CR:
        /* WDRSTT (with key) reloads the counter; we never fire, so no-op. */
        break;
    case WDT_MR:
        if (!s->mr_written) {       /* write-once */
            s->mr = val;
            s->mr_written = true;
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-wdt: write to unimplemented "
                      "offset 0x%02" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps wdt_ops = {
    .read = wdt_read,
    .write = wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void wdt_reset(DeviceState *dev)
{
    AT91WdtState *s = AT91_WDT(dev);
    s->mr = WDT_MR_RESET;
    s->mr_written = false;
}

static void wdt_dev_init(Object *obj)
{
    AT91WdtState *s = AT91_WDT(obj);
    memory_region_init_io(&s->iomem, obj, &wdt_ops, s, "at91-wdt", 0x10);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_at91_wdt = {
    .name = "at91-wdt",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91WdtState),
        VMSTATE_BOOL(mr_written, AT91WdtState),
        VMSTATE_END_OF_LIST()
    }
};

static void wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, wdt_reset);
    dc->vmsd = &vmstate_at91_wdt;
}

static const TypeInfo wdt_type = {
    .name = TYPE_AT91_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91WdtState),
    .instance_init = wdt_dev_init,
    .class_init = wdt_class_init,
};

static void at91_sysc_register_types(void)
{
    type_register_static(&rstc_type);
    type_register_static(&shdwc_type);
    type_register_static(&wdt_type);
}

type_init(at91_sysc_register_types)
