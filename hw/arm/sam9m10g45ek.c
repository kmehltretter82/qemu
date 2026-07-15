/*
 * Atmel/Microchip SAM9M10-G45-EK board + AT91SAM9G45 SoC (ARM926EJ-S).
 *
 * Bring-up target: boot a mainline arm multi_v5 zImage to an initramfs shell
 * over the DBGU console.  Register-level references are from the Atmel-6438
 * datasheet; see qemu/.localdir/qemu-at91sam9g45-plan.md.
 *
 * Modelled so far:
 *   - machine scaffold: arm926 + 128 MB DDR2 at 0x70000000
 *   - DBGU  (0xFFFFEE00) console UART, with RX + IRQ
 *   - AIC   (0xFFFFF000) Advanced Interrupt Controller
 *   - PIT   (0xFFFFFD30) Periodic Interval Timer (system tick)
 *   - PMC   (0xFFFFFC00) Power Management Controller (permissive clock stub)
 *
 * The system interrupt (AIC source 1) is the wired-OR of DBGU + PIT (+ other
 * system peripherals on real HW); it is modelled with an OR gate.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/or-irq.h"
#include "migration/vmstate.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/runstate.h"
#include "system/blockdev.h"
#include "chardev/char-fe.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "net/net.h"
#include "hw/net/at91_macb.h"
#include "hw/display/at91_lcdc.h"
#include "hw/dma/at91_dmac.h"
#include "hw/sd/at91_hsmci.h"
#include "hw/gpio/at91_pio.h"
#include "hw/char/at91_dbgu.h"
#include "hw/intc/at91_aic.h"
#include "hw/timer/at91_pit.h"
#include "ui/console.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"
#include "trace.h"

/* ---- SoC memory map (datasheet Fig 5-1, section 6.1) -------------------- */
#define SAM9G45_SDRAM_BASE   0x70000000   /* DDRSDRC0 chip select (main RAM) */
#define SAM9G45_SRAM_BASE    0x00300000   /* internal SRAM                    */
#define SAM9G45_SRAM_SIZE    0x00010000   /* 64 KB                            */
#define SAM9G45_PERIPH_BASE  0xFFF78000   /* start of APB peripheral window  */
#define SAM9G45_PERIPH_SIZE  0x00088000   /* ...through 0xFFFFFFFF            */
#define SAM9G45_DBGU_BASE    0xFFFFEE00   /* Debug Unit (console UART)        */
#define SAM9G45_AIC_BASE     0xFFFFF000   /* Advanced Interrupt Controller    */
#define SAM9G45_PIOA_BASE    0xFFFFF200   /* Parallel I/O Controller A         */
#define SAM9G45_PIOB_BASE    0xFFFFF400
#define SAM9G45_PIOC_BASE    0xFFFFF600
#define SAM9G45_PIOD_BASE    0xFFFFF800
#define SAM9G45_PIOE_BASE    0xFFFFFA00
#define SAM9G45_DMAC_BASE    0xFFFFEC00   /* DMA Controller                    */
#define SAM9G45_PMC_BASE     0xFFFFFC00   /* Power Management Controller       */
#define SAM9G45_RSTC_BASE    0xFFFFFD00   /* Reset Controller                  */
#define SAM9G45_SHDWC_BASE   0xFFFFFD10   /* Shutdown Controller               */
#define SAM9G45_PIT_BASE     0xFFFFFD30   /* Periodic Interval Timer           */
#define SAM9G45_WDT_BASE     0xFFFFFD40   /* Watchdog Timer                    */
#define SAM9G45_HSMCI0_BASE  0xFFF80000   /* High Speed MMC Interface 0        */
#define SAM9G45_HSMCI1_BASE  0xFFFD0000   /* High Speed MMC Interface 1        */
#define SAM9G45_OHCI_BASE    0x00700000   /* USB Host OHCI (full/low speed)    */
#define SAM9G45_EHCI_BASE    0x00800000   /* USB Host EHCI (high speed)        */
#define SAM9G45_EMAC_BASE    0xFFFBC000   /* Ethernet MAC (EMAC / Cadence macb) */
#define SAM9G45_LCDC_BASE    0x00500000   /* LCD Controller                    */

#define SAM9G45_DEFAULT_RAM  (128 * MiB)  /* SAM9M10-G45-EK: 128 MB DDR2      */

/* Peripheral interrupt IDs (AIC source numbers, datasheet Table 7-1). */
#define SAM9G45_IRQ_HSMCI0   11
#define SAM9G45_IRQ_HSMCI1   29
#define SAM9G45_IRQ_UHPHS    22   /* USB host (OHCI + EHCI share this) */
#define SAM9G45_IRQ_DMAC     21
#define SAM9G45_IRQ_EMAC     25
#define SAM9G45_IRQ_LCDC     23
#define SAM9G45_IRQ_PIOA     2
#define SAM9G45_IRQ_PIOB     3
#define SAM9G45_IRQ_PIOC     4
#define SAM9G45_IRQ_PIODE    5   /* PIOD and PIOE share this */

/* mmc card-detect lines on PIOD (board DT: cd-gpios = <&pioD 10/11 ...>). */
#define SAM9G45_MMC0_CD_PIN  10
#define SAM9G45_MMC1_CD_PIN  11

/*
 * Master clock.  The PMC reset registers below model a boot-loader-configured
 * clock tree (PLLA locked at 792 MHz from the 12 MHz crystal, MCK = 792/2/3),
 * matching a real running board; the PIT is clocked from the same MCK so guest
 * timekeeping stays self-consistent.
 */
#define SAM9G45_MCK_HZ       132000000    /* 792 MHz PLLA / 2 / 3 */

/* AIC source 1 is the shared "system interrupt" (DBGU, PIT, RTT, ...). */
#define SAM9G45_IRQ_SYS      1

/* ======================================================================== */
/*  PMC - Power Management Controller (datasheet section 25)                 */
/*                                                                           */
/*  Permissive clock stub: registers are read/write storage, all lock/ready  */
/*  status bits read as set (the at91 clk driver spin-waits on them), and the */
/*  master clock is reported as sourced from the 12 MHz main crystal.         */
/* ======================================================================== */

#define PMC_SCER    0x00   /* System Clock Enable      */
#define PMC_SCDR    0x04   /* System Clock Disable     */
#define PMC_SCSR    0x08   /* System Clock Status      */
#define PMC_PCER    0x10   /* Peripheral Clock Enable  */
#define PMC_PCDR    0x14   /* Peripheral Clock Disable */
#define PMC_PCSR    0x18   /* Peripheral Clock Status  */
#define CKGR_UCKR   0x1C   /* UTMI Clock Config        */
#define CKGR_MOR    0x20   /* Main Oscillator          */
#define CKGR_MCFR   0x24   /* Main Clock Frequency     */
#define CKGR_PLLAR  0x28   /* PLLA Register            */
#define PMC_MCKR    0x30   /* Master Clock Register    */
#define PMC_USB     0x38   /* USB Clock Register       */
#define PMC_PCK0    0x40   /* Programmable Clock 0     */
#define PMC_PCK1    0x44   /* Programmable Clock 1     */
#define PMC_IER     0x60   /* Interrupt Enable         */
#define PMC_IDR     0x64   /* Interrupt Disable        */
#define PMC_SR      0x68   /* Status Register          */
#define PMC_IMR     0x6C   /* Interrupt Mask           */
#define PMC_PLLICPR 0x80   /* PLL Charge Pump Current  */

/* PMC_SR status bits we always report ready (datasheet 25.12.16). */
#define PMC_SR_READY \
    ((1u << 0)  /* MOSCS   */ | (1u << 1)  /* LOCKA    */ | \
     (1u << 3)  /* MCKRDY  */ | (1u << 6)  /* LOCKU    */ | \
     (1u << 7)  /* OSCSELS */ | (1u << 8)  /* PCKRDY0  */ | \
     (1u << 9)  /* PCKRDY1 */ | (1u << 16) /* MOSCSELS */ | \
     (1u << 17) /* MOSCRCS */)

/* CKGR_MCFR: MAINF measured in 16 slow-clock periods for a 12 MHz main clock. */
#define PMC_MAINF_12MHZ  ((12000000u * 16u) / 32768u)   /* ~5859 */
#define PMC_MCFR_VALUE   ((1u << 16) /* MAINRDY */ | PMC_MAINF_12MHZ)

/*
 * Reset values modelling a boot-loader-configured clock tree (at91sam9g45 uses
 * the RM9200 PLL layout: PLLA = MAINCK/DIVA*(MUL+1); MCK = source/2^PRES/MDIV
 * with MDIV divisors {1,2,4,3}).
 *   PLLAR: DIVA=1, MULA=65  -> PLLA = 12 MHz / 1 * 66 = 792 MHz
 *   MCKR:  CSS=PLLA(2), PRES=/2, MDIV=/3 (idx 3) -> MCK = 792/2/3 = 132 MHz
 */
#define PMC_PLLAR_RESET  ((65u << 16) | 1u)              /* 0x00410001 */
#define PMC_MCKR_RESET   (2u | (1u << 2) | (3u << 8))    /* 0x00000306 */

#define TYPE_AT91_PMC "at91-pmc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91PmcState, AT91_PMC)

struct AT91PmcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t scsr;
    uint32_t pcsr;
    uint32_t uckr;
    uint32_t mor;
    uint32_t pllar;
    uint32_t mckr;
    uint32_t usb;
    uint32_t pck[2];
    uint32_t imr;
    uint32_t pllicpr;
};

static uint64_t pmc_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91PmcState *s = AT91_PMC(opaque);

    switch (offset) {
    case PMC_SCSR:   return s->scsr;
    case PMC_PCSR:   return s->pcsr;
    case CKGR_UCKR:  return s->uckr;
    case CKGR_MOR:   return s->mor;
    case CKGR_MCFR:  return PMC_MCFR_VALUE;
    case CKGR_PLLAR: return s->pllar;
    case PMC_MCKR:   return s->mckr;
    case PMC_USB:    return s->usb;
    case PMC_PCK0:   return s->pck[0];
    case PMC_PCK1:   return s->pck[1];
    case PMC_SR:     return PMC_SR_READY;
    case PMC_IMR:    return s->imr;
    case PMC_PLLICPR: return s->pllicpr;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-pmc: read from unimplemented "
                      "offset 0x%03" HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void pmc_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91PmcState *s = AT91_PMC(opaque);
    uint32_t val = value;

    switch (offset) {
    case PMC_SCER:   s->scsr |= val;   break;
    case PMC_SCDR:   s->scsr &= ~val;  break;
    case PMC_PCER:   s->pcsr |= val;   break;
    case PMC_PCDR:   s->pcsr &= ~val;  break;
    case CKGR_UCKR:  s->uckr = val;    break;
    case CKGR_MOR:   s->mor = val;     break;
    case CKGR_PLLAR: s->pllar = val;   break;
    case PMC_MCKR:   s->mckr = val;    break;
    case PMC_USB:    s->usb = val;     break;
    case PMC_PCK0:   s->pck[0] = val;  break;
    case PMC_PCK1:   s->pck[1] = val;  break;
    case PMC_IER:    s->imr |= val;    break;
    case PMC_IDR:    s->imr &= ~val;   break;
    case PMC_PLLICPR: s->pllicpr = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "at91-pmc: write to unimplemented "
                      "offset 0x%03" HWADDR_PRIx " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps pmc_ops = {
    .read = pmc_read,
    .write = pmc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 4, .max_access_size = 4 },
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void pmc_reset(DeviceState *dev)
{
    AT91PmcState *s = AT91_PMC(dev);

    s->scsr = 0x01;
    s->pcsr = 0;
    s->uckr = 0x10200800;
    s->mor = 0;
    s->pllar = PMC_PLLAR_RESET;
    s->mckr = PMC_MCKR_RESET;
    s->usb = 0;
    s->pck[0] = 0;
    s->pck[1] = 0;
    s->imr = 0;
    s->pllicpr = 0;
}

static void pmc_dev_init(Object *obj)
{
    AT91PmcState *s = AT91_PMC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pmc_ops, s, "at91-pmc", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_at91_pmc = {
    .name = "at91-pmc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(scsr, AT91PmcState),
        VMSTATE_UINT32(pcsr, AT91PmcState),
        VMSTATE_UINT32(uckr, AT91PmcState),
        VMSTATE_UINT32(mor, AT91PmcState),
        VMSTATE_UINT32(pllar, AT91PmcState),
        VMSTATE_UINT32(mckr, AT91PmcState),
        VMSTATE_UINT32(usb, AT91PmcState),
        VMSTATE_UINT32_ARRAY(pck, AT91PmcState, 2),
        VMSTATE_UINT32(imr, AT91PmcState),
        VMSTATE_UINT32(pllicpr, AT91PmcState),
        VMSTATE_END_OF_LIST()
    }
};

static void pmc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, pmc_reset);
    dc->vmsd = &vmstate_at91_pmc;
}

static const TypeInfo pmc_type = {
    .name = TYPE_AT91_PMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PmcState),
    .instance_init = pmc_dev_init,
    .class_init = pmc_class_init,
};

/* ======================================================================== */
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

#define TYPE_AT91_RSTC "at91-rstc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91RstcState, AT91_RSTC)
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

#define TYPE_AT91_SHDWC "at91-shdwc"
OBJECT_DECLARE_SIMPLE_TYPE(AT91ShdwcState, AT91_SHDWC)
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

#define TYPE_AT91_WDT "at91-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(AT91WdtState, AT91_WDT)
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

static void at91_register_types(void)
{
    type_register_static(&pmc_type);
    type_register_static(&rstc_type);
    type_register_static(&shdwc_type);
    type_register_static(&wdt_type);
}

type_init(at91_register_types)

/* ======================================================================== */
/*  SAM9M10-G45-EK board                                                     */
/* ======================================================================== */

static struct arm_boot_info sam9m10g45ek_binfo;

/* Create an HSMCI controller, wire its interrupt to the AIC, and attach an
 * SD card from the matching -sd drive (if any). */
static void sam9_create_hsmci(hwaddr base, DeviceState *aic, int irqno,
                              int sd_unit)
{
    DeviceState *mci = qdev_new(TYPE_AT91_HSMCI);
    DriveInfo *di = drive_get(IF_SD, 0, sd_unit);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(mci), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mci), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(mci), 0, qdev_get_gpio_in(aic, irqno));

    if (di) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(mci, "sd-bus"),
                               &error_fatal);
    }
}

static void sam9m10g45ek_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram;
    Object *cpuobj;
    ARMCPU *cpu;
    DeviceState *dbgu, *aic, *pit, *pmc, *orgate;

    cpuobj = object_new(machine->cpu_type);
    /* Older ARMv5 cores have no EL3; guard is harmless if absent. */
    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* DDR2 SDRAM at the DDRSDRC0 chip select. */
    memory_region_add_subregion(sysmem, SAM9G45_SDRAM_BASE, machine->ram);

    /* 64 KB internal SRAM (0x00300000) - the kernel maps it as an mmio-sram
     * pool and reads back what it writes, so it needs real backing memory. */
    sram = g_new(MemoryRegion, 1);
    memory_region_init_ram(sram, NULL, "at91.sram", SAM9G45_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SAM9G45_SRAM_BASE, sram);

    /* Log (rather than abort on) any access to a not-yet-modelled peripheral,
     * so a boot shows how far it gets.  This covers the whole internal APB /
     * system-controller window (0xFFF78000-0xFFFFFFFF); together with the
     * modelled devices and the SRAM/DDR above, every address the guest touches
     * is backed, so ignore_memory_transaction_failures is not needed. */
    create_unimplemented_device("at91-periph", SAM9G45_PERIPH_BASE,
                                SAM9G45_PERIPH_SIZE);

    /* AIC: outputs drive the CPU's nIRQ/nFIQ. */
    aic = qdev_new(TYPE_AT91_AIC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(aic), 0, SAM9G45_AIC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(aic), 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(aic), 1,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));

    /* System interrupt (AIC source 1) is the wired-OR of DBGU + PIT. */
    orgate = qdev_new(TYPE_OR_IRQ);
    qdev_prop_set_uint16(orgate, "num-lines", 2);
    qdev_realize_and_unref(orgate, NULL, &error_fatal);
    qdev_connect_gpio_out(orgate, 0,
                          qdev_get_gpio_in(aic, SAM9G45_IRQ_SYS));

    /* PMC clock controller. */
    pmc = qdev_new(TYPE_AT91_PMC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pmc), 0, SAM9G45_PMC_BASE);

    /* LCD controller. */
    {
        DeviceState *lcdc = qdev_new(TYPE_AT91_LCDC);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(lcdc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(lcdc), 0, SAM9G45_LCDC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(lcdc), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_LCDC));
    }

    /* DMA controller. */
    {
        DeviceState *dmac = qdev_new(TYPE_AT91_DMAC);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dmac), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 0, SAM9G45_DMAC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(dmac), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_DMAC));
    }

    /* Reset / shutdown / watchdog controllers. */
    sysbus_create_simple(TYPE_AT91_RSTC, SAM9G45_RSTC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SHDWC, SAM9G45_SHDWC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_WDT, SAM9G45_WDT_BASE, NULL);

    /* Parallel I/O controllers (GPIO).  PIOA/B/C get their own AIC source;
     * PIOD + PIOE share source 5 (via an OR gate). */
    {
        static const struct { hwaddr base; int irq; } abc[] = {
            { SAM9G45_PIOA_BASE, SAM9G45_IRQ_PIOA },
            { SAM9G45_PIOB_BASE, SAM9G45_IRQ_PIOB },
            { SAM9G45_PIOC_BASE, SAM9G45_IRQ_PIOC },
        };
        DeviceState *pio_or, *piod, *pioe, *p;
        int i;

        for (i = 0; i < 3; i++) {
            p = qdev_new(TYPE_AT91_PIO);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(p), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(p), 0, abc[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(p), 0,
                               qdev_get_gpio_in(aic, abc[i].irq));
        }

        pio_or = qdev_new(TYPE_OR_IRQ);
        qdev_prop_set_uint16(pio_or, "num-lines", 2);
        qdev_realize_and_unref(pio_or, NULL, &error_fatal);
        qdev_connect_gpio_out(pio_or, 0,
                              qdev_get_gpio_in(aic, SAM9G45_IRQ_PIODE));

        piod = qdev_new(TYPE_AT91_PIO);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(piod), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(piod), 0, SAM9G45_PIOD_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(piod), 0,
                           qdev_get_gpio_in(pio_or, 0));

        pioe = qdev_new(TYPE_AT91_PIO);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pioe), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pioe), 0, SAM9G45_PIOE_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(pioe), 0,
                           qdev_get_gpio_in(pio_or, 1));

        /* mmc card-detect lines live on PIOD.  The mmc/gpio stack treats the
         * line as effectively active-low here (card present => pin low), so
         * drive low when a -sd drive is present and high otherwise. */
        pio_set_reset_input(piod, SAM9G45_MMC0_CD_PIN,
                            drive_get(IF_SD, 0, 0) == NULL);
        pio_set_reset_input(piod, SAM9G45_MMC1_CD_PIN,
                            drive_get(IF_SD, 0, 1) == NULL);
    }

    /* High Speed MMC interfaces (SD via -sd / -drive if=sd). */
    sam9_create_hsmci(SAM9G45_HSMCI0_BASE, aic, SAM9G45_IRQ_HSMCI0, 0);
    sam9_create_hsmci(SAM9G45_HSMCI1_BASE, aic, SAM9G45_IRQ_HSMCI1, 1);

    /* Ethernet (10/100 EMAC, Cadence "macb"). */
    {
        DeviceState *emac = qdev_new(TYPE_AT91_MACB);
        qemu_configure_nic_device(emac, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(emac), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(emac), 0, SAM9G45_EMAC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(emac), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_EMAC));
    }

    /* USB host: EHCI (HS) + OHCI (FS/LS) as a companion pair sharing AIC
     * source 22 (UHPHS).  The EHCI owns the ports and hands full/low-speed
     * devices off to the OHCI companion (like the real UHP HS block), rather
     * than the two being independent controllers. */
    {
        DeviceState *ohci, *ehci, *usb_or;

        usb_or = qdev_new(TYPE_OR_IRQ);
        qdev_prop_set_uint16(usb_or, "num-lines", 2);
        qdev_realize_and_unref(usb_or, NULL, &error_fatal);
        qdev_connect_gpio_out(usb_or, 0,
                              qdev_get_gpio_in(aic, SAM9G45_IRQ_UHPHS));

        /* EHCI realized first so its "usb-bus.0" exists for the companion. */
        ehci = qdev_new(TYPE_PLATFORM_EHCI);
        object_property_set_bool(OBJECT(ehci), "companion-enable", true,
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ehci), 0, SAM9G45_EHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ehci), 0,
                           qdev_get_gpio_in(usb_or, 0));

        ohci = qdev_new(TYPE_SYSBUS_OHCI);
        qdev_prop_set_string(ohci, "masterbus", "usb-bus.0");
        qdev_prop_set_uint32(ohci, "num-ports", 2);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ohci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ohci), 0, SAM9G45_OHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ohci), 0,
                           qdev_get_gpio_in(usb_or, 1));
    }

    /* DBGU console -> OR gate input 0. */
    dbgu = qdev_new(TYPE_AT91_DBGU);
    qdev_prop_set_chr(dbgu, "chardev", serial_hd(0));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dbgu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dbgu), 0, SAM9G45_DBGU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dbgu), 0,
                       qdev_get_gpio_in(orgate, 0));

    /* PIT system tick -> OR gate input 1.  Clock it from the board MCK so
     * guest timekeeping matches the modelled PMC clock tree. */
    pit = qdev_new(TYPE_AT91_PIT);
    qdev_prop_set_uint32(pit, "mck-frequency", SAM9G45_MCK_HZ);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pit), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pit), 0, SAM9G45_PIT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(pit), 0,
                       qdev_get_gpio_in(orgate, 1));

    sam9m10g45ek_binfo.loader_start = SAM9G45_SDRAM_BASE;
    sam9m10g45ek_binfo.ram_size = machine->ram_size;
    arm_load_kernel(cpu, machine, &sam9m10g45ek_binfo);
}

static void sam9m10g45ek_machine_init(MachineClass *mc)
{
    mc->desc = "Atmel SAM9M10-G45-EK (AT91SAM9G45, ARM926EJ-S)";
    mc->init = sam9m10g45ek_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "sam9g45.ram";
    mc->default_ram_size = SAM9G45_DEFAULT_RAM;
    /* Every peripheral window is either modelled or covered by an
     * unimplemented-device catch-all (see sam9m10g45ek_init), so real bus
     * faults are not masked. */
}

DEFINE_MACHINE_ARM("sam9m10g45ek", sam9m10g45ek_machine_init)
