/*
 * Atmel/Microchip AT91 Power Management Controller (PMC).
 *
 * Models a boot-loader-configured clock tree (PLLA locked, MCK derived) with
 * all lock/ready status bits reported set, so the at91 clk driver's spin-waits
 * complete.  Rates are not re-derived on a programmatic PLL/prescaler change.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/misc/at91_pmc.h"

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

static void at91_pmc_register_types(void)
{
    type_register_static(&pmc_type);
}

type_init(at91_pmc_register_types)
