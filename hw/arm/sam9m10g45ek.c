/*
 * Atmel/Microchip SAM9M10-G45-EK board + AT91SAM9G45 SoC (ARM926EJ-S).
 *
 * This file is the board glue: it instantiates the CPU, RAM/SRAM and the AT91
 * peripheral models and wires up their MMIO windows and interrupts.  The
 * peripherals themselves live in the upstream per-subsystem locations:
 *   DBGU  -> hw/char/at91_dbgu.c        AIC   -> hw/intc/at91_aic.c
 *   PIT   -> hw/timer/at91_pit.c        PMC   -> hw/misc/at91_pmc.c
 *   RSTC/SHDWC/WDT -> hw/misc/at91_sysc.c
 *   HSMCI -> hw/sd/at91_hsmci.c         DMAC  -> hw/dma/at91_dmac.c
 *   EMAC  -> hw/net/at91_macb.c         LCDC  -> hw/display/at91_lcdc.c
 *   PIO   -> hw/gpio/at91_pio.c         TSADCC -> hw/adc/at91_tsadcc.c
 *   AC97C -> hw/audio/at91_ac97.c
 *   TRNG  -> hw/misc/at91_trng.c
 *   DDRAMC/SMC/MATRIX/ECC -> hw/misc/at91_memc.c
 *   SCKC  -> hw/misc/at91_sckc.c
 *   SSC   -> hw/ssi/at91_ssc.c
 *   UDPHS -> hw/usb/at91_udphs.c
 *   EBI static SRAM / CFI NOR / CompactFlash -> optional CS attachments below
 *
 * Register-level references are from the Atmel-6438 datasheet; see
 * qemu/.localdir/qemu-at91sam9g45-plan.md.  The system interrupt (AIC source 1)
 * is the wired-OR of DBGU + PIT (+ other system peripherals on real HW),
 * modelled with an OR gate.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qapi/qapi-visit-common.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/or-irq.h"
#include "hw/arm/at91_bootrom.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/block/flash.h"
#include "hw/block/nand.h"
#include "hw/block/at91_nand.h"
#include "hw/ide/mmio.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/at91_udphs.h"
#include "net/net.h"
#include "hw/net/at91_macb.h"
#include "hw/display/at91_lcdc.h"
#include "hw/dma/at91_dmac.h"
#include "hw/sd/at91_hsmci.h"
#include "hw/gpio/at91_pio.h"
#include "hw/i2c/at91_twi.h"
#include "hw/i2c/i2c.h"
#include "hw/misc/at91_isi.h"
#include "hw/char/at91_usart.h"
#include "hw/ssi/at91_spi.h"
#include "hw/ssi/at91_ssc.h"
#include "hw/ssi/ssi.h"
#include "hw/adc/at91_tsadcc.h"
#include "hw/audio/at91_ac97.h"
#include "hw/intc/at91_aic.h"
#include "hw/timer/at91_pit.h"
#include "hw/timer/at91_tc.h"
#include "hw/misc/at91_pmc.h"
#include "hw/misc/at91_sysc.h"
#include "hw/misc/at91_pwm.h"
#include "hw/misc/at91_trng.h"
#include "hw/misc/at91_memc.h"
#include "hw/misc/at91_sckc.h"
#include "hw/rtc/at91_rtc.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

/* ---- SoC memory map (datasheet Fig 5-1, section 6.1) -------------------- */
#define SAM9G45_SDRAM_BASE   0x70000000   /* DDRSDRC0 chip select (main RAM) */
#define SAM9G45_ITCM_BASE     0x00100000   /* SRAM A through the AHB matrix   */
#define SAM9G45_DTCM_BASE     0x00200000   /* SRAM B through the AHB matrix   */
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
#define SAM9G45_TWI0_BASE    0xFFF84000   /* Two-Wire Interface 0 (I2C)        */
#define SAM9G45_TWI1_BASE    0xFFF88000   /* Two-Wire Interface 1 (I2C)        */
#define SAM9G45_USART0_BASE  0xFFF8C000   /* USART0 (ttyS1)                    */
#define SAM9G45_USART1_BASE  0xFFF90000   /* USART1 (ttyS2)                    */
#define SAM9G45_USART2_BASE  0xFFF94000   /* USART2 (ttyS3)                    */
#define SAM9G45_USART3_BASE  0xFFF98000   /* USART3 (ttyS4)                    */
#define SAM9G45_SPI0_BASE    0xFFFA4000   /* Serial Peripheral Interface 0     */
#define SAM9G45_SPI1_BASE    0xFFFA8000   /* Serial Peripheral Interface 1     */
#define SAM9G45_SSC0_BASE    0xFFF9C000
#define SAM9G45_SSC1_BASE    0xFFFA0000

/* Debug Unit chip identification (datasheet section 6.3). */
#define SAM9G45_CIDR  0x819B05A2
#define SAM9G45_EXID  0x00000004
#define SAM9G45_PMC_BASE     0xFFFFFC00   /* Power Management Controller       */
#define SAM9G45_RSTC_BASE    0xFFFFFD00   /* Reset Controller                  */
#define SAM9G45_SHDWC_BASE   0xFFFFFD10   /* Shutdown Controller               */
#define SAM9G45_PIT_BASE     0xFFFFFD30   /* Periodic Interval Timer           */
#define SAM9G45_WDT_BASE     0xFFFFFD40   /* Watchdog Timer                    */
#define SAM9G45_RTT_BASE     0xFFFFFD20
#define SAM9G45_GPBR_BASE    0xFFFFFD60
#define SAM9G45_RTC_BASE     0xFFFFFDB0
#define SAM9G45_HSMCI0_BASE  0xFFF80000   /* High Speed MMC Interface 0        */
#define SAM9G45_HSMCI1_BASE  0xFFFD0000   /* High Speed MMC Interface 1        */
#define SAM9G45_TCB0_BASE    0xFFF7C000   /* Timer Counter block 0 (TC0-2)     */
#define SAM9G45_TCB1_BASE    0xFFFD4000   /* Timer Counter block 1 (TC3-5)     */
#define SAM9G45_OHCI_BASE    0x00700000   /* USB Host OHCI (full/low speed)    */
#define SAM9G45_EHCI_BASE    0x00800000   /* USB Host EHCI (high speed)        */
#define SAM9G45_UDPHS_FIFO   0x00600000   /* USB device endpoint FIFO aperture */
#define SAM9G45_UDPHS_BASE   0xFFF78000   /* USB device controller registers   */
#define SAM9G45_EMAC_BASE    0xFFFBC000   /* Ethernet MAC (EMAC / Cadence macb) */
#define SAM9G45_LCDC_BASE    0x00500000   /* LCD Controller                    */
#define SAM9G45_TSADCC_BASE  0xFFFB0000   /* Touch Screen ADC Controller       */
#define SAM9G45_ISI_BASE     0xFFFB4000   /* Image Sensor Interface            */
#define SAM9G45_PWM_BASE     0xFFFB8000   /* Pulse Width Modulation controller */
#define SAM9G45_AC97_BASE    0xFFFAC000   /* AC97 Controller                   */
#define SAM9G45_TRNG_BASE    0xFFFCC000   /* True Random Number Generator      */
#define SAM9G45_ECC_BASE     0xFFFFE200
#define SAM9G45_DDRAMC0_BASE 0xFFFFE400
#define SAM9G45_DDRAMC1_BASE 0xFFFFE600
#define SAM9G45_SMC_BASE     0xFFFFE800
#define SAM9G45_MATRIX_BASE  0xFFFFEA00
#define SAM9G45_SCKC_BASE    0xFFFFFD50

#define SAM9G45_EBI_CS_BASE(n)  (0x10000000ULL + (n) * 0x10000000ULL)
#define SAM9G45_EBI_CS_SIZE     0x10000000ULL
#define SAM9G45_EBI_CS_COUNT    6
#define SAM9G45_EBI_NAND_CS     3
#define SAM9G45_EBI_NOR_SECTOR  (64 * KiB)

/* True-IDE register aliases generated by the EBI CompactFlash address lines. */
#define SAM9G45_CF_IDE_OFFSET      0x00C00000
#define SAM9G45_CF_ALT_IDE_OFFSET  0x00E00006

#define SAM9G45_DEFAULT_RAM  (128 * MiB)  /* SAM9M10-G45-EK: 128 MB DDR2      */

/* Peripheral interrupt IDs (AIC source numbers, datasheet Table 7-1). */
#define SAM9G45_IRQ_HSMCI0   11
#define SAM9G45_IRQ_HSMCI1   29
#define SAM9G45_IRQ_UHPHS    22   /* USB host (OHCI + EHCI share this) */
#define SAM9G45_IRQ_UDPHS    27
#define SAM9G45_IRQ_DMAC     21
#define SAM9G45_IRQ_TWI0     12
#define SAM9G45_IRQ_TWI1     13
#define SAM9G45_IRQ_USART0   7
#define SAM9G45_IRQ_USART1   8
#define SAM9G45_IRQ_USART2   9
#define SAM9G45_IRQ_USART3   10
#define SAM9G45_IRQ_SPI0     14
#define SAM9G45_IRQ_SPI1     15
#define SAM9G45_IRQ_SSC0     16
#define SAM9G45_IRQ_SSC1     17
#define SAM9G45_IRQ_EMAC     25
#define SAM9G45_IRQ_LCDC     23
#define SAM9G45_IRQ_TSADCC   20
#define SAM9G45_IRQ_ISI      26
#define SAM9G45_IRQ_PWM      19
#define SAM9G45_IRQ_AC97     24
#define SAM9G45_IRQ_TRNG     6
#define SAM9G45_IRQ_PIOA     2
#define SAM9G45_IRQ_PIOB     3
#define SAM9G45_IRQ_PIOC     4
#define SAM9G45_IRQ_PIODE    5   /* PIOD and PIOE share this */
#define SAM9G45_IRQ_TCB      18  /* both TC blocks share this */

/* mmc card-detect lines on PIOD (board DT: cd-gpios = <&pioD 10/11 ...>). */
#define SAM9G45_MMC0_CD_PIN  10
#define SAM9G45_MMC1_CD_PIN  11

/* NAND flash sits in the EBI chip-select 3 window; its ready/busy output
 * is a PIO input (board DT: gpios = <&pioC 8 ...> in the nand node). */
#define SAM9G45_NAND_RB_PIN  8

/* Board USB-device connector VBUS sense (active high). */
#define SAM9G45_UDPHS_VBUS_PIN  19

/* OV2640 camera module SCCB address (stock DT camera@30 on i2c0). */
#define SAM9G45_OV2640_ADDR  0x30

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
/*  SAM9M10-G45-EK board                                                     */
/* ======================================================================== */

static struct arm_boot_info sam9m10g45ek_binfo;

#define TYPE_SAM9M10G45EK_MACHINE MACHINE_TYPE_NAME("sam9m10g45ek")

OBJECT_DECLARE_SIMPLE_TYPE(Sam9m10g45ekMachineState, SAM9M10G45EK_MACHINE)

struct Sam9m10g45ekMachineState {
    MachineState parent_obj;

    uint64_t ebi_sram_size;
    uint8_t ebi_sram_cs;
    uint8_t ebi_nor_cs;
    uint8_t ebi_cf_cs;
    bool udphs_loopback;
    MemoryRegion sram;
    MemoryRegion sram_alias;
    MemoryRegion ebi_sram;
};

static bool sam9_get_udphs_loopback(Object *obj, Error **errp)
{
    return SAM9M10G45EK_MACHINE(obj)->udphs_loopback;
}

static void sam9_set_udphs_loopback(Object *obj, bool value, Error **errp)
{
    SAM9M10G45EK_MACHINE(obj)->udphs_loopback = value;
}

enum Sam9EbiCsProperty {
    SAM9_EBI_SRAM_CS,
    SAM9_EBI_NOR_CS,
    SAM9_EBI_CF_CS,
};

static uint8_t *sam9_ebi_cs_field(Sam9m10g45ekMachineState *s,
                                  enum Sam9EbiCsProperty prop)
{
    switch (prop) {
    case SAM9_EBI_SRAM_CS:
        return &s->ebi_sram_cs;
    case SAM9_EBI_NOR_CS:
        return &s->ebi_nor_cs;
    case SAM9_EBI_CF_CS:
        return &s->ebi_cf_cs;
    default:
        g_assert_not_reached();
    }
}

static void sam9_ebi_get_cs(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(obj);
    uint8_t cs = *sam9_ebi_cs_field(s, GPOINTER_TO_UINT(opaque));

    visit_type_uint8(v, name, &cs, errp);
}

static void sam9_ebi_set_cs(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(obj);
    uint8_t cs;

    if (!visit_type_uint8(v, name, &cs, errp)) {
        return;
    }
    if (cs >= SAM9G45_EBI_CS_COUNT) {
        error_setg(errp, "%s must be in the range 0..%u", name,
                   SAM9G45_EBI_CS_COUNT - 1);
        return;
    }
    if (cs == SAM9G45_EBI_NAND_CS) {
        error_setg(errp, "%s cannot select CS3, which is wired to NAND", name);
        return;
    }

    *sam9_ebi_cs_field(s, GPOINTER_TO_UINT(opaque)) = cs;
}

static void sam9_ebi_get_sram_size(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(obj);
    uint64_t size = s->ebi_sram_size;

    visit_type_size(v, name, &size, errp);
}

static void sam9_ebi_set_sram_size(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(obj);
    uint64_t size;

    if (!visit_type_size(v, name, &size, errp)) {
        return;
    }
    if (size > SAM9G45_EBI_CS_SIZE) {
        error_setg(errp, "%s cannot exceed the 256 MiB EBI CS window", name);
        return;
    }

    s->ebi_sram_size = size;
}

static void sam9_validate_ebi_cs(Sam9m10g45ekMachineState *s,
                                 bool have_nor, bool have_cf)
{
    unsigned used = 1U << SAM9G45_EBI_NAND_CS;

    if (s->ebi_sram_size) {
        used |= 1U << s->ebi_sram_cs;
    }
    if (have_nor) {
        if (used & (1U << s->ebi_nor_cs)) {
            error_report("EBI NOR chip select CS%u is already in use",
                         s->ebi_nor_cs);
            exit(EXIT_FAILURE);
        }
        used |= 1U << s->ebi_nor_cs;
    }
    if (have_cf && (used & (1U << s->ebi_cf_cs))) {
        error_report("EBI CompactFlash chip select CS%u is already in use",
                     s->ebi_cf_cs);
        exit(EXIT_FAILURE);
    }
}

static void sam9_create_ebi_devices(MachineState *machine)
{
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    DriveInfo *nor_di = drive_get(IF_PFLASH, 0, 0);
    DriveInfo *cf_di = drive_get(IF_IDE, 0, 0);

    sam9_validate_ebi_cs(s, nor_di != NULL, cf_di != NULL);

    if (s->ebi_sram_size) {
        memory_region_init_ram(&s->ebi_sram, NULL, "at91.ebi-sram",
                               s->ebi_sram_size, &error_fatal);
        memory_region_add_subregion(sysmem,
                                    SAM9G45_EBI_CS_BASE(s->ebi_sram_cs),
                                    &s->ebi_sram);
    }

    if (nor_di) {
        BlockBackend *blk = blk_by_legacy_dinfo(nor_di);
        int64_t size = blk_getlength(blk);

        if (size <= 0 || size > SAM9G45_EBI_CS_SIZE ||
            !QEMU_IS_ALIGNED(size, SAM9G45_EBI_NOR_SECTOR) ||
            (size & (size - 1))) {
            error_report("EBI NOR image size must be a power of two, aligned "
                         "to 64 KiB, and no larger than 256 MiB");
            exit(EXIT_FAILURE);
        }

        /* One 16-bit Intel-compatible CFI device on the selected CS bank. */
        pflash_cfi01_register(SAM9G45_EBI_CS_BASE(s->ebi_nor_cs),
                              "at91.ebi-nor", size, blk,
                              SAM9G45_EBI_NOR_SECTOR, 2,
                              0x0089, 0x0018, 0, 0, 0);
    }

    if (cf_di) {
        hwaddr base = SAM9G45_EBI_CS_BASE(s->ebi_cf_cs);
        DeviceState *ide = qdev_new(TYPE_MMIO_IDE);
        DeviceState *card = qdev_new("ide-cf");
        BusState *bus;

        qdev_prop_set_uint32(ide, "shift", 0);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ide), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ide), 0,
                        base + SAM9G45_CF_IDE_OFFSET);
        sysbus_mmio_map(SYS_BUS_DEVICE(ide), 1,
                        base + SAM9G45_CF_ALT_IDE_OFFSET);

        bus = qdev_get_child_bus(ide, "ide.0");
        if (!bus) {
            error_report("AT91 CompactFlash controller has no IDE bus");
            exit(EXIT_FAILURE);
        }
        qdev_prop_set_uint32(card, "unit", 0);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(cf_di),
                                &error_fatal);
        qdev_realize_and_unref(card, bus, &error_fatal);
        /* The historical pata_at91 driver supports IRQ-less PIO polling. */
    }
}

/* Create an HSMCI controller, wire its interrupt to the AIC, and attach an
 * SD card from the matching -sd drive (if any). */
static void sam9_create_hsmci(hwaddr base, DeviceState *aic, int irqno,
                              int sd_unit, DeviceState *dmac, int dma_request)
{
    DeviceState *mci = qdev_new(TYPE_AT91_HSMCI);
    DriveInfo *di = drive_get(IF_SD, 0, sd_unit);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(mci), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mci), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(mci), 0, qdev_get_gpio_in(aic, irqno));
    qdev_connect_gpio_out_named(mci, AT91_HSMCI_DMA_REQUEST, 0,
        qdev_get_gpio_in_named(dmac, AT91_DMAC_REQUEST_GPIO, dma_request));

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
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;
    DeviceState *dbgu, *aic, *pit, *pmc, *matrix, *orgate, *dmac, *wdt;
    DeviceState *piob = NULL;
    Clock *mck;

    cpuobj = object_new(machine->cpu_type);
    /* Older ARMv5 cores have no EL3; guard is harmless if absent. */
    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* DDR2 SDRAM at the DDRSDRC0 chip select. */
    memory_region_add_subregion(sysmem, SAM9G45_SDRAM_BASE, machine->ram);

    /*
     * The MATRIX maps this shared 64 KiB backing as ordinary SRAM, DTCM, or
     * split ITCM/DTCM.  At reset all of it appears at 0x00300000.
     */
    memory_region_init_ram(&s->sram, NULL, "at91.sram", SAM9G45_SRAM_SIZE,
                           &error_fatal);

    /*
     * Optional static-memory attachments on EBI CS0..CS5.  CS3 remains
     * reserved for the board's NAND below.
     */
    sam9_create_ebi_devices(machine);

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

    /*
     * System interrupt (AIC source 1) is the wired-OR of DBGU, PIT, RTC, RTT,
     * and other system peripherals on real hardware.
     */
    orgate = qdev_new(TYPE_OR_IRQ);
    qdev_prop_set_uint16(orgate, "num-lines", 5);
    qdev_realize_and_unref(orgate, NULL, &error_fatal);
    qdev_connect_gpio_out(orgate, 0,
                          qdev_get_gpio_in(aic, SAM9G45_IRQ_SYS));

    /* PMC clock controller. */
    pmc = qdev_new(TYPE_AT91_PMC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pmc), 0, SAM9G45_PMC_BASE);
    mck = qdev_get_clock_out(pmc, "mck");

    /*
     * Memory/system configuration plane.  SDRAM itself is the flat RAM
     * region above; these devices retain firmware/Linux timing and mode
     * registers without imposing artificial transaction delays.
     */
    sysbus_create_simple(TYPE_AT91_ECC, SAM9G45_ECC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_DDRAMC, SAM9G45_DDRAMC0_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_DDRAMC, SAM9G45_DDRAMC1_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SMC, SAM9G45_SMC_BASE, NULL);
    matrix = qdev_new(TYPE_AT91_MATRIX);
    object_property_set_link(OBJECT(matrix), "cpu", OBJECT(cpu),
                             &error_fatal);
    object_property_set_link(OBJECT(matrix), "sram", OBJECT(&s->sram),
                             &error_fatal);
    object_property_set_link(OBJECT(matrix), "memory", OBJECT(sysmem),
                             &error_fatal);
    qdev_prop_set_uint64(matrix, "itcm-ahb-base", SAM9G45_ITCM_BASE);
    qdev_prop_set_uint64(matrix, "dtcm-ahb-base", SAM9G45_DTCM_BASE);
    qdev_prop_set_uint64(matrix, "sram-ahb-base", SAM9G45_SRAM_BASE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(matrix), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(matrix), 0, SAM9G45_MATRIX_BASE);
    sysbus_create_simple(TYPE_AT91_SCKC, SAM9G45_SCKC_BASE, NULL);

    /* LCD controller. */
    {
        DeviceState *lcdc = qdev_new(TYPE_AT91_LCDC);

        qdev_connect_clock_in(lcdc, "mck", mck);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(lcdc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(lcdc), 0, SAM9G45_LCDC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(lcdc), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_LCDC));
    }

    /* DMA controller. */
    dmac = qdev_new(TYPE_AT91_DMAC);
    qdev_prop_set_uint64(dmac, AT91_DMAC_REQUEST_MASK,
                         (UINT64_C(1) << 0) | (UINT64_C(1) << 13));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dmac), 0, SAM9G45_DMAC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dmac), 0,
                       qdev_get_gpio_in(aic, SAM9G45_IRQ_DMAC));

    /* Reset / shutdown / watchdog controllers. */
    sysbus_create_simple(TYPE_AT91_RSTC, SAM9G45_RSTC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SHDWC, SAM9G45_SHDWC_BASE, NULL);
    wdt = qdev_new(TYPE_AT91_WDT);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(wdt), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(wdt), 0, SAM9G45_WDT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(wdt), 0,
                       qdev_get_gpio_in(orgate, 4));

    /*
     * Backup-area timekeeping: RTC + RTT use system OR-gate inputs 2 and 3;
     * GPBR is plain scratch storage.
     */
    {
        DeviceState *rtc = qdev_new(TYPE_AT91_RTC);
        DeviceState *rtt = qdev_new(TYPE_AT91_RTT);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(rtc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rtc), 0, SAM9G45_RTC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(rtc), 0,
                           qdev_get_gpio_in(orgate, 2));

        sysbus_realize_and_unref(SYS_BUS_DEVICE(rtt), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rtt), 0, SAM9G45_RTT_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(rtt), 0,
                           qdev_get_gpio_in(orgate, 3));

        sysbus_create_simple(TYPE_AT91_GPBR, SAM9G45_GPBR_BASE, NULL);
    }

    /* Parallel I/O controllers (GPIO).  PIOA/B/C get their own AIC source;
     * PIOD + PIOE share source 5 (via an OR gate). */
    {
        static const struct {
            const char *name;
            hwaddr base;
            int irq;
        } abc[] = {
            { "pioa", SAM9G45_PIOA_BASE, SAM9G45_IRQ_PIOA },
            { "piob", SAM9G45_PIOB_BASE, SAM9G45_IRQ_PIOB },
            { "pioc", SAM9G45_PIOC_BASE, SAM9G45_IRQ_PIOC },
        };
        DeviceState *pio_or, *piod, *pioe, *p, *pioc = NULL;
        int i;

        for (i = 0; i < 3; i++) {
            p = qdev_new(TYPE_AT91_PIO);
            object_property_add_child(OBJECT(machine), abc[i].name, OBJECT(p));
            qdev_connect_clock_in(p, "mck", mck);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(p), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(p), 0, abc[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(p), 0,
                               qdev_get_gpio_in(aic, abc[i].irq));
            if (abc[i].base == SAM9G45_PIOC_BASE) {
                pioc = p;
            } else if (abc[i].base == SAM9G45_PIOB_BASE) {
                piob = p;
            }
        }

        pio_or = qdev_new(TYPE_OR_IRQ);
        qdev_prop_set_uint16(pio_or, "num-lines", 2);
        qdev_realize_and_unref(pio_or, NULL, &error_fatal);
        qdev_connect_gpio_out(pio_or, 0,
                              qdev_get_gpio_in(aic, SAM9G45_IRQ_PIODE));

        piod = qdev_new(TYPE_AT91_PIO);
        object_property_add_child(OBJECT(machine), "piod", OBJECT(piod));
        qdev_connect_clock_in(piod, "mck", mck);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(piod), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(piod), 0, SAM9G45_PIOD_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(piod), 0,
                           qdev_get_gpio_in(pio_or, 0));

        pioe = qdev_new(TYPE_AT91_PIO);
        object_property_add_child(OBJECT(machine), "pioe", OBJECT(pioe));
        qdev_connect_clock_in(pioe, "mck", mck);
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

        /* NAND flash on EBI chip select 3 (attach with -drive if=mtd,
         * format=raw; the image uses the raw page + OOB layout, 2112 bytes
         * per page).  Without a drive the window reads as zeros, so a
         * READID probe finds no chip and the guest driver bails out
         * cleanly. */
        {
            DriveInfo *di = drive_get(IF_MTD, 0, 0);

            if (di) {
                DeviceState *chip = qdev_new(TYPE_NAND);
                DeviceState *ebi = qdev_new(TYPE_AT91_NAND);

                qdev_prop_set_uint8(chip, "manufacturer_id", NAND_MFR_MICRON);
                /* 2 Gbit x8: 256 MB, 2K pages, 128K blocks (MT29F2G08). */
                qdev_prop_set_uint8(chip, "chip_id", 0xda);
                qdev_prop_set_drive_err(chip, "drive",
                                        blk_by_legacy_dinfo(di), &error_fatal);
                qdev_realize_and_unref(chip, NULL, &error_fatal);

                object_property_set_link(OBJECT(ebi), "nand", OBJECT(chip),
                                         &error_fatal);
                sysbus_realize_and_unref(SYS_BUS_DEVICE(ebi), &error_fatal);
                sysbus_mmio_map(SYS_BUS_DEVICE(ebi), 0,
                                SAM9G45_EBI_CS_BASE(SAM9G45_EBI_NAND_CS));
            } else {
                create_unimplemented_device("at91-ebi-cs3",
                              SAM9G45_EBI_CS_BASE(SAM9G45_EBI_NAND_CS),
                              SAM9G45_EBI_CS_SIZE);
            }

            /* Ready/busy PIO input: the modelled chip completes operations
             * synchronously, so it is always ready. */
            pio_set_reset_input(pioc, SAM9G45_NAND_RB_PIN, true);
        }
    }

    /* High Speed MMC interfaces (SD via -sd / -drive if=sd). */
    sam9_create_hsmci(SAM9G45_HSMCI0_BASE, aic, SAM9G45_IRQ_HSMCI0, 0,
                      dmac, 0);
    sam9_create_hsmci(SAM9G45_HSMCI1_BASE, aic, SAM9G45_IRQ_HSMCI1, 1,
                      dmac, 13);

    /* TWI (I2C) controllers.  Slaves attach with e.g.
     * -device at24c-eeprom,bus=i2c-bus.0,address=0x50.  The board's OV2640
     * camera module control interface sits on TWI0 (EK schematic: ISI
     * connector), matching the stock device tree's camera@30 node. */
    {
        static const struct { hwaddr base; int irq; } twi[] = {
            { SAM9G45_TWI0_BASE, SAM9G45_IRQ_TWI0 },
            { SAM9G45_TWI1_BASE, SAM9G45_IRQ_TWI1 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(twi); i++) {
            DeviceState *dev = qdev_new(TYPE_AT91_TWI);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, twi[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               qdev_get_gpio_in(aic, twi[i].irq));
            if (twi[i].base == SAM9G45_TWI0_BASE) {
                i2c_slave_create_simple(I2C_BUS(qdev_get_child_bus(dev,
                                                                   "i2c-bus.0")),
                                        "ov2640", SAM9G45_OV2640_ADDR);
            }
        }
    }

    /* Image Sensor Interface; frames are synthesized by the model. */
    {
        DeviceState *isi = qdev_new(TYPE_AT91_ISI);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(isi), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(isi), 0, SAM9G45_ISI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(isi), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_ISI));
    }

    /* Touch Screen ADC Controller.  Absolute-pointer input (display click
     * or QMP input-send-event) becomes 4-wire pen contacts; the analog
     * channels convert to synthetic values. */
    {
        DeviceState *adc = qdev_new(TYPE_AT91_TSADCC);
        qdev_prop_set_uint32(adc, "mck-frequency", SAM9G45_MCK_HZ);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(adc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(adc), 0, SAM9G45_TSADCC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(adc), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_TSADCC));
    }

    /* Pulse Width Modulation controller (4 channels). */
    {
        DeviceState *pwm = qdev_new(TYPE_AT91_PWM);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(pwm), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(pwm), 0, SAM9G45_PWM_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(pwm), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_PWM));
    }

    /* AC97 controller and LM4549-compatible codec. */
    {
        DeviceState *ac97 = qdev_new(TYPE_AT91_AC97);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(ac97), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ac97), 0, SAM9G45_AC97_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ac97), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_AC97));
    }

    /* True random number generator. */
    {
        DeviceState *trng = qdev_new(TYPE_AT91_TRNG);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(trng), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(trng), 0, SAM9G45_TRNG_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(trng), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_TRNG));
    }

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
        DeviceState *ohci, *ehci, *usb_or, *udphs;
        Sam9m10g45ekMachineState *sms = SAM9M10G45EK_MACHINE(machine);

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

        /* USB device port.  The register/FIFO model always exists.  With the
         * opt-in loopback property, its physical cable endpoint occupies one
         * EHCI port and VBUS is driven high, allowing the guest's own gadget
         * and host stacks to exercise each other end-to-end. */
        udphs = qdev_new(TYPE_AT91_UDPHS);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(udphs), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(udphs), 0, SAM9G45_UDPHS_FIFO);
        sysbus_mmio_map(SYS_BUS_DEVICE(udphs), 1, SAM9G45_UDPHS_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(udphs), 0,
                           qdev_get_gpio_in(aic, SAM9G45_IRQ_UDPHS));

        if (sms->udphs_loopback) {
            USBDevice *cable = USB_DEVICE(qdev_new(TYPE_AT91_UDPHS_USB));
            BusState *bus = qdev_get_child_bus(ehci, "usb-bus.0");

            g_assert(bus);
            object_property_set_link(OBJECT(cable), "controller",
                                     OBJECT(udphs), &error_fatal);
            usb_realize_and_unref(cable, USB_BUS(bus), &error_fatal);
            pio_set_reset_input(piob, SAM9G45_UDPHS_VBUS_PIN, true);
        }
    }

    /* DBGU console -> OR gate input 0.  The DBGU is a cut-down USART that also
     * exposes the SoC Chip ID registers, so it is the USART model with a
     * non-zero chip-id. */
    dbgu = qdev_new(TYPE_AT91_USART);
    qdev_prop_set_chr(dbgu, "chardev", serial_hd(0));
    qdev_prop_set_uint32(dbgu, "chip-id", SAM9G45_CIDR);
    qdev_prop_set_uint32(dbgu, "chip-exid", SAM9G45_EXID);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dbgu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dbgu), 0, SAM9G45_DBGU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dbgu), 0,
                       qdev_get_gpio_in(orgate, 0));

    /* USART0-3 -> ttyS1..4 on serial_hd(1..4), each on its own AIC source. */
    {
        static const struct { hwaddr base; int irq; } usart[] = {
            { SAM9G45_USART0_BASE, SAM9G45_IRQ_USART0 },
            { SAM9G45_USART1_BASE, SAM9G45_IRQ_USART1 },
            { SAM9G45_USART2_BASE, SAM9G45_IRQ_USART2 },
            { SAM9G45_USART3_BASE, SAM9G45_IRQ_USART3 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(usart); i++) {
            DeviceState *dev = qdev_new(TYPE_AT91_USART);
            qdev_prop_set_chr(dev, "chardev", serial_hd(1 + i));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, usart[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               qdev_get_gpio_in(aic, usart[i].irq));
        }
    }

    /* SPI controllers.  SPI0 carries an SPI-NOR flash (n25q032, a stand-in for
     * the board's AT45 DataFlash) on native chip-select NPCS0; the controller's
     * "cs" gpio output drives the flash's chip-select input so each command is
     * framed by CS.  Attach a backing image with -drive if=mtd,index=1
     * (index 0 is the NAND).  The stock DTB's flash node is atmel,at45; use a
     * jedec,spi-nor node to exercise it (mtd_dataflash won't drive an m25p80). */
    {
        static const struct { hwaddr base; int irq; } spi[] = {
            { SAM9G45_SPI0_BASE, SAM9G45_IRQ_SPI0 },
            { SAM9G45_SPI1_BASE, SAM9G45_IRQ_SPI1 },
        };
        DeviceState *spi0 = NULL;
        int i;

        for (i = 0; i < ARRAY_SIZE(spi); i++) {
            DeviceState *dev = qdev_new(TYPE_AT91_SPI);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, spi[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               qdev_get_gpio_in(aic, spi[i].irq));
            if (i == 0) {
                spi0 = dev;
            }
        }

        {
            SSIBus *bus = (SSIBus *)qdev_get_child_bus(spi0, "spi");
            DeviceState *flash = qdev_new("n25q032");
            DriveInfo *di = drive_get(IF_MTD, 0, 1);

            if (di) {
                qdev_prop_set_drive_err(flash, "drive",
                                        blk_by_legacy_dinfo(di), &error_fatal);
            }
            qdev_prop_set_uint8(flash, "cs", 0);
            qdev_realize_and_unref(flash, BUS(bus), &error_fatal);
            qdev_connect_gpio_out_named(spi0, "cs", 0,
                                        qdev_get_gpio_in_named(flash,
                                                               SSI_GPIO_CS, 0));
        }
    }

    /*
     * Synchronous Serial Controllers.  The models provide PIO/PDC data paths
     * and local receiver loopback; no external I2S codec is wired by default.
     */
    {
        static const struct { hwaddr base; int irq; } ssc[] = {
            { SAM9G45_SSC0_BASE, SAM9G45_IRQ_SSC0 },
            { SAM9G45_SSC1_BASE, SAM9G45_IRQ_SSC1 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(ssc); i++) {
            DeviceState *dev = qdev_new(TYPE_AT91_SSC);

            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, ssc[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               qdev_get_gpio_in(aic, ssc[i].irq));
        }
    }

    /* Timer Counter blocks (TC0-2 / TC3-5).  Both share AIC source 18.
     * Linux's tcb_clksrc uses TCB0 as clocksource + tick clockevent when
     * enabled, in preference to the PIT. */
    {
        static const hwaddr tcb[] = { SAM9G45_TCB0_BASE, SAM9G45_TCB1_BASE };
        DeviceState *tc_or = qdev_new(TYPE_OR_IRQ);
        int i;

        qdev_prop_set_uint16(tc_or, "num-lines", 2);
        qdev_realize_and_unref(tc_or, NULL, &error_fatal);
        qdev_connect_gpio_out(tc_or, 0,
                              qdev_get_gpio_in(aic, SAM9G45_IRQ_TCB));

        for (i = 0; i < ARRAY_SIZE(tcb); i++) {
            DeviceState *tc = qdev_new(TYPE_AT91_TC);
            qdev_connect_clock_in(tc, "mck", mck);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tc), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(tc), 0, tcb[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(tc), 0,
                               qdev_get_gpio_in(tc_or, i));
        }
    }

    /* PIT system tick -> OR gate input 1.  Clock it from the board MCK so
     * guest timekeeping matches the modelled PMC clock tree. */
    pit = qdev_new(TYPE_AT91_PIT);
    qdev_connect_clock_in(pit, "mck", mck);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pit), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pit), 0, SAM9G45_PIT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(pit), 0,
                       qdev_get_gpio_in(orgate, 1));

    /*
     * With no direct kernel, model the internal ROM's SD-card boot path.
     * ROM code finds and validates BOOT.BIN, copies it into internal SRAM,
     * remaps that SRAM at address zero, then branches to zero.  Parsing the
     * card on the host models the proprietary ROM while BOOT.BIN and every
     * later stage access the card through the guest-visible HSMCI device.
     */
    if (!machine->kernel_filename) {
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
        Error *err = NULL;

        if (blk && blk_is_available(blk) &&
            at91_bootrom_load_sd(blk, SAM9G45_SRAM_BASE, SAM9G45_SRAM_SIZE,
                                 &err)) {
            memory_region_init_alias(&s->sram_alias, NULL, "at91.sram-remap",
                                     &s->sram, 0,
                                     SAM9G45_SRAM_SIZE);
            memory_region_add_subregion(sysmem, 0, &s->sram_alias);
        } else if (err) {
            warn_report_err(err);
        }
    }

    sam9m10g45ek_binfo.loader_start = SAM9G45_SDRAM_BASE;
    sam9m10g45ek_binfo.ram_size = machine->ram_size;
    arm_load_kernel(cpu, machine, &sam9m10g45ek_binfo);
}

static void sam9m10g45ek_machine_instance_init(Object *obj)
{
    Sam9m10g45ekMachineState *s = SAM9M10G45EK_MACHINE(obj);

    s->ebi_sram_cs = 1;
    s->ebi_nor_cs = 0;
    s->ebi_cf_cs = 4;
}

static void sam9m10g45ek_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Atmel SAM9M10-G45-EK (AT91SAM9G45, ARM926EJ-S)";
    mc->init = sam9m10g45ek_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "sam9g45.ram";
    mc->default_ram_size = SAM9G45_DEFAULT_RAM;
    /* Every peripheral window is either modelled or covered by an
     * unimplemented-device catch-all (see sam9m10g45ek_init), so real bus
     * faults are not masked. */

    object_class_property_add(oc, "ebi-sram-size", "size",
                              sam9_ebi_get_sram_size,
                              sam9_ebi_set_sram_size, NULL, NULL);
    object_class_property_set_description(oc, "ebi-sram-size",
        "Optional static SRAM size (default 0, maximum 256 MiB)");

    object_class_property_add(oc, "ebi-sram-cs", "uint8",
                              sam9_ebi_get_cs, sam9_ebi_set_cs, NULL,
                              GUINT_TO_POINTER(SAM9_EBI_SRAM_CS));
    object_class_property_set_description(oc, "ebi-sram-cs",
        "EBI chip select for optional SRAM (default 1)");

    object_class_property_add(oc, "ebi-nor-cs", "uint8",
                              sam9_ebi_get_cs, sam9_ebi_set_cs, NULL,
                              GUINT_TO_POINTER(SAM9_EBI_NOR_CS));
    object_class_property_set_description(oc, "ebi-nor-cs",
        "EBI chip select for -drive if=pflash (default 0)");

    object_class_property_add(oc, "ebi-cf-cs", "uint8",
                              sam9_ebi_get_cs, sam9_ebi_set_cs, NULL,
                              GUINT_TO_POINTER(SAM9_EBI_CF_CS));
    object_class_property_set_description(oc, "ebi-cf-cs",
        "EBI chip select for -drive if=ide CompactFlash (default 4)");

    object_class_property_add_bool(oc, "udphs-loopback",
                                   sam9_get_udphs_loopback,
                                   sam9_set_udphs_loopback);
    object_class_property_set_description(oc, "udphs-loopback",
        "Connect the USB device port to the board EHCI host (default off)");
}

static const TypeInfo sam9m10g45ek_machine_typeinfo = {
    .name = TYPE_SAM9M10G45EK_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Sam9m10g45ekMachineState),
    .instance_init = sam9m10g45ek_machine_instance_init,
    .class_init = sam9m10g45ek_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void sam9m10g45ek_machine_register_types(void)
{
    type_register_static(&sam9m10g45ek_machine_typeinfo);
}

type_init(sam9m10g45ek_machine_register_types)
