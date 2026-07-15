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
 *   PIO   -> hw/gpio/at91_pio.c
 *
 * Register-level references are from the Atmel-6438 datasheet; see
 * qemu/.localdir/qemu-at91sam9g45-plan.md.  The system interrupt (AIC source 1)
 * is the wired-OR of DBGU + PIT (+ other system peripherals on real HW),
 * modelled with an OR gate.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/or-irq.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "net/net.h"
#include "hw/net/at91_macb.h"
#include "hw/display/at91_lcdc.h"
#include "hw/dma/at91_dmac.h"
#include "hw/sd/at91_hsmci.h"
#include "hw/gpio/at91_pio.h"
#include "hw/i2c/at91_twi.h"
#include "hw/char/at91_usart.h"
#include "hw/intc/at91_aic.h"
#include "hw/timer/at91_pit.h"
#include "hw/misc/at91_pmc.h"
#include "hw/misc/at91_sysc.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

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
#define SAM9G45_TWI0_BASE    0xFFF84000   /* Two-Wire Interface 0 (I2C)        */
#define SAM9G45_TWI1_BASE    0xFFF88000   /* Two-Wire Interface 1 (I2C)        */
#define SAM9G45_USART0_BASE  0xFFF8C000   /* USART0 (ttyS1)                    */
#define SAM9G45_USART1_BASE  0xFFF90000   /* USART1 (ttyS2)                    */
#define SAM9G45_USART2_BASE  0xFFF94000   /* USART2 (ttyS3)                    */
#define SAM9G45_USART3_BASE  0xFFF98000   /* USART3 (ttyS4)                    */

/* Debug Unit chip identification (datasheet section 6.3). */
#define SAM9G45_CIDR  0x819B05A2
#define SAM9G45_EXID  0x00000004
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
#define SAM9G45_IRQ_TWI0     12
#define SAM9G45_IRQ_TWI1     13
#define SAM9G45_IRQ_USART0   7
#define SAM9G45_IRQ_USART1   8
#define SAM9G45_IRQ_USART2   9
#define SAM9G45_IRQ_USART3   10
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

    /* TWI (I2C) controllers.  Slaves attach with e.g.
     * -device at24c-eeprom,bus=i2c-bus.0,address=0x50. */
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
        }
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
