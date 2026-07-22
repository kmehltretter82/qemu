/*
 * Atmel/Microchip AT91SAM9G35-EK board + AT91SAM9G35 SoC (SAM9x5 family,
 * ARM926EJ-S).
 *
 * The G35 reuses the same peripheral IP as the SAM9G45 (modelled in
 * sam9m10g45ek.c) but on the SAM9x5 memory map: SDRAM at 0x20000000 and the
 * peripherals split between the 0xF00x/0xF80x block and the 0xFFFFxxxx system
 * controller.  Addresses and AIC source numbers are taken from the kernel DT
 * (arch/arm/boot/dts/microchip/at91sam9x5.dtsi + at91sam9g35.dtsi); the chip ID
 * is DBGU_CIDR = 0x819A05A1, DBGU_EXID = 0x00000001 (SAM9x5 family base CIDR
 * 0x819A05A0, G35 EXID 1 - see Linux drivers/soc/atmel/soc.h).
 *
 * Phase 1: boot-critical + core peripherals (CPU/RAM, AIC, PMC, memory
 * controllers, DBGU console, PIT/TCB timers, PIO, macb Ethernet, HSMCI SD,
 * USART/UART).  The SAM9x5-specific HLCDC and ADC (different IP from the G45
 * LCDC/TSADCC), plus SPI/SSC/TWI/USB/ISI, are left as unimplemented windows.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/or-irq.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "system/system.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "hw/sd/sd.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "net/net.h"
#include "hw/net/at91_macb.h"
#include "hw/sd/at91_hsmci.h"
#include "hw/gpio/at91_pio.h"
#include "hw/char/at91_usart.h"
#include "hw/intc/at91_aic.h"
#include "hw/timer/at91_pit.h"
#include "hw/timer/at91_tc.h"
#include "hw/dma/at91_dmac.h"
#include "hw/misc/at91_pmc.h"
#include "hw/misc/at91_sysc.h"
#include "hw/misc/at91_memc.h"
#include "hw/misc/at91_sckc.h"
#include "hw/rtc/at91_rtc.h"
#include "hw/misc/unimp.h"
#include "qom/object.h"
#include "target/arm/cpu-qom.h"

/* ---- SAM9x5 SoC memory map (at91sam9x5.dtsi) --------------------------- */
#define SAM9X5_SDRAM_BASE     0x20000000   /* DDRSDRC chip select (main RAM) */
#define SAM9X5_SRAM_BASE      0x00300000
#define SAM9X5_SRAM_SIZE      0x00010000   /* 64 KB internal SRAM */

/* Not-yet-modelled peripherals are covered by two catch-all windows: the low
 * peripheral block (0xF0000000..) and the system-controller area (0xFFFFC000..).
 * Modelled devices are mapped at default priority and override these. */
#define SAM9X5_PERIPH_LO_BASE 0xF0000000
#define SAM9X5_PERIPH_LO_SIZE 0x08100000
#define SAM9X5_PERIPH_HI_BASE 0xFFFFC000
#define SAM9X5_PERIPH_HI_SIZE 0x00004000

#define SAM9X5_MATRIX_BASE    0xFFFFDE00
#define SAM9X5_ECC_BASE       0xFFFFE000
#define SAM9X5_DDRAMC_BASE    0xFFFFE800
#define SAM9X5_SMC_BASE       0xFFFFEA00
#define SAM9X5_AIC_BASE       0xFFFFF000
#define SAM9X5_DBGU_BASE      0xFFFFF200
#define SAM9X5_PIOA_BASE      0xFFFFF400
#define SAM9X5_PIOB_BASE      0xFFFFF600
#define SAM9X5_PIOC_BASE      0xFFFFF800
#define SAM9X5_PIOD_BASE      0xFFFFFA00
#define SAM9X5_PMC_BASE       0xFFFFFC00
#define SAM9X5_RSTC_BASE      0xFFFFFE00
#define SAM9X5_SHDWC_BASE     0xFFFFFE10
#define SAM9X5_PIT_BASE       0xFFFFFE30
#define SAM9X5_WDT_BASE       0xFFFFFE40
#define SAM9X5_SCKC_BASE      0xFFFFFE50
#define SAM9X5_RTC_BASE       0xFFFFFEB0

#define SAM9X5_HSMCI0_BASE    0xF0008000
#define SAM9X5_HSMCI1_BASE    0xF000C000
#define SAM9X5_TCB0_BASE      0xF8008000
#define SAM9X5_TCB1_BASE      0xF800C000
#define SAM9X5_USART0_BASE    0xF801C000
#define SAM9X5_USART1_BASE    0xF8020000
#define SAM9X5_USART2_BASE    0xF8024000
#define SAM9X5_MACB0_BASE     0xF802C000
#define SAM9X5_UART0_BASE     0xF8040000
#define SAM9X5_UART1_BASE     0xF8044000
#define SAM9X5_OHCI_BASE      0x00600000   /* USB Host OHCI (full/low speed) */
#define SAM9X5_EHCI_BASE      0x00700000   /* USB Host EHCI (high speed)     */
#define SAM9X5_EBI_CS3_BASE   0x40000000   /* External Bus Interface CS3 (NAND) */
#define SAM9X5_EBI_CS3_SIZE   0x10000000
#define SAM9X5_DMAC0_BASE     0xFFFFEC00   /* DMA Controller 0 */
#define SAM9X5_DMAC1_BASE     0xFFFFEE00   /* DMA Controller 1 (USART DMA)      */

/* Debug Unit chip identification (SAM9x5 base CIDR + G35 extension ID). */
#define SAM9X5_CIDR           0x819A05A1
#define SAM9G35_EXID          0x00000001

/* AIC source numbers (at91sam9x5.dtsi). */
#define SAM9X5_IRQ_SYS        1    /* DBGU + PIT + RTC + WDT + RSTC (wired-OR) */
#define SAM9X5_IRQ_PIOAB      2    /* PIOA + PIOB share this          */
#define SAM9X5_IRQ_PIOCD      3    /* PIOC + PIOD share this          */
#define SAM9X5_IRQ_USART0     5
#define SAM9X5_IRQ_USART1     6
#define SAM9X5_IRQ_USART2     7
#define SAM9X5_IRQ_HSMCI0     12
#define SAM9X5_IRQ_UART0      15
#define SAM9X5_IRQ_UART1      16
#define SAM9X5_IRQ_TCB        17   /* both TC blocks share this       */
#define SAM9X5_IRQ_DMA0       20
#define SAM9X5_IRQ_DMA1       21
#define SAM9X5_IRQ_UHPHS      22   /* USB host (OHCI + EHCI share this) */
#define SAM9X5_IRQ_MACB0      24
#define SAM9X5_IRQ_HSMCI1     26

/* Must match what the guest derives from the PMC reset values above:
 * PLLA = 12 MHz * 66 = 792 MHz, MCK = PLLA / PRES(2) / MDIV(3) = 132 MHz. */
#define SAM9X5_MCK_HZ         132000000
#define SAM9G35_DEFAULT_RAM   (128 * MiB)

static struct arm_boot_info sam9g35ek_binfo;

#define TYPE_SAM9G35EK_MACHINE MACHINE_TYPE_NAME("sam9g35ek")

OBJECT_DECLARE_SIMPLE_TYPE(Sam9g35ekMachineState, SAM9G35EK_MACHINE)

struct Sam9g35ekMachineState {
    MachineState parent_obj;
};

/* Create an HSMCI controller, wire its IRQ to the AIC, and attach an SD card
 * from the matching -sd drive (if any). */
static void sam9x5_create_hsmci(hwaddr base, DeviceState *aic, int irqno,
                                int sd_unit, DeviceState *dmac)
{
    DeviceState *mci = qdev_new(TYPE_AT91_HSMCI);
    DriveInfo *di = drive_get(IF_SD, 0, sd_unit);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(mci), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(mci), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(mci), 0, qdev_get_gpio_in(aic, irqno));
    qdev_connect_gpio_out_named(mci, AT91_HSMCI_DMA_REQUEST, 0,
        qdev_get_gpio_in_named(dmac, AT91_DMAC_REQUEST_GPIO, 0));

    if (di) {
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(mci, "sd-bus"),
                               &error_fatal);
    }
}

static void sam9g35ek_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *sram;
    Object *cpuobj;
    ARMCPU *cpu;
    DeviceState *aic, *pit, *pmc, *dbgu, *sys_or, *pioab_or, *piocd_or;
    DeviceState *wdt;
    DeviceState *dmac_dev[2];
    Clock *mck;

    cpuobj = object_new(machine->cpu_type);
    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    /* DDR SDRAM. */
    memory_region_add_subregion(sysmem, SAM9X5_SDRAM_BASE, machine->ram);

    /* 64 KB internal SRAM (mapped read/write pool by the kernel). */
    sram = g_new(MemoryRegion, 1);
    memory_region_init_ram(sram, NULL, "at91.sram", SAM9X5_SRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, SAM9X5_SRAM_BASE, sram);

    /* Catch-all windows for not-yet-modelled peripherals; modelled devices,
     * mapped at default priority below, override these. */
    create_unimplemented_device("sam9x5-periph-lo", SAM9X5_PERIPH_LO_BASE,
                                SAM9X5_PERIPH_LO_SIZE);
    create_unimplemented_device("sam9x5-periph-hi", SAM9X5_PERIPH_HI_BASE,
                                SAM9X5_PERIPH_HI_SIZE);

    /* AIC: outputs drive the CPU nIRQ/nFIQ. */
    aic = qdev_new(TYPE_AT91_AIC);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(aic), 0, SAM9X5_AIC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(aic), 0,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(aic), 1,
                       qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));

    /* System interrupt (AIC source 1) is the wired-OR of DBGU, PIT, RTC and
     * other system-controller peripherals. */
    sys_or = qdev_new(TYPE_OR_IRQ);
    qdev_prop_set_uint16(sys_or, "num-lines", 4);
    qdev_realize_and_unref(sys_or, NULL, &error_fatal);
    qdev_connect_gpio_out(sys_or, 0, qdev_get_gpio_in(aic, SAM9X5_IRQ_SYS));

    /* PMC clock controller (permissive stub; register-compatible with the
     * at91sam9x5-pmc for the ready-bit spin-waits the clk driver does). */
    pmc = qdev_new(TYPE_AT91_PMC);
    /* SAM9x5 decodes MCKR with at91sam9x5_master_layout (PRES at bit 4), not
     * the rm9200/sam9g45 layout the device defaults to.  Without this the
     * guest reads PRES as /1 and derives MCK = 264 MHz instead of 132 MHz,
     * running every guest timer at half speed. */
    qdev_prop_set_uint32(pmc, "mckr-reset", AT91_PMC_MCKR_RESET_SAM9X5);
    qdev_prop_set_uint32(pmc, "pres-shift", AT91_PMC_PRES_SHIFT_SAM9X5);
    qdev_prop_set_bit(pmc, "pres-div3", true);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pmc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pmc), 0, SAM9X5_PMC_BASE);
    mck = qdev_get_clock_out(pmc, "mck");

    /* Memory/system configuration plane (register storage; SDRAM is flat). */
    sysbus_create_simple(TYPE_AT91_ECC, SAM9X5_ECC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_DDRAMC, SAM9X5_DDRAMC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SMC, SAM9X5_SMC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_MATRIX, SAM9X5_MATRIX_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SCKC, SAM9X5_SCKC_BASE, NULL);

    /* Two AHB DMA controllers (atmel,at91sam9g45-dma).  The g25 DTB routes the
     * DBGU/USART, HSMCI, i2c and NAND through these; without a model, their
     * descriptor-driven transfers never complete and e.g. userspace console
     * writes (which go through USART TX DMA on DMA1) hang forever. */
    {
        static const struct { hwaddr base; int irq; } dmac[] = {
            { SAM9X5_DMAC0_BASE, SAM9X5_IRQ_DMA0 },
            { SAM9X5_DMAC1_BASE, SAM9X5_IRQ_DMA1 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(dmac); i++) {
            DeviceState *d = qdev_new(TYPE_AT91_DMAC);

            qdev_prop_set_uint64(d, AT91_DMAC_REQUEST_MASK, UINT64_C(1));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(d), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(d), 0, dmac[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(d), 0,
                               qdev_get_gpio_in(aic, dmac[i].irq));
            dmac_dev[i] = d;
        }
    }

    /* NAND flash chip-select (EBI CS3). No NAND chip is modelled; back the
     * window so the atmel-nand controller's exec_op reads 0 (-> "no device
     * found") and fails its probe cleanly, rather than taking an external
     * abort on the command/address latch cycles. */
    create_unimplemented_device("sam9x5-ebi-cs3-nand", SAM9X5_EBI_CS3_BASE,
                                SAM9X5_EBI_CS3_SIZE);

    /* Reset / shutdown / watchdog controllers. */
    sysbus_create_simple(TYPE_AT91_RSTC, SAM9X5_RSTC_BASE, NULL);
    sysbus_create_simple(TYPE_AT91_SHDWC, SAM9X5_SHDWC_BASE, NULL);
    wdt = qdev_new(TYPE_AT91_WDT);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(wdt), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(wdt), 0, SAM9X5_WDT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(wdt), 0,
                       qdev_get_gpio_in(sys_or, 3));

    /* Real-time clock -> system OR-gate input 2. */
    {
        DeviceState *rtc = qdev_new(TYPE_AT91_RTC);

        sysbus_realize_and_unref(SYS_BUS_DEVICE(rtc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(rtc), 0, SAM9X5_RTC_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(rtc), 0,
                           qdev_get_gpio_in(sys_or, 2));
    }

    /* Parallel I/O controllers.  PIOA+PIOB share AIC source 2, PIOC+PIOD
     * share source 3. */
    pioab_or = qdev_new(TYPE_OR_IRQ);
    qdev_prop_set_uint16(pioab_or, "num-lines", 2);
    qdev_realize_and_unref(pioab_or, NULL, &error_fatal);
    qdev_connect_gpio_out(pioab_or, 0, qdev_get_gpio_in(aic, SAM9X5_IRQ_PIOAB));

    piocd_or = qdev_new(TYPE_OR_IRQ);
    qdev_prop_set_uint16(piocd_or, "num-lines", 2);
    qdev_realize_and_unref(piocd_or, NULL, &error_fatal);
    qdev_connect_gpio_out(piocd_or, 0, qdev_get_gpio_in(aic, SAM9X5_IRQ_PIOCD));

    {
        static const struct { hwaddr base; bool cd; int line; } pio[] = {
            { SAM9X5_PIOA_BASE, false, 0 }, { SAM9X5_PIOB_BASE, false, 1 },
            { SAM9X5_PIOC_BASE, true,  0 }, { SAM9X5_PIOD_BASE, true,  1 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(pio); i++) {
            DeviceState *p = qdev_new(TYPE_AT91_PIO);

            sysbus_realize_and_unref(SYS_BUS_DEVICE(p), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(p), 0, pio[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(p), 0,
                               qdev_get_gpio_in(pio[i].cd ? piocd_or : pioab_or,
                                                pio[i].line));
        }
    }

    /* Ethernet (10/100 EMAC, Cadence "macb"). */
    {
        DeviceState *emac = qdev_new(TYPE_AT91_MACB);

        qemu_configure_nic_device(emac, true, NULL);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(emac), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(emac), 0, SAM9X5_MACB0_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(emac), 0,
                           qdev_get_gpio_in(aic, SAM9X5_IRQ_MACB0));
    }

    /* USB host: EHCI (HS) + OHCI (FS/LS) companion pair sharing AIC source 22.
     * EHCI is realized first so its "usb-bus.0" exists for the OHCI companion. */
    {
        DeviceState *ohci, *ehci, *usb_or;

        usb_or = qdev_new(TYPE_OR_IRQ);
        qdev_prop_set_uint16(usb_or, "num-lines", 2);
        qdev_realize_and_unref(usb_or, NULL, &error_fatal);
        qdev_connect_gpio_out(usb_or, 0,
                              qdev_get_gpio_in(aic, SAM9X5_IRQ_UHPHS));

        ehci = qdev_new(TYPE_PLATFORM_EHCI);
        object_property_set_bool(OBJECT(ehci), "companion-enable", true,
                                 &error_fatal);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ehci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ehci), 0, SAM9X5_EHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ehci), 0,
                           qdev_get_gpio_in(usb_or, 0));

        ohci = qdev_new(TYPE_SYSBUS_OHCI);
        qdev_prop_set_string(ohci, "masterbus", "usb-bus.0");
        /* The SAM9x5-EK board wires 3 OHCI root-hub ports (at91sam9x5ek.dtsi
         * "num-ports = <3>"); the at91_ohci driver reads that from the DT, so
         * the model must expose the same count or the guest polls a phantom
         * port 3 (out-of-range reads return 0xffffffff -> stuck over-current). */
        qdev_prop_set_uint32(ohci, "num-ports", 3);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ohci), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(ohci), 0, SAM9X5_OHCI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(ohci), 0,
                           qdev_get_gpio_in(usb_or, 1));
    }

    /* High Speed MMC interfaces (SD via -sd / -drive if=sd). */
    sam9x5_create_hsmci(SAM9X5_HSMCI0_BASE, aic, SAM9X5_IRQ_HSMCI0, 0,
                        dmac_dev[0]);
    sam9x5_create_hsmci(SAM9X5_HSMCI1_BASE, aic, SAM9X5_IRQ_HSMCI1, 1,
                        dmac_dev[1]);

    /* DBGU console -> OR-gate input 0.  The DBGU is a cut-down USART that also
     * exposes the SoC Chip ID registers. */
    dbgu = qdev_new(TYPE_AT91_USART);
    qdev_prop_set_chr(dbgu, "chardev", serial_hd(0));
    qdev_prop_set_uint32(dbgu, "chip-id", SAM9X5_CIDR);
    qdev_prop_set_uint32(dbgu, "chip-exid", SAM9G35_EXID);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dbgu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dbgu), 0, SAM9X5_DBGU_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dbgu), 0, qdev_get_gpio_in(sys_or, 0));

    /* USART0-2 (ttyS1..3) + UART0-1 (ttyS4..5) on serial_hd(1..5). */
    {
        static const struct { hwaddr base; int irq; } uart[] = {
            { SAM9X5_USART0_BASE, SAM9X5_IRQ_USART0 },
            { SAM9X5_USART1_BASE, SAM9X5_IRQ_USART1 },
            { SAM9X5_USART2_BASE, SAM9X5_IRQ_USART2 },
            { SAM9X5_UART0_BASE,  SAM9X5_IRQ_UART0 },
            { SAM9X5_UART1_BASE,  SAM9X5_IRQ_UART1 },
        };
        int i;

        for (i = 0; i < ARRAY_SIZE(uart); i++) {
            DeviceState *dev = qdev_new(TYPE_AT91_USART);

            qdev_prop_set_chr(dev, "chardev", serial_hd(1 + i));
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, uart[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               qdev_get_gpio_in(aic, uart[i].irq));
        }
    }

    /* Timer Counter blocks (TC0-2 / TC3-5), both on AIC source 17.  Linux's
     * tcb_clksrc uses TCB0 as clocksource + tick clockevent. */
    {
        static const hwaddr tcb[] = { SAM9X5_TCB0_BASE, SAM9X5_TCB1_BASE };
        DeviceState *tc_or = qdev_new(TYPE_OR_IRQ);
        int i;

        qdev_prop_set_uint16(tc_or, "num-lines", 2);
        qdev_realize_and_unref(tc_or, NULL, &error_fatal);
        qdev_connect_gpio_out(tc_or, 0, qdev_get_gpio_in(aic, SAM9X5_IRQ_TCB));

        for (i = 0; i < ARRAY_SIZE(tcb); i++) {
            DeviceState *tc = qdev_new(TYPE_AT91_TC);

            qdev_connect_clock_in(tc, "mck", mck);
            /* SAM9x5 TC channels are 32-bit; Linux tcb_clksrc uses a single
             * channel as a free-running 32-bit clocksource (not the 16-bit
             * two-channel chain), so the counter must wrap at 2^32. */
            qdev_prop_set_uint32(tc, "counter-width", 32);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(tc), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(tc), 0, tcb[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(tc), 0,
                               qdev_get_gpio_in(tc_or, i));
        }
    }

    /* PIT system tick -> OR-gate input 1. */
    pit = qdev_new(TYPE_AT91_PIT);
    qdev_connect_clock_in(pit, "mck", mck);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pit), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pit), 0, SAM9X5_PIT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(pit), 0, qdev_get_gpio_in(sys_or, 1));

    sam9g35ek_binfo.loader_start = SAM9X5_SDRAM_BASE;
    sam9g35ek_binfo.ram_size = machine->ram_size;
    arm_load_kernel(cpu, machine, &sam9g35ek_binfo);
}

static void sam9g35ek_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Atmel AT91SAM9G35-EK (SAM9x5 family, ARM926EJ-S)";
    mc->init = sam9g35ek_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
    mc->default_ram_id = "sam9g35.ram";
    mc->default_ram_size = SAM9G35_DEFAULT_RAM;
}

static const TypeInfo sam9g35ek_machine_typeinfo = {
    .name = TYPE_SAM9G35EK_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Sam9g35ekMachineState),
    .class_init = sam9g35ek_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void sam9g35ek_machine_register_types(void)
{
    type_register_static(&sam9g35ek_machine_typeinfo);
}

type_init(sam9g35ek_machine_register_types)
