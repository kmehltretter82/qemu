/*
 * Rebel.com NetWinder (StrongARM SA-110 + DC21285 "Footbridge").
 *
 * Memory map (see Linux arch/arm/include/asm/hardware/dec21285.h):
 *   0x00000000  SDRAM
 *   0x41000000  flash (NeTTrom firmware)
 *   0x42000000  21285 CSR block
 *   0x79000000  PCI IACK (ISA interrupt acknowledge)
 *   0x7c000000  PCI I/O window (SuperIO/ISA lives here)
 *   0x80000000  PCI memory window
 *
 * Serial wiring: -serial 0/1 are reserved for the SuperIO 16550s
 * (ttyS0/ttyS1, the machine's real console); -serial 2 is the 21285
 * internal UART (Linux ttyFB, also the DEBUG_LL/earlycon port).
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/qdev-properties.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/dc21285.h"
#include "hw/block/flash.h"
#include "hw/isa/isa.h"
#include "hw/intc/i8259.h"
#include "hw/char/serial-isa.h"
#include "hw/timer/i8254.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/dc21285-pci.h"
#include "hw/ide/pci.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/system.h"
#include "target/arm/cpu-qom.h"
#include "qom/object.h"

#define NETWINDER_FLASH_BASE    0x41000000
#define NETWINDER_FLASH_SIZE    (4 * MiB)
#define NETWINDER_CSR_BASE      0x42000000
#define NETWINDER_PCI_IACK      0x79000000
#define NETWINDER_PCI_IO_BASE   0x7c000000

#define MACH_TYPE_NETWINDER     5

struct NetwinderMachineState {
    MachineState parent;

    ARMCPU *cpu;
    DC21285State *fb;
    DC21285PCIState *pci;
    bool old_param;
};

#define TYPE_NETWINDER_MACHINE MACHINE_TYPE_NAME("netwinder")
OBJECT_DECLARE_SIMPLE_TYPE(NetwinderMachineState, NETWINDER_MACHINE)

static struct arm_boot_info netwinder_binfo = {
    .loader_start = 0,
    .board_id = MACH_TYPE_NETWINDER,
    /*
     * NeTTrom loads kernels at 0x8000, and NetWinder-era vmlinuz
     * images (e.g. Debian's) carry a shim that assumes exactly that
     * address and self-corrupts when started anywhere else.
     */
    .kernel_load_offset = 0x8000,
};

/*
 * The 21285 turns a read from the IACK region into a PCI interrupt
 * acknowledge cycle, which the ISA bridge answers with the 8259 vector.
 * Linux's isa_irq_handler reads one byte from here to find the ISA IRQ.
 */
static uint64_t netwinder_iack_read(void *opaque, hwaddr addr, unsigned size)
{
    return pic_read_irq(isa_pic);
}

static void netwinder_iack_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
}

static const MemoryRegionOps netwinder_iack_ops = {
    .read = netwinder_iack_read,
    .write = netwinder_iack_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void netwinder_init(MachineState *machine)
{
    NetwinderMachineState *nms = NETWINDER_MACHINE(machine);
    DeviceState *dev, *pcidev;
    DriveInfo *dinfo;
    MemoryRegion *isa_region, *iack_region, *dma_alias;
    ISABus *isa_bus;
    qemu_irq *isa_irqs;

    nms->cpu = ARM_CPU(cpu_create(machine->cpu_type));

    memory_region_add_subregion(get_system_memory(), 0, machine->ram);

    dev = qdev_new(TYPE_DC21285);
    nms->fb = DC21285(dev);
    qdev_prop_set_chr(dev, "chardev", serial_hd(2));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, NETWINDER_CSR_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(DEVICE(nms->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       qdev_get_gpio_in(DEVICE(nms->cpu), ARM_CPU_FIQ));

    dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi01_register(NETWINDER_FLASH_BASE, "netwinder.flash",
                          NETWINDER_FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          64 * KiB, 4, 0x00, 0x00, 0x00, 0x00, 0);

    /* PCI host bridge: config windows, memory window, I/O window */
    pcidev = qdev_new(TYPE_DC21285_PCI);
    nms->pci = DC21285_PCI(pcidev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pcidev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pcidev), 0, 0x7b000000); /* type 0 cfg */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcidev), 1, 0x7a000000); /* type 1 cfg */
    sysbus_mmio_map(SYS_BUS_DEVICE(pcidev), 2, 0x80000000); /* PCI memory */
    for (int i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(pcidev), i,
                           qdev_get_gpio_in(dev, DC21285_IRQ_IN0 + i));
    }

    /*
     * Inbound window: the 21285 exposes system RAM to PCI bus
     * masters at bus address 0xe0000000 (Linux's BUS_OFFSET).
     */
    dma_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(dma_alias, NULL, "pci-inbound-ram",
                             machine->ram, 0, machine->ram_size);
    memory_region_add_subregion(&nms->pci->pci_mem, 0xe0000000, dma_alias);

    /*
     * The SuperIO's ISA ports decode in the PCI I/O window; the ISA
     * bus therefore lives in the PCI host's I/O space.
     */
    isa_region = &nms->pci->pci_io;
    memory_region_add_subregion(get_system_memory(), NETWINDER_PCI_IO_BASE,
                                isa_region);
    isa_bus = isa_bus_new(NULL, get_system_memory(), isa_region,
                          &error_abort);
    isa_irqs = i8259_init(isa_bus,
                          qdev_get_gpio_in(dev, DC21285_IRQ_IN3));
    isa_bus_register_input_irqs(isa_bus, isa_irqs);
    serial_hds_isa_init(isa_bus, 0, 2);
    /* NetWinder timekeeping is the SuperIO's i8254 PIT on ISA IRQ 0 */
    i8254_pit_init(isa_bus, 0x40, 0, NULL);

    /* Onboard DC21143 ethernet in slot 10 (-> 21285 IN1) */
    pci_init_nic_in_slot(PCI_HOST_BRIDGE(nms->pci)->bus, "tulip", NULL,
                         "0a");

    /*
     * Winbond W83C553/SL82C105 pair in slot 12: the IDE function is
     * function 1 and runs in legacy mode (ports 0x1f0, ISA IRQ 14);
     * pata_sl82c105 requires the bridge shell at function 0.
     */
    pci_create_simple_multifunction(PCI_HOST_BRIDGE(nms->pci)->bus,
                                    PCI_DEVFN(12, 0), "w83c553-isa");
    pci_ide_create_devs(pci_create_simple(PCI_HOST_BRIDGE(nms->pci)->bus,
                                          PCI_DEVFN(12, 1), "sl82c105-ide"));

    iack_region = g_new(MemoryRegion, 1);
    memory_region_init_io(iack_region, NULL, &netwinder_iack_ops, NULL,
                          "pci-iack", 4);
    memory_region_add_subregion(get_system_memory(), NETWINDER_PCI_IACK,
                                iack_region);

    netwinder_binfo.ram_size = machine->ram_size;
    netwinder_binfo.old_param = nms->old_param;
    arm_load_kernel(nms->cpu, machine, &netwinder_binfo);
}

static bool netwinder_get_old_param(Object *obj, Error **errp)
{
    return NETWINDER_MACHINE(obj)->old_param;
}

static void netwinder_set_old_param(Object *obj, bool value, Error **errp)
{
    NETWINDER_MACHINE(obj)->old_param = value;
}

static void netwinder_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("sa110"),
        NULL
    };

    mc->desc = "Rebel NetWinder (SA-110)";
    mc->init = netwinder_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("sa110");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "netwinder.ram";

    object_class_property_add_bool(oc, "old-param",
                                   netwinder_get_old_param,
                                   netwinder_set_old_param);
    object_class_property_set_description(oc, "old-param",
        "Pass boot parameters as a NeTTrom-style param_struct instead of "
        "ATAGs (needed by vendor-era 2.2/2.4 kernels)");
}

static const TypeInfo netwinder_machine_typeinfo = {
    .name = TYPE_NETWINDER_MACHINE,
    .parent = TYPE_MACHINE,
    .class_init = netwinder_machine_class_init,
    .instance_size = sizeof(NetwinderMachineState),
    .interfaces = arm_machine_interfaces,
};

static void netwinder_machine_register_types(void)
{
    type_register_static(&netwinder_machine_typeinfo);
}
type_init(netwinder_machine_register_types);
