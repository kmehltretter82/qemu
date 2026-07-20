/*
 * Acorn RiscPC (StrongARM SA-110 processor card + IOMD + VIDC20).
 *
 * Memory map (Linux arch/arm/mach-rpc/include/mach/hardware.h):
 *   0x00000000  ROM (RISC OS; unmapped when direct-kernel booting)
 *   0x02000000  VRAM
 *   0x03000000  I/O space: SuperIO at +0x010000 (byte regs on word
 *               boundaries, so COM1 0x3f8 appears at 0x03010fe0),
 *               IOMD at +0x200000, VIDC20 at +0x400000
 *   0x10000000  RAM (PHYS_OFFSET is non-zero!)
 *
 * The serial console is the SuperIO's first 16550 (ttyS0) on IOMD
 * bank B bit 2. VIDC20 video, the KART keyboard link and podules are
 * not modelled yet.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/arm/boot.h"
#include "hw/arm/machines-qom.h"
#include "hw/misc/acorn-iomd.h"
#include "hw/char/serial-mm.h"
#include "hw/ide/mmio.h"
#include "hw/core/qdev-properties.h"
#include "system/blockdev.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "target/arm/cpu-qom.h"
#include "qom/object.h"

#define RISCPC_RAM_BASE     0x10000000
#define RISCPC_IOMD_BASE    0x03200000
#define RISCPC_SERIAL_BASE  0x03010fe0
#define RISCPC_IDE_CMD_BASE 0x030107c0
#define RISCPC_IDE_CTL_BASE 0x03010fd8

#define MACH_TYPE_RISCPC    1

struct RiscPCMachineState {
    MachineState parent;

    ARMCPU *cpu;
    AcornIOMDState *iomd;
    bool old_param;
};

#define TYPE_RISCPC_MACHINE MACHINE_TYPE_NAME("riscpc")
OBJECT_DECLARE_SIMPLE_TYPE(RiscPCMachineState, RISCPC_MACHINE)

static struct arm_boot_info riscpc_binfo = {
    .loader_start = RISCPC_RAM_BASE,
    .board_id = MACH_TYPE_RISCPC,
};

/*
 * Empty bus cycles float high on the RiscPC; the expansion card
 * (podule) probe relies on reading 0xff from empty slots.
 */
static uint64_t riscpc_bus_read(void *opaque, hwaddr addr, unsigned size)
{
    return (uint64_t)-1;
}

static void riscpc_bus_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned size)
{
}

static const MemoryRegionOps riscpc_bus_ops = {
    .read = riscpc_bus_read,
    .write = riscpc_bus_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void riscpc_init(MachineState *machine)
{
    RiscPCMachineState *rms = RISCPC_MACHINE(machine);
    DeviceState *dev, *ide;

    MemoryRegion *iobus = g_new(MemoryRegion, 1);
    MemoryRegion *podule = g_new(MemoryRegion, 1);

    rms->cpu = ARM_CPU(cpu_create(machine->cpu_type));

    memory_region_add_subregion(get_system_memory(), RISCPC_RAM_BASE,
                                machine->ram);

    /* I/O space and EASI podule space float high where nothing decodes */
    memory_region_init_io(iobus, NULL, &riscpc_bus_ops, NULL,
                          "riscpc-io-bus", 0x01000000);
    memory_region_add_subregion_overlap(get_system_memory(), 0x03000000,
                                        iobus, -1);
    memory_region_init_io(podule, NULL, &riscpc_bus_ops, NULL,
                          "riscpc-easi", 0x08000000);
    memory_region_add_subregion_overlap(get_system_memory(), 0x08000000,
                                        podule, -1);

    dev = qdev_new(TYPE_ACORN_IOMD);
    rms->iomd = ACORN_IOMD(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, RISCPC_IOMD_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(DEVICE(rms->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       qdev_get_gpio_in(DEVICE(rms->cpu), ARM_CPU_FIQ));

    serial_mm_init(get_system_memory(), RISCPC_SERIAL_BASE, 2,
                   qdev_get_gpio_in(dev, ACORN_IOMD_IRQ_SERIAL),
                   1843200, serial_hd(0), DEVICE_LITTLE_ENDIAN);

    /*
     * Onboard SuperIO IDE (Linux pata_platform: cmd 0x030107c0,
     * ctl 0x03010fd8, byte registers on word boundaries, IOMD
     * bank B "harddisk" interrupt).
     */
    ide = qdev_new(TYPE_MMIO_IDE);
    qdev_prop_set_uint32(ide, "shift", 2);
    /* connect before realize: the IDE bus latches its IRQ there */
    sysbus_connect_irq(SYS_BUS_DEVICE(ide), 0,
                       qdev_get_gpio_in(dev, ACORN_IOMD_IRQ_HARDDISK));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(ide), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(ide), 0, RISCPC_IDE_CMD_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(ide), 1, RISCPC_IDE_CTL_BASE);
    mmio_ide_init_drives(ide, drive_get(IF_IDE, 0, 0),
                         drive_get(IF_IDE, 0, 1));

    riscpc_binfo.ram_size = machine->ram_size;
    riscpc_binfo.old_param = rms->old_param;
    arm_load_kernel(rms->cpu, machine, &riscpc_binfo);
}

static bool riscpc_get_old_param(Object *obj, Error **errp)
{
    return RISCPC_MACHINE(obj)->old_param;
}

static void riscpc_set_old_param(Object *obj, bool value, Error **errp)
{
    RISCPC_MACHINE(obj)->old_param = value;
}

static void riscpc_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("sa110"),
        NULL
    };

    mc->desc = "Acorn RiscPC (SA-110)";
    mc->init = riscpc_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("sa110");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "riscpc.ram";

    object_class_property_add_bool(oc, "old-param",
                                   riscpc_get_old_param,
                                   riscpc_set_old_param);
    object_class_property_set_description(oc, "old-param",
        "Pass boot parameters as a RISC OS loader style param_struct "
        "instead of ATAGs (needed by vendor-era 2.2/2.4 kernels)");
}

static const TypeInfo riscpc_machine_typeinfo = {
    .name = TYPE_RISCPC_MACHINE,
    .parent = TYPE_MACHINE,
    .class_init = riscpc_machine_class_init,
    .instance_size = sizeof(RiscPCMachineState),
    .interfaces = arm_machine_interfaces,
};

static void riscpc_machine_register_types(void)
{
    type_register_static(&riscpc_machine_typeinfo);
}
type_init(riscpc_machine_register_types);
