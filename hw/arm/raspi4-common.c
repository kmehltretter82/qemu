/*
 * Raspberry Pi 4 common machine support (c) 2012 Gregory Estrade
 * Upstreaming code cleanup [including bcm2835_*] (c) 2013 Jan Petrous
 *
 * Raspberry Pi 2 emulation Copyright (c) 2015, Microsoft
 * Written by Andrew Baumann
 *
 * Raspberry Pi 3 emulation Copyright (c) 2018 Zoltán Baldaszti
 * Upstream code cleanup (c) 2018 Pekka Enberg
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/arm/boot.h"
#include "hw/arm/bcm2838.h"
#include "hw/arm/raspi4_platform.h"
#include "hw/core/registerfields.h"
#include "qemu/error-report.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "qom/object.h"
#include "system/reset.h"
#include "target/arm/cpu.h"

#define SMPBOOT_ADDR    0x300
#define FIRMWARE_ADDR   0x80000
#define SPINTABLE_ADDR  0xd8

FIELD(REV_CODE, REVISION,           0, 4);
FIELD(REV_CODE, MEMORY_SIZE,       20, 3);
FIELD(REV_CODE, STYLE,             23, 1);

uint64_t raspi4_board_ram_size(uint32_t board_rev)
{
    assert(FIELD_EX32(board_rev, REV_CODE, STYLE));
    return 256 * MiB << FIELD_EX32(board_rev, REV_CODE, MEMORY_SIZE);
}

static void write_smpboot64(ARMCPU *cpu, const struct arm_boot_info *info)
{
    AddressSpace *as = arm_boot_address_space(cpu, info);
    static const ARMInsnFixup smpboot[] = {
        { 0xd2801b05 }, /*        mov     x5, 0xd8 */
        { 0xd53800a6 }, /*        mrs     x6, mpidr_el1 */
        { 0x924004c6 }, /*        and     x6, x6, #0x3 */
        { 0xd503205f }, /* spin:  wfe */
        { 0xf86678a4 }, /*        ldr     x4, [x5,x6,lsl #3] */
        { 0xb4ffffc4 }, /*        cbz     x4, spin */
        { 0xd2800000 }, /*        mov     x0, #0x0 */
        { 0xd2800001 }, /*        mov     x1, #0x0 */
        { 0xd2800002 }, /*        mov     x2, #0x0 */
        { 0xd2800003 }, /*        mov     x3, #0x0 */
        { 0xd61f0080 }, /*        br      x4 */
        { 0, FIXUP_TERMINATOR }
    };
    static const uint32_t fixupcontext[FIXUP_MAX] = { 0 };
    static const uint64_t spintables[] = { 0, 0, 0, 0 };

    arm_write_bootloader("raspi_smpboot", as, info->smp_loader_start,
                         smpboot, fixupcontext);
    rom_add_blob_fixed_as("raspi_spintables", spintables, sizeof(spintables),
                          SPINTABLE_ADDR, as);
}

static void reset_secondary(ARMCPU *cpu, const struct arm_boot_info *info)
{
    cpu_set_pc(CPU(cpu), info->smp_loader_start);
}

static void setup_boot(MachineState *machine, ARMCPU *cpu, size_t ram_size)
{
    Raspi4BaseMachineState *s = RASPI4_BASE_MACHINE(machine);
    int r;

    s->binfo.ram_size = ram_size;
    s->binfo.smp_loader_start = SMPBOOT_ADDR;
    s->binfo.write_secondary_boot = write_smpboot64;
    s->binfo.secondary_cpu_reset_hook = reset_secondary;

    if (machine->firmware) {
        r = load_image_targphys(machine->firmware, FIRMWARE_ADDR,
                                ram_size - FIRMWARE_ADDR, NULL);
        if (r < 0) {
            error_report("Failed to load firmware from %s", machine->firmware);
            exit(1);
        }

        s->binfo.entry = FIRMWARE_ADDR;
        s->binfo.firmware_loaded = true;
    }

    arm_exact_poison_regs = s->exact_poison;
    arm_exact_poison_seed = s->poison_seed;
    arm_load_kernel(cpu, machine, &s->binfo);
}

static void raspi4_machine_reset(MachineState *machine, ResetType type)
{
    Raspi4BaseMachineState *s = RASPI4_BASE_MACHINE(machine);

    if (s->exact_poison) {
        arm_exact_poison_region(machine->ram, s->poison_byte,
                                s->poison_seed);
    }
    qemu_devices_reset(type);
}

static bool raspi4_get_exact_poison(Object *obj, Error **errp)
{
    return RASPI4_BASE_MACHINE(obj)->exact_poison;
}

static void raspi4_set_exact_poison(Object *obj, bool value, Error **errp)
{
    RASPI4_BASE_MACHINE(obj)->exact_poison = value;
}

static void raspi4_get_poison_byte(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint8_t value = RASPI4_BASE_MACHINE(obj)->poison_byte;

    visit_type_uint8(v, name, &value, errp);
}

static void raspi4_set_poison_byte(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint8_t value;

    if (visit_type_uint8(v, name, &value, errp)) {
        RASPI4_BASE_MACHINE(obj)->poison_byte = value;
    }
}

static void raspi4_get_poison_seed(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint64_t value = RASPI4_BASE_MACHINE(obj)->poison_seed;

    visit_type_uint64(v, name, &value, errp);
}

static void raspi4_set_poison_seed(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint64_t value;

    if (visit_type_uint64(v, name, &value, errp)) {
        RASPI4_BASE_MACHINE(obj)->poison_seed = value;
    }
}

void raspi4_common_machine_init(MachineState *machine, BCM2838State *soc)
{
    Raspi4BaseMachineClass *mc = RASPI4_BASE_MACHINE_GET_CLASS(machine);
    uint32_t board_rev = mc->board_rev;
    uint64_t ram_size = raspi4_board_ram_size(board_rev);
    uint32_t vcram_base, vcram_size;
    size_t boot_ram_size;
    DriveInfo *di;
    BlockBackend *blk;
    BusState *bus;
    DeviceState *carddev;

    if (machine->ram_size != ram_size) {
        char *size_str = size_to_str(ram_size);
        error_report("Invalid RAM size, should be %s", size_str);
        g_free(size_str);
        exit(1);
    }

    memory_region_add_subregion_overlap(get_system_memory(), 0,
                                        machine->ram, 0);

    object_property_add_const_link(OBJECT(soc), "ram", OBJECT(machine->ram));
    object_property_set_int(OBJECT(soc), "board-rev", board_rev,
                            &error_abort);
    object_property_set_str(OBJECT(soc), "command-line",
                            machine->kernel_cmdline, &error_abort);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    di = drive_get(IF_SD, 0, 0);
    blk = di ? blk_by_legacy_dinfo(di) : NULL;
    bus = qdev_get_child_bus(DEVICE(soc), "sd-bus");
    if (bus == NULL) {
        error_report("No SD bus found in SOC object");
        exit(1);
    }
    carddev = qdev_new(TYPE_SD_CARD);
    qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
    qdev_realize_and_unref(carddev, bus, &error_fatal);

    vcram_size = object_property_get_uint(OBJECT(soc), "vcram-size",
                                          &error_abort);
    vcram_base = object_property_get_uint(OBJECT(soc), "vcram-base",
                                          &error_abort);

    if (vcram_base == 0) {
        vcram_base = ram_size - vcram_size;
    }
    boot_ram_size = MIN(vcram_base, UPPER_RAM_BASE - vcram_size);

    setup_boot(machine, &soc->cpu[0].core, boot_ram_size);
}

void raspi4_common_machine_class_init(MachineClass *mc, uint32_t board_rev,
                                      const char *model)
{
    mc->desc = g_strdup_printf("Raspberry Pi %s (revision 1.%u)", model,
                               FIELD_EX32(board_rev, REV_CODE, REVISION));
    mc->block_default_type = IF_SD;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->default_cpus = mc->min_cpus = mc->max_cpus = BCM2838_NCPUS;
    mc->default_ram_size = raspi4_board_ram_size(board_rev);
    mc->default_ram_id = "ram";
}

static void raspi4_base_machine_init(Object *obj)
{
    Raspi4BaseMachineState *s = RASPI4_BASE_MACHINE(obj);

    s->exact_poison = false;
    s->poison_byte = 0xdf;
    s->poison_seed = 0;
}

static void raspi4_base_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->reset = raspi4_machine_reset;
    object_class_property_add_bool(oc, "x-exact-poison",
                                   raspi4_get_exact_poison,
                                   raspi4_set_exact_poison);
    object_class_property_set_description(oc, "x-exact-poison",
                                          "qemu-exact: poison RAM and UNKNOWN "
                                          "registers at reset");
    object_class_property_add(oc, "x-poison-byte", "uint8",
                              raspi4_get_poison_byte, raspi4_set_poison_byte,
                              NULL, NULL);
    object_class_property_add(oc, "x-poison-seed", "uint64",
                              raspi4_get_poison_seed, raspi4_set_poison_seed,
                              NULL, NULL);
}

static const TypeInfo raspi4_base_machine_type = {
    .name           = TYPE_RASPI4_BASE_MACHINE,
    .parent         = TYPE_MACHINE,
    .instance_size  = sizeof(Raspi4BaseMachineState),
    .instance_init  = raspi4_base_machine_init,
    .class_size     = sizeof(Raspi4BaseMachineClass),
    .class_init     = raspi4_base_machine_class_init,
    .abstract       = true,
};

static void raspi4_base_machine_register_type(void)
{
    type_register_static(&raspi4_base_machine_type);
}

type_init(raspi4_base_machine_register_type)
