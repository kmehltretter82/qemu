/*
 * Raspberry Pi 4B emulation
 *
 * Copyright (C) 2022 Ovchinnikov Vitalii <vitalii.ovchinnikov@auriga.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/arm/machines-qom.h"
#include "hw/arm/raspi_platform.h"
#include "hw/display/bcm2835_fb.h"
#include "hw/core/registerfields.h"
#include "qemu/error-report.h"
#include "system/device_tree.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/arm/boot.h"
#include "qom/object.h"
#include "hw/arm/bcm2838.h"
#include <libfdt.h>

#define TYPE_RASPI4B_MACHINE MACHINE_TYPE_NAME("raspi4b")
OBJECT_DECLARE_SIMPLE_TYPE(Raspi4bMachineState, RASPI4B_MACHINE)

struct Raspi4bMachineState {
    RaspiBaseMachineState parent_obj;
    BCM2838State soc;
    MemoryRegion highmem_alias;
};

/*
 * Add second memory region if board RAM amount exceeds VC base address
 * (see https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
 * 1.2 Address Map)
 */
static void raspi_add_memory_node(void *fdt, hwaddr mem_base, hwaddr mem_len)
{
    uint32_t acells, scells;
    char *nodename = g_strdup_printf("/memory@%" PRIx64, mem_base);

    acells = qemu_fdt_getprop_cell(fdt, "/", "#address-cells",
                                   NULL, &error_fatal);
    scells = qemu_fdt_getprop_cell(fdt, "/", "#size-cells",
                                   NULL, &error_fatal);
    /* validated by arm_load_dtb */
    g_assert(acells && scells);

    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "device_type", "memory");
    qemu_fdt_setprop_sized_cells(fdt, nodename, "reg",
                                        acells, mem_base,
                                        scells, mem_len);

    g_free(nodename);
}

static void raspi4_modify_dtb(const struct arm_boot_info *info, void *fdt)
{
    uint64_t ram_size;

    /*
     * The upstream BCM2711 DT describes the firmware's AArch64 spin table in
     * each CPU node.  AArch32 Linux has no spin-table SMP backend; real Pi
     * firmware instead leaves it using the brcm,bcm2836-smp mailbox method.
     * Match that direct-boot ABI when the selected CPU runs in AArch32.
     */
    if (!arm_feature(&info->primary_cpu->env, ARM_FEATURE_AARCH64)) {
        unsigned int i;

        for (i = 0; i < BCM283X_NCPUS; i++) {
            g_autofree char *path = g_strdup_printf("/cpus/cpu@%u", i);

            qemu_fdt_setprop_string(fdt, path, "enable-method",
                                    "brcm,bcm2836-smp");
        }
    }

    /* Temporarily disable following devices until they are implemented */
    const char *nodes_to_remove[] = {
        "brcm,bcm2711-pcie",
        "brcm,bcm2711-rng200",
        "brcm,bcm2711-thermal",
        "brcm,bcm2711-genet-v5",
        "brcm,bcm2711-l2-intc",
        "brcm,bcm2711-hdmi-i2c",
        "brcm,brcm2711-dvp",
    };

    for (int i = 0; i < ARRAY_SIZE(nodes_to_remove); i++) {
        const char *dev_str = nodes_to_remove[i];
        int offset;

        /*
         * NOP every node with this compatible. fdt_nop_node() invalidates the
         * offset it was given, so the search must restart from -1 each time;
         * a NOP'd node no longer matches, so this converges and removes all
         * instances (e.g. bcm2711 has two hdmi-i2c nodes).
         */
        offset = fdt_node_offset_by_compatible(fdt, -1, dev_str);
        while (offset >= 0) {
            if (fdt_nop_node(fdt, offset) == 0) {
                warn_report("bcm2711 dtb: %s has been disabled!", dev_str);
            }
            offset = fdt_node_offset_by_compatible(fdt, -1, dev_str);
        }
    }

    /*
     * Enable the SoC-internal xHCI (now modelled) and disable the dwc2 OTG
     * controller: they share a USB PHY and the DT notes enabling both locks up.
     */
    {
        int off = fdt_node_offset_by_compatible(fdt, -1, "brcm,bcm2711-xhci");
        if (off >= 0) {
            fdt_setprop_string(fdt, off, "status", "okay");
        }
        off = fdt_node_offset_by_compatible(fdt, -1, "brcm,bcm2835-usb");
        if (off >= 0) {
            fdt_setprop_string(fdt, off, "status", "disabled");
        }
    }

    ram_size = board_ram_size(info->board_id);

    if (ram_size > UPPER_RAM_BASE) {
        /*
         * The upper DRAM node must stop at the peripheral window. The SoC
         * aliases its peripheral and GIC registers over
         * [BCM2838_PERI_LOW_BASE, 4 GiB) (0xfc000000..0xffffffff), so any RAM
         * described there would make the guest treat live MMIO as memory and
         * corrupt the timer, CPRMAN, GPIO and mailbox on first use. Boards up
         * to ~3 GiB (raspi4b at 2 GiB) never reach the window; a 4 GiB raspi400
         * does. The DRAM behind the window is instead described above 4 GiB by
         * the high node below, matching how the real BCM2711 relocates it.
         */
        uint64_t upper_end = MIN(ram_size, BCM2838_PERI_LOW_BASE);

        raspi_add_memory_node(fdt, UPPER_RAM_BASE, upper_end - UPPER_RAM_BASE);
    }

    /*
     * DRAM that would fall under the peripheral window is aliased above 4 GiB
     * (see raspi4b_machine_init); describe it there so the guest can use it.
     */
    if (ram_size > BCM2838_PERI_LOW_BASE) {
        raspi_add_memory_node(fdt, 4 * GiB, ram_size - BCM2838_PERI_LOW_BASE);
    }
}

static void raspi4b_machine_init(MachineState *machine)
{
    Raspi4bMachineState *s = RASPI4B_MACHINE(machine);
    RaspiBaseMachineState *s_base = RASPI_BASE_MACHINE(machine);
    RaspiBaseMachineClass *mc = RASPI_BASE_MACHINE_GET_CLASS(machine);
    BCM2838State *soc = &s->soc;

    s_base->binfo.modify_dtb = raspi4_modify_dtb;
    s_base->binfo.board_id = mc->board_rev;

    object_initialize_child(OBJECT(machine), "soc", soc,
                            board_soc_type(mc->board_rev));

    raspi_base_machine_init(machine, &soc->parent_obj);

    /*
     * On a board large enough to reach the peripheral window the DRAM behind it
     * ([BCM2838_PERI_LOW_BASE, board size)) is shadowed by MMIO in the flat
     * mapping at 0. The real BCM2711 relocates that DRAM above 4 GiB; mirror
     * that so a 4 GiB raspi400 exposes its full memory instead of losing the
     * top ~64 MiB. The matching /memory node is added in raspi4_modify_dtb().
     */
    if (machine->ram_size > BCM2838_PERI_LOW_BASE) {
        uint64_t hi_size = machine->ram_size - BCM2838_PERI_LOW_BASE;

        memory_region_init_alias(&s->highmem_alias, OBJECT(machine),
                                 "raspi-highmem", machine->ram,
                                 BCM2838_PERI_LOW_BASE, hi_size);
        memory_region_add_subregion(get_system_memory(), 4 * GiB,
                                    &s->highmem_alias);
    }
}

static void raspi4b_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    RaspiBaseMachineClass *rmc = RASPI_BASE_MACHINE_CLASS(oc);

#if HOST_LONG_BITS == 32
    rmc->board_rev = 0xa03111; /* Revision 1.1, 1 Gb RAM */
#else
    rmc->board_rev = 0xb03115; /* Revision 1.5, 2 Gb RAM */
#endif
    raspi_machine_class_common_init(mc, rmc->board_rev);
    mc->auto_create_sdcard = true;
    mc->init = raspi4b_machine_init;
}

static const TypeInfo raspi4b_machine_type = {
    .name           = TYPE_RASPI4B_MACHINE,
    .parent         = TYPE_RASPI_BASE_MACHINE,
    .instance_size  = sizeof(Raspi4bMachineState),
    .class_init     = raspi4b_machine_class_init,
    .interfaces     = aarch64_machine_interfaces,
};

/*
 * The Raspberry Pi 400 uses the same BCM2711 SoC as the Pi 4B, so its machine
 * is the Pi 4B machine with a different board revision: 0xc03130 decodes to
 * type 0x13 (400), processor 3 (BCM2838), memory size 4 (4 GiB), new style.
 * board_soc_type() therefore still selects TYPE_BCM2838 and the Pi 4B init and
 * DTB-fixup paths are reused unchanged.
 */
static void raspi400_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    RaspiBaseMachineClass *rmc = RASPI_BASE_MACHINE_CLASS(oc);

    rmc->board_rev = 0xc03130; /* Revision 1.0, 4 GiB RAM */
    raspi_machine_class_common_init(mc, rmc->board_rev);
}

static const TypeInfo raspi400_machine_type = {
    .name           = MACHINE_TYPE_NAME("raspi400"),
    .parent         = TYPE_RASPI4B_MACHINE,
    .class_init     = raspi400_machine_class_init,
};

static void raspi4b_machine_register_type(void)
{
    type_register_static(&raspi4b_machine_type);
    type_register_static(&raspi400_machine_type);
}

type_init(raspi4b_machine_register_type)
