/*
 * Renesas Urquell board (SH7786, two SH-X3 cores)
 *
 * Memory map (arch/sh/include/mach-common/mach/urquell.h and
 * arch/sh/boards/board-urquell.c in Linux, SH7786 hardware manual 1.5):
 *   0x00000000 NOR flash (CS0), 64 MiB, 16-bit
 *   0x05000000 FPGA board registers (CS1)
 *   0x05800300 SMC91C111 Ethernet (not modelled yet, it is on IRL)
 *   0x08000000 DDR3 (CS2/CS3 windows onto DBSC 2 and 3)
 *   0x40000000 DDR3, 512 MiB
 *
 * -smp 1 or 2. With -kernel the board does what the boot firmware would:
 * the kernel (ELF vmlinux or zImage) is loaded, the boot parameters are
 * written, CPU0 starts at the entry point and CPU1 waits in module stop
 * for Linux to set C1RESETVEC. The FPGA registers hold what is written;
 * SRSTR = H'A5A5 powers off, as Linux's urquell_power_off() does.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "cpu.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/cpu.h"
#include "exec/tswap.h"
#include "elf.h"
#include "hw/block/flash.h"
#include "system/blockdev.h"
#include "hw/sh4/sh.h"
#include "hw/sh4/sh7785.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"

#define DDR_BASE            0x40000000
#define DDR_SIZE            (512 * MiB)
#define CS2_BASE            0x08000000
#define CS3_BASE            0x0c000000
#define CS_SIZE             (64 * MiB)

#define LINUX_LOAD_OFFSET   0x00800000
#define INITRD_LOAD_OFFSET  0x01800000
#define ZERO_PAGE_OFFSET    0x00001000

#define FLASH_BASE          0x00000000
#define FLASH_SIZE          (64 * MiB)
#define FPGA_BASE           0x05000000
#define FPGA_SIZE           0x4000
#define FPGA_SRSTR          0x0000      /* H'A5A5: system reset / power off */
#define FPGA_FPVERR         0x0150
#define FPGA_MDSWMR         0x1040      /* mode switches: 0 = EXTAL, x64 */

/*
 * FRQMR1 (CPG) as firmware leaves it: PLL = 33.33 MHz x 64; Ick /2,
 * SHck /2, Bck /32, DDRck /2, DUck /8, Pck /32 = 66.67 MHz, which is what
 * Linux's clock-sh7786.c then computes and what the TMUs count.
 * TODO(manual): the real firmware setting is not documented; any legal
 * combination works for Linux as long as Pck matches.
 */
#define FRQMR1              0xffc40014
#define FRQMR1_VAL          0x10191049
#define PCLK_HZ             66666666

#define TYPE_URQUELL_MACHINE MACHINE_TYPE_NAME("urquell")
OBJECT_DECLARE_SIMPLE_TYPE(UrquellMachineState, URQUELL_MACHINE)

struct UrquellMachineState {
    MachineState parent_obj;

    SuperHCPU *cpu[2];
    int ncpus;
    SH7785State *soc;
    uint32_t vector;
    MemoryRegion cs2, cs3, fpga;
    uint16_t fpga_regs[FPGA_SIZE / 2];
};

static uint64_t fpga_read(void *opaque, hwaddr addr, unsigned size)
{
    UrquellMachineState *s = opaque;

    if (addr == FPGA_FPVERR) {
        return 0x0001;
    }
    if (addr == FPGA_MDSWMR) {
        return 0;
    }
    return s->fpga_regs[addr / 2];
}

static void fpga_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    UrquellMachineState *s = opaque;

    if (addr == FPGA_SRSTR && val == 0xa5a5) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        return;
    }
    s->fpga_regs[addr / 2] = val;
}

static const MemoryRegionOps fpga_ops = {
    .read = fpga_read,
    .write = fpga_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 2,
    .valid.max_access_size = 2,
};

static void urquell_reset(void *opaque)
{
    UrquellMachineState *s = opaque;

    for (int i = 0; i < s->ncpus; i++) {
        cpu_reset(CPU(s->cpu[i]));
    }
    sh7786_reset_cores(s->soc);
    s->cpu[0]->env.pc = s->vector;
}

static struct QEMU_PACKED {
    int32_t mount_root_rdonly;
    int32_t ramdisk_flags;
    int32_t orig_root_dev;
    int32_t loader_type;
    int32_t initrd_start;
    int32_t initrd_size;
    char pad[232];
    char kernel_cmdline[256] QEMU_NONSTRING;
} boot_params;

/* head_32.S: the boot parameters the kernel reads, at _text */
static uint64_t boot_params_sym;

static void urquell_elf_sym(const char *name, int info, uint64_t value,
                            uint64_t size)
{
    if (!strcmp(name, "boot_params_page")) {
        boot_params_sym = value;
    }
}

static uint64_t urquell_elf_to_phys(void *opaque, uint64_t addr)
{
    return addr & 0x1fffffff;
}

static void urquell_load_kernel(UrquellMachineState *s, MachineState *machine)
{
    bool elf_kernel;
    uint64_t entry;
    hwaddr zero_page;
    void *params;

    boot_params_sym = 0;
    elf_kernel = load_elf_ram_sym(machine->kernel_filename, NULL,
                                  urquell_elf_to_phys, s, &entry, NULL,
                                  NULL, NULL, ELFDATA2LSB, EM_SH, 0, 0,
                                  NULL, true, urquell_elf_sym) > 0;
    if (elf_kernel) {
        s->vector = entry;
    } else if (load_image_targphys(machine->kernel_filename,
                                   CS2_BASE + LINUX_LOAD_OFFSET,
                                   INITRD_LOAD_OFFSET - LINUX_LOAD_OFFSET,
                                   NULL) < 0) {
        error_report("could not load kernel '%s'", machine->kernel_filename);
        exit(1);
    } else {
        s->vector = 0xa0000000 | (CS2_BASE + LINUX_LOAD_OFFSET);
    }

    memset(&boot_params, 0, sizeof(boot_params));
    if (machine->initrd_filename) {
        int size = load_image_targphys(machine->initrd_filename,
                                       CS2_BASE + INITRD_LOAD_OFFSET,
                                       128 * MiB - INITRD_LOAD_OFFSET, NULL);
        if (size < 0) {
            error_report("could not load initrd '%s'",
                         machine->initrd_filename);
            exit(1);
        }
        boot_params.loader_type = tswap32(1);
        boot_params.initrd_start = tswap32(INITRD_LOAD_OFFSET);
        boot_params.initrd_size = tswap32(size);
    }
    if (machine->kernel_cmdline) {
        strncpy(boot_params.kernel_cmdline, machine->kernel_cmdline,
                sizeof(boot_params.kernel_cmdline));
    }
    zero_page = elf_kernel && boot_params_sym ?
                urquell_elf_to_phys(s, boot_params_sym) :
                CS2_BASE + ZERO_PAGE_OFFSET;
    params = elf_kernel ? rom_ptr(zero_page, sizeof(boot_params)) : NULL;
    if (params) {
        memcpy(params, &boot_params, sizeof(boot_params));
    } else {
        rom_add_blob_fixed("boot_params", &boot_params, sizeof(boot_params),
                           zero_page);
    }
}

static void urquell_init(MachineState *machine)
{
    UrquellMachineState *s = URQUELL_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 0);

    if (machine->ram_size != DDR_SIZE) {
        error_report("urquell has %d MiB of RAM", (int)(DDR_SIZE / MiB));
        exit(1);
    }

    s->ncpus = machine->smp.cpus;
    for (int i = 0; i < s->ncpus; i++) {
        Object *cpu = object_new(machine->cpu_type);

        /* CPU1 powers up in module stop (5.6.1) */
        if (i) {
            object_property_set_bool(cpu, "start-powered-off", true,
                                     &error_abort);
        }
        qdev_realize(DEVICE(cpu), NULL, &error_fatal);
        object_unref(cpu);
        s->cpu[i] = SUPERH_CPU(cpu);
    }
    s->vector = 0xa0000000;

    memory_region_add_subregion(sysmem, DDR_BASE, machine->ram);
    memory_region_init_alias(&s->cs2, NULL, "urquell.ddr-cs2", machine->ram,
                             CS2_BASE, CS_SIZE);
    memory_region_init_alias(&s->cs3, NULL, "urquell.ddr-cs3", machine->ram,
                             CS3_BASE, CS_SIZE);
    memory_region_add_subregion(sysmem, CS2_BASE, &s->cs2);
    memory_region_add_subregion(sysmem, CS3_BASE, &s->cs3);

    s->soc = sh7786_init(s->cpu, s->ncpus, sysmem, PCLK_HZ,
                         dinfo && !machine->kernel_filename);
    sh7785_set_reg(s->soc, FRQMR1, FRQMR1_VAL);
    qemu_register_reset(urquell_reset, s);

    pflash_cfi02_register(FLASH_BASE, "urquell.flash", FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          128 * KiB, 1, 2, 0x0001, 0x227e, 0x2223, 0x2201,
                          0x555, 0x2aa, 0);

    memory_region_init_io(&s->fpga, NULL, &fpga_ops, s, "urquell-fpga",
                          FPGA_SIZE);
    memory_region_add_subregion(sysmem, FPGA_BASE, &s->fpga);

    if (machine->kernel_filename) {
        urquell_load_kernel(s, machine);
    }
}

static void urquell_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Renesas Urquell (SH7786, two SH-X3 cores)";
    mc->init = urquell_init;
    mc->default_cpu_type = TYPE_SH7786_CPU;
    mc->default_ram_size = DDR_SIZE;
    mc->default_ram_id = "urquell.ddr";
    mc->max_cpus = 2;
    mc->default_cpus = 2;
}

static const TypeInfo urquell_info = {
    .name = TYPE_URQUELL_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(UrquellMachineState),
    .class_init = urquell_class_init,
};

static void urquell_register_types(void)
{
    type_register_static(&urquell_info);
}

type_init(urquell_register_types)
