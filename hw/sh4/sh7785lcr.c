/*
 * Renesas SH7785LCR (R0P7785LC0011RL) evaluation board
 *
 * Memory map in 29-bit mode (arch/sh/include/mach-common/mach/sh7785lcr.h
 * in Linux, and the SH7785 hardware manual 1.5 for the areas):
 *   0x00000000 NOR flash (CS0, 64 MiB, 32-bit bus), -drive if=pflash
 *   0x04000000 PLD registers (CS1)
 *   0x06000000 PCA9564 I2C (CS1)             - not modelled yet
 *   0x08000000 DDR2 SDRAM, 128 MiB (areas 2 and 3)
 *   0x10000000 SM107 graphics (CS4)          - not modelled yet
 *
 * The board runs in clock mode 16 with a 33.33 MHz EXTAL (the factory DIP
 * switch settings Linux assumes in sh7785lcr_mode_pins()): FRQMR1 reads
 * H'1225 2448 (hardware manual table 15.3) and Pck is 50 MHz.
 *
 * With -kernel, a zImage is loaded at 0x08800000 and entered through P2,
 * with the boot parameters (command line, initrd) in the zero page at
 * 0x08001000, like the r2d machine does.
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
#include "exec/tswap.h"
#include "hw/block/flash.h"
#include "system/blockdev.h"
#include "hw/sh4/sh.h"
#include "hw/sh4/sh7785.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "system/system.h"

#define SDRAM_BASE          0x08000000
#define SDRAM_SIZE          (128 * MiB)
#define LINUX_LOAD_OFFSET   0x00800000
#define INITRD_LOAD_OFFSET  0x01800000
#define ZERO_PAGE_OFFSET    0x00001000

#define FLASH_BASE          0x00000000
#define FLASH_SIZE          (64 * MiB)
#define PLD_BASE            0x04000000
#define PLD_POFCR           0x06    /* write 1: power off */
#define PLD_VERSR           0x0c

#define FRQMR1              0xffc80014
#define FRQMR1_MODE16       0x12252448
#define PCLK_HZ             50000000

typedef struct {
    SuperHCPU *cpu;
    uint32_t vector;
} ResetData;

static void main_cpu_reset(void *opaque)
{
    ResetData *s = opaque;

    cpu_reset(CPU(s->cpu));
    s->cpu->env.pc = s->vector;
}

/* PLD: only the registers the kernel and the test rig use */
static uint64_t pld_read(void *opaque, hwaddr addr, unsigned size)
{
    uint16_t *regs = opaque;

    if (addr == PLD_VERSR) {
        return 0x0001;
    }
    return addr < 0x10 ? regs[addr / 2] : 0;
}

static void pld_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    uint16_t *regs = opaque;

    if (addr == PLD_POFCR && (val & 1)) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        return;
    }
    if (addr < 0x10) {
        regs[addr / 2] = val;
    }
}

static const MemoryRegionOps pld_ops = {
    .read = pld_read,
    .write = pld_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 2,
    .impl.min_access_size = 1,
    .impl.max_access_size = 2,
};

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

static void sh7785lcr_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *pld = g_new(MemoryRegion, 1);
    SuperHCPU *cpu;
    SH7785State *soc;
    ResetData *reset_info;
    DriveInfo *dinfo;

    if (machine->ram_size != SDRAM_SIZE) {
        error_report("sh7785lcr has %d MiB of RAM", (int)(SDRAM_SIZE / MiB));
        exit(1);
    }

    cpu = SUPERH_CPU(cpu_create(machine->cpu_type));
    reset_info = g_new0(ResetData, 1);
    reset_info->cpu = cpu;
    reset_info->vector = cpu->env.pc;
    qemu_register_reset(main_cpu_reset, reset_info);

    memory_region_add_subregion(sysmem, SDRAM_BASE, machine->ram);

    soc = sh7785_init(cpu, sysmem, PCLK_HZ);
    sh7785_set_reg(soc, FRQMR1, FRQMR1_MODE16);

    /*
     * NOR flash: Linux registers it as physmap-flash with bankwidth 4.
     * TODO(manual): the exact part is not verified; AMD command set with
     * Spansion S29GL512 IDs is assumed.
     */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    pflash_cfi02_register(FLASH_BASE, "sh7785lcr.flash", FLASH_SIZE,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          128 * KiB, 1, 4, 0x0001, 0x227e, 0x2223, 0x2201,
                          0x555, 0x2aa, 0);

    memory_region_init_io(pld, NULL, &pld_ops, g_new0(uint16_t, 8),
                          "sh7785lcr-pld", 0x10);
    memory_region_add_subregion(sysmem, PLD_BASE, pld);

    memset(&boot_params, 0, sizeof(boot_params));
    if (machine->kernel_filename) {
        if (load_image_targphys(machine->kernel_filename,
                                SDRAM_BASE + LINUX_LOAD_OFFSET,
                                INITRD_LOAD_OFFSET - LINUX_LOAD_OFFSET,
                                NULL) < 0) {
            error_report("could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }
        reset_info->vector = (SDRAM_BASE + LINUX_LOAD_OFFSET) | 0xa0000000;
    }
    if (machine->initrd_filename) {
        int size = load_image_targphys(machine->initrd_filename,
                                       SDRAM_BASE + INITRD_LOAD_OFFSET,
                                       SDRAM_SIZE - INITRD_LOAD_OFFSET,
                                       NULL);
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
    rom_add_blob_fixed("boot_params", &boot_params, sizeof(boot_params),
                       SDRAM_BASE + ZERO_PAGE_OFFSET);
}

static void sh7785lcr_machine_init(MachineClass *mc)
{
    mc->desc = "Renesas SH7785LCR (SH-4A)";
    mc->init = sh7785lcr_init;
    mc->default_cpu_type = TYPE_SH7785_CPU;
    mc->default_ram_size = SDRAM_SIZE;
    mc->default_ram_id = "sh7785lcr.sdram";
}

DEFINE_MACHINE("sh7785lcr", sh7785lcr_machine_init)
