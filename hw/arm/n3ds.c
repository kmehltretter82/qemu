/*
 * Nintendo New 3DS / New 2DS XL ("KTR", SoC "LGR") machine model.
 *
 * Four ARM11 MPCore cores with the MPCore private peripherals and an
 * L2C-310, plus the ARM946E-S "ARM9" security processor with its own
 * address space. The machine boots a FIRM image the way the Luma3DS
 * chainloader would: sections are copied to their physical addresses,
 * the ARM9 starts at the ARM9 entry point with Luma's argc/argv and
 * the ARM11 entry point is published in the AXI WRAM mailbox that
 * core 0 polls. No boot ROM is required.
 *
 *   qemu-system-arm -M n3ds -kernel payload.firm \
 *       -drive if=sd,index=0,format=raw,file=sd.img
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/arm/machines-qom.h"
#include "hw/cpu/cluster.h"
#include "hw/cpu/arm11mpcore.h"
#include "hw/misc/unimp.h"
#include "hw/misc/n3ds.h"
#include "hw/i2c/i2c.h"
#include "hw/sd/sd.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/memory.h"
#include "system/reset.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpu-qom.h"
#include "qom/object.h"

#define N3DS_NUM_ARM11 4

/* Luma3DS chainloader hand-off state */
#define N3DS_BOOTARGS_ADDR   0x080FF000
#define N3DS_BOOTARGS_ARGV   (N3DS_BOOTARGS_ADDR + 0x000)
#define N3DS_BOOTARGS_PATH   (N3DS_BOOTARGS_ADDR + 0x100)
#define N3DS_BOOTARGS_FBS    (N3DS_BOOTARGS_ADDR + 0x200)
#define N3DS_BOOTARGS_SIZE   0x300
#define N3DS_ARM9_MAGIC      0x4BEEF
#define N3DS_ARM9_SCTLR      0x00052078  /* ITCM, DTCM, high vectors */
#define N3DS_ARM11_SCTLR     0x00054078  /* as set by Luma3DS' start.s */

struct N3DSMachineState {
    MachineState parent_obj;

    CPUClusterState arm11_cluster;
    CPUClusterState arm9_cluster;
    ARMCPU arm11[N3DS_NUM_ARM11];
    ARMCPU arm9;

    MemoryRegion arm9_mem;
    AddressSpace arm9_as;
    MemoryRegion itcm;
    MemoryRegion itcm_alias[2];
    MemoryRegion arm9_wram;
    MemoryRegion arm9_wram_ext;
    MemoryRegion dtcm;
    MemoryRegion boot9;
    MemoryRegion boot11;
    MemoryRegion boot11_alias[2];
    MemoryRegion vram;
    MemoryRegion qtm;
    MemoryRegion dsp;
    MemoryRegion axi_wram;
    MemoryRegion shared_io;
    MemoryRegion arm9_alias_vram;
    MemoryRegion arm9_alias_dsp;
    MemoryRegion arm9_alias_axi;
    MemoryRegion arm9_alias_fcram;
    MemoryRegion arm9_alias_io;

    DeviceState *mpcore;
    DeviceState *sysctl;
    DeviceState *irq9;
    DeviceState *timer9;
    DeviceState *tmio;
    DeviceState *pxi;
    DeviceState *i2c[3];
    DeviceState *hid;
    DeviceState *lcd;

    uint32_t arm9_entry;
    uint32_t arm11_entry;
    bool dtcm_mapped;
};

#define TYPE_N3DS_MACHINE MACHINE_TYPE_NAME("n3ds")
OBJECT_DECLARE_SIMPLE_TYPE(N3DSMachineState, N3DS_MACHINE)

/*
 * boot11 stub: core 0 enables the GIC distributor (firmware chainloaded
 * by Luma3DS relies on interrupts being deliverable) and waits for the
 * Luma ARM11 entry mailbox; cores 1-3 park in WFI until their mailbox at
 * 0x1FFFFFDC is written and an SGI wakes them.
 */
static const uint32_t boot11_stub[] = {
    0xea000006, 0xeafffffe, 0xeafffffe, 0xeafffffe,
    0xeafffffe, 0xeafffffe, 0xeafffffe, 0xeafffffe,
    0xee100fb0, 0xe2000003, 0xe3500000, 0x1a000007,
    0xe59f3044, 0xe3a04001, 0xe5834000, 0xe3e0123e,
    0xe5912000, 0xe3520000, 0x0afffffc, 0xe12fff12,
    0xe59f3028, 0xe3a04001, 0xe5834000, 0xe3a040f0,
    0xe5834004, 0xe59f1018, 0xe320f003, 0xe5912000,
    0xe3520000, 0x0afffffb, 0xe12fff12, 0x17e01000,
    0x17e00100, 0x1fffffdc,
};

/* boot9 stub: hanging exception vectors, `bx lr` at the two cache
 * maintenance entry points Luma3DS calls (0x830, 0xAB4). */
#define BOOT9_STUB_SIZE 0xB00
static void n3ds_make_boot9(uint8_t *buf)
{
    int i;

    memset(buf, 0, BOOT9_STUB_SIZE);
    for (i = 0; i < 8; i++) {
        stl_le_p(buf + 4 * i, 0xeafffffe);       /* b . */
    }
    stl_le_p(buf + 0x830, 0xe12fff1e);           /* bx lr */
    stl_le_p(buf + 0xab4, 0xe12fff1e);           /* bx lr */
}

static void n3ds_unimp(MemoryRegion *container, const char *name,
                       hwaddr base, hwaddr size, int priority)
{
    DeviceState *dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion_overlap(container, base,
        sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0), priority);
}

static void n3ds_map(MemoryRegion *container, hwaddr container_base,
                     DeviceState *dev, int n, hwaddr addr)
{
    memory_region_add_subregion(container, addr - container_base,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), n));
}

static qemu_irq n3ds_gic_irq(N3DSMachineState *s, int gic_id)
{
    return qdev_get_gpio_in(s->mpcore, N3DS_GIC_SPI(gic_id));
}

/*
 * ARM946E-S DTCM region register: [31:12] base, [5:1] size code. The
 * 16 KiB DTCM is mapped at the window base (mirroring inside larger
 * windows is not modelled). The ITCM window is fixed at 0 on this core.
 */
static void n3ds_arm9_map_dtcm(N3DSMachineState *s, uint32_t reg)
{
    hwaddr base = reg & 0xFFFFF000u;

    if (s->dtcm_mapped) {
        memory_region_del_subregion(&s->arm9_mem, &s->dtcm);
    }
    memory_region_add_subregion_overlap(&s->arm9_mem, base, &s->dtcm, 2);
    s->dtcm_mapped = true;
}

static void n3ds_arm9_tcm_changed(ARMCPU *cpu, int which, uint32_t value,
                                  void *opaque)
{
    N3DSMachineState *s = opaque;

    if (which == 0) {
        n3ds_arm9_map_dtcm(s, value);
    }
}

#define N3DS_ARM9_DTCM_REGION 0xFFF0000Au   /* 16 KiB at 0xFFF00000 (Luma3DS) */
#define N3DS_ARM9_ITCM_REGION 0x00000024u   /* 128 MiB window at 0 */

static void n3ds_cpu_reset(void *opaque)
{
    N3DSMachineState *s = opaque;
    int i;

    for (i = 0; i < N3DS_NUM_ARM11; i++) {
        CPUState *cs = CPU(&s->arm11[i]);
        CPUARMState *env = &s->arm11[i].env;

        cpu_reset(cs);
        env->regs[15] = 0;               /* boot11 stub at physical 0 */
        if (i == 0) {
            env->cp15.sctlr_ns = N3DS_ARM11_SCTLR;
            env->cp15.sctlr_s = N3DS_ARM11_SCTLR;
        }
        arm_rebuild_hflags(env);
    }

    {
        CPUState *cs = CPU(&s->arm9);
        CPUARMState *env = &s->arm9.env;

        cpu_reset(cs);
        env->cp15.sctlr_ns = N3DS_ARM9_SCTLR;
        env->cp15.sctlr_s = N3DS_ARM9_SCTLR;
        env->cp15.pmsav5_tcm[0] = N3DS_ARM9_DTCM_REGION;
        env->cp15.pmsav5_tcm[1] = N3DS_ARM9_ITCM_REGION;
        n3ds_arm9_map_dtcm(s, N3DS_ARM9_DTCM_REGION);
        if (s->arm9_entry) {
            env->regs[0] = 2;
            env->regs[1] = N3DS_BOOTARGS_ARGV;
            env->regs[2] = N3DS_ARM9_MAGIC;
            env->regs[15] = s->arm9_entry;
        } else {
            env->regs[15] = N3DS_BOOTROM_BASE;
        }
        arm_rebuild_hflags(env);
    }
}

static void n3ds_load_firm(N3DSMachineState *s, const char *filename)
{
    g_autofree gchar *data = NULL;
    g_autofree gchar *base = NULL;
    g_autofree gchar *path = NULL;
    GError *gerr = NULL;
    gsize len;
    uint8_t args[N3DS_BOOTARGS_SIZE];
    uint32_t zero = 0;
    int i;

    if (!g_file_get_contents(filename, &data, &len, &gerr)) {
        error_report("n3ds: cannot read FIRM '%s': %s", filename, gerr->message);
        exit(1);
    }
    if (len < 0x200 || memcmp(data, "FIRM", 4) != 0) {
        error_report("n3ds: '%s' is not a FIRM image", filename);
        exit(1);
    }
    s->arm11_entry = ldl_le_p(data + 0x8);
    s->arm9_entry = ldl_le_p(data + 0xC);

    for (i = 0; i < 4; i++) {
        const uint8_t *hdr = (const uint8_t *)data + 0x40 + i * 0x30;
        uint32_t off = ldl_le_p(hdr + 0);
        uint32_t addr = ldl_le_p(hdr + 4);
        uint32_t size = ldl_le_p(hdr + 8);
        g_autofree char *name = NULL;

        if (size == 0) {
            continue;
        }
        if ((uint64_t)off + size > len) {
            error_report("n3ds: FIRM section %d exceeds the file", i);
            exit(1);
        }
        name = g_strdup_printf("n3ds.firm.section%d", i);
        rom_add_blob_fixed_as(name, data + off, size, addr, &s->arm9_as);
    }

    /* Luma3DS passes argc=2, argv = { path on the SD card, framebuffers } */
    memset(args, 0, sizeof(args));
    stl_le_p(args + 0x000, N3DS_BOOTARGS_PATH);
    stl_le_p(args + 0x004, N3DS_BOOTARGS_FBS);
    base = g_path_get_basename(filename);
    path = g_strdup_printf("sdmc:/luma/payloads/%s", base);
    g_strlcpy((char *)args + 0x100, path, 0x100);
    stl_le_p(args + 0x200, 0x18300000);  /* fbs[0].top_left */
    stl_le_p(args + 0x204, 0x18300000);  /* fbs[0].top_right */
    stl_le_p(args + 0x208, 0x18346500);  /* fbs[0].bottom */
    stl_le_p(args + 0x20C, 0x18400000);  /* fbs[1].top_left */
    stl_le_p(args + 0x210, 0x18400000);  /* fbs[1].top_right */
    stl_le_p(args + 0x214, 0x18446500);  /* fbs[1].bottom */
    rom_add_blob_fixed_as("n3ds.bootargs", args, sizeof(args),
                          N3DS_BOOTARGS_ADDR, &s->arm9_as);

    /* Luma writes the ARM11 entry point right before jumping to ARM9 */
    if (s->arm11_entry) {
        uint32_t entry = cpu_to_le32(s->arm11_entry);
        rom_add_blob_fixed_as("n3ds.arm11-mailbox", &entry, sizeof(entry),
                              N3DS_ARM11_ENTRY_MAILBOX, &address_space_memory);
    }
    rom_add_blob_fixed_as("n3ds.core1-mailbox", &zero, sizeof(zero),
                          N3DS_CORE1_ENTRY_MAILBOX, &address_space_memory);
}

static void n3ds_init(MachineState *machine)
{
    N3DSMachineState *s = N3DS_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cluster;
    uint8_t boot9[BOOT9_STUB_SIZE];
    DriveInfo *di;
    DeviceState *dev;
    I2CBus *i2cbus;
    int i;

    if (machine->ram_size != N3DS_FCRAM_SIZE_KTR &&
        machine->ram_size != N3DS_FCRAM_SIZE_CTR) {
        error_report("n3ds: FCRAM size must be 128M (CTR) or 256M (KTR)");
        exit(1);
    }

    /* ---- CPUs ---- */
    cluster = OBJECT(&s->arm11_cluster);
    object_initialize_child(OBJECT(machine), "arm11-cluster", &s->arm11_cluster,
                            TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 0);
    for (i = 0; i < N3DS_NUM_ARM11; i++) {
        Object *cpu;

        object_initialize_child(cluster, "arm11[*]", &s->arm11[i],
                                ARM_CPU_TYPE_NAME("arm11mpcore"));
        cpu = OBJECT(&s->arm11[i]);
        object_property_set_int(cpu, "mp-affinity", i, &error_abort);
        object_property_set_link(cpu, "memory", OBJECT(sysmem), &error_abort);
        /* New 3DS: cores 2 and 3 are held in reset until PDN releases them */
        object_property_set_bool(cpu, "start-powered-off", i >= 2, &error_abort);
        qdev_realize(DEVICE(cpu), NULL, &error_fatal);
    }
    qdev_realize(DEVICE(cluster), NULL, &error_fatal);

    memory_region_init(&s->arm9_mem, OBJECT(machine), "n3ds.arm9-mem",
                       1ULL << 32);
    address_space_init(&s->arm9_as, &s->arm9_mem, "n3ds-arm9");

    cluster = OBJECT(&s->arm9_cluster);
    object_initialize_child(OBJECT(machine), "arm9-cluster", &s->arm9_cluster,
                            TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(cluster), "cluster-id", 1);
    object_initialize_child(cluster, "arm9", &s->arm9,
                            ARM_CPU_TYPE_NAME("arm946"));
    object_property_set_link(OBJECT(&s->arm9), "memory", OBJECT(&s->arm9_mem),
                             &error_abort);
    s->arm9.tcm_region_changed = n3ds_arm9_tcm_changed;
    s->arm9.tcm_region_opaque = s;
    qdev_realize(DEVICE(&s->arm9), NULL, &error_fatal);
    qdev_realize(DEVICE(cluster), NULL, &error_fatal);

    /* ---- memory, shared ---- */
    memory_region_add_subregion(sysmem, N3DS_FCRAM_BASE, machine->ram);
    memory_region_init_ram(&s->vram, NULL, "n3ds.vram", N3DS_VRAM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, N3DS_VRAM_BASE, &s->vram);
    memory_region_init_ram(&s->dsp, NULL, "n3ds.dsp-ram", N3DS_DSP_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, N3DS_DSP_BASE, &s->dsp);
    memory_region_init_ram(&s->axi_wram, NULL, "n3ds.axi-wram",
                           N3DS_AXI_WRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, N3DS_AXI_WRAM_BASE, &s->axi_wram);

    memory_region_init_alias(&s->arm9_alias_fcram, NULL, "n3ds.arm9.fcram",
                             machine->ram, 0, machine->ram_size);
    memory_region_add_subregion(&s->arm9_mem, N3DS_FCRAM_BASE,
                                &s->arm9_alias_fcram);
    memory_region_init_alias(&s->arm9_alias_vram, NULL, "n3ds.arm9.vram",
                             &s->vram, 0, N3DS_VRAM_SIZE);
    memory_region_add_subregion(&s->arm9_mem, N3DS_VRAM_BASE,
                                &s->arm9_alias_vram);
    memory_region_init_alias(&s->arm9_alias_dsp, NULL, "n3ds.arm9.dsp-ram",
                             &s->dsp, 0, N3DS_DSP_SIZE);
    memory_region_add_subregion(&s->arm9_mem, N3DS_DSP_BASE, &s->arm9_alias_dsp);
    memory_region_init_alias(&s->arm9_alias_axi, NULL, "n3ds.arm9.axi-wram",
                             &s->axi_wram, 0, N3DS_AXI_WRAM_SIZE);
    memory_region_add_subregion(&s->arm9_mem, N3DS_AXI_WRAM_BASE,
                                &s->arm9_alias_axi);

    /* ---- memory, ARM11 only ---- */
    memory_region_init_ram(&s->qtm, NULL, "n3ds.qtm-ram", N3DS_QTM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, N3DS_QTM_BASE, &s->qtm);
    memory_region_init_rom(&s->boot11, NULL, "n3ds.boot11", N3DS_BOOTROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(sysmem, 0, &s->boot11);
    memory_region_init_alias(&s->boot11_alias[0], NULL, "n3ds.boot11.mirror",
                             &s->boot11, 0, N3DS_BOOTROM_SIZE);
    memory_region_add_subregion(sysmem, N3DS_BOOTROM_SIZE, &s->boot11_alias[0]);
    memory_region_init_alias(&s->boot11_alias[1], NULL, "n3ds.boot11.high",
                             &s->boot11, 0, N3DS_BOOTROM_SIZE);
    memory_region_add_subregion(sysmem, N3DS_BOOTROM_BASE, &s->boot11_alias[1]);
    rom_add_blob_fixed_as("n3ds.boot11-stub", boot11_stub, sizeof(boot11_stub),
                          0, &address_space_memory);

    /* ---- memory, ARM9 only ---- */
    memory_region_init_ram(&s->itcm, NULL, "n3ds.itcm", N3DS_ITCM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(&s->arm9_mem, N3DS_ITCM_BASE, &s->itcm);
    memory_region_init_alias(&s->itcm_alias[0], NULL, "n3ds.itcm.kernel",
                             &s->itcm, 0, N3DS_ITCM_SIZE);
    memory_region_add_subregion(&s->arm9_mem, 0x01FF8000, &s->itcm_alias[0]);
    memory_region_init_alias(&s->itcm_alias[1], NULL, "n3ds.itcm.bootrom",
                             &s->itcm, 0, N3DS_ITCM_SIZE);
    memory_region_add_subregion(&s->arm9_mem, 0x07FF8000, &s->itcm_alias[1]);
    memory_region_init_ram(&s->arm9_wram, NULL, "n3ds.arm9-wram",
                           N3DS_ARM9_WRAM_SIZE, &error_fatal);
    memory_region_add_subregion(&s->arm9_mem, N3DS_ARM9_WRAM_BASE, &s->arm9_wram);
    memory_region_init_ram(&s->arm9_wram_ext, NULL, "n3ds.arm9-wram-ext",
                           N3DS_ARM9_WRAM_EXT_SIZE, &error_fatal);
    memory_region_add_subregion(&s->arm9_mem, N3DS_ARM9_WRAM_EXT,
                                &s->arm9_wram_ext);
    memory_region_init_ram(&s->dtcm, NULL, "n3ds.dtcm", N3DS_DTCM_SIZE,
                           &error_fatal);
    n3ds_arm9_map_dtcm(s, N3DS_ARM9_DTCM_REGION);
    memory_region_init_rom(&s->boot9, NULL, "n3ds.boot9", N3DS_BOOTROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(&s->arm9_mem, N3DS_BOOTROM_BASE, &s->boot9);
    n3ds_make_boot9(boot9);
    rom_add_blob_fixed_as("n3ds.boot9-stub", boot9, sizeof(boot9),
                          N3DS_BOOTROM_BASE, &s->arm9_as);

    /* ---- shared IO window ---- */
    memory_region_init(&s->shared_io, OBJECT(machine), "n3ds.io-shared",
                       N3DS_IO_SHARED_SIZE);
    memory_region_add_subregion(sysmem, N3DS_IO_SHARED_BASE, &s->shared_io);
    memory_region_init_alias(&s->arm9_alias_io, NULL, "n3ds.arm9.io-shared",
                             &s->shared_io, 0, N3DS_IO_SHARED_SIZE);
    memory_region_add_subregion(&s->arm9_mem, N3DS_IO_SHARED_BASE,
                                &s->arm9_alias_io);

    /* ---- ARM11 MPCore private peripherals, L2C, CDMA ---- */
    s->mpcore = qdev_new(TYPE_ARM11MPCORE_PRIV);
    qdev_prop_set_uint32(s->mpcore, "num-cpu", N3DS_NUM_ARM11);
    qdev_prop_set_uint32(s->mpcore, "num-irq", 128);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->mpcore), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->mpcore), 0, N3DS_MPCORE_BASE);
    for (i = 0; i < N3DS_NUM_ARM11; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->mpcore), i,
                           qdev_get_gpio_in(DEVICE(&s->arm11[i]), ARM_CPU_IRQ));
    }
    sysbus_create_simple("l2x0", N3DS_L2C_BASE, NULL);
    {
        static const hwaddr cdma_bases[2] = { N3DS_CDMA_BASE, N3DS_CDMA2_BASE };

        for (i = 0; i < 2; i++) {
            dev = qdev_new("pl330");
            object_property_set_link(OBJECT(dev), "memory", OBJECT(sysmem),
                                     &error_abort);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, cdma_bases[i]);
        }
    }

    /* ---- ARM9 interrupt controller and timers ---- */
    s->irq9 = qdev_new(TYPE_N3DS_IRQ);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->irq9), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->irq9, 0, N3DS_IRQ9_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->irq9), 0,
                       qdev_get_gpio_in(DEVICE(&s->arm9), ARM_CPU_IRQ));

    s->timer9 = qdev_new(TYPE_N3DS_TIMER);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->timer9), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->timer9, 0, N3DS_TIMER9_BASE);
    for (i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->timer9), i,
                           qdev_get_gpio_in(s->irq9, N3DS_IRQ9_TIMER0 + i));
    }

    /* ---- system control ---- */
    s->sysctl = qdev_new(TYPE_N3DS_SYSCTL);
    object_property_set_link(OBJECT(s->sysctl), "extwram",
                             OBJECT(&s->arm9_wram_ext), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->sysctl), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 0, N3DS_CFG9_BASE);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 1, N3DS_CFG_BASE);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 2, N3DS_PRNG_BASE);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->sysctl, 3, N3DS_CFG11_BASE);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->sysctl, 4, N3DS_PDN_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->sysctl), 0,
                       n3ds_gic_irq(s, N3DS_GIC_SOCMODE));

    /* ---- SD/MMC host (ARM9 side): SD card on port 0, eMMC on port 1 ---- */
    s->tmio = qdev_new(TYPE_N3DS_TMIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->tmio), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->tmio, 0, N3DS_SDMMC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->tmio), 0,
                       qdev_get_gpio_in(s->irq9, N3DS_IRQ9_SDIO1));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->tmio), 1,
                       qdev_get_gpio_in(s->irq9, N3DS_IRQ9_SDIO1_ASYNC));
    di = drive_get(IF_SD, 0, 0);
    if (di) {
        dev = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
        qdev_realize_and_unref(dev, qdev_get_child_bus(s->tmio, "sd-bus"),
                               &error_fatal);
    }
    di = drive_get(IF_SD, 0, 1);
    dev = qdev_new(TYPE_EMMC);
    if (di) {
        qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
    }
    qdev_realize_and_unref(dev, qdev_get_child_bus(s->tmio, "nand-bus"),
                           &error_fatal);

    /* ---- PXI ---- */
    s->pxi = qdev_new(TYPE_N3DS_PXI);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->pxi), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->pxi, 0, N3DS_PXI9_BASE);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->pxi, 1, N3DS_PXI11_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->pxi), 0,
                       qdev_get_gpio_in(s->irq9, N3DS_IRQ9_PXI_SYNC));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->pxi), 1,
                       qdev_get_gpio_in(s->irq9, N3DS_IRQ9_PXI_SEND));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->pxi), 2,
                       qdev_get_gpio_in(s->irq9, N3DS_IRQ9_PXI_RECV));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->pxi), 3,
                       n3ds_gic_irq(s, N3DS_GIC_PXI_SYNC));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->pxi), 4,
                       n3ds_gic_irq(s, N3DS_GIC_PXI_SEND));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->pxi), 5,
                       n3ds_gic_irq(s, N3DS_GIC_PXI_RECV));

    /* ---- I2C buses and the MCU ---- */
    {
        static const struct { hwaddr base; int irq; } i2c_defs[3] = {
            { N3DS_I2C1_BASE, N3DS_GIC_I2C1 },
            { N3DS_I2C2_BASE, N3DS_GIC_I2C2 },
            { N3DS_I2C3_BASE, N3DS_GIC_I2C3 },
        };

        for (i = 0; i < 3; i++) {
            s->i2c[i] = qdev_new(TYPE_N3DS_I2C);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(s->i2c[i]), &error_fatal);
            n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->i2c[i], 0,
                     i2c_defs[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(s->i2c[i]), 0,
                               n3ds_gic_irq(s, i2c_defs[i].irq));
        }
    }
    i2cbus = I2C_BUS(qdev_get_child_bus(s->i2c[1], "i2c"));
    i2c_slave_create_simple(i2cbus, TYPE_N3DS_MCU, 0x25);

    /* ---- SPI buses, GPIO, WiFi SDIO host ---- */
    {
        static const hwaddr spi_bases[3] = {
            N3DS_SPI2_BASE, N3DS_SPI3_BASE, N3DS_SPI1_BASE
        };

        for (i = 0; i < 3; i++) {
            dev = qdev_new(TYPE_N3DS_SPI);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, dev, 0, spi_bases[i]);
        }
    }
    dev = qdev_new(TYPE_N3DS_GPIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, dev, 0, N3DS_GPIO_BASE);

    dev = qdev_new(TYPE_N3DS_TMIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, dev, 0, N3DS_WIFI_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, n3ds_gic_irq(s, N3DS_GIC_SDIO3));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       n3ds_gic_irq(s, N3DS_GIC_SDIO3_ASYNC));

    /* ---- HID, LCD/GPU ---- */
    s->hid = qdev_new(TYPE_N3DS_HID);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->hid), &error_fatal);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->hid, 0, N3DS_HID_BASE);

    s->lcd = qdev_new(TYPE_N3DS_LCD);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->lcd), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->lcd), 0, N3DS_LCD_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->lcd), 1, N3DS_GPU_BASE);

    /* ---- everything else logs as unimplemented ---- */
    n3ds_unimp(&s->arm9_mem, "n3ds.io9", N3DS_IO9_BASE, 0x100000, -10);
    n3ds_unimp(&s->shared_io, "n3ds.io-shared", 0, N3DS_IO_SHARED_SIZE, -10);
    n3ds_unimp(sysmem, "n3ds.io11", N3DS_IO11_BASE, 0x100000, -10);
    n3ds_unimp(sysmem, "n3ds.gpu", N3DS_GPU_BASE, 0x100000, -10);

    /* ---- boot ---- */
    if (machine->kernel_filename) {
        n3ds_load_firm(s, machine->kernel_filename);
    }
    qemu_register_reset(n3ds_cpu_reset, s);
}

static void n3ds_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Nintendo New 3DS / New 2DS XL (4x ARM11 MPCore + ARM946E-S)";
    mc->init = n3ds_init;
    mc->min_cpus = N3DS_NUM_ARM11 + 1;
    mc->max_cpus = N3DS_NUM_ARM11 + 1;
    mc->default_cpus = N3DS_NUM_ARM11 + 1;
    mc->default_ram_id = "n3ds.fcram";
    mc->default_ram_size = N3DS_FCRAM_SIZE_KTR;
    mc->block_default_type = IF_SD;
    mc->no_parallel = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
}

static const TypeInfo n3ds_machine_type = {
    .name = TYPE_N3DS_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(N3DSMachineState),
    .class_init = n3ds_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void n3ds_machine_register_types(void)
{
    type_register_static(&n3ds_machine_type);
}

type_init(n3ds_machine_register_types)
