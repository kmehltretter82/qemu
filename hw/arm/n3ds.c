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
#include "qapi/visitor.h"
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
#include "hw/sd/n3ds_sdio.h"
#include "hw/arm/n3ds_bootstub.h"
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
#define N3DS_ARM11_MIDR      0x410FB025  /* ARM11 MPCore r0p5, read on a New 2DS XL */
#define N3DS_ARM11_SCTLR     0x00054078  /* as set by Luma3DS' start.s */

struct N3DSMachineState {
    MachineState parent_obj;

    CPUClusterState arm11_cluster;
    CPUClusterState arm9_cluster;
    char *boot9_path;            /* real boot ROM images instead of the stubs */
    char *boot11_path;
    char *otp_path;
    char *nand_cid;              /* 16 bytes hex as the console reads them */
    uint32_t arm11_icount_weight;
    uint32_t arm9_icount_weight;
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
    MemoryRegion boot11_lock;        /* protected-half overlays (CFG9_SYSPROT*) */
    MemoryRegion boot11_lock_alias[3];
    MemoryRegion boot11_zero;
    MemoryRegion boot9_lock;
    MemoryRegion boot9_lock_alias;
    MemoryRegion boot9_zero;
    MemoryRegion b9s_scratch;        /* boot9strap loads a FIRM section at 0xC0000000 */
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
        if (i == 0 && !s->boot9_path) {
            /* CP15 state as Luma3DS leaves it (n3ds-probes cp15_entry) */
            env->cp15.sctlr_ns = N3DS_ARM11_SCTLR;
            env->cp15.sctlr_s = N3DS_ARM11_SCTLR;
            env->cp15.c1_auxcr = 0xF;
            env->cp15.cpacr_el1 = 0x00F00000;
            env->cp15.ttbr0_ns = 0x1FFF4002;
            env->cp15.ttbr1_ns = 0x1FFF4002;
            env->cp15.dacr_ns = 0x55555555;
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

/*
 * The 3DS DMA-330s are configured with all peripheral requests and
 * interrupts usable from the non-secure manager/channels (the boot ROMs
 * issue non-secure DMAGO and DMAFLUSHP on peripheral 0).
 */
static void n3ds_pl330_config(DeviceState *dev)
{
    qdev_prop_set_uint8(dev, "num_periph_req", 32);
    qdev_prop_set_uint8(dev, "num_events", 32);
    qdev_prop_set_uint32(dev, "PNS", 0xFFFFFFFF);
    qdev_prop_set_uint32(dev, "INS", 0xFFFFFFFF);
}

/* Load a boot ROM dump (at most 64 KiB) at the given address. */
static void n3ds_load_rom(const char *path, const char *name, hwaddr addr,
                          AddressSpace *as)
{
    g_autofree char *data = NULL;
    gsize len = 0;
    GError *gerr = NULL;

    if (!g_file_get_contents(path, &data, &len, &gerr)) {
        error_report("n3ds: cannot read %s: %s", path, gerr->message);
        exit(1);
    }
    if (len == 0 || len > N3DS_BOOTROM_SIZE) {
        error_report("n3ds: %s: bad boot ROM size %zu", path, (size_t)len);
        exit(1);
    }
    rom_add_blob_fixed_as(name, data, len, addr, as);
}

#define N3DS_PATH_PROP(field)                                              \
static char *n3ds_get_##field(Object *obj, Error **errp)                   \
{                                                                          \
    return g_strdup(N3DS_MACHINE(obj)->field##_path);                      \
}                                                                          \
static void n3ds_set_##field(Object *obj, const char *v, Error **errp)     \
{                                                                          \
    N3DSMachineState *s = N3DS_MACHINE(obj);                               \
    g_free(s->field##_path);                                               \
    s->field##_path = g_strdup(v);                                         \
}
N3DS_PATH_PROP(boot9)
N3DS_PATH_PROP(boot11)
N3DS_PATH_PROP(otp)
static char *n3ds_get_nand_cid(Object *obj, Error **errp)
{
    return g_strdup(N3DS_MACHINE(obj)->nand_cid);
}
static void n3ds_set_nand_cid(Object *obj, const char *v, Error **errp)
{
    N3DSMachineState *s = N3DS_MACHINE(obj);
    g_free(s->nand_cid);
    s->nand_cid = g_strdup(v);
}

static void n3ds_get_arm11_icount_weight(Object *obj, Visitor *v,
                                          const char *name, void *opaque,
                                          Error **errp)
{
    uint32_t value = N3DS_MACHINE(obj)->arm11_icount_weight;

    visit_type_uint32(v, name, &value, errp);
}

static void n3ds_set_arm11_icount_weight(Object *obj, Visitor *v,
                                          const char *name, void *opaque,
                                          Error **errp)
{
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value < 1 || value > 1024) {
        error_setg(errp, "%s must be between 1 and 1024", name);
        return;
    }
    N3DS_MACHINE(obj)->arm11_icount_weight = value;
}

static void n3ds_get_arm9_icount_weight(Object *obj, Visitor *v,
                                         const char *name, void *opaque,
                                         Error **errp)
{
    uint32_t value = N3DS_MACHINE(obj)->arm9_icount_weight;

    visit_type_uint32(v, name, &value, errp);
}

static void n3ds_set_arm9_icount_weight(Object *obj, Visitor *v,
                                         const char *name, void *opaque,
                                         Error **errp)
{
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value < 1 || value > 1024) {
        error_setg(errp, "%s must be between 1 and 1024", name);
        return;
    }
    N3DS_MACHINE(obj)->arm9_icount_weight = value;
}

static void n3ds_init(MachineState *machine)
{
    N3DSMachineState *s = N3DS_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cluster;
    DriveInfo *di;
    DeviceState *dev;
    DeviceState *l2c_dev;
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
        object_property_set_int(cpu, "midr", N3DS_ARM11_MIDR, &error_abort);
        object_property_set_link(cpu, "memory", OBJECT(sysmem), &error_abort);
        /* New 3DS: cores 2 and 3 are held in reset until PDN releases them */
        object_property_set_bool(cpu, "start-powered-off", i >= 2, &error_abort);
        /*
         * The secondaries do not come up with the architectural SCTLR: the
         * console's core 1 has the round-robin replacement bit set and cores
         * 2 and 3 have that and high vectors as well (n3ds-probes
         * core1_boot, core23_boot).
         */
        if (i == 1) {
            ARM_CPU(cpu)->reset_sctlr = 0x00054078;
        } else if (i >= 2) {
            ARM_CPU(cpu)->reset_sctlr = 0x00056078;
        }
        /* ARM11 MPCore runs at 268 MHz vs the ARM9 at 134 MHz: give it
         * twice the per-round icount budget so the two interleave in
         * roughly the right ratio (only takes effect under -icount). */
        CPU(cpu)->tcg_exec_weight = s->arm11_icount_weight;
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
    CPU(&s->arm9)->tcg_exec_weight = s->arm9_icount_weight;
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
    /* reads-as-zero overlay on the protected upper 32 KiB of boot11 and its mirrors */
    memory_region_init_rom(&s->boot11_zero, NULL, "n3ds.boot11.locked",
                           N3DS_BOOTROM_SIZE / 2, &error_fatal);
    memory_region_init(&s->boot11_lock, OBJECT(machine), "n3ds.boot11-lock", 1ull << 32);
    {
        static const hwaddr lock11[3] = { N3DS_BOOTROM_SIZE / 2,
                                          N3DS_BOOTROM_SIZE + N3DS_BOOTROM_SIZE / 2,
                                          N3DS_BOOTROM_BASE + N3DS_BOOTROM_SIZE / 2 };
        for (i = 0; i < 3; i++) {
            memory_region_init_alias(&s->boot11_lock_alias[i], NULL, "n3ds.boot11.lock",
                                     &s->boot11_zero, 0, N3DS_BOOTROM_SIZE / 2);
            memory_region_add_subregion(&s->boot11_lock, lock11[i], &s->boot11_lock_alias[i]);
        }
    }
    memory_region_add_subregion_overlap(sysmem, 0, &s->boot11_lock, 1);
    memory_region_set_enabled(&s->boot11_lock, false);
    if (s->boot11_path) {
        n3ds_load_rom(s->boot11_path, "n3ds.boot11", 0, &address_space_memory);
    } else {
        rom_add_blob_fixed_as("n3ds.boot11-stub", n3ds_boot11_stub, sizeof(n3ds_boot11_stub),
                              0, &address_space_memory);
    }

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
    memory_region_init_ram(&s->b9s_scratch, NULL, "n3ds.b9s-scratch",
                           0x1000, &error_fatal);
    memory_region_add_subregion(&s->arm9_mem, 0xC0000000, &s->b9s_scratch);
    memory_region_init_rom(&s->boot9_zero, NULL, "n3ds.boot9.locked",
                           N3DS_BOOTROM_SIZE / 2, &error_fatal);
    memory_region_init(&s->boot9_lock, OBJECT(machine), "n3ds.boot9-lock", 1ull << 32);
    memory_region_init_alias(&s->boot9_lock_alias, NULL, "n3ds.boot9.lock",
                             &s->boot9_zero, 0, N3DS_BOOTROM_SIZE / 2);
    memory_region_add_subregion(&s->boot9_lock, N3DS_BOOTROM_BASE + N3DS_BOOTROM_SIZE / 2,
                                &s->boot9_lock_alias);
    memory_region_add_subregion_overlap(&s->arm9_mem, 0, &s->boot9_lock, 1);
    memory_region_set_enabled(&s->boot9_lock, false);
    if (s->boot9_path) {
        n3ds_load_rom(s->boot9_path, "n3ds.boot9", N3DS_BOOTROM_BASE, &s->arm9_as);
    } else {
        rom_add_blob_fixed_as("n3ds.boot9-stub", n3ds_boot9_stub, sizeof(n3ds_boot9_stub),
                              N3DS_BOOTROM_BASE, &s->arm9_as);
    }

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
    {
        /*
         * What the console's SCU reads at hand-off (n3ds-probes io_scu):
         * control 0x1ffe with the enable bit clear, a configuration whose
         * "CPUs in coherency" nibble is still zero, and cores 2 and 3
         * marked powered down.
         */
        Object *scu = object_resolve_path_component(OBJECT(s->mpcore), "scu");

        object_property_set_uint(scu, "control-reset", 0x1ffe, &error_abort);
        if (!s->boot9_path) {
            /*
             * Every interrupt priority reads 0x80 on a console booted through
             * the chainloader - the firmware ranks them all equally before
             * handing over (n3ds-probes gic_prio_bits).
             */
            Object *gic = object_resolve_path_component(OBJECT(s->mpcore),
                                                        "gic");

            object_property_set_uint(gic, "priority-reset", 0x80,
                                     &error_abort);
            /* and every shared interrupt pointed at core 0 */
            object_property_set_uint(gic, "target-reset", 0x01, &error_abort);
            /*
             * The chainloader leaves CPU0's SGIs in 1:N mode while the
             * secondary CPUs retain the N:N reset mode.
             */
            object_property_set_uint(gic, "sgi-model-reset-cpu0", 0xffff,
                                     &error_abort);
            /*
             * The display controllers, the PXI sync line and the MCU's I2C
             * bus have all pulsed by the time we take over, and nothing has
             * acknowledged them: ids 40-43, 80 and 85 read pending, the same
             * on seven consecutive boots (n3ds-probes gic_enable_state).
             */
            object_property_set_uint(gic, "pending-reset",
                                     0x0021000000000f00ULL, &error_abort);
        }
        object_property_set_uint(scu, "config", 0x5003, &error_abort);
        object_property_set_uint(scu, "cpu-status-reset", 0xf0, &error_abort);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->mpcore), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(s->mpcore), 0, N3DS_MPCORE_BASE);
    for (i = 0; i < N3DS_NUM_ARM11; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->mpcore), i,
                           qdev_get_gpio_in(DEVICE(&s->arm11[i]), ARM_CPU_IRQ));
    }
    l2c_dev = qdev_new("l2x0");
    sysbus_realize_and_unref(SYS_BUS_DEVICE(l2c_dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(l2c_dev), 0, N3DS_L2C_BASE);
    {
        static const hwaddr cdma_bases[2] = { N3DS_CDMA_BASE, N3DS_CDMA2_BASE };

        for (i = 0; i < 2; i++) {
            dev = qdev_new("pl330");
            object_property_set_link(OBJECT(dev), "memory", OBJECT(sysmem),
                                     &error_abort);
            n3ds_pl330_config(dev);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, cdma_bases[i]);
        }
    }

    /* ---- ARM9 interrupt controller and timers ---- */
    s->irq9 = qdev_new(TYPE_N3DS_IRQ);
    qdev_prop_set_bit(s->irq9, "luma-state", !s->boot9_path);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->irq9), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->irq9, 0, N3DS_IRQ9_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->irq9), 0,
                       qdev_get_gpio_in(DEVICE(&s->arm9), ARM_CPU_IRQ));

    {
        DeviceState *aes = qdev_new("n3ds-aes");
        DeviceState *sha = qdev_new("n3ds-sha");

        sysbus_realize_and_unref(SYS_BUS_DEVICE(aes), &error_fatal);
        n3ds_map(&s->arm9_mem, 0, aes, 0, N3DS_AES9_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(aes), 0,
                           qdev_get_gpio_in(s->irq9, N3DS_IRQ9_AES));
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sha), &error_fatal);
        n3ds_map(&s->arm9_mem, 0, sha, 0, N3DS_SHA9_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(sha), 0,
                           qdev_get_gpio_in(s->irq9, N3DS_IRQ9_SHA));
    }
    s->timer9 = qdev_new(TYPE_N3DS_TIMER);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->timer9), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->timer9, 0, N3DS_TIMER9_BASE);
    for (i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(s->timer9), i,
                           qdev_get_gpio_in(s->irq9, N3DS_IRQ9_TIMER0 + i));
    }

    /* ---- system control ---- */
    s->sysctl = qdev_new(TYPE_N3DS_SYSCTL);
    object_property_set_link(OBJECT(s->sysctl), "boot9-lock", OBJECT(&s->boot9_lock),
                             &error_abort);
    object_property_set_link(OBJECT(s->sysctl), "boot11-lock", OBJECT(&s->boot11_lock),
                             &error_abort);
    qdev_prop_set_bit(s->sysctl, "luma-state", !s->boot9_path);
    object_property_set_link(OBJECT(s->sysctl), "l2c",
                             OBJECT(sysbus_mmio_get_region(
                                 SYS_BUS_DEVICE(l2c_dev), 0)), &error_abort);
    object_property_set_link(OBJECT(s->sysctl), "extwram",
                             OBJECT(&s->arm9_wram_ext), &error_abort);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->sysctl), &error_fatal);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 0, N3DS_CFG9_BASE);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 1, N3DS_CFG_BASE);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 2, N3DS_PRNG_BASE);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->sysctl, 3, N3DS_CFG11_BASE);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->sysctl, 4, N3DS_PDN_BASE);
    n3ds_map(&s->arm9_mem, 0, s->sysctl, 5, N3DS_OTP9_BASE);
    /* past the end of the shared I/O window, so map it directly */
    sysbus_mmio_map(SYS_BUS_DEVICE(s->sysctl), 6, N3DS_FCRAMCFG_BASE);
    if (s->otp_path) {
        g_autofree char *otp = NULL;
        gsize len = 0;
        GError *gerr = NULL;

        if (!g_file_get_contents(s->otp_path, &otp, &len, &gerr)) {
            error_report("n3ds: cannot read %s: %s", s->otp_path, gerr->message);
            exit(1);
        }
        n3ds_sysctl_set_otp(s->sysctl, (const uint8_t *)otp, len);
    }
    {
        DeviceState *ndma = qdev_new("n3ds-ndma");
        DeviceState *xdma = qdev_new("pl330");
        int ch;

        object_property_set_link(OBJECT(ndma), "memory", OBJECT(&s->arm9_mem),
                                 &error_abort);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(ndma), &error_fatal);
        n3ds_map(&s->arm9_mem, 0, ndma, 0, N3DS_NDMA9_BASE);
        for (ch = 0; ch < 8; ch++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(ndma), ch,
                               qdev_get_gpio_in(s->irq9, ch));
        }
        /* ARM9 XDMA: a PL330 in the ARM9 address space */
        object_property_set_link(OBJECT(xdma), "memory", OBJECT(&s->arm9_mem),
                                 &error_abort);
        n3ds_pl330_config(xdma);
        /* the console's ARM9 DMA manager runs non-secure (io_xdma) */
        qdev_prop_set_uint8(xdma, "mgr_ns_at_rst", 1);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(xdma), &error_fatal);
        n3ds_map(&s->arm9_mem, 0, xdma, 0, N3DS_XDMA_BASE);
    }
    {
        DeviceState *rsa = qdev_new("n3ds-rsa");

        sysbus_realize_and_unref(SYS_BUS_DEVICE(rsa), &error_fatal);
        n3ds_map(&s->arm9_mem, 0, rsa, 0, N3DS_RSA9_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(rsa), 0,
                           qdev_get_gpio_in(s->irq9, N3DS_IRQ9_RSA));
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(s->sysctl), 0,
                       n3ds_gic_irq(s, N3DS_GIC_SOCMODE));

    /* ---- SD/MMC host (ARM9 side): SD card on port 0, eMMC on port 1 ---- */
    s->tmio = qdev_new(TYPE_N3DS_TMIO);
    qdev_prop_set_bit(s->tmio, "luma-state", !s->boot9_path);
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
    if (s->nand_cid && strlen(s->nand_cid) == 32) {
        /*
         * The console reads the CID as R2 response bits 127..8 in little-
         * endian byte order (byte 15 = 0); convert back to raw CID bytes.
         */
        char raw[33];
        int i;

        for (i = 0; i < 15; i++) {
            memcpy(raw + 2 * i, s->nand_cid + 2 * (14 - i), 2);
        }
        memcpy(raw + 30, "00", 2);
        raw[32] = 0;
        qdev_prop_set_string(dev, "cid", raw);
    }
    if (di) {
        qdev_prop_set_drive_err(dev, "drive", blk_by_legacy_dinfo(di),
                                &error_fatal);
    }
    qdev_realize_and_unref(dev, qdev_get_child_bus(s->tmio, "nand-bus"),
                           &error_fatal);

    /* ---- PXI ---- */
    s->pxi = qdev_new(TYPE_N3DS_PXI);
    qdev_prop_set_bit(s->pxi, "luma-state", !s->boot9_path);
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
            if (i2c_defs[i].base == N3DS_I2C2_BASE) {
                /* the MCU bus keeps the boot ROM's last transfer state */
                qdev_prop_set_uint8(s->i2c[i], "cnt-reset", 0x61);
            }
            sysbus_realize_and_unref(SYS_BUS_DEVICE(s->i2c[i]), &error_fatal);
            n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, s->i2c[i], 0,
                     i2c_defs[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(s->i2c[i]), 0,
                               n3ds_gic_irq(s, i2c_defs[i].irq));
        }
    }
    i2cbus = I2C_BUS(qdev_get_child_bus(s->i2c[1], "i2c"));
    {
        I2CSlave *mcu = i2c_slave_create_simple(i2cbus, TYPE_N3DS_MCU, 0x25);

        /* MCU interrupt (GPIO on the real SoC) -> GIC 0x71 */
        qdev_connect_gpio_out(DEVICE(mcu), 0, n3ds_gic_irq(s, 0x71));
    }

    /* ---- SPI buses, GPIO, WiFi SDIO host ---- */
    {
        static const struct { hwaddr base; int irq; } spi_defs[3] = {
            { N3DS_SPI2_BASE, 0x57 },   /* spi@10142800: touchscreen/codec */
            { N3DS_SPI3_BASE, 0x24 },   /* spi@10143800 */
            { N3DS_SPI1_BASE, 0x56 },   /* spi@10160800: NVRAM */
        };

        for (i = 0; i < 3; i++) {
            dev = qdev_new(TYPE_N3DS_SPI);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
            n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, dev, 0,
                     spi_defs[i].base);
            sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                               n3ds_gic_irq(s, spi_defs[i].irq));
        }
    }
    {
        /*
         * The third SD host controller. Nothing is known to use it, but the
         * console answers on it, so model it rather than leaving a hole.
         */
        DeviceState *sdio3 = qdev_new(TYPE_N3DS_TMIO);

        qdev_prop_set_bit(sdio3, "port1-present", true);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sdio3), &error_fatal);
        n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, sdio3, 0, 0x10100000);
    }
    dev = qdev_new(TYPE_N3DS_GPIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    n3ds_map(&s->shared_io, N3DS_IO_SHARED_BASE, dev, 0, N3DS_GPIO_BASE);

    {
        DeviceState *sdio = qdev_new(TYPE_N3DS_SDIO);

        qdev_realize_and_unref(sdio, NULL, &error_fatal);
        dev = qdev_new(TYPE_N3DS_TMIO);
        object_property_set_link(OBJECT(dev), "sdio", OBJECT(sdio),
                                 &error_abort);
        qdev_prop_set_uint8(dev, "nports", 1);   /* just the WiFi part */
    }
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
    sysbus_connect_irq(SYS_BUS_DEVICE(s->lcd), 0, n3ds_gic_irq(s, 0x2A));
    sysbus_connect_irq(SYS_BUS_DEVICE(s->lcd), 1, n3ds_gic_irq(s, 0x2B));

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

static void n3ds_machine_instance_init(Object *obj)
{
    N3DSMachineState *s = N3DS_MACHINE(obj);

    s->arm11_icount_weight = 2;
    s->arm9_icount_weight = 1;
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
    object_class_property_add_str(oc, "boot9", n3ds_get_boot9, n3ds_set_boot9);
    object_class_property_set_description(oc, "boot9",
        "ARM9 boot ROM dump (boot9.bin) used instead of the built-in stub");
    object_class_property_add_str(oc, "boot11", n3ds_get_boot11, n3ds_set_boot11);
    object_class_property_set_description(oc, "boot11",
        "ARM11 boot ROM dump (boot11.bin) used instead of the built-in stub");
    object_class_property_add_str(oc, "otp", n3ds_get_otp, n3ds_set_otp);
    object_class_property_add_str(oc, "nand-cid", n3ds_get_nand_cid, n3ds_set_nand_cid);
    object_class_property_set_description(oc, "nand-cid",
        "eMMC CID (32 hex digits) as the console reports it (essential.exefs nand_cid)");
    object_class_property_set_description(oc, "otp",
        "OTP dump (256 bytes) presented at 0x10012000");
    object_class_property_add(oc, "arm11-icount-weight", "uint32",
                              n3ds_get_arm11_icount_weight,
                              n3ds_set_arm11_icount_weight, NULL, NULL);
    object_class_property_set_description(oc, "arm11-icount-weight",
        "Relative icount execution budget for each ARM11 CPU (default: 2)");
    object_class_property_add(oc, "arm9-icount-weight", "uint32",
                              n3ds_get_arm9_icount_weight,
                              n3ds_set_arm9_icount_weight, NULL, NULL);
    object_class_property_set_description(oc, "arm9-icount-weight",
        "Relative icount execution budget for the ARM9 CPU (default: 1)");
}

static const TypeInfo n3ds_machine_type = {
    .name = TYPE_N3DS_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(N3DSMachineState),
    .instance_init = n3ds_machine_instance_init,
    .class_init = n3ds_machine_class_init,
    .interfaces = arm_machine_interfaces,
};

static void n3ds_machine_register_types(void)
{
    type_register_static(&n3ds_machine_type);
}

type_init(n3ds_machine_register_types)
