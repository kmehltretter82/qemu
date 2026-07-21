/*
 * NetBSD/acorn32 boot support for the Acorn RiscPC.
 *
 * On real hardware the kernel is started by !BtNetBSD, a RISC OS
 * application.  That matters more than it sounds: RISC OS runs with the
 * MMU enabled, so BtNetBSD hands the kernel a machine that is *already
 * translating*.  The kernel relies on it - sys/arch/acorn32 links at
 * 0xf0000000 and its entry point
 *
 *   start:  mrs r1, CPSR
 *           add r1, pc, #68
 *           ldm r1, {r1, r2, r8, sp}   ; absolute 0xf03f.... addresses
 *           str r4, [r1], #4           ; zeroes BSS
 *           bl  initarm
 *
 * loads sp and the BSS bounds from absolute virtual addresses and never
 * touches CP15.  RiscPC RAM is physically at 0x10000000, so entering with
 * the MMU off would fault on the first store.  initarm() does build page
 * tables, but they are the kernel's *final* tables; by then it is already
 * running mapped.
 *
 * So this file does what BtNetBSD did:
 *
 *   1. load the ELF, translating its virtual addresses to physical,
 *   2. build an L1 section table (kernel at 0xf0000000, RAM and I/O
 *      identity-mapped - initarm dereferences VIDC_HW_BASE 0x03400000 and
 *      IOMD_HW_BASE raw, see acorn32/include/vidc_machdep.h),
 *   3. fill in a struct bootconfig, which is the ABI between loader and
 *      kernel (acorn32/include/bootconfig.h),
 *   4. run a short stub that enables the MMU and jumps to the kernel with
 *      r0 pointing at the bootconfig.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "system/memory.h"
#include "hw/core/loader.h"
#include "hw/arm/acorn32-boot.h"
#include "elf.h"
#include "system/address-spaces.h"
#include "cpu.h"

/* Kernel link address: KERNEL_BASE in acorn32/include/vmparam.h */
#define ACORN32_KERNEL_VA   0xf0000000

/*
 * Physical layout.
 *
 * The kernel must go at the very base of DRAM.  initarm() does not read
 * where it was loaded - it assumes it (rpc_machdep.c:555):
 *
 *     kernel_start = physical_start;              // lowest dram[] address
 *     physical_freestart = kernel_start;
 *     physical_freestart += bootconfig.kernsize;
 *
 * so anything below physical_start + kernsize is "the kernel" and
 * everything above it is free for the kernel's own page tables.  Load the
 * image anywhere else and it allocates straight over itself.
 *
 * Our own structures therefore live in the *top* megabyte, which initarm()
 * will not reach: it allocates upward from just past the kernel, and is
 * finished with all three of them (the stub before entry, the L1 table
 * once it installs its own TTBR, the bootconfig once canonicalise_
 * bootconfig() has copied it) long before allocation could get that far.
 * The top of RAM is 1 MiB aligned, so the L1 table's 16 KiB alignment
 * comes for free.
 */
#define KERNEL_OFF          0x00000000
#define BOOTSCRATCH_SIZE    (4 * MiB)
#define L1TABLE_OFF         0x00000000  /* within the scratch region */
#define STUB_OFF            0x00004000
#define BOOTCONFIG_OFF      0x00008000
#define FRAMEBUFFER_OFF     0x00200000  /* 1 MiB aligned */
#define FRAMEBUFFER_SIZE    (2 * MiB)   /* acornfb's MAX_SIZE, and enough
                                         * for 640x480x8 */
#define FB_WIDTH            640
#define FB_HEIGHT           480
#define FB_LOG2_BPP         3           /* 8 bpp */

#define L1_ENTRIES          4096
#define L1_SIZE             (L1_ENTRIES * 4)

/*
 * ARMv4 L1 section descriptor: base | AP=11 | domain 0 | bit4 | type=2.
 * cb carries the C and B bits (0xc for cached RAM, 0 for device I/O).
 */
#define L1_SECTION(pa, cb)  (((pa) & 0xfff00000) | (3 << 10) | (1 << 4) | (cb) | 2)
#define L1_CB_RAM           0x0c
#define L1_CB_DEV           0x00

/* struct bootconfig, acorn32/include/bootconfig.h */
#define BOOTCONFIG_MAGIC    0x43112233
#define BOOTCONFIG_VERSION  2
#define DRAM_BLOCKS         32
#define VRAM_BLOCKS         16

typedef struct {
    uint32_t address;
    uint32_t pages;
    uint32_t flags;
} QEMU_PACKED phys_mem;

struct acorn32_bootconfig {
    uint32_t magic;
    uint32_t version;
    uint8_t  machine_id[4];
    char     kernelname[80];
    char     args[512];
    uint32_t kernvirtualbase;
    uint32_t kernphysicalbase;
    uint32_t kernsize;
    uint32_t scratchvirtualbase;
    uint32_t scratchphysicalbase;
    uint32_t scratchsize;
    uint32_t ksym_start;
    uint32_t ksym_end;
    uint32_t MDFvirtualbase;
    uint32_t MDFphysicalbase;
    uint32_t MDFsize;
    uint32_t display_phys;
    uint32_t display_start;
    uint32_t display_size;
    uint32_t width;
    uint32_t height;
    uint32_t log2_bpp;
    uint32_t framerate;
    char     reserved[512];
    uint32_t pagesize;
    uint32_t drampages;
    uint32_t vrampages;
    uint32_t dramblocks;
    uint32_t vramblocks;
    phys_mem dram[DRAM_BLOCKS];
    phys_mem vram[VRAM_BLOCKS];
} QEMU_PACKED;

typedef struct {
    hwaddr ram_base;
    hwaddr kernel_pa;
} Acorn32LoadCtx;

bool acorn32_kernel_p(const char *filename)
{
    Elf32_Ehdr hdr;
    bool is64 = false;

    if (!filename) {
        return false;
    }
    /* Not an ELF at all is the normal case: a Linux zImage. */
    if (!load_elf_hdr(filename, &hdr, &is64, NULL) || is64) {
        return false;
    }
    return le16_to_cpu(hdr.e_machine) == EM_ARM &&
           le32_to_cpu(hdr.e_entry) == ACORN32_KERNEL_VA;
}

/*
 * Virtual span the kernel needs mapped, from the program headers.
 *
 * This must come from p_memsz, not from what load_elf() actually copies:
 * the last LOAD segment carries .bss, and start() zeroes it before doing
 * anything else (0xf03f064c..0xf040fc88 in the 10.1 GENERIC kernel).  Map
 * only the file-backed part and the kernel faults on its own BSS.
 */
static bool acorn32_elf_span(const char *filename, uint32_t *span, Error **errp)
{
    g_autofree Elf32_Phdr *phdr = NULL;
    Elf32_Ehdr ehdr;
    g_autoptr(GError) gerr = NULL;
    g_autofree char *data = NULL;
    gsize len = 0;
    uint64_t end = 0;
    int i;

    if (!g_file_get_contents(filename, &data, &len, &gerr)) {
        error_setg(errp, "cannot read '%s': %s", filename, gerr->message);
        return false;
    }
    if (len < sizeof(ehdr)) {
        error_setg(errp, "'%s' is too short to be an ELF", filename);
        return false;
    }
    memcpy(&ehdr, data, sizeof(ehdr));

    if (le32_to_cpu(ehdr.e_phoff) + (uint64_t)le16_to_cpu(ehdr.e_phnum) *
        le16_to_cpu(ehdr.e_phentsize) > len) {
        error_setg(errp, "'%s' has truncated program headers", filename);
        return false;
    }

    for (i = 0; i < le16_to_cpu(ehdr.e_phnum); i++) {
        Elf32_Phdr p;

        memcpy(&p, data + le32_to_cpu(ehdr.e_phoff) +
               (size_t)i * le16_to_cpu(ehdr.e_phentsize), sizeof(p));
        if (le32_to_cpu(p.p_type) != PT_LOAD) {
            continue;
        }
        end = MAX(end, (uint64_t)le32_to_cpu(p.p_vaddr) +
                       le32_to_cpu(p.p_memsz));
    }

    if (end <= ACORN32_KERNEL_VA) {
        error_setg(errp, "'%s' has no loadable segment at 0x%x",
                   filename, ACORN32_KERNEL_VA);
        return false;
    }

    *span = ROUND_UP(end - ACORN32_KERNEL_VA, MiB);
    return true;
}

/*
 * The ELF has a single LOAD segment with vaddr == paddr == 0xf0000000.
 * That paddr is a lie - it is the virtual link address - so translate it
 * rather than honouring it.
 */
static uint64_t acorn32_translate(void *opaque, uint64_t addr)
{
    Acorn32LoadCtx *ctx = opaque;

    return addr - ACORN32_KERNEL_VA + ctx->kernel_pa;
}

static void acorn32_build_l1(hwaddr l1_pa, hwaddr ram_base, ram_addr_t ram_size,
                             hwaddr kernel_pa, uint32_t kernel_span)
{
    g_autofree uint32_t *l1 = g_new0(uint32_t, L1_ENTRIES);
    uint32_t i;

    /* RAM, identity mapped: bootconfig and the stub stay addressable. */
    for (i = 0; i < ram_size / MiB; i++) {
        hwaddr pa = ram_base + (hwaddr)i * MiB;
        l1[pa >> 20] = L1_SECTION(pa, L1_CB_RAM);
    }

    /*
     * I/O space, identity mapped and uncached.  initarm() uses the raw
     * hardware addresses (vidc_base = VIDC_HW_BASE) before it installs
     * its own tables.
     */
    for (i = 0; i < 16; i++) {
        hwaddr pa = 0x03000000 + (hwaddr)i * MiB;
        l1[pa >> 20] = L1_SECTION(pa, L1_CB_DEV);
    }

    /*
     * The kernel's virtual window over DRAM.
     *
     * This has to cover all of DRAM, not just the kernel image.  initarm()
     * derives virtual addresses for every page it allocates with
     * (rpc_machdep.c:579)
     *
     *     pv_va = KERNEL_BASE + pv_pa - kernel_start;
     *
     * so as soon as it allocates a page table past the image it accesses
     * it up here.  Mapping only the image gets you as far as
     * pmap_map_chunk() touching 0xf05.... and then a silent stop.
     */
    for (i = 0; i < ram_size / MiB; i++) {
        hwaddr va = ACORN32_KERNEL_VA + (hwaddr)i * MiB;
        l1[va >> 20] = L1_SECTION(ram_base + (hwaddr)i * MiB, L1_CB_RAM);
    }

    for (i = 0; i < L1_ENTRIES; i++) {
        l1[i] = cpu_to_le32(l1[i]);
    }
    rom_add_blob_fixed("netbsd-l1pt", l1, L1_SIZE, l1_pa);
}

/*
 * Enable the MMU and jump.  Runs with translation off, from identity
 * mapped RAM, so the same addresses stay valid across the switch.
 */
static void acorn32_write_stub(hwaddr stub_pa, hwaddr l1_pa,
                               hwaddr bootconfig_va)
{
    uint32_t code[] = {
        0xe3a02000,     /* mov  r2, #0                    */
        0xee072f17,     /* mcr  p15, 0, r2, c7, c7, 0     ; inval caches */
        0xee082f17,     /* mcr  p15, 0, r2, c8, c7, 0     ; inval TLBs   */
        0xe59f101c,     /* ldr  r1, [pc, #28]             ; L1 base      */
        0xee021f10,     /* mcr  p15, 0, r1, c2, c0, 0     ; TTBR0        */
        0xe3e02000,     /* mvn  r2, #0                                    */
        0xee032f10,     /* mcr  p15, 0, r2, c3, c0, 0     ; DACR=manager */
        0xee113f10,     /* mrc  p15, 0, r3, c1, c0, 0                     */
        0xe3833001,     /* orr  r3, r3, #1                ; M bit         */
        0xee013f10,     /* mcr  p15, 0, r3, c1, c0, 0     ; MMU on        */
        0xe59f0004,     /* ldr  r0, [pc, #4]              ; bootconfig    */
        0xe59ff004,     /* ldr  pc, [pc, #4]              ; kernel entry  */
        0,              /* .word L1 base                                  */
        0,              /* .word bootconfig                               */
        0,              /* .word 0xf0000000                               */
    };
    int i;

    code[12] = l1_pa;
    code[13] = bootconfig_va;
    code[14] = ACORN32_KERNEL_VA;

    for (i = 0; i < ARRAY_SIZE(code); i++) {
        code[i] = cpu_to_le32(code[i]);
    }
    rom_add_blob_fixed("netbsd-stub", code, sizeof(code), stub_pa);
}

static void acorn32_write_bootconfig(hwaddr pa, hwaddr ram_base,
                                     ram_addr_t ram_size, hwaddr kernel_pa,
                                     uint32_t kernel_span, hwaddr fb_pa,
                                     const char *kernel_filename,
                                     const char *cmdline)
{
    g_autofree struct acorn32_bootconfig *bc =
        g_new0(struct acorn32_bootconfig, 1);

    bc->magic   = cpu_to_le32(BOOTCONFIG_MAGIC);
    bc->version = cpu_to_le32(BOOTCONFIG_VERSION);

    pstrcpy(bc->kernelname, sizeof(bc->kernelname),
            kernel_filename ? kernel_filename : "netbsd");
    if (cmdline) {
        pstrcpy(bc->args, sizeof(bc->args), cmdline);
    }

    bc->kernvirtualbase  = cpu_to_le32(ACORN32_KERNEL_VA);
    bc->kernphysicalbase = cpu_to_le32(kernel_pa);
    bc->kernsize         = cpu_to_le32(kernel_span);

    bc->pagesize   = cpu_to_le32(4096);
    bc->drampages  = cpu_to_le32(ram_size / 4096);
    bc->dramblocks = cpu_to_le32(1);
    bc->dram[0].address = cpu_to_le32(ram_base);
    bc->dram[0].pages   = cpu_to_le32(ram_size / 4096);
    bc->dram[0].flags   = cpu_to_le32(0);   /* PHYSMEM_TYPE_GENERIC */

    /*
     * No VRAM: vramblocks == 0 makes initarm() pick VIDEOMEM_TYPE_DRAM,
     * matching a RiscPC with no VRAM fitted.
     *
     * The display fields must still be filled in even before a VIDC20
     * model exists.  NetBSD's console on acorn32 is vidcvideo, not the
     * serial port, so consinit() runs vidcvideo_cnattach() during boot and
     * writes into this memory; handing it display_phys == 0 faults in
     * data_abort_entry.  Point it at a real chunk of DRAM in our scratch
     * region.  With no VIDC20 the pixels simply go nowhere - but the
     * kernel gets past its console attach, which is what we need to
     * proceed.
     */
    bc->vrampages  = cpu_to_le32(0);
    bc->vramblocks = cpu_to_le32(0);

    bc->display_phys  = cpu_to_le32(fb_pa);
    bc->display_start = cpu_to_le32(ACORN32_KERNEL_VA + (fb_pa - ram_base));
    bc->display_size  = cpu_to_le32(FRAMEBUFFER_SIZE);
    bc->width         = cpu_to_le32(FB_WIDTH);
    bc->height        = cpu_to_le32(FB_HEIGHT);
    bc->log2_bpp      = cpu_to_le32(FB_LOG2_BPP);
    bc->framerate     = cpu_to_le32(60);

    rom_add_blob_fixed("netbsd-bootconfig", bc, sizeof(*bc), pa);
}

bool acorn32_load_netbsd(MachineState *machine, hwaddr ram_base,
                         hwaddr *entry, Error **errp)
{
    Acorn32LoadCtx ctx;
    uint64_t elf_entry, lowaddr, highaddr;
    ram_addr_t ram_size = machine->ram_size;
    hwaddr scratch = ram_base + ram_size - BOOTSCRATCH_SIZE;
    hwaddr l1_pa   = scratch + L1TABLE_OFF;
    hwaddr stub_pa = scratch + STUB_OFF;
    hwaddr bc_pa   = scratch + BOOTCONFIG_OFF;
    hwaddr fb_pa   = scratch + FRAMEBUFFER_OFF;
    uint32_t kernel_span;
    ssize_t size;

    ctx.ram_base  = ram_base;
    ctx.kernel_pa = ram_base + KERNEL_OFF;

    if (!acorn32_elf_span(machine->kernel_filename, &kernel_span, errp)) {
        return false;
    }

    size = load_elf(machine->kernel_filename, NULL, acorn32_translate, &ctx,
                    &elf_entry, &lowaddr, &highaddr, NULL,
                    ELFDATA2LSB, EM_ARM, 1, 0);
    if (size <= 0) {
        error_setg(errp, "could not load NetBSD kernel '%s': %s",
                   machine->kernel_filename, load_elf_strerror(size));
        return false;
    }

    /*
     * load_elf() translates load addresses but reports e_entry raw, so
     * this stays in the kernel's virtual world.
     */
    if (elf_entry != ACORN32_KERNEL_VA) {
        error_setg(errp, "unexpected NetBSD entry point 0x%" PRIx64
                   " (expected 0x%x)", elf_entry, ACORN32_KERNEL_VA);
        return false;
    }

    if (KERNEL_OFF + kernel_span + BOOTSCRATCH_SIZE > ram_size) {
        error_setg(errp, "NetBSD kernel (%u MiB) does not fit in %" PRIu64
                   " MiB of RAM", kernel_span / (uint32_t)MiB,
                   (uint64_t)(ram_size / MiB));
        return false;
    }

    acorn32_build_l1(l1_pa, ram_base, ram_size, ctx.kernel_pa, kernel_span);
    acorn32_write_bootconfig(bc_pa, ram_base, ram_size, ctx.kernel_pa,
                             kernel_span, fb_pa, machine->kernel_filename,
                             machine->kernel_cmdline);
    acorn32_write_stub(stub_pa, l1_pa, bc_pa);

    *entry = stub_pa;
    return true;
}
