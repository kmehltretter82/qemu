/*
 * Renesas SH7780/SH7785 PCI controller (PCIC), host mode
 *
 * SH7785 hardware manual (REJ09B0261-0100), section 13. The PCIC's own
 * configuration header and local registers are at H'FE04 0000. The CPU
 * reaches PCI memory through three windows (13.4.3: H'FD00 0000 16 MiB,
 * H'1000 0000 64 MiB when MMSELR gives area 4 to the PCIC, H'C000 0000
 * 512 MiB in 32-bit address mode) and PCI I/O through H'FE20 0000; the
 * upper address bits come from PCIMBRn/PCIIOBR, the middle bits from the
 * bus address or the bank register as PCIMBMRn/PCIIOBMR select, and bits
 * 17:0 pass through. PCI masters reach local memory through two target
 * windows (13.4.4): PCIMBARn at the size PCILSRn gives, translated to
 * PCILARn when PCILSRn.MBARE is set.
 *
 * The host is not visible in configuration space (a real SH7785LCR lists
 * only its RTL8169 and SiI3512 on bus 0). INTA to INTD go to the PCIINTA
 * to PCIINTD sources of the INTC; devices are swizzled by slot, which is
 * what Linux's R7780RP/SH7785LCR fixups assume (IRQ = PCIINTA + slot).
 *
 * Not modelled: error reporting (PCIIR, PCIAINT, SERR), cache snoop
 * (PCICSCR/PCICSAR, stored only), prefetch, byte swapping (PCICR.BSWP),
 * normal (non-host) mode, power management.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/sh7785_pcic.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define PCIC_VENDOR_RENESAS     0x1912
#define PCIC_DEVICE_SH7785      0x0007

/* Local registers (13.3.3), offsets from H'FE04 0000 */
#define PCICR           0x100
#define PCILSR0         0x104
#define PCILSR1         0x108
#define PCILAR0         0x10c
#define PCILAR1         0x110
#define PCIIR           0x114
#define PCIIMR          0x118
#define PCIAIR          0x11c
#define PCICIR          0x120
#define PCIAINT         0x130
#define PCIAINTM        0x134
#define PCIBMIR         0x138
#define PCIPAR          0x1c0
#define PCIPINT         0x1cc
#define PCIPINTM        0x1d0
#define PCIMBR0         0x1e0
#define PCIMBMR0        0x1e4
#define PCIMBR1         0x1e8
#define PCIMBMR1        0x1ec
#define PCIMBR2         0x1f0
#define PCIMBMR2        0x1f4
#define PCIIOBR         0x1f8
#define PCIIOBMR        0x1fc
#define PCICSCR0        0x210
#define PCICSCR1        0x214
#define PCICSAR0        0x218
#define PCICSAR1        0x21c
#define PCIPDR          0x220

#define PCICR_PREFIX    0xa5000000
#define PCILSR_MBARE    1

/* PCI memory space n: bits above hi come from PCIMBRn, mid from either */
static const struct {
    uint32_t base, size, mid;
} pcic_mem[3] = {
    { 0xfd000000, 16 * MiB,  0x00fc0000 },
    { 0x10000000, 64 * MiB,  0x03fc0000 },
    { 0xc0000000, 512 * MiB, 0x1ffc0000 },
};

static uint32_t pcic_master_addr(uint32_t local, uint32_t bank,
                                 uint32_t mask, uint32_t mid)
{
    uint32_t hi = ~(mid | 0x3ffff);

    return (bank & hi) | (((local & mask) | (bank & ~mask)) & mid) |
           (local & 0x3ffff);
}

/* Configuration header: writable bits (SH side) */
static void pcic_config_reset(SH7785PCICState *s)
{
    uint8_t *c = s->config;

    memset(c, 0, sizeof(s->config));
    pci_set_word(c + PCI_VENDOR_ID, PCIC_VENDOR_RENESAS);
    pci_set_word(c + PCI_DEVICE_ID, PCIC_DEVICE_SH7785);
    pci_set_word(c + PCI_STATUS, PCI_STATUS_CAP_LIST | PCI_STATUS_FAST_BACK |
                 PCI_STATUS_DEVSEL_MEDIUM);
    pci_set_byte(c + PCI_REVISION_ID, 1);
    pci_set_word(c + PCI_CLASS_DEVICE, PCI_CLASS_BRIDGE_HOST);
    pci_set_long(c + PCI_BASE_ADDRESS_0, PCI_BASE_ADDRESS_SPACE_IO);
    pci_set_byte(c + PCI_CAPABILITY_LIST, 0x40);
    pci_set_byte(c + PCI_INTERRUPT_PIN, 1);
    pci_set_byte(c + 0x40, PCI_CAP_ID_PM);
    pci_set_word(c + 0x42, 0x0002);
}

static uint32_t pcic_config_wmask(unsigned off)
{
    switch (off & ~3) {
    case PCI_COMMAND:
        return 0x0000ffff;              /* status is write-1-to-clear */
    case PCI_CLASS_REVISION:
        return 0xffffff00;
    case PCI_CACHE_LINE_SIZE:
        return 0x0000ff00;              /* latency timer */
    case PCI_BASE_ADDRESS_0:
        return 0xffffff00;
    case PCI_BASE_ADDRESS_1:
    case PCI_BASE_ADDRESS_2:
        return 0xfff00000;
    case PCI_SUBSYSTEM_VENDOR_ID:
        return 0xffffffff;
    case PCI_INTERRUPT_LINE:
        return 0x0000ffff;
    case 0x44:
        return 0xffff;
    default:
        return 0;
    }
}

static void pcic_update_dma(SH7785PCICState *s)
{
    memory_region_transaction_begin();
    for (int i = 0; i < 2; i++) {
        uint32_t lsr = s->lsr[i];
        uint32_t size = ((lsr & 0x1ff00000) | 0xfffff) + 1;
        uint32_t mbar = pci_get_long(s->config + PCI_BASE_ADDRESS_1 + 4 * i);
        uint32_t base = mbar & ~(size - 1);
        uint32_t local = lsr & PCILSR_MBARE ? s->lar[i] & ~(size - 1) : base;

        if (memory_region_is_mapped(&s->dma_win[i])) {
            memory_region_del_subregion(&s->dma_root, &s->dma_win[i]);
        }
        if (size & (size - 1)) {
            qemu_log_mask(LOG_GUEST_ERROR, "sh7785-pcic: PCILSR%d 0x%08x is"
                          " a prohibited setting\n", i, lsr);
            continue;
        }
        memory_region_set_size(&s->dma_win[i], size);
        memory_region_set_alias_offset(&s->dma_win[i], local);
        memory_region_add_subregion_overlap(&s->dma_root, base,
                                            &s->dma_win[i], i ? 0 : 1);
    }
    memory_region_transaction_commit();
}

static uint32_t pcic_reg_read32(SH7785PCICState *s, unsigned off)
{
    PCIHostState *phb = PCI_HOST_BRIDGE(s);

    if (off < 0x100) {
        return pci_get_long(s->config + off);
    }
    switch (off) {
    case PCICR:     return s->cr;
    case PCILSR0:   return s->lsr[0];
    case PCILSR1:   return s->lsr[1];
    case PCILAR0:   return s->lar[0];
    case PCILAR1:   return s->lar[1];
    case PCIIR:     return s->ir;
    case PCIIMR:    return s->imr;
    case PCIAIR:
    case PCICIR:
    case PCIBMIR:   return 0;
    case PCIAINT:   return s->aint;
    case PCIAINTM:  return s->aintm;
    case PCIPAR:    return s->par;
    case PCIPINT:   return s->pint;
    case PCIPINTM:  return s->pintm;
    case PCIMBR0:   return s->mbr[0];
    case PCIMBMR0:  return s->mbmr[0];
    case PCIMBR1:   return s->mbr[1];
    case PCIMBMR1:  return s->mbmr[1];
    case PCIMBR2:   return s->mbr[2];
    case PCIMBMR2:  return s->mbmr[2];
    case PCIIOBR:   return s->iobr;
    case PCIIOBMR:  return s->iobmr;
    case PCICSCR0:  return s->cscr[0];
    case PCICSCR1:  return s->cscr[1];
    case PCICSAR0:  return s->csar[0];
    case PCICSAR1:  return s->csar[1];
    case PCIPDR:
        return s->par & 0x80000000 ? pci_data_read(phb->bus, s->par, 4)
                                   : 0xffffffff;
    }
    qemu_log_mask(LOG_UNIMP, "sh7785-pcic: read of 0x%03x\n", off);
    return 0;
}

static void pcic_reg_write32(SH7785PCICState *s, unsigned off, uint32_t val,
                             uint32_t lanes)
{
    uint32_t old;

    if (off < 0x100) {
        uint32_t wmask = pcic_config_wmask(off) & lanes;

        old = pci_get_long(s->config + off);
        if (off == PCI_COMMAND) {
            /* status error bits are write-1-to-clear, 66MHz writable */
            uint32_t w1c = (val & lanes & 0xf9000000);

            old &= ~w1c;
            wmask |= lanes & PCI_STATUS_66MHZ << 16;
        }
        pci_set_long(s->config + off, (old & ~wmask) | (val & wmask));
        if (off == PCI_BASE_ADDRESS_1 || off == PCI_BASE_ADDRESS_2) {
            pcic_update_dma(s);
        }
        return;
    }
    old = pcic_reg_read32(s, off);
    val = (old & ~lanes) | (val & lanes);
    switch (off) {
    case PCICR:
        if ((val & 0xff000000) == PCICR_PREFIX) {
            s->cr = val & 0x00000fff;
        }
        break;
    case PCILSR0:
    case PCILSR1:
        s->lsr[off == PCILSR1] = val & 0x1ff00001;
        pcic_update_dma(s);
        break;
    case PCILAR0:
    case PCILAR1:
        s->lar[off == PCILAR1] = val & 0xfff00000;
        pcic_update_dma(s);
        break;
    case PCIIR:     s->ir &= ~(val & lanes); break;
    case PCIIMR:    s->imr = val; break;
    case PCIAINT:   s->aint &= ~(val & lanes); break;
    case PCIAINTM:  s->aintm = val; break;
    case PCIPAR:    s->par = val; break;
    case PCIPINT:   s->pint &= ~(val & lanes); break;
    case PCIPINTM:  s->pintm = val; break;
    case PCIMBR0:   s->mbr[0] = val & 0xfffc0000; break;
    case PCIMBMR0:  s->mbmr[0] = val & 0x00fc0000; break;
    case PCIMBR1:   s->mbr[1] = val & 0xfffc0000; break;
    case PCIMBMR1:  s->mbmr[1] = val & 0x03fc0000; break;
    case PCIMBR2:   s->mbr[2] = val & 0xfffc0000; break;
    case PCIMBMR2:  s->mbmr[2] = val & 0x1ffc0000; break;
    case PCIIOBR:   s->iobr = val & 0xfffc0000; break;
    case PCIIOBMR:  s->iobmr = val & 0x001c0000; break;
    case PCICSCR0:  s->cscr[0] = val; break;
    case PCICSCR1:  s->cscr[1] = val; break;
    case PCICSAR0:  s->csar[0] = val; break;
    case PCICSAR1:  s->csar[1] = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "sh7785-pcic: write 0x%08x to 0x%03x\n",
                      val, off);
    }
}

static uint64_t pcic_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    SH7785PCICState *s = opaque;
    unsigned shift = (addr & 3) * 8;

    if ((addr & ~3) == PCIPDR) {
        PCIHostState *phb = PCI_HOST_BRIDGE(s);

        if (!(s->par & 0x80000000)) {
            return MAKE_64BIT_MASK(0, size * 8);
        }
        return pci_data_read(phb->bus, s->par | (addr & 3), size);
    }
    return extract32(pcic_reg_read32(s, addr & ~3), shift, size * 8);
}

static void pcic_reg_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    SH7785PCICState *s = opaque;
    unsigned shift = (addr & 3) * 8;

    if ((addr & ~3) == PCIPDR) {
        PCIHostState *phb = PCI_HOST_BRIDGE(s);

        if (s->par & 0x80000000) {
            pci_data_write(phb->bus, s->par | (addr & 3), val, size);
        }
        return;
    }
    pcic_reg_write32(s, addr & ~3, val << shift,
                     MAKE_64BIT_MASK(shift, size * 8));
}

static const MemoryRegionOps pcic_reg_ops = {
    .read = pcic_reg_read,
    .write = pcic_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* CPU accesses to PCI memory and I/O space; a master abort reads ~0 */
static MemTxResult pcic_master_access(AddressSpace *as, uint32_t pci,
                                      uint64_t *val, unsigned size,
                                      bool write, MemTxAttrs attrs)
{
    uint8_t buf[8];

    if (write) {
        stn_le_p(buf, size, *val);
        address_space_write(as, pci, attrs, buf, size);
    } else if (address_space_read(as, pci, attrs, buf, size) == MEMTX_OK) {
        *val = ldn_le_p(buf, size);
    } else {
        *val = MAKE_64BIT_MASK(0, size * 8);
    }
    return MEMTX_OK;
}

#define PCIC_MEM_OPS(n)                                                     \
static MemTxResult pcic_mem##n##_read(void *opaque, hwaddr addr,            \
                                      uint64_t *val, unsigned size,         \
                                      MemTxAttrs attrs)                     \
{                                                                           \
    SH7785PCICState *s = opaque;                                            \
    uint32_t pci = pcic_master_addr(pcic_mem[n].base + addr, s->mbr[n],     \
                                    s->mbmr[n], pcic_mem[n].mid);           \
                                                                            \
    return pcic_master_access(&s->as_mem, pci, val, size, false,         \
                              attrs);                                       \
}                                                                           \
static MemTxResult pcic_mem##n##_write(void *opaque, hwaddr addr,           \
                                       uint64_t val, unsigned size,         \
                                       MemTxAttrs attrs)                    \
{                                                                           \
    SH7785PCICState *s = opaque;                                            \
    uint32_t pci = pcic_master_addr(pcic_mem[n].base + addr, s->mbr[n],     \
                                    s->mbmr[n], pcic_mem[n].mid);           \
                                                                            \
    return pcic_master_access(&s->as_mem, pci, &val, size, true,         \
                              attrs);                                       \
}                                                                           \
static const MemoryRegionOps pcic_mem##n##_ops = {                          \
    .read_with_attrs = pcic_mem##n##_read,                                  \
    .write_with_attrs = pcic_mem##n##_write,                                \
    .endianness = DEVICE_LITTLE_ENDIAN,                                     \
    .valid.min_access_size = 1,                                             \
    .valid.max_access_size = 4,                                             \
};

PCIC_MEM_OPS(0)
PCIC_MEM_OPS(1)
PCIC_MEM_OPS(2)

static MemTxResult pcic_io_read(void *opaque, hwaddr addr, uint64_t *val,
                                unsigned size, MemTxAttrs attrs)
{
    SH7785PCICState *s = opaque;
    uint32_t pci = pcic_master_addr(0xfe200000 + addr, s->iobr, s->iobmr,
                                    0x001c0000);

    return pcic_master_access(&s->as_io, pci, val, size, false, attrs);
}

static MemTxResult pcic_io_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size, MemTxAttrs attrs)
{
    SH7785PCICState *s = opaque;
    uint32_t pci = pcic_master_addr(0xfe200000 + addr, s->iobr, s->iobmr,
                                    0x001c0000);

    return pcic_master_access(&s->as_io, pci, &val, size, true, attrs);
}

static const MemoryRegionOps pcic_io_ops = {
    .read_with_attrs = pcic_io_read,
    .write_with_attrs = pcic_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static int pcic_map_irq(PCIDevice *d, int pin)
{
    return (PCI_SLOT(d->devfn) + pin) & 3;
}

static void pcic_set_irq(void *opaque, int n, int level)
{
    SH7785PCICState *s = opaque;

    qemu_set_irq(s->irq[n], level);
}

static AddressSpace *pcic_dma_as(PCIBus *bus, void *opaque, int devfn)
{
    SH7785PCICState *s = opaque;

    return &s->dma_as;
}

static const PCIIOMMUOps pcic_iommu_ops = {
    .get_address_space = pcic_dma_as,
};

void sh7785_pcic_preset_target(SH7785PCICState *s, uint32_t mbar0)
{
    /* kept across resets, as the firmware would set it again */
    s->mbar0_preset = mbar0 & 0xfff00000;
    pci_set_long(s->config + PCI_BASE_ADDRESS_1, s->mbar0_preset);
    pcic_update_dma(s);
}

static void pcic_reset(DeviceState *dev)
{
    SH7785PCICState *s = SH7785_PCIC(dev);

    pcic_config_reset(s);
    s->cr = s->ir = s->imr = s->aint = s->aintm = s->pint = s->pintm = 0;
    s->par = 0x80000000;
    s->iobr = s->iobmr = 0;
    memset(s->lsr, 0, sizeof(s->lsr));
    memset(s->lar, 0, sizeof(s->lar));
    memset(s->mbr, 0, sizeof(s->mbr));
    memset(s->mbmr, 0, sizeof(s->mbmr));
    memset(s->cscr, 0, sizeof(s->cscr));
    memset(s->csar, 0, sizeof(s->csar));
    pci_set_long(s->config + PCI_BASE_ADDRESS_1, s->mbar0_preset);
    pcic_update_dma(s);
}

static void pcic_realize(DeviceState *dev, Error **errp)
{
    SH7785PCICState *s = SH7785_PCIC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    Object *o = OBJECT(dev);

    memory_region_init(&s->pci_mem, o, "sh7785-pcic.pci-mem", 4 * GiB);
    memory_region_init(&s->pci_io, o, "sh7785-pcic.pci-io", 4 * GiB);
    address_space_init(&s->as_mem, &s->pci_mem, "sh7785-pcic-mem");
    address_space_init(&s->as_io, &s->pci_io, "sh7785-pcic-io");

    memory_region_init_io(&s->regs, o, &pcic_reg_ops, s, "sh7785-pcic", 0x400);
    memory_region_init_io(&s->mem0, o, &pcic_mem0_ops, s, "sh7785-pcic.mem0",
                          pcic_mem[0].size);
    memory_region_init_io(&s->mem1, o, &pcic_mem1_ops, s, "sh7785-pcic.mem1",
                          pcic_mem[1].size);
    memory_region_init_io(&s->mem2, o, &pcic_mem2_ops, s, "sh7785-pcic.mem2",
                          pcic_mem[2].size);
    memory_region_init_io(&s->io, o, &pcic_io_ops, s, "sh7785-pcic.io",
                          2 * MiB);
    sysbus_init_mmio(sbd, &s->regs);
    sysbus_init_mmio(sbd, &s->mem0);
    sysbus_init_mmio(sbd, &s->mem1);
    sysbus_init_mmio(sbd, &s->mem2);
    sysbus_init_mmio(sbd, &s->io);
    for (int i = 0; i < 4; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }

    /* Target windows: PCI bus master accesses into local memory */
    memory_region_init(&s->dma_root, o, "sh7785-pcic.dma", 4 * GiB);
    for (int i = 0; i < 2; i++) {
        memory_region_init_alias(&s->dma_win[i], o,
                                 i ? "sh7785-pcic.target1"
                                   : "sh7785-pcic.target0",
                                 get_system_memory(), 0, 1 * MiB);
    }
    address_space_init(&s->dma_as, &s->dma_root, "sh7785-pcic-dma");

    phb->bus = pci_register_root_bus(dev, "pci", pcic_set_irq, pcic_map_irq,
                                     s, &s->pci_mem, &s->pci_io,
                                     PCI_DEVFN(0, 0), 4, TYPE_PCI_BUS);
    pci_setup_iommu(phb->bus, &pcic_iommu_ops, s);
}

static void pcic_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pcic_realize;
    device_class_set_legacy_reset(dc, pcic_reset);
    dc->user_creatable = false;
}

static const TypeInfo pcic_types[] = {
    {
        .name           = TYPE_SH7785_PCIC,
        .parent         = TYPE_PCI_HOST_BRIDGE,
        .instance_size  = sizeof(SH7785PCICState),
        .class_init     = pcic_class_init,
    },
};

DEFINE_TYPES(pcic_types)
