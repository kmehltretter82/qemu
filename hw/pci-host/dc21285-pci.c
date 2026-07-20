/*
 * DC21285 "Footbridge" PCI host bridge (NetWinder, EBSA-285).
 *
 * The 21285 generates PCI configuration cycles through two 16MB
 * physical windows (Linux arch/arm/mach-footbridge/dc21285.c):
 *   0x7b000000  type 0: offset = 0xc00000 | ((devfn - 8) << 8) | reg
 *               (devfn 0, the 21285 itself, is configured through its
 *               CSR block instead and never appears here)
 *   0x7a000000  type 1: offset = (bus << 16) | (devfn << 8) | reg
 * The PCI I/O window lives at 0x7c000000 (shared with the SuperIO's
 * ISA ports on the NetWinder) and PCI memory at 0x80000000 with
 * bus addresses equal to physical addresses.
 *
 * Interrupt pins are board-wired to the 21285 IN0..IN3 inputs.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_host.h"
#include "hw/pci-host/dc21285-pci.h"

static uint64_t dc21285_cfg0_read(void *opaque, hwaddr offset, unsigned size)
{
    DC21285PCIState *s = opaque;
    PCIHostState *phb = PCI_HOST_BRIDGE(s);

    if ((offset & 0xc00000) != 0xc00000) {
        return (uint64_t)-1;
    }
    return pci_data_read(phb->bus,
                         (((offset >> 8) & 0xff) + 8) << 8 | (offset & 0xff),
                         size);
}

static void dc21285_cfg0_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    DC21285PCIState *s = opaque;
    PCIHostState *phb = PCI_HOST_BRIDGE(s);

    if ((offset & 0xc00000) != 0xc00000) {
        return;
    }
    pci_data_write(phb->bus,
                   (((offset >> 8) & 0xff) + 8) << 8 | (offset & 0xff),
                   value, size);
}

static const MemoryRegionOps dc21285_cfg0_ops = {
    .read = dc21285_cfg0_read,
    .write = dc21285_cfg0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* No devices behind subordinate buses: master-abort */
static uint64_t dc21285_cfg1_read(void *opaque, hwaddr offset, unsigned size)
{
    return (uint64_t)-1;
}

static void dc21285_cfg1_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
}

static const MemoryRegionOps dc21285_cfg1_ops = {
    .read = dc21285_cfg1_read,
    .write = dc21285_cfg1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* Empty I/O cycles float high (shared with the NetWinder ISA bus). */
static uint64_t dc21285_io_bg_read(void *opaque, hwaddr addr, unsigned size)
{
    return (uint64_t)-1;
}

static void dc21285_io_bg_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
}

static const MemoryRegionOps dc21285_io_bg_ops = {
    .read = dc21285_io_bg_read,
    .write = dc21285_io_bg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void dc21285_pci_set_irq(void *opaque, int irq_num, int level)
{
    qemu_irq *irqs = opaque;

    qemu_set_irq(irqs[irq_num], level);
}

/*
 * Interrupt routing is a board property; the NetWinder wires its
 * onboard devices by slot (arch/arm/mach-footbridge/netwinder-pci.c):
 * slot 13 (89C940F ether10) -> IN0, slot 10 (21143) -> IN1.
 */
static int dc21285_pci_map_irq(PCIDevice *d, int irq_num)
{
    switch (PCI_SLOT(d->devfn)) {
    case 13:
        return 0;   /* IN0 */
    case 10:
        return 1;   /* IN1 */
    default:
        return 2;   /* IN2 */
    }
}

/*
 * Bus-master DMA goes through the 21285's inbound windows, i.e. the
 * PCI memory space (where the board maps RAM at BUS_OFFSET), not the
 * CPU's view of the world.
 */
static AddressSpace *dc21285_pci_dma_as(PCIBus *bus, void *opaque, int devfn)
{
    DC21285PCIState *s = opaque;

    return &s->dma_as;
}

static const PCIIOMMUOps dc21285_pci_iommu_ops = {
    .get_address_space = dc21285_pci_dma_as,
};

static void dc21285_pci_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    DC21285PCIState *s = DC21285_PCI(dev);
    PCIHostState *phb = PCI_HOST_BRIDGE(s);
    int i;

    for (i = 0; i < 4; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }

    memory_region_init(&s->pci_mem, OBJECT(s), "pci-mem", UINT32_MAX);
    memory_region_init(&s->pci_io, OBJECT(s), "pci-io", 0x10000);
    memory_region_init_io(&s->io_bg, OBJECT(s), &dc21285_io_bg_ops, s,
                          "pci-io-bg", 0x10000);
    memory_region_add_subregion_overlap(&s->pci_io, 0, &s->io_bg, -1);

    phb->bus = pci_register_root_bus(dev, "pci",
                                     dc21285_pci_set_irq,
                                     dc21285_pci_map_irq,
                                     s->irq, &s->pci_mem, &s->pci_io,
                                     PCI_DEVFN(1, 0), 4, TYPE_PCI_BUS);
    address_space_init(&s->dma_as, &s->pci_mem, "dc21285-pci-dma");
    pci_setup_iommu(phb->bus, &dc21285_pci_iommu_ops, s);

    memory_region_init_io(&s->cfg0, OBJECT(s), &dc21285_cfg0_ops, s,
                          "dc21285-pci-cfg0", 0x1000000);
    memory_region_init_io(&s->cfg1, OBJECT(s), &dc21285_cfg1_ops, s,
                          "dc21285-pci-cfg1", 0x1000000);
    /*
     * CPU-to-PCI memory window: physical 0x80000000..0xffffffff
     * generates PCI bus addresses 0x00000000..0x7fffffff (the address
     * extension register is not modelled; Linux leaves it at zero).
     */
    memory_region_init_alias(&s->mem_alias, OBJECT(s), "pci-mem-window",
                             &s->pci_mem, 0, 0x80000000);

    sysbus_init_mmio(sbd, &s->cfg0);
    sysbus_init_mmio(sbd, &s->cfg1);
    sysbus_init_mmio(sbd, &s->mem_alias);
}

static void dc21285_pci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "DC21285 PCI host bridge";
    dc->realize = dc21285_pci_realize;
    dc->fw_name = "pci";
}

static const TypeInfo dc21285_pci_info = {
    .name          = TYPE_DC21285_PCI,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(DC21285PCIState),
    .class_init    = dc21285_pci_class_init,
};

static void dc21285_pci_register_types(void)
{
    type_register_static(&dc21285_pci_info);
}

type_init(dc21285_pci_register_types)
