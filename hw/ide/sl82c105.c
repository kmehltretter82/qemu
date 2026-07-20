/*
 * Winbond W83C553 southbridge stub + SL82C105 (W83C554F) PCI IDE,
 * the NetWinder's disk controller pair in PCI slot 12.
 *
 * The IDE function operates in legacy compatibility mode (ports
 * 0x1f0/0x170, ISA IRQs 14/15), like the PIIX model this is derived
 * from. Linux's pata_sl82c105 requires the W83C553 ISA-bridge shell
 * as function 0 of the same slot and reads its revision to decide
 * whether DMA is trustworthy; we report revision 6 or later so the
 * driver uses the BMDMA engine (BAR 4).
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/ide/pci.h"
#include "hw/isa/isa.h"
#include "hw/core/qdev-properties.h"
#include "ide-internal.h"

#define TYPE_SL82C105_IDE "sl82c105-ide"
#define TYPE_W83C553_ISA  "w83c553-isa"

OBJECT_DECLARE_SIMPLE_TYPE(SL82C105State, SL82C105_IDE)

struct SL82C105State {
    PCIIDEState parent_obj;
    bool legacy;
};

#define PCI_VENDOR_ID_WINBOND_2      0x10ad
#define PCI_DEVICE_ID_WINBOND_82C105 0x0105
#define PCI_DEVICE_ID_WINBOND_83C553 0x0565

/* BMDMA register block, PIIX-compatible layout */

static uint64_t sl82c105_bmdma_read(void *opaque, hwaddr addr, unsigned size)
{
    BMDMAState *bm = opaque;

    if (size != 1) {
        return ((uint64_t)1 << (size * 8)) - 1;
    }

    switch (addr & 3) {
    case 0:
        return bm->cmd;
    case 2:
        return bm->status;
    default:
        return 0xff;
    }
}

static void sl82c105_bmdma_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    BMDMAState *bm = opaque;

    if (size != 1) {
        return;
    }

    switch (addr & 3) {
    case 0:
        bmdma_cmd_writeb(bm, val);
        break;
    case 2:
        bmdma_status_writeb(bm, val);
        break;
    }
}

static const MemoryRegionOps sl82c105_bmdma_ops = {
    .read = sl82c105_bmdma_read,
    .write = sl82c105_bmdma_write,
};

static void sl82c105_bmdma_setup_bar(PCIIDEState *d)
{
    int i;

    memory_region_init(&d->bmdma_bar, OBJECT(d), "sl82c105-bmdma-container",
                       16);
    for (i = 0; i < 2; i++) {
        BMDMAState *bm = &d->bmdma[i];

        memory_region_init_io(&bm->extra_io, OBJECT(d), &sl82c105_bmdma_ops,
                              bm, "sl82c105-bmdma", 4);
        memory_region_add_subregion(&d->bmdma_bar, i * 8, &bm->extra_io);
        memory_region_init_io(&bm->addr_ioport, OBJECT(d),
                              &bmdma_addr_ioport_ops, bm, "bmdma", 4);
        memory_region_add_subregion(&d->bmdma_bar, i * 8 + 4,
                                    &bm->addr_ioport);
    }
}

/* SL82C105 IDE function */

static void sl82c105_ide_reset(DeviceState *dev)
{
    PCIIDEState *d = PCI_IDE(dev);
    PCIDevice *pd = PCI_DEVICE(d);
    uint8_t *pci_conf = pd->config;
    int i;

    for (i = 0; i < 2; i++) {
        ide_bus_reset(&d->bus[i]);
    }

    pci_set_word(pci_conf + PCI_COMMAND, 0x0000);
    pci_set_word(pci_conf + PCI_STATUS,
                 PCI_STATUS_DEVSEL_MEDIUM | PCI_STATUS_FAST_BACK);
    pci_set_long(pci_conf + 0x20, 0x1);  /* BMIBA */
}

static void sl82c105_ide_realize(PCIDevice *dev, Error **errp)
{
    PCIIDEState *d = PCI_IDE(dev);
    SL82C105State *s = SL82C105_IDE(dev);
    uint8_t *pci_conf = dev->config;
    /* On the NetWinder the W83C553 routes the IDE interrupts to
     * the ISA controller (IRQ_ISA_HARDDISK1/2), not to PCI INTx. */
    static const int isairq[2] = {14, 15};
    unsigned i;

    if (s->legacy) {
        /*
         * Legacy/compatibility mode, as the NetWinder straps the part:
         * fixed ports 0x1f0/0x170, no address BARs. Linux's PCI core
         * then installs the fixed "legacy IDE quirk" resources.
         */
        pci_conf[PCI_CLASS_PROG] = 0x80;

        sl82c105_bmdma_setup_bar(d);
        pci_register_bar(dev, 4, PCI_BASE_ADDRESS_SPACE_IO, &d->bmdma_bar);
        pci_conf[PCI_INTERRUPT_PIN] = 0x01;

        for (i = 0; i < 2; i++) {
            static const struct { int cmd, ctl; } ports[2] = {
                {0x1f0, 0x3f6}, {0x170, 0x376}
            };
            int ret;

            ide_bus_init(&d->bus[i], sizeof(d->bus[i]), DEVICE(d), i, 2);
            ret = ide_init_ioport(&d->bus[i], NULL, ports[i].cmd,
                                  ports[i].ctl);
            if (ret) {
                error_setg_errno(errp, -ret, "Failed to realize %s port %u",
                                 object_get_typename(OBJECT(d)), i);
                return;
            }
            ide_bus_init_output_irq(&d->bus[i], isa_get_irq(NULL, isairq[i]));
            bmdma_init(&d->bus[i], &d->bmdma[i], d);
            ide_bus_register_restart_cb(&d->bus[i]);
        }
        return;
    }

    /* native mode, both channels */
    pci_conf[PCI_CLASS_PROG] = 0x8f;

    memory_region_init_io(&d->data_bar[0], OBJECT(d), &pci_ide_data_le_ops,
                          &d->bus[0], "sl82c105-data0", 8);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &d->data_bar[0]);
    memory_region_init_io(&d->cmd_bar[0], OBJECT(d), &pci_ide_cmd_le_ops,
                          &d->bus[0], "sl82c105-cmd0", 4);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->cmd_bar[0]);
    memory_region_init_io(&d->data_bar[1], OBJECT(d), &pci_ide_data_le_ops,
                          &d->bus[1], "sl82c105-data1", 8);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_IO, &d->data_bar[1]);
    memory_region_init_io(&d->cmd_bar[1], OBJECT(d), &pci_ide_cmd_le_ops,
                          &d->bus[1], "sl82c105-cmd1", 4);
    pci_register_bar(dev, 3, PCI_BASE_ADDRESS_SPACE_IO, &d->cmd_bar[1]);

    sl82c105_bmdma_setup_bar(d);
    pci_register_bar(dev, 4, PCI_BASE_ADDRESS_SPACE_IO, &d->bmdma_bar);

    /*
     * Advertise INTA so the kernel's slot-based map_irq runs; the
     * board routes the interrupts to ISA 14/15 (IRQ_ISA_HARDDISK1/2).
     */
    pci_conf[PCI_INTERRUPT_PIN] = 0x01;

    for (i = 0; i < 2; i++) {
        ide_bus_init(&d->bus[i], sizeof(d->bus[i]), DEVICE(d), i, 2);
        ide_bus_init_output_irq(&d->bus[i], isa_get_irq(NULL, isairq[i]));

        bmdma_init(&d->bus[i], &d->bmdma[i], d);
        ide_bus_register_restart_cb(&d->bus[i]);
    }
}

static void sl82c105_ide_exitfn(PCIDevice *dev)
{
    PCIIDEState *d = PCI_IDE(dev);
    unsigned i;

    for (i = 0; i < 2; ++i) {
        memory_region_del_subregion(&d->bmdma_bar, &d->bmdma[i].extra_io);
        memory_region_del_subregion(&d->bmdma_bar, &d->bmdma[i].addr_ioport);
    }
}

static const Property sl82c105_properties[] = {
    DEFINE_PROP_BOOL("legacy", SL82C105State, legacy, false),
};

static void sl82c105_ide_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, sl82c105_ide_reset);
    dc->vmsd = &vmstate_ide_pci;
    k->realize = sl82c105_ide_realize;
    k->exit = sl82c105_ide_exitfn;
    k->vendor_id = PCI_VENDOR_ID_WINBOND_2;
    k->device_id = PCI_DEVICE_ID_WINBOND_82C105;
    k->class_id = PCI_CLASS_STORAGE_IDE;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->hotpluggable = false;
    device_class_set_props(dc, sl82c105_properties);
}

static const TypeInfo sl82c105_ide_info = {
    .name          = TYPE_SL82C105_IDE,
    .parent        = TYPE_PCI_IDE,
    .instance_size = sizeof(SL82C105State),
    .class_init    = sl82c105_ide_class_init,
};

/* W83C553 ISA bridge function: configuration shell only */

static void w83c553_realize(PCIDevice *dev, Error **errp)
{
}

static void w83c553_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = w83c553_realize;
    k->vendor_id = PCI_VENDOR_ID_WINBOND_2;
    k->device_id = PCI_DEVICE_ID_WINBOND_83C553;
    k->revision = 6;
    k->class_id = PCI_CLASS_BRIDGE_ISA;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->desc = "W83C553 PCI-ISA bridge (configuration shell)";
    dc->hotpluggable = false;
}

static const TypeInfo w83c553_info = {
    .name          = TYPE_W83C553_ISA,
    .parent        = TYPE_PCI_DEVICE,
    .class_init    = w83c553_class_init,
    .interfaces    = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void sl82c105_register_types(void)
{
    type_register_static(&sl82c105_ide_info);
    type_register_static(&w83c553_info);
}

type_init(sl82c105_register_types)
