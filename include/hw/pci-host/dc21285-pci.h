/*
 * DC21285 "Footbridge" PCI host bridge.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_PCI_HOST_DC21285_PCI_H
#define HW_PCI_HOST_DC21285_PCI_H

#include "hw/pci/pci_host.h"
#include "qom/object.h"

#define TYPE_DC21285_PCI "dc21285-pci"
OBJECT_DECLARE_SIMPLE_TYPE(DC21285PCIState, DC21285_PCI)

struct DC21285PCIState {
    PCIHostState parent_obj;

    MemoryRegion pci_mem;
    MemoryRegion pci_io;
    MemoryRegion io_bg;
    MemoryRegion cfg0;
    MemoryRegion cfg1;
    MemoryRegion mem_alias;

    qemu_irq irq[4];
    AddressSpace dma_as;
};

#endif
