/*
 * Renesas SH7780/SH7785 PCI controller (PCIC), host mode
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_PCI_HOST_SH7785_PCIC_H
#define HW_PCI_HOST_SH7785_PCIC_H

#include "hw/pci/pci_host.h"
#include "qom/object.h"

#define TYPE_SH7785_PCIC "sh7785-pcic"
OBJECT_DECLARE_SIMPLE_TYPE(SH7785PCICState, SH7785_PCIC)

/* sysbus MMIO regions, mapped by the SoC and board */
enum {
    SH7785_PCIC_MMIO_REGS,      /* H'FE04 0000, 1 KiB */
    SH7785_PCIC_MMIO_MEM0,      /* H'FD00 0000, 16 MiB */
    SH7785_PCIC_MMIO_MEM1,      /* H'1000 0000 when MMSELR selects PCIC */
    SH7785_PCIC_MMIO_MEM2,      /* H'C000 0000, 32-bit address mode */
    SH7785_PCIC_MMIO_IO,        /* H'FE20 0000, 2 MiB */
};

struct SH7785PCICState {
    PCIHostState parent_obj;

    qemu_irq irq[4];            /* INTA to INTD */
    MemoryRegion regs, mem0, mem1, mem2, io;
    MemoryRegion pci_mem, pci_io;
    AddressSpace as_mem, as_io;
    MemoryRegion dma_root, dma_win[2];
    AddressSpace dma_as;

    uint8_t config[256];        /* the PCIC's own configuration header */
    uint32_t cr, lsr[2], lar[2], ir, imr, aint, aintm, par, pint, pintm;
    uint32_t mbr[3], mbmr[3], iobr, iobmr, cscr[2], csar[2];
    uint32_t mbar0_preset;      /* PCIMBAR0 as boot firmware leaves it */
};

/* What boot firmware (U-Boot) leaves in PCIMBAR0 for an identity window */
void sh7785_pcic_preset_target(SH7785PCICState *s, uint32_t mbar0);

#endif
