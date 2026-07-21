/*
 * Acorn VIDC20 video controller (RiscPC).
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_VIDC20_H
#define HW_DISPLAY_VIDC20_H

#include "hw/core/sysbus.h"
#include "hw/misc/acorn-iomd.h"
#include "system/memory.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_VIDC20 "vidc20"
OBJECT_DECLARE_SIMPLE_TYPE(VIDC20State, VIDC20)

struct VIDC20State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion *fbmem;            /* where the frame store lives */
    MemoryRegionSection fbsection;
    AcornIOMDState *iomd;           /* supplies the DMA address */
    QemuConsole *con;

    /* 28-bit palette entries as written, plus a cached ARGB form */
    uint32_t palette[256];
    uint32_t pal_argb[256];
    uint32_t cursor_palette[3];
    uint32_t palindex;
    uint32_t border;

    uint32_t control;               /* pixel depth lives here */
    uint32_t dctl;
    uint32_t ectl;

    uint32_t hdsr, hder;            /* horizontal display start/end */
    uint32_t vdsr, vder;            /* vertical display start/end */

    /* what the console is currently sized for */
    int last_width, last_height, last_bpp;
    hwaddr last_base;
    bool invalidate;
};

#endif
