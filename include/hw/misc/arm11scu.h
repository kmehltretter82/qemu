/*
 * ARM11MPCore Snoop Control Unit (SCU) emulation
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Copyright (c) 2013 SUSE LINUX Products GmbH
 * Written by Paul Brook and Andreas Färber
 *
 * This code is licensed under the GPL.
 */

#ifndef HW_MISC_ARM11SCU_H
#define HW_MISC_ARM11SCU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_ARM11_SCU "arm11-scu"
OBJECT_DECLARE_SIMPLE_TYPE(ARM11SCUState, ARM11_SCU)

struct ARM11SCUState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    uint32_t control;
    uint32_t cpu_status;
    uint32_t num_cpu;
    uint32_t control_reset;
    uint32_t config;
    uint8_t cpu_status_reset;
    MemoryRegion iomem;
};

#endif
