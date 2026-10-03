/*
 * Minimal TI CPSW register model for Linux driver probe testing.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_TI_CPSW_TEST_H
#define HW_NET_TI_CPSW_TEST_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "system/memory.h"

#define TYPE_TI_CPSW_TEST "ti-cpsw-test"
OBJECT_DECLARE_SIMPLE_TYPE(TICPSWTestState, TI_CPSW_TEST)

#define TI_CPSW_TEST_MAIN_SIZE 0x4000
#define TI_CPSW_TEST_WR_SIZE   0x1000
#define TI_CPSW_TEST_NUM_IRQS  4

struct TICPSWTestState {
    SysBusDevice parent_obj;

    MemoryRegion main_mmio;
    MemoryRegion wr_mmio;
    qemu_irq irq[TI_CPSW_TEST_NUM_IRQS];
    uint32_t main_regs[TI_CPSW_TEST_MAIN_SIZE / sizeof(uint32_t)];
    uint32_t wr_regs[TI_CPSW_TEST_WR_SIZE / sizeof(uint32_t)];
};

#endif
