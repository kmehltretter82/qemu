/*
 * Minimal TI CPSW register model for Linux driver probe testing.
 *
 * This intentionally implements only register storage and the reset/teardown
 * handshakes needed to exercise the legacy Linux CPSW driver's probe, open,
 * stop, and probe-error cleanup paths. It is not a functional NIC model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/net/ti_cpsw_test.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

#define CPSW_IDVER                  0x0000
#define CPSW_SOFT_RESET             0x0008
#define CPSW_CPDMA_TX_TEARDOWN      0x0808
#define CPSW_CPDMA_RX_TEARDOWN      0x0818
#define CPSW_CPDMA_SOFT_RESET       0x081c
#define CPSW_STATERAM_TX_CP         0x0a40
#define CPSW_STATERAM_RX_CP         0x0a60
#define CPSW_ALE_IDVER              0x0d00
#define CPSW_SLIVER0_MACSTATUS      0x0d88
#define CPSW_SLIVER0_SOFT_RESET     0x0d8c

#define CPSW_VERSION_2              0x0019010c
#define CPSW_ALE_VERSION_1_3        0x00000103
#define CPDMA_TEARDOWN_VALUE        0xfffffffc
#define CPDMA_NUM_CHANNELS          8
#define CPSW_SLIVER_STATUS_IDLE     (1U << 31)

#define CPSW_WR_SOFT_RESET          0x0004

static bool ti_cpsw_test_is_reset(hwaddr addr)
{
    return addr == CPSW_SOFT_RESET ||
           addr == CPSW_CPDMA_SOFT_RESET ||
           addr == CPSW_SLIVER0_SOFT_RESET;
}

static uint64_t ti_cpsw_test_main_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    TICPSWTestState *s = opaque;

    if (addr == CPSW_IDVER) {
        return CPSW_VERSION_2;
    }
    if (addr == CPSW_ALE_IDVER) {
        return CPSW_ALE_VERSION_1_3;
    }
    if (addr == CPSW_SLIVER0_MACSTATUS) {
        return CPSW_SLIVER_STATUS_IDLE;
    }
    if (ti_cpsw_test_is_reset(addr)) {
        return 0;
    }
    return s->main_regs[addr / sizeof(uint32_t)];
}

static void ti_cpsw_test_main_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned size)
{
    TICPSWTestState *s = opaque;
    hwaddr cp;

    if (ti_cpsw_test_is_reset(addr)) {
        s->main_regs[addr / sizeof(uint32_t)] = 0;
        return;
    }

    if ((addr == CPSW_CPDMA_TX_TEARDOWN ||
         addr == CPSW_CPDMA_RX_TEARDOWN) && value < CPDMA_NUM_CHANNELS) {
        cp = addr == CPSW_CPDMA_TX_TEARDOWN ? CPSW_STATERAM_TX_CP
                                            : CPSW_STATERAM_RX_CP;
        cp += value * sizeof(uint32_t);
        s->main_regs[cp / sizeof(uint32_t)] = CPDMA_TEARDOWN_VALUE;
    }

    s->main_regs[addr / sizeof(uint32_t)] = value;
}

static uint64_t ti_cpsw_test_wr_read(void *opaque, hwaddr addr,
                                     unsigned size)
{
    TICPSWTestState *s = opaque;

    if (addr == CPSW_WR_SOFT_RESET) {
        return 0;
    }
    return s->wr_regs[addr / sizeof(uint32_t)];
}

static void ti_cpsw_test_wr_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned size)
{
    TICPSWTestState *s = opaque;

    if (addr == CPSW_WR_SOFT_RESET) {
        s->wr_regs[addr / sizeof(uint32_t)] = 0;
        return;
    }
    s->wr_regs[addr / sizeof(uint32_t)] = value;
}

static const MemoryRegionOps ti_cpsw_test_main_ops = {
    .read = ti_cpsw_test_main_read,
    .write = ti_cpsw_test_main_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static const MemoryRegionOps ti_cpsw_test_wr_ops = {
    .read = ti_cpsw_test_wr_read,
    .write = ti_cpsw_test_wr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void ti_cpsw_test_reset(DeviceState *dev)
{
    TICPSWTestState *s = TI_CPSW_TEST(dev);

    memset(s->main_regs, 0, sizeof(s->main_regs));
    memset(s->wr_regs, 0, sizeof(s->wr_regs));
}

static void ti_cpsw_test_init(Object *obj)
{
    TICPSWTestState *s = TI_CPSW_TEST(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    int i;

    memory_region_init_io(&s->main_mmio, obj, &ti_cpsw_test_main_ops, s,
                          "ti-cpsw-test-main", TI_CPSW_TEST_MAIN_SIZE);
    sysbus_init_mmio(sbd, &s->main_mmio);

    memory_region_init_io(&s->wr_mmio, obj, &ti_cpsw_test_wr_ops, s,
                          "ti-cpsw-test-wrapper", TI_CPSW_TEST_WR_SIZE);
    sysbus_init_mmio(sbd, &s->wr_mmio);

    for (i = 0; i < TI_CPSW_TEST_NUM_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static const VMStateDescription vmstate_ti_cpsw_test = {
    .name = TYPE_TI_CPSW_TEST,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(main_regs, TICPSWTestState,
                             TI_CPSW_TEST_MAIN_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(wr_regs, TICPSWTestState,
                             TI_CPSW_TEST_WR_SIZE / sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    },
};

static void ti_cpsw_test_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Minimal TI CPSW probe-test model";
    dc->legacy_reset = ti_cpsw_test_reset;
    dc->vmsd = &vmstate_ti_cpsw_test;
}

static const TypeInfo ti_cpsw_test_info = {
    .name = TYPE_TI_CPSW_TEST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TICPSWTestState),
    .instance_init = ti_cpsw_test_init,
    .class_init = ti_cpsw_test_class_init,
};

static void ti_cpsw_test_register_types(void)
{
    type_register_static(&ti_cpsw_test_info);
}

type_init(ti_cpsw_test_register_types)
