/*
 * Atmel/Microchip AT91 EBI NAND flash interface (CS3 window glue).
 *
 * The AT91 SoCs have no dedicated NAND controller in the classic sense:
 * the raw chip sits on the External Bus Interface, and the CPU talks to
 * it with plain byte accesses in the chip-select window.  Two address
 * lines are wired to the chip's latch pins (on the SAM9G45 EK boards
 * A21 = ALE, A22 = CLE, matching the DT properties
 * atmel,nand-addr-offset = <21> / atmel,nand-cmd-offset = <22>), so
 *   base + 0x000000  = data cycles
 *   base + 0x200000  = address cycles
 *   base + 0x400000  = command cycles
 * This device decodes those accesses onto a TYPE_NAND chip model.
 *
 * The chip's GND ("SmartMedia grounded") pin is held asserted: with it
 * released the chip model stops sequential output at the end of the
 * 2048-byte page, and column addresses inside the spare area return
 * nothing - but Linux reads the 64-byte OOB exactly that way (bad-block
 * scans, on-flash BBT, soft-ECC bytes).  Asserting GND extends the
 * readable window to page + OOB, which is how the real chip behaves.
 *
 * All glue state lives in the chip model (pins, command/address latches),
 * so this device itself has nothing to migrate.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/block/nand.h"
#include "hw/block/at91_nand.h"
#include "qom/object.h"
#include "trace.h"

#define AT91_NAND_ALE   (1 << 21)
#define AT91_NAND_CLE   (1 << 22)

/* One EBI chip-select window (CS3 on the SAM9G45). */
#define AT91_NAND_WINDOW_SIZE  0x10000000

struct AT91NandState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    DeviceState *chip;
};

static uint64_t at91_nand_read(void *opaque, hwaddr addr, unsigned size)
{
    AT91NandState *s = opaque;

    /* Data cycles: CLE/ALE low, CE asserted, WP high (unprotected). */
    nand_setpins(s->chip, 0, 0, 0, 1, 1);
    return nand_getio(s->chip);
}

static void at91_nand_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned size)
{
    AT91NandState *s = opaque;
    bool cle = !!(addr & AT91_NAND_CLE);
    bool ale = !!(addr & AT91_NAND_ALE);

    if (cle || ale) {
        trace_at91_nand_ctrl(cle ? "cmd" : "addr", (uint8_t)value);
    }
    nand_setpins(s->chip, cle, ale, 0, 1, 1);
    nand_setio(s->chip, value);
}

static const MemoryRegionOps at91_nand_ops = {
    .read = at91_nand_read,
    .write = at91_nand_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    /* The bus is 8 bits wide; wider CPU accesses (readsl-style data
     * bursts) are byte-split into sequential I/O cycles. */
    .impl.min_access_size = 1,
    .impl.max_access_size = 1,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void at91_nand_realize(DeviceState *dev, Error **errp)
{
    AT91NandState *s = AT91_NAND(dev);

    if (!s->chip) {
        error_setg(errp, "at91-nand: 'nand' chip link property not set");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &at91_nand_ops, s,
                          "at91-nand", AT91_NAND_WINDOW_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static const Property at91_nand_properties[] = {
    DEFINE_PROP_LINK("nand", AT91NandState, chip, TYPE_NAND, DeviceState *),
};

static void at91_nand_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = at91_nand_realize;
    device_class_set_props(dc, at91_nand_properties);
}

static const TypeInfo at91_nand_info = {
    .name          = TYPE_AT91_NAND,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91NandState),
    .class_init    = at91_nand_class_init,
};

static void at91_nand_register_types(void)
{
    type_register_static(&at91_nand_info);
}

type_init(at91_nand_register_types)
