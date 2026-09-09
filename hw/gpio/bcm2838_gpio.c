/*
 * Raspberry Pi (BCM2838) GPIO Controller
 * This implementation is based on bcm2835_gpio (hw/gpio/bcm2835_gpio.c)
 *
 * Copyright (c) 2022 Auriga LLC
 *
 * Authors:
 *  Lotosh, Aleksey <aleksey.lotosh@auriga.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "hw/sd/sd.h"
#include "hw/gpio/bcm2838_gpio.h"
#include "hw/core/irq.h"

#define GPFSEL0   0x00
#define GPFSEL1   0x04
#define GPFSEL2   0x08
#define GPFSEL3   0x0C
#define GPFSEL4   0x10
#define GPFSEL5   0x14
#define GPSET0    0x1C
#define GPSET1    0x20
#define GPCLR0    0x28
#define GPCLR1    0x2C
#define GPLEV0    0x34
#define GPLEV1    0x38
#define GPEDS0    0x40
#define GPEDS1    0x44
#define GPREN0    0x4C
#define GPREN1    0x50
#define GPFEN0    0x58
#define GPFEN1    0x5C
#define GPHEN0    0x64
#define GPHEN1    0x68
#define GPLEN0    0x70
#define GPLEN1    0x74
#define GPAREN0   0x7C
#define GPAREN1   0x80
#define GPAFEN0   0x88
#define GPAFEN1   0x8C

#define GPIO_PUP_PDN_CNTRL_REG0 0xE4
#define GPIO_PUP_PDN_CNTRL_REG1 0xE8
#define GPIO_PUP_PDN_CNTRL_REG2 0xEC
#define GPIO_PUP_PDN_CNTRL_REG3 0xF0

#define RESET_VAL_CNTRL_REG0 0xAAA95555
#define RESET_VAL_CNTRL_REG1 0xA0AAAAAA
#define RESET_VAL_CNTRL_REG2 0x50AAA95A
#define RESET_VAL_CNTRL_REG3 0x00055555

#define NUM_FSELN_IN_GPFSELN 10
#define NUM_BITS_FSELN       3
#define MASK_FSELN           0x7

#define BYTES_IN_WORD        4

/* bcm,function property */
#define BCM2838_FSEL_GPIO_IN    0
#define BCM2838_FSEL_GPIO_OUT   1
#define BCM2838_FSEL_ALT5       2
#define BCM2838_FSEL_ALT4       3
#define BCM2838_FSEL_ALT0       4
#define BCM2838_FSEL_ALT1       5
#define BCM2838_FSEL_ALT2       6
#define BCM2838_FSEL_ALT3       7

static uint32_t gpfsel_get(BCM2838GpioState *s, uint8_t reg)
{
    int i;
    uint32_t value = 0;
    for (i = 0; i < NUM_FSELN_IN_GPFSELN; i++) {
        uint32_t index = NUM_FSELN_IN_GPFSELN * reg + i;
        if (index < sizeof(s->fsel)) {
            value |= (s->fsel[index] & MASK_FSELN) << (NUM_BITS_FSELN * i);
        }
    }
    return value;
}

static void gpfsel_set(BCM2838GpioState *s, uint8_t reg, uint32_t value)
{
    int i;
    for (i = 0; i < NUM_FSELN_IN_GPFSELN; i++) {
        uint32_t index = NUM_FSELN_IN_GPFSELN * reg + i;
        if (index < sizeof(s->fsel)) {
            int fsel = (value >> (NUM_BITS_FSELN * i)) & MASK_FSELN;
            s->fsel[index] = fsel;
        }
    }

    /* SD controller selection (48-53) */
    if (s->sd_fsel != BCM2838_FSEL_GPIO_IN
        && (s->fsel[48] == BCM2838_FSEL_GPIO_IN)
        && (s->fsel[49] == BCM2838_FSEL_GPIO_IN)
        && (s->fsel[50] == BCM2838_FSEL_GPIO_IN)
        && (s->fsel[51] == BCM2838_FSEL_GPIO_IN)
        && (s->fsel[52] == BCM2838_FSEL_GPIO_IN)
        && (s->fsel[53] == BCM2838_FSEL_GPIO_IN)
       ) {
        /* SDHCI controller selected */
        sdbus_reparent_card(s->sdbus_sdhost, s->sdbus_sdhci);
        s->sd_fsel = BCM2838_FSEL_GPIO_IN;
    } else if (s->sd_fsel != BCM2838_FSEL_ALT0
               && (s->fsel[48] == BCM2838_FSEL_ALT0) /* SD_CLK_R */
               && (s->fsel[49] == BCM2838_FSEL_ALT0) /* SD_CMD_R */
               && (s->fsel[50] == BCM2838_FSEL_ALT0) /* SD_DATA0_R */
               && (s->fsel[51] == BCM2838_FSEL_ALT0) /* SD_DATA1_R */
               && (s->fsel[52] == BCM2838_FSEL_ALT0) /* SD_DATA2_R */
               && (s->fsel[53] == BCM2838_FSEL_ALT0) /* SD_DATA3_R */
              ) {
        /* SDHost controller selected */
        sdbus_reparent_card(s->sdbus_sdhci, s->sdbus_sdhost);
        s->sd_fsel = BCM2838_FSEL_ALT0;
    }
}

static int gpfsel_is_out(BCM2838GpioState *s, int index)
{
    if (index >= 0 && index < BCM2838_GPIO_NUM) {
        return s->fsel[index] == 1;
    }
    return 0;
}

static void bcm2838_gpio_set_out(BCM2838GpioState *s, int index, bool level)
{
    s->out_level = deposit64(s->out_level, index, 1, level);
    qemu_set_irq(s->out[index], level);
}

static void bcm2838_gpio_replay_outputs(BCM2838GpioState *s)
{
    for (int i = 0; i < ARRAY_SIZE(s->out); i++) {
        qemu_set_irq(s->out[i], extract64(s->out_level, i, 1));
    }
}

/*
 * Assert each of the 3 GPIO interrupt lines based on which EDS bits are set.
 * Mapping matches the Linux pinctrl-bcm2835 grouping:
 *   line 0: bank0 GPIO 0-27
 *   line 1: bank0 GPIO 28-31 and bank1 GPIO 32-45 (bank1 bits 0-13)
 *   line 2: bank1 GPIO 46-53 (bank1 bits 14-21)
 */
static void bcm2838_gpio_update_irq(BCM2838GpioState *s)
{
    qemu_set_irq(s->irq[0], (s->eds[0] & 0x0fffffff) != 0);
    qemu_set_irq(s->irq[1],
                 ((s->eds[0] & 0xf0000000) || (s->eds[1] & 0x00003fff)) != 0);
    qemu_set_irq(s->irq[2], (s->eds[1] & 0x003fc000) != 0);
}

/* Evaluate level (high/low) detection for a bank against its current level. */
static void bcm2838_gpio_level_detect(BCM2838GpioState *s, int bank,
                                      uint32_t level)
{
    s->eds[bank] |= (s->hen[bank] & level);
    s->eds[bank] |= (s->len[bank] & ~level);
}

/* Evaluate edge detection for the bits that just changed in a bank. */
static void bcm2838_gpio_edge_detect(BCM2838GpioState *s, int bank,
                                     uint32_t rising, uint32_t falling)
{
    s->eds[bank] |= ((s->ren[bank] | s->aren[bank]) & rising);
    s->eds[bank] |= ((s->fen[bank] | s->afen[bank]) & falling);
}

static void gpset(BCM2838GpioState *s, uint32_t val, uint8_t start,
                  uint8_t count, uint32_t *lev)
{
    uint32_t changes = val & ~*lev;
    uint32_t cur = 1;

    int i;
    for (i = 0; i < count; i++) {
        if ((changes & cur) && (gpfsel_is_out(s, start + i))) {
            bcm2838_gpio_set_out(s, start + i, true);
        }
        cur <<= 1;
    }

    *lev |= val;

    /* Rising edges + high-level detection on the affected bank. */
    {
        int bank = (start >= 32) ? 1 : 0;
        bcm2838_gpio_edge_detect(s, bank, changes, 0);
        bcm2838_gpio_level_detect(s, bank, *lev);
        bcm2838_gpio_update_irq(s);
    }
}

static void gpclr(BCM2838GpioState *s, uint32_t val, uint8_t start,
                  uint8_t count, uint32_t *lev)
{
    uint32_t changes = val & *lev;
    uint32_t cur = 1;

    int i;
    for (i = 0; i < count; i++) {
        if ((changes & cur) && (gpfsel_is_out(s, start + i))) {
            bcm2838_gpio_set_out(s, start + i, false);
        }
        cur <<= 1;
    }

    *lev &= ~val;

    /* Falling edges + low-level detection on the affected bank. */
    {
        int bank = (start >= 32) ? 1 : 0;
        bcm2838_gpio_edge_detect(s, bank, 0, changes);
        bcm2838_gpio_level_detect(s, bank, *lev);
        bcm2838_gpio_update_irq(s);
    }
}

static uint64_t bcm2838_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    BCM2838GpioState *s = (BCM2838GpioState *)opaque;
    uint64_t value = 0;

    switch (offset) {
    case GPFSEL0:
    case GPFSEL1:
    case GPFSEL2:
    case GPFSEL3:
    case GPFSEL4:
    case GPFSEL5:
        value = gpfsel_get(s, offset / BYTES_IN_WORD);
        break;
    case GPSET0:
    case GPSET1:
    case GPCLR0:
    case GPCLR1:
        /* Write Only */
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: Attempt reading from write only"
                      " register. 0x%"PRIx64" will be returned."
                      " Address 0x%"HWADDR_PRIx", size %u\n",
                      TYPE_BCM2838_GPIO, __func__, value, offset, size);
        break;
    case GPLEV0:
        value = s->lev0;
        break;
    case GPLEV1:
        value = s->lev1;
        break;
    case GPEDS0:
        value = s->eds[0];
        break;
    case GPEDS1:
        value = s->eds[1];
        break;
    case GPREN0:
        value = s->ren[0];
        break;
    case GPREN1:
        value = s->ren[1];
        break;
    case GPFEN0:
        value = s->fen[0];
        break;
    case GPFEN1:
        value = s->fen[1];
        break;
    case GPHEN0:
        value = s->hen[0];
        break;
    case GPHEN1:
        value = s->hen[1];
        break;
    case GPLEN0:
        value = s->len[0];
        break;
    case GPLEN1:
        value = s->len[1];
        break;
    case GPAREN0:
        value = s->aren[0];
        break;
    case GPAREN1:
        value = s->aren[1];
        break;
    case GPAFEN0:
        value = s->afen[0];
        break;
    case GPAFEN1:
        value = s->afen[1];
        break;
    case GPIO_PUP_PDN_CNTRL_REG0:
    case GPIO_PUP_PDN_CNTRL_REG1:
    case GPIO_PUP_PDN_CNTRL_REG2:
    case GPIO_PUP_PDN_CNTRL_REG3:
        value = s->pup_cntrl_reg[(offset - GPIO_PUP_PDN_CNTRL_REG0)
                                 / sizeof(s->pup_cntrl_reg[0])];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: bad offset %"HWADDR_PRIx"\n",
                      TYPE_BCM2838_GPIO, __func__, offset);
        break;
    }

    return value;
}

static void bcm2838_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    BCM2838GpioState *s = (BCM2838GpioState *)opaque;

    switch (offset) {
    case GPFSEL0:
    case GPFSEL1:
    case GPFSEL2:
    case GPFSEL3:
    case GPFSEL4:
    case GPFSEL5:
        gpfsel_set(s, offset / BYTES_IN_WORD, value);
        break;
    case GPSET0:
        gpset(s, value, 0, 32, &s->lev0);
        break;
    case GPSET1:
        gpset(s, value, 32, BCM2838_GPIO_NUM - 32, &s->lev1);
        break;
    case GPCLR0:
        gpclr(s, value, 0, 32, &s->lev0);
        break;
    case GPCLR1:
        gpclr(s, value, 32, BCM2838_GPIO_NUM - 32, &s->lev1);
        break;
    case GPLEV0:
    case GPLEV1:
        /* Read Only */
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: Attempt writing 0x%"PRIx64""
                      " to read only register. Ignored."
                      " Address 0x%"HWADDR_PRIx", size %u\n",
                      TYPE_BCM2838_GPIO, __func__, value, offset, size);
        break;
    case GPEDS0: /* write-1-to-clear */
        s->eds[0] &= ~(uint32_t)value;
        bcm2838_gpio_update_irq(s);
        break;
    case GPEDS1:
        s->eds[1] &= ~(uint32_t)value;
        bcm2838_gpio_update_irq(s);
        break;
    case GPREN0:
        s->ren[0] = value;
        goto reeval;
    case GPREN1:
        s->ren[1] = value;
        goto reeval;
    case GPFEN0:
        s->fen[0] = value;
        goto reeval;
    case GPFEN1:
        s->fen[1] = value;
        goto reeval;
    case GPHEN0:
        s->hen[0] = value;
        goto reeval;
    case GPHEN1:
        s->hen[1] = value;
        goto reeval;
    case GPLEN0:
        s->len[0] = value;
        goto reeval;
    case GPLEN1:
        s->len[1] = value;
        goto reeval;
    case GPAREN0:
        s->aren[0] = value;
        goto reeval;
    case GPAREN1:
        s->aren[1] = value;
        goto reeval;
    case GPAFEN0:
        s->afen[0] = value;
        goto reeval;
    case GPAFEN1:
        s->afen[1] = value;
        goto reeval;
    reeval:
        bcm2838_gpio_level_detect(s, 0, s->lev0);
        bcm2838_gpio_level_detect(s, 1, s->lev1);
        bcm2838_gpio_update_irq(s);
        break;
    case GPIO_PUP_PDN_CNTRL_REG0:
    case GPIO_PUP_PDN_CNTRL_REG1:
    case GPIO_PUP_PDN_CNTRL_REG2:
    case GPIO_PUP_PDN_CNTRL_REG3:
        s->pup_cntrl_reg[(offset - GPIO_PUP_PDN_CNTRL_REG0)
                         / sizeof(s->pup_cntrl_reg[0])] = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: %s: bad offset %"HWADDR_PRIx"\n",
                  TYPE_BCM2838_GPIO, __func__, offset);
    }
}

static void bcm2838_gpio_reset(DeviceState *dev)
{
    BCM2838GpioState *s = BCM2838_GPIO(dev);
    int i;

    /*
     * Clear the function-select registers through gpfsel_set() so the SD-mux
     * side effect runs: if the guest had switched the card to SDHost, driving
     * the pins back to inputs reparents it to SDHCI. A bare memset() would
     * clear fsel without the reparent, leaving the card stranded on SDHost
     * while sd_fsel claims SDHCI is selected.
     */
    for (i = 0; i < 6; i++) {
        gpfsel_set(s, i, 0);
    }

    s->sd_fsel = 0;

    /* SDHCI is selected by default */
    sdbus_reparent_card(&s->sdbus, s->sdbus_sdhci);

    s->lev0 = 0;
    s->lev1 = 0;
    s->out_level = 0;
    bcm2838_gpio_replay_outputs(s);

    memset(s->ren, 0, sizeof(s->ren));
    memset(s->fen, 0, sizeof(s->fen));
    memset(s->hen, 0, sizeof(s->hen));
    memset(s->len, 0, sizeof(s->len));
    memset(s->aren, 0, sizeof(s->aren));
    memset(s->afen, 0, sizeof(s->afen));
    memset(s->eds, 0, sizeof(s->eds));

    s->pup_cntrl_reg[0] = RESET_VAL_CNTRL_REG0;
    s->pup_cntrl_reg[1] = RESET_VAL_CNTRL_REG1;
    s->pup_cntrl_reg[2] = RESET_VAL_CNTRL_REG2;
    s->pup_cntrl_reg[3] = RESET_VAL_CNTRL_REG3;
    bcm2838_gpio_update_irq(s);
}

static const MemoryRegionOps bcm2838_gpio_ops = {
    .read = bcm2838_gpio_read,
    .write = bcm2838_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int bcm2838_gpio_post_load(void *opaque, int version_id)
{
    BCM2838GpioState *s = opaque;

    bcm2838_gpio_replay_outputs(s);
    bcm2838_gpio_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_bcm2838_gpio = {
    .name = "bcm2838_gpio",
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = bcm2838_gpio_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(fsel, BCM2838GpioState, BCM2838_GPIO_NUM),
        VMSTATE_UINT32(lev0, BCM2838GpioState),
        VMSTATE_UINT32(lev1, BCM2838GpioState),
        VMSTATE_UINT8(sd_fsel, BCM2838GpioState),
        VMSTATE_UINT32_ARRAY(pup_cntrl_reg, BCM2838GpioState,
                             GPIO_PUP_PDN_CNTRL_NUM),
        VMSTATE_UINT32_ARRAY_V(ren, BCM2838GpioState, 2, 2),
        VMSTATE_UINT32_ARRAY_V(fen, BCM2838GpioState, 2, 2),
        VMSTATE_UINT32_ARRAY_V(hen, BCM2838GpioState, 2, 2),
        VMSTATE_UINT32_ARRAY_V(len, BCM2838GpioState, 2, 2),
        VMSTATE_UINT32_ARRAY_V(aren, BCM2838GpioState, 2, 2),
        VMSTATE_UINT32_ARRAY_V(afen, BCM2838GpioState, 2, 2),
        VMSTATE_UINT32_ARRAY_V(eds, BCM2838GpioState, 2, 2),
        VMSTATE_UINT64_V(out_level, BCM2838GpioState, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm2838_gpio_init(Object *obj)
{
    BCM2838GpioState *s = BCM2838_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_SD_BUS, DEVICE(s), "sd-bus");

    memory_region_init_io(&s->iomem, obj, &bcm2838_gpio_ops, s,
                          "bcm2838_gpio", BCM2838_GPIO_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out(dev, s->out, BCM2838_GPIO_NUM);

    for (int i = 0; i < BCM2838_GPIO_NUM_IRQS; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void bcm2838_gpio_realize(DeviceState *dev, Error **errp)
{
    BCM2838GpioState *s = BCM2838_GPIO(dev);
    Object *obj;

    obj = object_property_get_link(OBJECT(dev), "sdbus-sdhci", &error_abort);
    s->sdbus_sdhci = SD_BUS(obj);

    obj = object_property_get_link(OBJECT(dev), "sdbus-sdhost", &error_abort);
    s->sdbus_sdhost = SD_BUS(obj);
}

static void bcm2838_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_bcm2838_gpio;
    dc->realize = &bcm2838_gpio_realize;
    device_class_set_legacy_reset(dc, bcm2838_gpio_reset);
}

static const TypeInfo bcm2838_gpio_info = {
    .name          = TYPE_BCM2838_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM2838GpioState),
    .instance_init = bcm2838_gpio_init,
    .class_init    = bcm2838_gpio_class_init,
};

static void bcm2838_gpio_register_types(void)
{
    type_register_static(&bcm2838_gpio_info);
}

type_init(bcm2838_gpio_register_types)
