/*
 * Nintendo 3DS power-management microcontroller (Renesas "MCU") as an
 * I2C slave at address 0x25 on I2C bus 2.
 *
 * Only the pieces early firmware and Linux rely on are modelled: the
 * register file with auto-increment, power off / reboot through
 * register 0x20, LCD/backlight power (0x22), LEDs, battery level, RTC
 * (host wall-clock, BCD) and the interrupt status/mask words.
 *
 * Documentation: 3dbrew I2C_Registers ("MCU" section).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/runstate.h"
#include "system/rtc.h"
#include "hw/misc/n3ds.h"
#include "qom/object.h"

OBJECT_DECLARE_SIMPLE_TYPE(N3DSMcuState, N3DS_MCU)

#define MCU_REG_VERSION_HI  0x00
#define MCU_REG_VERSION_LO  0x01
#define MCU_REG_BATTERY     0x0B
#define MCU_REG_POWER_STAT  0x0F
#define MCU_REG_IRQ_STATUS  0x10  /* 4 bytes, read clears */
#define MCU_REG_IRQ_MASK    0x18  /* 4 bytes */
#define MCU_REG_POWER_CTL   0x20
#define MCU_REG_LCD_POWER   0x22
#define MCU_REG_LED_STATE   0x2D
#define MCU_REG_RTC_SEC     0x30
#define MCU_REG_RTC_YEAR    0x36

#define POWER_CTL_POWEROFF  0x01
#define POWER_CTL_REBOOT    0x04

struct N3DSMcuState {
    I2CSlave parent_obj;

    uint8_t regs[256];
    uint8_t reg;
    bool addr_pending;
};

static uint8_t to_bcd(int v)
{
    return ((v / 10) << 4) | (v % 10);
}

static void n3ds_mcu_update_rtc(N3DSMcuState *s)
{
    struct tm tm;

    qemu_get_timedate(&tm, 0);
    s->regs[MCU_REG_RTC_SEC + 0] = to_bcd(tm.tm_sec);
    s->regs[MCU_REG_RTC_SEC + 1] = to_bcd(tm.tm_min);
    s->regs[MCU_REG_RTC_SEC + 2] = to_bcd(tm.tm_hour);
    s->regs[MCU_REG_RTC_SEC + 3] = to_bcd(tm.tm_wday);
    s->regs[MCU_REG_RTC_SEC + 4] = to_bcd(tm.tm_mday);
    s->regs[MCU_REG_RTC_SEC + 5] = to_bcd(tm.tm_mon + 1);
    s->regs[MCU_REG_RTC_YEAR] = to_bcd(tm.tm_year % 100);
}

static int n3ds_mcu_event(I2CSlave *i2c, enum i2c_event event)
{
    N3DSMcuState *s = N3DS_MCU(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->addr_pending = true;
        break;
    case I2C_START_RECV:
        n3ds_mcu_update_rtc(s);
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t n3ds_mcu_recv(I2CSlave *i2c)
{
    N3DSMcuState *s = N3DS_MCU(i2c);
    uint8_t v = s->regs[s->reg];

    if (s->reg >= MCU_REG_IRQ_STATUS && s->reg < MCU_REG_IRQ_STATUS + 4) {
        s->regs[s->reg] = 0;
    }
    s->reg++;
    return v;
}

static int n3ds_mcu_send(I2CSlave *i2c, uint8_t data)
{
    N3DSMcuState *s = N3DS_MCU(i2c);

    if (s->addr_pending) {
        s->reg = data;
        s->addr_pending = false;
        return 0;
    }

    switch (s->reg) {
    case MCU_REG_POWER_CTL:
        if (data & POWER_CTL_POWEROFF) {
            qemu_log("n3ds-mcu: power off requested\n");
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        } else if (data & POWER_CTL_REBOOT) {
            qemu_log("n3ds-mcu: reboot requested\n");
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        break;
    case MCU_REG_IRQ_STATUS ... MCU_REG_IRQ_STATUS + 3:
        break;
    default:
        s->regs[s->reg] = data;
        break;
    }
    s->reg++;
    return 0;
}

static void n3ds_mcu_reset(DeviceState *dev)
{
    N3DSMcuState *s = N3DS_MCU(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[MCU_REG_VERSION_HI] = 0x03;
    s->regs[MCU_REG_VERSION_LO] = 0x05;
    s->regs[MCU_REG_BATTERY] = 100;
    s->regs[MCU_REG_POWER_STAT] = 0x00;
    s->regs[MCU_REG_LCD_POWER] = 0x00;
    memset(&s->regs[MCU_REG_IRQ_MASK], 0xFF, 4);
    s->reg = 0;
    s->addr_pending = false;
    n3ds_mcu_update_rtc(s);
}

static void n3ds_mcu_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->event = n3ds_mcu_event;
    k->recv = n3ds_mcu_recv;
    k->send = n3ds_mcu_send;
    device_class_set_legacy_reset(dc, n3ds_mcu_reset);
}

static const TypeInfo n3ds_mcu_info = {
    .name          = TYPE_N3DS_MCU,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(N3DSMcuState),
    .class_init    = n3ds_mcu_class_init,
};

static void n3ds_mcu_register_types(void)
{
    type_register_static(&n3ds_mcu_info);
}

type_init(n3ds_mcu_register_types)
