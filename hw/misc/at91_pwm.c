/*
 * Atmel/Microchip AT91 Pulse Width Modulation (PWM) controller (PWMV1).
 *
 * Register model of the SAM9 PWM as driven by the Linux pwm-atmel driver
 * ("atmel,at91sam9rl-pwm"): 4 channels, each with a period (CPRD) and duty
 * (CDTY).  QEMU is a functional (not analog-timing) emulator, so this models
 * the register/enable state that the driver reads back - it does not generate a
 * real waveform.  The channel config is exposed so /sys/class/pwm reflects what
 * the guest programmed.
 *
 * Driver contract notes:
 *  - PWM_ENA/PWM_DIS set/clear per-channel bits in PWM_SR; disable() spins on
 *    PWM_SR until the bit clears, so DIS must clear it immediately.
 *  - PWM_ISR (clear-on-read) reports channels that started a new period; the
 *    driver's update_pending logic waits on it before reading back an updated
 *    duty.  We report every enabled channel as "period elapsed" so a queued
 *    CUPD is seen as applied at once.
 *  - CUPD updates CDTY or CPRD depending on CMR.UPD_CDTY.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/memory.h"
#include "hw/misc/at91_pwm.h"
#include "trace.h"

/* Global registers */
#define PWM_MR      0x00   /* Mode                 */
#define PWM_ENA     0x04   /* Enable               */
#define PWM_DIS     0x08   /* Disable              */
#define PWM_SR      0x0c   /* Status               */
#define PWM_IER     0x10   /* Interrupt Enable     */
#define PWM_IDR     0x14   /* Interrupt Disable    */
#define PWM_IMR     0x18   /* Interrupt Mask       */
#define PWM_ISR     0x1c   /* Interrupt Status     */

/* Per-channel registers at 0x200 + ch * 0x20 */
#define PWM_CH_BASE 0x200
#define PWM_CH_SIZE 0x20
#define PWM_CMR     0x00   /* Channel Mode         */
#define PWM_CDTY    0x04   /* Channel Duty Cycle   */
#define PWM_CPRD    0x08   /* Channel Period       */
#define PWM_CCNT    0x0c   /* Channel Counter      */
#define PWM_CUPD    0x10   /* Channel Update       */

#define CMR_UPD_CDTY (1u << 10)

#define PWM_NUM_CH  4
#define PWM_CH_MASK 0x0f

typedef struct AT91PwmChan {
    uint32_t cmr;
    uint32_t cdty;
    uint32_t cprd;
} AT91PwmChan;

struct AT91PwmState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t mr;
    uint32_t sr;    /* enabled-channel bitmap */
    uint32_t imr;
    AT91PwmChan chan[PWM_NUM_CH];
};

static void pwm_update_irq(AT91PwmState *s)
{
    /* No real periods are generated, so no channel interrupt is ever pending. */
    qemu_set_irq(s->irq, 0);
}

static void pwm_reset(DeviceState *dev)
{
    AT91PwmState *s = AT91_PWM(dev);

    s->mr = 0;
    s->sr = 0;
    s->imr = 0;
    memset(s->chan, 0, sizeof(s->chan));
}

static uint64_t pwm_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91PwmState *s = opaque;

    if (offset >= PWM_CH_BASE) {
        unsigned ch = (offset - PWM_CH_BASE) / PWM_CH_SIZE;
        unsigned reg = (offset - PWM_CH_BASE) % PWM_CH_SIZE;

        if (ch >= PWM_NUM_CH) {
            return 0;
        }
        switch (reg) {
        case PWM_CMR:  return s->chan[ch].cmr;
        case PWM_CDTY: return s->chan[ch].cdty;
        case PWM_CPRD: return s->chan[ch].cprd;
        case PWM_CCNT: return 0;   /* free-running counter, unmodelled */
        default:       return 0;
        }
    }

    switch (offset) {
    case PWM_MR:  return s->mr;
    case PWM_SR:  return s->sr;
    case PWM_IMR: return s->imr;
    case PWM_ISR:
        /* Clear-on-read; report every enabled channel as having started a new
         * period so the driver's wait-for-update loop resolves immediately. */
        return s->sr & PWM_CH_MASK;
    default:
        return 0;
    }
}

static void pwm_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    AT91PwmState *s = opaque;
    uint32_t val = value;

    if (offset >= PWM_CH_BASE) {
        unsigned ch = (offset - PWM_CH_BASE) / PWM_CH_SIZE;
        unsigned reg = (offset - PWM_CH_BASE) % PWM_CH_SIZE;

        if (ch >= PWM_NUM_CH) {
            return;
        }
        switch (reg) {
        case PWM_CMR:  s->chan[ch].cmr = val; break;
        case PWM_CDTY: s->chan[ch].cdty = val; break;
        case PWM_CPRD: s->chan[ch].cprd = val; break;
        case PWM_CUPD:
            /* Queued update: target CDTY or CPRD per CMR.UPD_CDTY. */
            if (s->chan[ch].cmr & CMR_UPD_CDTY) {
                s->chan[ch].cdty = val;
            } else {
                s->chan[ch].cprd = val;
            }
            break;
        default:
            break;
        }
        return;
    }

    switch (offset) {
    case PWM_MR:  s->mr = val; break;
    case PWM_ENA:
        s->sr |= val & PWM_CH_MASK;
        trace_at91_pwm_enable(val & PWM_CH_MASK, s->sr);
        break;
    case PWM_DIS:
        s->sr &= ~(val & PWM_CH_MASK);
        trace_at91_pwm_enable(0, s->sr);
        break;
    case PWM_IER: s->imr |= val; pwm_update_irq(s); break;
    case PWM_IDR: s->imr &= ~val; pwm_update_irq(s); break;
    default:
        break;
    }
}

static const MemoryRegionOps pwm_ops = {
    .read = pwm_read,
    .write = pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static const VMStateDescription vmstate_at91_pwm_chan = {
    .name = "at91-pwm-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cmr, AT91PwmChan),
        VMSTATE_UINT32(cdty, AT91PwmChan),
        VMSTATE_UINT32(cprd, AT91PwmChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_at91_pwm = {
    .name = "at91-pwm",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91PwmState),
        VMSTATE_UINT32(sr, AT91PwmState),
        VMSTATE_UINT32(imr, AT91PwmState),
        VMSTATE_STRUCT_ARRAY(chan, AT91PwmState, PWM_NUM_CH, 1,
                             vmstate_at91_pwm_chan, AT91PwmChan),
        VMSTATE_END_OF_LIST()
    }
};

static void pwm_init(Object *obj)
{
    AT91PwmState *s = AT91_PWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pwm_ops, s, "at91-pwm", 0x300);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, pwm_reset);
    dc->vmsd = &vmstate_at91_pwm;
}

static const TypeInfo pwm_type = {
    .name = TYPE_AT91_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91PwmState),
    .instance_init = pwm_init,
    .class_init = pwm_class_init,
};

static void at91_pwm_register_types(void)
{
    type_register_static(&pwm_type);
}

type_init(at91_pwm_register_types)
