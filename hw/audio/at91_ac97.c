/*
 * Atmel AT91 AC97 controller and codec.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This models the command channel used by the Linux AC97 bus and channel A's
 * PDC playback/capture path.  The attached codec has LM4549-compatible IDs
 * and exposes variable-rate stereo PCM to a QEMU audio backend.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/audio/at91_ac97.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/audio.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include "trace.h"

#define AC97C_MR        0x008
#define AC97C_ICA       0x010
#define AC97C_OCA       0x014
#define AC97C_CARHR     0x020
#define AC97C_CATHR     0x024
#define AC97C_CASR      0x028
#define AC97C_CAMR      0x02c
#define AC97C_CORHR     0x040
#define AC97C_COTHR     0x044
#define AC97C_COSR      0x048
#define AC97C_COMR      0x04c
#define AC97C_SR        0x050
#define AC97C_IER       0x054
#define AC97C_IDR       0x058
#define AC97C_IMR       0x05c
#define AC97C_VERSION   0x0fc

#define PDC_RPR         0x100
#define PDC_RCR         0x104
#define PDC_TPR         0x108
#define PDC_TCR         0x10c
#define PDC_RNPR        0x110
#define PDC_RNCR        0x114
#define PDC_TNPR        0x118
#define PDC_TNCR        0x11c
#define PDC_PTCR        0x120
#define PDC_PTSR        0x124

#define AC97C_IOMEM_SIZE    0x128
#define AC97C_VERSION_VALUE 0x100

#define MR_ENA          BIT(0)
#define MR_WRST         BIT(1)

#define CSR_TXRDY       BIT(0)
#define CSR_TXEMPTY     BIT(1)
#define CSR_RXRDY       BIT(4)
#define CSR_ENDTX       BIT(10)
#define CSR_ENDRX       BIT(14)

#define CMR_CENA        BIT(21)
#define CMR_DMAEN       BIT(22)

#define SR_COEVT        BIT(2)
#define SR_CAEVT        BIT(3)

#define PDC_RXTEN       BIT(0)
#define PDC_RXTDIS      BIT(1)
#define PDC_TXTEN       BIT(8)
#define PDC_TXTDIS      BIT(9)

#define AC97_RESET              0x00
#define AC97_MASTER_VOL         0x02
#define AC97_HEADPHONE_VOL      0x04
#define AC97_MASTER_MONO_VOL    0x06
#define AC97_PC_BEEP_VOL        0x0a
#define AC97_PHONE_VOL          0x0c
#define AC97_MIC_VOL            0x0e
#define AC97_LINE_IN_VOL        0x10
#define AC97_CD_VOL             0x12
#define AC97_VIDEO_VOL          0x14
#define AC97_AUX_VOL            0x16
#define AC97_PCM_VOL            0x18
#define AC97_REC_SEL            0x1a
#define AC97_REC_GAIN           0x1c
#define AC97_GENERAL_PURPOSE    0x20
#define AC97_3D_CONTROL         0x22
#define AC97_POWERDOWN          0x26
#define AC97_EXT_AUDIO_ID       0x28
#define AC97_EXT_AUDIO_CTRL     0x2a
#define AC97_PCM_DAC_RATE       0x2c
#define AC97_PCM_ADC_RATE       0x32
#define AC97_VENDOR_ID1         0x7c
#define AC97_VENDOR_ID2         0x7e

#define AC97_DEFAULT_RATE       48000
#define AC97_MIN_RATE           4000
#define AC97_DMA_CHUNK          4096

struct AT91AC97State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    AudioBackend *audio_be;
    SWVoiceOut *voice_out;
    SWVoiceIn *voice_in;

    uint32_t mr;
    uint32_t ica;
    uint32_t oca;
    uint32_t casr;
    uint32_t camr;
    uint32_t corhr;
    uint32_t cosr;
    uint32_t comr;
    uint32_t imr;

    uint32_t rpr;
    uint32_t rcr;
    uint32_t tpr;
    uint32_t tcr;
    uint32_t rnpr;
    uint32_t rncr;
    uint32_t tnpr;
    uint32_t tncr;
    bool rxten;
    bool txten;

    uint16_t codec[128];
};

static void at91_ac97_out_cb(void *opaque, int free);
static void at91_ac97_in_cb(void *opaque, int avail);

static uint32_t at91_ac97_status(AT91AC97State *s)
{
    uint32_t status = 0;

    if (s->casr & s->camr) {
        status |= SR_CAEVT;
    }
    if (s->cosr & s->comr) {
        status |= SR_COEVT;
    }
    return status;
}

static void at91_ac97_update_irq(AT91AC97State *s)
{
    qemu_set_irq(s->irq, (at91_ac97_status(s) & s->imr) != 0);
}

static bool at91_ac97_playback_enabled(AT91AC97State *s)
{
    return (s->mr & MR_ENA) && s->txten &&
           (s->camr & (CMR_CENA | CMR_DMAEN)) ==
           (CMR_CENA | CMR_DMAEN);
}

static bool at91_ac97_capture_enabled(AT91AC97State *s)
{
    return (s->mr & MR_ENA) && s->rxten &&
           (s->camr & (CMR_CENA | CMR_DMAEN)) ==
           (CMR_CENA | CMR_DMAEN);
}

static void at91_ac97_update_audio(AT91AC97State *s)
{
    if (s->voice_out) {
        audio_be_set_active_out(s->audio_be, s->voice_out,
                                at91_ac97_playback_enabled(s));
    }
    if (s->voice_in) {
        audio_be_set_active_in(s->audio_be, s->voice_in,
                               at91_ac97_capture_enabled(s));
    }
}

static unsigned at91_ac97_rate(AT91AC97State *s, unsigned reg)
{
    unsigned rate = s->codec[reg];

    if (rate < AC97_MIN_RATE || rate > AC97_DEFAULT_RATE) {
        rate = AC97_DEFAULT_RATE;
    }
    return rate;
}

static void at91_ac97_open_out(AT91AC97State *s)
{
    struct audsettings settings = {
        .freq = at91_ac97_rate(s, AC97_PCM_DAC_RATE),
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = false,
    };

    s->voice_out = audio_be_open_out(s->audio_be, s->voice_out,
                                     "at91-ac97.out", s,
                                     at91_ac97_out_cb, &settings);
    if (s->voice_out) {
        audio_be_set_volume_out_lr(s->audio_be, s->voice_out, false,
                                   255, 255);
    }
}

static void at91_ac97_open_in(AT91AC97State *s)
{
    struct audsettings settings = {
        .freq = at91_ac97_rate(s, AC97_PCM_ADC_RATE),
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = false,
    };

    s->voice_in = audio_be_open_in(s->audio_be, s->voice_in,
                                   "at91-ac97.in", s,
                                   at91_ac97_in_cb, &settings);
    if (s->voice_in) {
        audio_be_set_volume_in_lr(s->audio_be, s->voice_in, false, 255, 255);
    }
}

static void at91_ac97_codec_reset(AT91AC97State *s)
{
    memset(s->codec, 0, sizeof(s->codec));
    s->codec[AC97_RESET] = 0x0d50;
    s->codec[AC97_MASTER_VOL] = 0x8008;
    s->codec[AC97_HEADPHONE_VOL] = 0x8000;
    s->codec[AC97_MASTER_MONO_VOL] = 0x8000;
    s->codec[AC97_PC_BEEP_VOL] = 0x0000;
    s->codec[AC97_PHONE_VOL] = 0x8008;
    s->codec[AC97_MIC_VOL] = 0x8008;
    s->codec[AC97_LINE_IN_VOL] = 0x8808;
    s->codec[AC97_CD_VOL] = 0x8808;
    s->codec[AC97_VIDEO_VOL] = 0x8808;
    s->codec[AC97_AUX_VOL] = 0x8808;
    s->codec[AC97_PCM_VOL] = 0x8808;
    s->codec[AC97_REC_SEL] = 0x0000;
    s->codec[AC97_REC_GAIN] = 0x8000;
    s->codec[AC97_GENERAL_PURPOSE] = 0x0000;
    s->codec[AC97_3D_CONTROL] = 0x0101;
    s->codec[AC97_POWERDOWN] = 0x000f;
    s->codec[AC97_EXT_AUDIO_ID] = 0x0001; /* variable-rate audio */
    s->codec[AC97_EXT_AUDIO_CTRL] = 0x0000;
    s->codec[AC97_PCM_DAC_RATE] = AC97_DEFAULT_RATE;
    s->codec[AC97_PCM_ADC_RATE] = AC97_DEFAULT_RATE;
    s->codec[AC97_VENDOR_ID1] = 0x4e53; /* National Semiconductor */
    s->codec[AC97_VENDOR_ID2] = 0x4331; /* LM4549-compatible codec */
}

static void at91_ac97_codec_write(AT91AC97State *s, unsigned reg,
                                  uint16_t value)
{
    reg &= 0x7e;

    switch (reg) {
    case AC97_RESET:
        at91_ac97_codec_reset(s);
        at91_ac97_open_out(s);
        if (s->voice_in) {
            at91_ac97_open_in(s);
        }
        at91_ac97_update_audio(s);
        break;
    case AC97_PCM_DAC_RATE:
        if (value >= AC97_MIN_RATE && value <= AC97_DEFAULT_RATE) {
            s->codec[reg] = value;
            at91_ac97_open_out(s);
            at91_ac97_update_audio(s);
        }
        break;
    case AC97_PCM_ADC_RATE:
        if (value >= AC97_MIN_RATE && value <= AC97_DEFAULT_RATE) {
            s->codec[reg] = value;
            if (s->voice_in) {
                at91_ac97_open_in(s);
            }
            at91_ac97_update_audio(s);
        }
        break;
    case AC97_POWERDOWN:
        s->codec[reg] = (value & ~0xf) | (s->codec[reg] & 0xf);
        break;
    case AC97_EXT_AUDIO_ID:
    case AC97_VENDOR_ID1:
    case AC97_VENDOR_ID2:
        break;
    default:
        s->codec[reg] = value;
        break;
    }
}

static void at91_ac97_finish_tx(AT91AC97State *s)
{
    trace_at91_ac97_period("playback", s->tpr);
    if (s->tncr) {
        s->tpr = s->tnpr;
        s->tcr = s->tncr;
        s->tnpr = 0;
        s->tncr = 0;
    }
    s->casr |= CSR_ENDTX;
    at91_ac97_update_irq(s);
}

static void at91_ac97_finish_rx(AT91AC97State *s)
{
    trace_at91_ac97_period("capture", s->rpr);
    if (s->rncr) {
        s->rpr = s->rnpr;
        s->rcr = s->rncr;
        s->rnpr = 0;
        s->rncr = 0;
    }
    s->casr |= CSR_ENDRX;
    at91_ac97_update_irq(s);
}

static void at91_ac97_out_cb(void *opaque, int free)
{
    AT91AC97State *s = opaque;
    uint8_t buffer[AC97_DMA_CHUNK];

    if (!at91_ac97_playback_enabled(s)) {
        return;
    }

    while (free >= 2 && s->tcr) {
        size_t amount = MIN((size_t)free, sizeof(buffer));
        size_t written;

        amount = MIN(amount, (size_t)s->tcr * 2);
        amount &= ~1ULL;
        address_space_read(&address_space_memory, s->tpr,
                           MEMTXATTRS_UNSPECIFIED, buffer, amount);
        written = audio_be_write(s->audio_be, s->voice_out, buffer, amount);
        written &= ~1ULL;
        if (!written) {
            break;
        }

        s->tpr += written;
        s->tcr -= written / 2;
        free -= written;
        if (!s->tcr) {
            at91_ac97_finish_tx(s);
            break;
        }
    }
}

static void at91_ac97_in_cb(void *opaque, int avail)
{
    AT91AC97State *s = opaque;
    uint8_t buffer[AC97_DMA_CHUNK];

    if (!at91_ac97_capture_enabled(s)) {
        return;
    }

    while (avail >= 2 && s->rcr) {
        size_t amount = MIN((size_t)avail, sizeof(buffer));
        size_t acquired;

        amount = MIN(amount, (size_t)s->rcr * 2);
        amount &= ~1ULL;
        acquired = audio_be_read(s->audio_be, s->voice_in, buffer, amount);
        acquired &= ~1ULL;
        if (!acquired) {
            break;
        }

        address_space_write(&address_space_memory, s->rpr,
                            MEMTXATTRS_UNSPECIFIED, buffer, acquired);
        s->rpr += acquired;
        s->rcr -= acquired / 2;
        avail -= acquired;
        if (!s->rcr) {
            at91_ac97_finish_rx(s);
            break;
        }
    }
}

static uint64_t at91_ac97_read(void *opaque, hwaddr offset, unsigned size)
{
    AT91AC97State *s = opaque;
    uint32_t value = 0;

    switch (offset) {
    case AC97C_MR:
        value = s->mr;
        break;
    case AC97C_ICA:
        value = s->ica;
        break;
    case AC97C_OCA:
        value = s->oca;
        break;
    case AC97C_CARHR:
    case AC97C_CATHR:
        break;
    case AC97C_CASR:
        value = s->casr;
        s->casr = 0;
        at91_ac97_update_irq(s);
        break;
    case AC97C_CAMR:
        value = s->camr;
        break;
    case AC97C_CORHR:
        value = s->corhr;
        s->cosr &= ~CSR_RXRDY;
        at91_ac97_update_irq(s);
        break;
    case AC97C_COSR:
        value = s->cosr | CSR_TXRDY | CSR_TXEMPTY;
        break;
    case AC97C_COMR:
        value = s->comr;
        break;
    case AC97C_SR:
        value = at91_ac97_status(s);
        break;
    case AC97C_IMR:
        value = s->imr;
        break;
    case AC97C_VERSION:
        value = AC97C_VERSION_VALUE;
        break;
    case PDC_RPR:
        value = s->rpr;
        break;
    case PDC_RCR:
        value = s->rcr;
        break;
    case PDC_TPR:
        value = s->tpr;
        break;
    case PDC_TCR:
        value = s->tcr;
        break;
    case PDC_RNPR:
        value = s->rnpr;
        break;
    case PDC_RNCR:
        value = s->rncr;
        break;
    case PDC_TNPR:
        value = s->tnpr;
        break;
    case PDC_TNCR:
        value = s->tncr;
        break;
    case PDC_PTSR:
        value = (s->rxten ? PDC_RXTEN : 0) |
                (s->txten ? PDC_TXTEN : 0);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-ac97: unimplemented read at 0x%03" HWADDR_PRIx
                      "\n", offset);
        break;
    }
    return value;
}

static void at91_ac97_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    AT91AC97State *s = opaque;
    uint32_t val = value;

    switch (offset) {
    case AC97C_MR:
        s->mr = val & (MR_ENA | MR_WRST | BIT(2));
        if (val & MR_WRST) {
            at91_ac97_codec_reset(s);
            at91_ac97_open_out(s);
            if (s->voice_in) {
                at91_ac97_open_in(s);
            }
        }
        at91_ac97_update_audio(s);
        break;
    case AC97C_ICA:
        s->ica = val;
        break;
    case AC97C_OCA:
        s->oca = val;
        break;
    case AC97C_CAMR:
        s->camr = val;
        at91_ac97_update_audio(s);
        at91_ac97_update_irq(s);
        break;
    case AC97C_COTHR:
    {
        unsigned reg = (val >> 16) & 0x7f;
        uint16_t codec_value = val;
        bool read = val & BIT(23);

        if (read) {
            reg &= 0x7e;
            s->corhr = s->codec[reg];
            s->cosr |= CSR_RXRDY;
            codec_value = s->corhr;
        } else {
            at91_ac97_codec_write(s, reg, val);
        }
        trace_at91_ac97_codec(read, reg, codec_value);
        at91_ac97_update_irq(s);
        break;
    }
    case AC97C_COMR:
        s->comr = val;
        at91_ac97_update_irq(s);
        break;
    case AC97C_IER:
        s->imr |= val;
        at91_ac97_update_irq(s);
        break;
    case AC97C_IDR:
        s->imr &= ~val;
        at91_ac97_update_irq(s);
        break;
    case PDC_RPR:
        s->rpr = val;
        break;
    case PDC_RCR:
        s->rcr = val;
        break;
    case PDC_TPR:
        s->tpr = val;
        break;
    case PDC_TCR:
        s->tcr = val;
        break;
    case PDC_RNPR:
        s->rnpr = val;
        break;
    case PDC_RNCR:
        s->rncr = val;
        break;
    case PDC_TNPR:
        s->tnpr = val;
        break;
    case PDC_TNCR:
        s->tncr = val;
        break;
    case PDC_PTCR:
        if (val & PDC_RXTEN) {
            s->rxten = true;
            if (!s->voice_in) {
                at91_ac97_open_in(s);
            }
        }
        if (val & PDC_RXTDIS) {
            s->rxten = false;
        }
        if (val & PDC_TXTEN) {
            s->txten = true;
        }
        if (val & PDC_TXTDIS) {
            s->txten = false;
        }
        at91_ac97_update_audio(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "at91-ac97: unimplemented write at 0x%03" HWADDR_PRIx
                      " value 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps at91_ac97_ops = {
    .read = at91_ac97_read,
    .write = at91_ac97_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void at91_ac97_reset(DeviceState *dev)
{
    AT91AC97State *s = AT91_AC97(dev);

    s->mr = 0;
    s->ica = 0;
    s->oca = 0;
    s->casr = 0;
    s->camr = 0;
    s->corhr = 0;
    s->cosr = CSR_TXRDY | CSR_TXEMPTY;
    s->comr = 0;
    s->imr = 0;
    s->rpr = 0;
    s->rcr = 0;
    s->tpr = 0;
    s->tcr = 0;
    s->rnpr = 0;
    s->rncr = 0;
    s->tnpr = 0;
    s->tncr = 0;
    s->rxten = false;
    s->txten = false;
    at91_ac97_codec_reset(s);
    if (s->voice_out) {
        at91_ac97_open_out(s);
    }
    if (s->voice_in) {
        at91_ac97_open_in(s);
    }
    at91_ac97_update_audio(s);
    at91_ac97_update_irq(s);
}

static int at91_ac97_post_load(void *opaque, int version_id)
{
    AT91AC97State *s = opaque;

    at91_ac97_open_out(s);
    if (s->rxten) {
        at91_ac97_open_in(s);
    }
    at91_ac97_update_audio(s);
    at91_ac97_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_at91_ac97 = {
    .name = "at91-ac97",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = at91_ac97_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mr, AT91AC97State),
        VMSTATE_UINT32(ica, AT91AC97State),
        VMSTATE_UINT32(oca, AT91AC97State),
        VMSTATE_UINT32(casr, AT91AC97State),
        VMSTATE_UINT32(camr, AT91AC97State),
        VMSTATE_UINT32(corhr, AT91AC97State),
        VMSTATE_UINT32(cosr, AT91AC97State),
        VMSTATE_UINT32(comr, AT91AC97State),
        VMSTATE_UINT32(imr, AT91AC97State),
        VMSTATE_UINT32(rpr, AT91AC97State),
        VMSTATE_UINT32(rcr, AT91AC97State),
        VMSTATE_UINT32(tpr, AT91AC97State),
        VMSTATE_UINT32(tcr, AT91AC97State),
        VMSTATE_UINT32(rnpr, AT91AC97State),
        VMSTATE_UINT32(rncr, AT91AC97State),
        VMSTATE_UINT32(tnpr, AT91AC97State),
        VMSTATE_UINT32(tncr, AT91AC97State),
        VMSTATE_BOOL(rxten, AT91AC97State),
        VMSTATE_BOOL(txten, AT91AC97State),
        VMSTATE_UINT16_ARRAY(codec, AT91AC97State, 128),
        VMSTATE_END_OF_LIST()
    },
};

static void at91_ac97_realize(DeviceState *dev, Error **errp)
{
    AT91AC97State *s = AT91_AC97(dev);

    if (!audio_be_check(&s->audio_be, errp)) {
        return;
    }

    at91_ac97_codec_reset(s);
    at91_ac97_open_out(s);
}

static void at91_ac97_unrealize(DeviceState *dev)
{
    AT91AC97State *s = AT91_AC97(dev);

    audio_be_close_out(s->audio_be, s->voice_out);
    audio_be_close_in(s->audio_be, s->voice_in);
    s->voice_out = NULL;
    s->voice_in = NULL;
}

static const Property at91_ac97_properties[] = {
    DEFINE_AUDIO_PROPERTIES(AT91AC97State, audio_be),
};

static void at91_ac97_init(Object *obj)
{
    AT91AC97State *s = AT91_AC97(obj);

    memory_region_init_io(&s->iomem, obj, &at91_ac97_ops, s,
                          TYPE_AT91_AC97, AC97C_IOMEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static void at91_ac97_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "Atmel AT91 AC97 Controller";
    dc->realize = at91_ac97_realize;
    dc->unrealize = at91_ac97_unrealize;
    dc->vmsd = &vmstate_at91_ac97;
    device_class_set_legacy_reset(dc, at91_ac97_reset);
    device_class_set_props(dc, at91_ac97_properties);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo at91_ac97_info = {
    .name = TYPE_AT91_AC97,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91AC97State),
    .instance_init = at91_ac97_init,
    .class_init = at91_ac97_class_init,
};

static void at91_ac97_register_types(void)
{
    type_register_static(&at91_ac97_info);
}

type_init(at91_ac97_register_types)
