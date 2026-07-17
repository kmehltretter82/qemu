/*
 * Atmel/Microchip AT91 EMAC - 10/100 Ethernet MAC (Cadence "macb" IP).
 *
 * Faithful model of the classic AT91 EMAC (Cadence "macb", pre-GEM), as
 * driven by the Linux "cdns,at91sam9260-macb" driver: the classic two-word
 * RX/TX buffer descriptors (128-byte RX buffers, frames span buffers),
 * MDIO/PHY maintenance with an emulated Davicom DM9161A PHY (the part on
 * the SAM9M10-G45-EK) auto-negotiated to 100M/full, and the
 * NCR/NCFGR/NSR/TSR/RSR/ISR register set.  The MID register reports
 * IDNUM < 2 so the driver takes the non-GEM path.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "net/net.h"
#include "hw/net/at91_macb.h"
#include "trace.h"

/* Register offsets (datasheet EMAC user interface). */
#define MACB_NCR    0x00   /* Network Control        */
#define MACB_NCFGR  0x04   /* Network Configuration  */
#define MACB_NSR    0x08   /* Network Status         */
#define MACB_TSR    0x14   /* Transmit Status        */
#define MACB_RBQP   0x18   /* RX Buffer Queue Ptr    */
#define MACB_TBQP   0x1c   /* TX Buffer Queue Ptr    */
#define MACB_RSR    0x20   /* Receive Status         */
#define MACB_ISR    0x24   /* Interrupt Status (RC)  */
#define MACB_IER    0x28   /* Interrupt Enable       */
#define MACB_IDR    0x2c   /* Interrupt Disable      */
#define MACB_IMR    0x30   /* Interrupt Mask (RO)    */
#define MACB_MAN    0x34   /* PHY Maintenance        */
#define MACB_HRB    0x90   /* Hash Register Bottom   */
#define MACB_HRT    0x94   /* Hash Register Top      */
#define MACB_SA1B   0x98   /* Specific Address 1..4  */
#define MACB_SA4T   0xb4
#define MACB_USRIO  0xc0   /* User I/O (MII/RMII)    */
#define MACB_MID    0xfc   /* Module ID              */

/* NCR - Network Control */
#define NCR_RE      (1u << 2)    /* receive enable          */
#define NCR_TE      (1u << 3)    /* transmit enable         */
#define NCR_TSTART  (1u << 9)    /* start transmission      */
#define NCR_THALT   (1u << 10)   /* transmit halt           */

/* NSR - Network Status */
#define NSR_MDIO    (1u << 1)    /* mdio_in pin state       */
#define NSR_IDLE    (1u << 2)    /* PHY management idle      */

/* Interrupt bits (ISR/IER/IDR/IMR) */
#define INT_MFD     (1u << 0)    /* management frame done   */
#define INT_RCOMP   (1u << 1)    /* receive complete        */
#define INT_RXUBR   (1u << 2)    /* RX used bit read        */
#define INT_TCOMP   (1u << 7)    /* transmit complete       */

/* TSR / RSR status bits */
#define TSR_UBR     (1u << 0)    /* TX used bit read        */
#define RSR_BNA     (1u << 0)    /* buffer not available    */
#define RSR_REC     (1u << 1)    /* frame received          */

/* RX descriptor word0 (address) bits */
#define RXD_USED    (1u << 0)    /* owned by software       */
#define RXD_WRAP    (1u << 1)    /* last descriptor in ring */
/* RX descriptor word1 (control) bits */
#define RXD_SOF     (1u << 14)
#define RXD_EOF     (1u << 15)

/* TX descriptor word1 (control) bits */
#define TXD_LAST    (1u << 15)   /* last buffer of frame    */
#define TXD_WRAP    (1u << 30)   /* last descriptor in ring */
#define TXD_USED    (1u << 31)   /* owned by MAC when clear */
#define TXD_LEN_MASK 0x7ffu

/* Emulated PHY: Davicom DM9161A, the part on the SAM9M10-G45-EK.  The
 * IEEE Clause-22 registers plus the Davicom vendor block (DSCR/DSCSR/
 * 10BTCSR/interrupt), so both the generic PHY driver and Linux's davicom
 * driver (CONFIG_DAVICOM_PHY, phy id 0x0181b8a0) are happy.  Vendor
 * registers are storage with datasheet reset values; DSCSR's operation
 * mode bits track the (always 100BASE-TX/full) negotiated link. */
#define MACB_PHY_CONTROL   0
#define MACB_PHY_STATUS    1
#define MACB_PHY_PHYID1    2
#define MACB_PHY_PHYID2    3
#define MACB_PHY_ANEGADV   4
#define MACB_PHY_LINKPABIL 5
#define MACB_PHY_ANER      6
#define DM9161_DSCR        16   /* Davicom specified configuration      */
#define DM9161_DSCSR       17   /* Davicom configuration and status     */
#define DM9161_10BTCSR     18   /* 10BASE-T configuration/status        */
#define DM9161_INTR        21   /* interrupt source/mask                */
#define PHY_CONTROL_RST    0x8000
#define PHY_CONTROL_LOOP   0x4000
#define PHY_CONTROL_ANEG   0x1000
#define PHY_CONTROL_ANRST  0x0200
#define PHY_STATUS_LINK    0x0004
#define PHY_STATUS_ANEGCMPL 0x0020
#define DSCSR_100FDX       0x8000
#define DSCSR_ANEG_DONE    0x0008

#define MACB_RX_BUFSIZE 128

struct AT91MacbState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t ncr;
    uint32_t ncfgr;
    uint32_t tsr;
    uint32_t rbqp;
    uint32_t tbqp;
    uint32_t rsr;
    uint32_t isr;
    uint32_t imr;      /* 1 = interrupt masked (disabled)          */
    uint32_t man;
    uint32_t hrb, hrt;
    uint32_t usrio;
    uint32_t sa[4][2]; /* specific-address bottom/top pairs         */
    uint32_t rx_desc;  /* next RX descriptor to fill (walks rbqp)    */
    uint32_t tx_desc;  /* next TX descriptor to send (walks tbqp)    */

    uint16_t phy_regs[32];
};

static void macb_update_irq(AT91MacbState *s)
{
    /* IMR bit set => masked, so an interrupt fires on (ISR & ~IMR). */
    qemu_set_irq(s->irq, !!(s->isr & ~s->imr));
}

static void macb_phy_update_link(AT91MacbState *s)
{
    if (qemu_get_queue(s->nic)->link_down) {
        s->phy_regs[MACB_PHY_STATUS] &= ~(PHY_STATUS_LINK | PHY_STATUS_ANEGCMPL);
        s->phy_regs[DM9161_DSCSR] = DSCSR_ANEG_DONE;
    } else {
        s->phy_regs[MACB_PHY_STATUS] |= (PHY_STATUS_LINK | PHY_STATUS_ANEGCMPL);
        s->phy_regs[DM9161_DSCSR] = DSCSR_100FDX | DSCSR_ANEG_DONE;
    }
}

static void macb_phy_reset(AT91MacbState *s)
{
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    s->phy_regs[MACB_PHY_CONTROL]   = 0x1140;  /* 100M, full duplex, aneg  */
    s->phy_regs[MACB_PHY_STATUS]    = 0x7969;  /* 100/10 caps, aneg able   */
    s->phy_regs[MACB_PHY_PHYID1]    = 0x0181;  /* Davicom DM9161A          */
    s->phy_regs[MACB_PHY_PHYID2]    = 0xb8a0;
    s->phy_regs[MACB_PHY_ANEGADV]   = 0x01e1;
    s->phy_regs[MACB_PHY_LINKPABIL] = 0xcde1;  /* link partner: 100/full   */
    s->phy_regs[MACB_PHY_ANER]      = 0x0001;  /* partner is aneg-able     */
    s->phy_regs[DM9161_DSCR]        = 0x0410;
    s->phy_regs[DM9161_10BTCSR]     = 0x7800;
    s->phy_regs[DM9161_INTR]        = 0x0f00;  /* all sources masked       */
    macb_phy_update_link(s);
}

static uint16_t macb_phy_read(AT91MacbState *s, unsigned reg)
{
    return s->phy_regs[reg & 0x1f];
}

static void macb_phy_write(AT91MacbState *s, unsigned reg, uint16_t val)
{
    reg &= 0x1f;
    if (reg == MACB_PHY_CONTROL) {
        if (val & PHY_CONTROL_RST) {
            macb_phy_reset(s);
            return;
        }
        if (val & PHY_CONTROL_ANEG) {
            /* Complete auto-negotiation immediately. */
            val &= ~(PHY_CONTROL_ANEG | PHY_CONTROL_ANRST);
        }
    }
    s->phy_regs[reg] = val;
}

/*
 * Transmit: walk the TX ring from TBQP.  Each buffer descriptor with the USED
 * bit clear is owned by the MAC; gather buffers until one carries the LAST bit,
 * transmit the assembled frame, then set USED on that frame's *first* buffer
 * descriptor (the driver only ever inspects the first one on reclaim).
 */
static void macb_do_tx(AT91MacbState *s)
{
    uint32_t desc = s->tx_desc;
    uint32_t frame_first = desc;
    uint8_t frame[2048];
    size_t frame_len = 0;
    bool in_frame = false;
    int guard = 0;

    if (!(s->ncr & NCR_TE)) {
        return;
    }

    while (guard++ < 1024) {
        uint32_t addr = address_space_ldl_le(&address_space_memory, desc,
                                             MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t ctrl = address_space_ldl_le(&address_space_memory, desc + 4,
                                             MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t len;

        if (ctrl & TXD_USED) {
            /* End of the queue (driver's used-bit sentinel). */
            s->tsr |= TSR_UBR;
            break;
        }
        if (!in_frame) {
            frame_first = desc;
            frame_len = 0;
            in_frame = true;
        }
        len = ctrl & TXD_LEN_MASK;
        if (frame_len + len <= sizeof(frame)) {
            address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, frame + frame_len, len);
            frame_len += len;
        }
        if (ctrl & TXD_LAST) {
            uint32_t fctrl;

            trace_at91_macb_tx(frame_len);
            qemu_send_packet(qemu_get_queue(s->nic), frame, frame_len);
            /* Hardware writes the USED bit back only on the frame's first
             * buffer descriptor; that is what the driver inspects on reclaim.
             */
            fctrl = address_space_ldl_le(&address_space_memory, frame_first + 4,
                                         MEMTXATTRS_UNSPECIFIED, NULL);
            address_space_stl_le(&address_space_memory, frame_first + 4,
                                 fctrl | TXD_USED, MEMTXATTRS_UNSPECIFIED, NULL);
            in_frame = false;
            s->isr |= INT_TCOMP;
        }
        desc = (ctrl & TXD_WRAP) ? s->tbqp : desc + 8;
    }

    /* Resume here on the next TSTART; the driver reuses the sentinel slot. */
    s->tx_desc = desc;
    macb_update_irq(s);
}

static bool macb_can_receive(NetClientState *nc)
{
    AT91MacbState *s = qemu_get_nic_opaque(nc);
    uint32_t addr;

    if (!(s->ncr & NCR_RE) || !s->rx_desc) {
        return false;
    }
    addr = address_space_ldl_le(&address_space_memory, s->rx_desc,
                                MEMTXATTRS_UNSPECIFIED, NULL);
    return !(addr & RXD_USED);
}

static ssize_t macb_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    AT91MacbState *s = qemu_get_nic_opaque(nc);
    const uint8_t *p = buf;
    size_t remaining = size;
    uint32_t desc = s->rx_desc;
    bool first = true;

    if (!(s->ncr & NCR_RE) || !desc) {
        /* RX disabled: swallow the frame like the real MAC.  Returning an
         * error here would make slirp treat the whole link as dead. */
        return size;
    }

    do {
        uint32_t addr = address_space_ldl_le(&address_space_memory, desc,
                                             MEMTXATTRS_UNSPECIFIED, NULL);
        uint32_t ctrl = 0;
        /* RBOF (NCFGR[15:14]): the frame starts this many bytes into the
         * first buffer so the driver can word-align the IP header; only the
         * first buffer carries the offset, subsequent ones start at 0.
         */
        uint32_t bufoff = first ? ((s->ncfgr >> 14) & 0x3) : 0;
        size_t space = MACB_RX_BUFSIZE - bufoff;
        size_t n;

        if (addr & RXD_USED) {
            /* No free buffer (can_receive only sees the first descriptor,
             * so a multi-buffer frame can still run out mid-frame): report
             * buffer-not-available and DROP the frame, like the hardware.
             * The frame counts as consumed - an error return would wedge
             * the peer (slirp already ate the bytes and never retries),
             * whereas a drop lets TCP retransmission recover.
             *
             * Reception resumes at THIS descriptor, not at the dropped
             * frame's first one: the driver's discard_partial_frame()
             * consumes the fragment's descriptors and expects the next
             * frame beyond them - rewinding desynchronizes the model's
             * write position from the driver's read position and the
             * guest goes permanently deaf. */
            s->rx_desc = desc;
            s->rsr |= RSR_BNA;
            s->isr |= INT_RXUBR;
            macb_update_irq(s);
            return size;
        }
        n = remaining < space ? remaining : space;
        if (n) {
            address_space_write(&address_space_memory, (addr & ~0x3u) + bufoff,
                                MEMTXATTRS_UNSPECIFIED, p, n);
        }
        if (first) {
            ctrl |= RXD_SOF;
        }
        remaining -= n;
        p += n;
        if (remaining == 0) {
            ctrl |= RXD_EOF | ((uint32_t)size & 0x1fff);
        }
        address_space_stl_le(&address_space_memory, desc + 4, ctrl,
                             MEMTXATTRS_UNSPECIFIED, NULL);
        address_space_stl_le(&address_space_memory, desc, addr | RXD_USED,
                             MEMTXATTRS_UNSPECIFIED, NULL);
        desc = (addr & RXD_WRAP) ? s->rbqp : desc + 8;
        first = false;
    } while (remaining > 0);

    s->rx_desc = desc;
    s->rsr |= RSR_REC;
    s->isr |= INT_RCOMP;
    macb_update_irq(s);
    trace_at91_macb_rx(size);
    return size;
}

static void macb_set_link(NetClientState *nc)
{
    AT91MacbState *s = qemu_get_nic_opaque(nc);

    macb_phy_update_link(s);
    macb_update_irq(s);
}

static uint64_t macb_read(void *opaque, hwaddr off, unsigned size)
{
    AT91MacbState *s = opaque;
    uint32_t v;

    switch (off) {
    case MACB_NCR:    return s->ncr;
    case MACB_NCFGR:  return s->ncfgr;
    case MACB_NSR:    return NSR_IDLE | NSR_MDIO;
    case MACB_TSR:    return s->tsr;
    case MACB_RBQP:   return s->rbqp;
    case MACB_TBQP:   return s->tbqp;
    case MACB_RSR:    return s->rsr;
    case MACB_ISR:                       /* clear-on-read */
        v = s->isr;
        s->isr = 0;
        macb_update_irq(s);
        return v;
    case MACB_IMR:    return s->imr;
    case MACB_MAN:    return s->man;
    case MACB_HRB:    return s->hrb;
    case MACB_HRT:    return s->hrt;
    case MACB_USRIO:  return s->usrio;
    case MACB_MID:    return 0;          /* IDNUM 0 (<2) => classic macb */
    default:
        if (off >= MACB_SA1B && off <= MACB_SA4T) {
            unsigned i = (off - MACB_SA1B) >> 2;
            return s->sa[i >> 1][i & 1];
        }
        return 0;                        /* statistics counters read 0 */
    }
}

static void macb_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    AT91MacbState *s = opaque;

    switch (off) {
    case MACB_NCR:
        if ((val & NCR_RE) && !(s->ncr & NCR_RE)) {
            s->rx_desc = s->rbqp;
        }
        s->ncr = val & ~NCR_TSTART;      /* TSTART is self-clearing */
        if (val & NCR_TSTART) {
            macb_do_tx(s);
        }
        if (val & NCR_RE) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case MACB_NCFGR:  s->ncfgr = val; break;
    case MACB_TSR:    s->tsr &= ~(uint32_t)val; break;      /* write-1-clear */
    case MACB_RBQP:
        s->rbqp = val & ~0x3u;
        s->rx_desc = s->rbqp;
        break;
    case MACB_TBQP:
        s->tbqp = val & ~0x3u;
        s->tx_desc = s->tbqp;
        break;
    case MACB_RSR:
        s->rsr &= ~(uint32_t)val;                          /* write-1-clear */
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case MACB_ISR:    s->isr &= ~(uint32_t)val; macb_update_irq(s); break;
    case MACB_IER:
        s->imr &= ~(uint32_t)val;
        macb_update_irq(s);
        /* The driver re-enables RX interrupts at the end of its NAPI poll
         * cycle, right after refilling the RX ring - the ring refill
         * itself is pure memory traffic we cannot observe, so this is the
         * hook to resume delivery of queued packets (can_receive() went
         * false while the ring was full). */
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        break;
    case MACB_IDR:    s->imr |= (uint32_t)val;  macb_update_irq(s); break;
    case MACB_MAN: {
        unsigned op   = (val >> 28) & 0x3;   /* 10=read, 01=write */
        unsigned phya = (val >> 23) & 0x1f;
        unsigned rega = (val >> 18) & 0x1f;
        trace_at91_macb_mdio(op, rega, val & 0xffff);
        if (phya != 0) {
            /* Single PHY on the board (DM9161A at address 0); the mdio
             * bus scan of the other 31 addresses reads an idle bus,
             * which floats high. */
            s->man = (op == 2) ? ((val & 0xffff0000u) | 0xffff) : val;
        } else if (op == 2) {
            s->man = (val & 0xffff0000u) | macb_phy_read(s, rega);
        } else {
            if (op == 1) {
                macb_phy_write(s, rega, val & 0xffff);
            }
            s->man = val;
        }
        s->isr |= INT_MFD;
        macb_update_irq(s);
        break;
    }
    case MACB_HRB:    s->hrb = val; break;
    case MACB_HRT:    s->hrt = val; break;
    case MACB_USRIO:  s->usrio = val; break;
    default:
        if (off >= MACB_SA1B && off <= MACB_SA4T) {
            unsigned i = (off - MACB_SA1B) >> 2;
            s->sa[i >> 1][i & 1] = val;
        }
        break;
    }
}

static const MemoryRegionOps macb_ops = {
    .read = macb_read,
    .write = macb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void macb_reset(DeviceState *dev)
{
    AT91MacbState *s = AT91_MACB(dev);

    s->ncr = s->ncfgr = s->tsr = s->rsr = s->isr = 0;
    s->rbqp = s->tbqp = s->rx_desc = s->tx_desc = 0;
    s->man = s->hrb = s->hrt = s->usrio = 0;
    s->imr = 0xffffffff;                 /* all interrupts masked (disabled) */
    memset(s->sa, 0, sizeof(s->sa));
    macb_phy_reset(s);
    macb_update_irq(s);
}

static NetClientInfo net_macb_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = macb_can_receive,
    .receive = macb_receive,
    .link_status_changed = macb_set_link,
};

static void macb_realize(DeviceState *dev, Error **errp)
{
    AT91MacbState *s = AT91_MACB(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_macb_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void macb_dev_init(Object *obj)
{
    AT91MacbState *s = AT91_MACB(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &macb_ops, s, "at91-macb", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const Property macb_properties[] = {
    DEFINE_NIC_PROPERTIES(AT91MacbState, conf),
};

static const VMStateDescription vmstate_at91_macb = {
    .name = "at91-macb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ncr, AT91MacbState),
        VMSTATE_UINT32(ncfgr, AT91MacbState),
        VMSTATE_UINT32(tsr, AT91MacbState),
        VMSTATE_UINT32(rbqp, AT91MacbState),
        VMSTATE_UINT32(tbqp, AT91MacbState),
        VMSTATE_UINT32(rsr, AT91MacbState),
        VMSTATE_UINT32(isr, AT91MacbState),
        VMSTATE_UINT32(imr, AT91MacbState),
        VMSTATE_UINT32(man, AT91MacbState),
        VMSTATE_UINT32(hrb, AT91MacbState),
        VMSTATE_UINT32(hrt, AT91MacbState),
        VMSTATE_UINT32(usrio, AT91MacbState),
        VMSTATE_UINT32(rx_desc, AT91MacbState),
        VMSTATE_UINT32(tx_desc, AT91MacbState),
        VMSTATE_UINT32_2DARRAY(sa, AT91MacbState, 4, 2),
        VMSTATE_UINT16_ARRAY(phy_regs, AT91MacbState, 32),
        VMSTATE_END_OF_LIST()
    }
};

static void macb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = macb_realize;
    device_class_set_legacy_reset(dc, macb_reset);
    device_class_set_props(dc, macb_properties);
    dc->vmsd = &vmstate_at91_macb;
}

static const TypeInfo macb_type = {
    .name = TYPE_AT91_MACB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AT91MacbState),
    .instance_init = macb_dev_init,
    .class_init = macb_class_init,
};

static void at91_macb_register_types(void)
{
    type_register_static(&macb_type);
}

type_init(at91_macb_register_types)
