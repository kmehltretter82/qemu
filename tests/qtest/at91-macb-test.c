/*
 * QTest tests for the AT91 classic EMAC ("macb") descriptor rings.
 *
 * A socketpair netdev carries the frames: the test injects RX frames as
 * 4-byte big-endian length-prefixed records and reads TX frames the same
 * way, so descriptor state can be compared against the exact bytes on
 * the wire.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define G45_MACB_BASE          0xfffbc000
#define G45_SDRAM_BASE         0x70000000

#define MACB_NCR               0x00
#define MACB_NCFGR             0x04
#define MACB_TSR               0x14
#define MACB_RBQP              0x18
#define MACB_TBQP              0x1c
#define MACB_RSR               0x20
#define MACB_ISR               0x24
#define MACB_IER               0x28
#define MACB_MID               0xfc

#define NCR_RE                 (1u << 2)
#define NCR_TE                 (1u << 3)
#define NCR_TSTART             (1u << 9)
#define NCFGR_CAF              (1u << 4)
#define NCFGR_RBOF_2           (2u << 14)   /* two-byte RX offset */
#define RSR_BNA                (1u << 0)
#define RSR_REC                (1u << 1)
#define INT_RCOMP              (1u << 1)
#define INT_TCOMP              (1u << 7)

#define RXD_USED               (1u << 0)
#define RXD_WRAP               (1u << 1)
#define RXD_SOF                (1u << 14)
#define RXD_EOF                (1u << 15)
#define RXD_LEN_MASK           0x7ffu

#define TXD_LAST               (1u << 15)
#define TXD_WRAP               (1u << 30)
#define TXD_USED               (1u << 31)

/* One max-size frame's worth of free descriptors gates can_receive(). */
#define RX_RING_DESCS          16
#define RX_BUF_SIZE            128

static void send_frame(int fd, const uint8_t *frame, size_t length)
{
    uint32_t prefix = htonl(length);
    ssize_t ret;

    ret = send(fd, &prefix, sizeof(prefix), 0);
    g_assert_cmpint(ret, ==, sizeof(prefix));
    ret = send(fd, frame, length, 0);
    g_assert_cmpint(ret, ==, length);
}

static void recv_exactly(int fd, uint8_t *buffer, size_t length)
{
    size_t received = 0;

    while (received < length) {
        ssize_t ret = recv(fd, buffer + received, length - received, 0);

        g_assert_cmpint(ret, >, 0);
        received += ret;
    }
}

static size_t recv_frame(int fd, uint8_t *frame, size_t max)
{
    uint32_t prefix;

    recv_exactly(fd, (uint8_t *)&prefix, sizeof(prefix));
    prefix = ntohl(prefix);
    g_assert_cmpuint(prefix, <=, max);
    recv_exactly(fd, frame, prefix);
    return prefix;
}

static const char *macb_machine = "sam9m10g45ek";
static uint64_t macb_base = G45_MACB_BASE;
static uint64_t macb_sdram = G45_SDRAM_BASE;

static QTestState *macb_start(int *fd)
{
    int pair[2];
    QTestState *qts;

    g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, pair), ==, 0);
    qts = qtest_initf("-machine %s "
                      "-nic socket,fd=%d,model=at91-macb,"
                      "mac=52:54:00:12:34:56 -S", macb_machine, pair[1]);
    close(pair[1]);
    *fd = pair[0];
    qtest_qmp_assert_success(qts, "{ 'execute': 'cont' }");
    g_assert_cmphex(qtest_readl(qts, macb_base + MACB_MID) & 0xffff0000,
                    !=, 0xffff0000);
    return qts;
}

static void build_frame(uint8_t *frame, size_t length, uint8_t seed)
{
    size_t i;

    memset(frame, 0xff, 6);                    /* broadcast destination */
    for (i = 6; i < 12; i++) {
        frame[i] = 0x40 + i;
    }
    frame[12] = 0x08;
    frame[13] = 0x00;
    for (i = 14; i < length; i++) {
        frame[i] = (uint8_t)(i * 7 + seed);
    }
}

/*
 * The TX ring pointer persists across TSTART writes: after two frames
 * complete, a third queued behind the all-USED sentinel must transmit
 * from where the ring stopped, not from TBQP.  Every frame's USED
 * writeback lands on its (single) first descriptor.
 */
static void test_macb_tx_ring_persist(void)
{
    const uint32_t ring = macb_sdram + 0x150000;
    const uint32_t buf0 = macb_sdram + 0x151000;
    const uint32_t buf1 = macb_sdram + 0x151100;
    const uint32_t buf2 = macb_sdram + 0x151200;
    uint8_t frame[128], wire[256];
    QTestState *qts;
    size_t got;
    int fd;

    qts = macb_start(&fd);

    build_frame(frame, 60, 0x11);
    qtest_bufwrite(qts, buf0, frame, 60);
    build_frame(frame, 64, 0x22);
    qtest_bufwrite(qts, buf1, frame, 64);

    qtest_writel(qts, ring + 0, buf0);
    qtest_writel(qts, ring + 4, 60 | TXD_LAST);
    qtest_writel(qts, ring + 8, buf1);
    qtest_writel(qts, ring + 12, 64 | TXD_LAST);
    qtest_writel(qts, ring + 16, 0);
    qtest_writel(qts, ring + 20, TXD_USED);     /* sentinel */
    qtest_writel(qts, ring + 24, 0);
    qtest_writel(qts, ring + 28, TXD_USED | TXD_WRAP);

    qtest_writel(qts, macb_base + MACB_TBQP, ring);
    qtest_writel(qts, macb_base + MACB_NCR, NCR_TE);
    qtest_writel(qts, macb_base + MACB_NCR, NCR_TE | NCR_TSTART);

    build_frame(frame, 60, 0x11);
    got = recv_frame(fd, wire, sizeof(wire));
    g_assert_cmpuint(got, ==, 60);
    g_assert_cmpmem(wire, got, frame, 60);
    build_frame(frame, 64, 0x22);
    got = recv_frame(fd, wire, sizeof(wire));
    g_assert_cmpuint(got, ==, 64);
    g_assert_cmpmem(wire, got, frame, 64);

    g_assert_cmphex(qtest_readl(qts, ring + 4) & TXD_USED, ==, TXD_USED);
    g_assert_cmphex(qtest_readl(qts, ring + 12) & TXD_USED, ==, TXD_USED);
    g_assert_cmphex(qtest_readl(qts, macb_base + MACB_ISR) & INT_TCOMP,
                    ==, INT_TCOMP);

    /* Queue a third frame at the stop position; TSTART resumes there. */
    build_frame(frame, 72, 0x33);
    qtest_bufwrite(qts, buf2, frame, 72);
    qtest_writel(qts, ring + 16, buf2);
    qtest_writel(qts, ring + 20, 72 | TXD_LAST);
    qtest_writel(qts, macb_base + MACB_NCR, NCR_TE | NCR_TSTART);
    got = recv_frame(fd, wire, sizeof(wire));
    g_assert_cmpuint(got, ==, 72);
    g_assert_cmpmem(wire, got, frame, 72);
    g_assert_cmphex(qtest_readl(qts, ring + 20) & TXD_USED, ==, TXD_USED);

    close(fd);
    qtest_quit(qts);
}

static void macb_setup_rx_ring(QTestState *qts, uint32_t ring,
                               uint32_t bufs)
{
    int i;

    for (i = 0; i < RX_RING_DESCS; i++) {
        uint32_t flags = (i == RX_RING_DESCS - 1) ? RXD_WRAP : 0;

        qtest_writel(qts, ring + 8 * i, (bufs + RX_BUF_SIZE * i) | flags);
        qtest_writel(qts, ring + 8 * i + 4, 0);
        qtest_memset(qts, bufs + RX_BUF_SIZE * i, 0xcc, RX_BUF_SIZE);
    }
    qtest_writel(qts, macb_base + MACB_RBQP, ring);
}

/* Poll until the descriptor's USED bit appears (delivery is async). */
static uint32_t wait_rx_used(QTestState *qts, uint32_t desc)
{
    int i;

    for (i = 0; i < 5000; i++) {
        uint32_t addr = qtest_readl(qts, desc);

        if (addr & RXD_USED) {
            return qtest_readl(qts, desc + 4);
        }
        g_usleep(1000);
    }
    g_assert_not_reached();
}

/*
 * RX frames land at the two-byte RBOF offset with SOF/EOF and exact
 * length, and the ring wraps - exercised driver-style, freeing each
 * descriptor and rewriting IER after every frame.  Then the ring is
 * deliberately left un-freed: the model's whole-max-frame preflight
 * (MACB_RX_MAX_DESC free descriptors) makes the backend queue the next
 * frame, and the driver's free-plus-IER sequence must flush it - the
 * refill contract from the Linux bring-up.
 */
static void test_macb_rx_wrap_rbof_refill(void)
{
    const uint32_t ring = macb_sdram + 0x152000;
    const uint32_t bufs = macb_sdram + 0x153000;
    uint8_t frame[64], readback[64];
    QTestState *qts;
    uint32_t status;
    int position = 0;
    int fd;
    int i;

    qts = macb_start(&fd);
    macb_setup_rx_ring(qts, ring, bufs);
    qtest_writel(qts, macb_base + MACB_NCFGR,
                 NCFGR_CAF | NCFGR_RBOF_2);
    qtest_writel(qts, macb_base + MACB_NCR, NCR_RE);

    /* Wrap coverage: eighteen frames, freeing after each delivery. */
    for (i = 0; i < RX_RING_DESCS + 2; i++) {
        uint32_t desc = ring + 8 * position;

        build_frame(frame, 60, (uint8_t)i);
        send_frame(fd, frame, 60);
        status = wait_rx_used(qts, desc);
        g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==,
                        RXD_SOF | RXD_EOF);
        g_assert_cmpuint(status & RXD_LEN_MASK, ==, 60);
        qtest_memread(qts, bufs + RX_BUF_SIZE * position + 2,
                      readback, 60);
        g_assert_cmpmem(readback, 60, frame, 60);

        qtest_writel(qts, desc,
                     qtest_readl(qts, desc) & ~(uint32_t)RXD_USED);
        qtest_writel(qts, desc + 4, 0);
        qtest_writel(qts, macb_base + MACB_IER, INT_RCOMP);
        position = (position + 1) % RX_RING_DESCS;
    }
    g_assert_cmphex(qtest_readl(qts, macb_base + MACB_RSR) & RSR_REC,
                    ==, RSR_REC);

    /*
     * Starve the ring: five un-freed frames leave fewer than
     * MACB_RX_MAX_DESC (12 of 16) free descriptors, so the sixth frame
     * queues in the backend rather than being delivered or dropped.
     */
    for (i = 0; i < 5; i++) {
        build_frame(frame, 60, (uint8_t)(0x50 + i));
        send_frame(fd, frame, 60);
        (void)wait_rx_used(qts, ring + 8 * ((position + i) %
                                            RX_RING_DESCS));
    }
    build_frame(frame, 60, 0xa5);
    send_frame(fd, frame, 60);
    g_usleep(100000);
    g_assert_cmphex(qtest_readl(qts, ring +
                                8 * ((position + 5) % RX_RING_DESCS)) &
                    RXD_USED, ==, 0);

    /* Free everything and rewrite IER: the queued frame must land. */
    for (i = 0; i < RX_RING_DESCS; i++) {
        uint32_t addr = qtest_readl(qts, ring + 8 * i);

        qtest_writel(qts, ring + 8 * i, addr & ~(uint32_t)RXD_USED);
        qtest_writel(qts, ring + 8 * i + 4, 0);
    }
    qtest_writel(qts, macb_base + MACB_IER, INT_RCOMP);
    position = (position + 5) % RX_RING_DESCS;
    status = wait_rx_used(qts, ring + 8 * position);
    g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==, RXD_SOF | RXD_EOF);
    qtest_memread(qts, bufs + RX_BUF_SIZE * position + 2, readback, 60);
    g_assert_cmpmem(readback, 60, frame, 60);

    close(fd);
    qtest_quit(qts);
}

/*
 * A 300-byte frame spans three 128-byte buffers: SOF only on the first
 * descriptor, EOF and the total length only on the last, data split at
 * the RBOF-adjusted boundaries.
 */
static void test_macb_rx_multibuffer_frame(void)
{
    const uint32_t ring = macb_sdram + 0x154000;
    const uint32_t bufs = macb_sdram + 0x155000;
    uint8_t frame[300], readback[300];
    QTestState *qts;
    uint32_t status;
    int fd;

    qts = macb_start(&fd);
    macb_setup_rx_ring(qts, ring, bufs);
    qtest_writel(qts, macb_base + MACB_NCFGR,
                 NCFGR_CAF | NCFGR_RBOF_2);
    qtest_writel(qts, macb_base + MACB_NCR, NCR_RE);

    build_frame(frame, sizeof(frame), 0x77);
    send_frame(fd, frame, sizeof(frame));

    status = wait_rx_used(qts, ring + 0);
    g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==, RXD_SOF);
    status = wait_rx_used(qts, ring + 8);
    g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==, 0);
    status = wait_rx_used(qts, ring + 16);
    g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==, RXD_EOF);
    g_assert_cmpuint(status & RXD_LEN_MASK, ==, sizeof(frame));

    /* First buffer holds 126 bytes after the offset, the rest follow. */
    qtest_memread(qts, bufs + 2, readback, RX_BUF_SIZE - 2);
    qtest_memread(qts, bufs + RX_BUF_SIZE, readback + RX_BUF_SIZE - 2,
                  RX_BUF_SIZE);
    qtest_memread(qts, bufs + 2 * RX_BUF_SIZE,
                  readback + 2 * RX_BUF_SIZE - 2,
                  sizeof(frame) - (2 * RX_BUF_SIZE - 2));
    g_assert_cmpmem(readback, sizeof(frame), frame, sizeof(frame));

    close(fd);
    qtest_quit(qts);
}

/*
 * A frame larger than the free ring capacity is dropped whole by the
 * preflight - BNA raised, the ring untouched, and the next normal frame
 * delivered at the expected position.  This is the safety net that
 * replaced mid-frame SOF-without-EOF fragments (the Linux macb driver
 * spins forever on those).
 */
static void test_macb_rx_oversize_preflight_drop(void)
{
    const uint32_t ring = macb_sdram + 0x156000;
    const uint32_t bufs = macb_sdram + 0x157000;
    uint8_t frame[1600], readback[64];
    QTestState *qts;
    uint32_t status;
    int fd;
    int i;

    qts = macb_start(&fd);
    macb_setup_rx_ring(qts, ring, bufs);
    /*
     * Pre-consume the LAST four descriptors: can_receive's 12-descriptor
     * window from the ring head stays free, but a 13-buffer frame hits
     * the used tail inside the delivery preflight.
     */
    for (i = RX_RING_DESCS - 4; i < RX_RING_DESCS; i++) {
        qtest_writel(qts, ring + 8 * i,
                     qtest_readl(qts, ring + 8 * i) | RXD_USED);
    }
    qtest_writel(qts, macb_base + MACB_NCFGR, NCFGR_CAF);
    qtest_writel(qts, macb_base + MACB_NCR, NCR_RE);

    /* Needs 13 buffers; only 12 are free: dropped whole with BNA. */
    build_frame(frame, sizeof(frame), 0x99);
    send_frame(fd, frame, sizeof(frame));
    g_usleep(100000);
    g_assert_cmphex(qtest_readl(qts, macb_base + MACB_RSR) & RSR_BNA,
                    ==, RSR_BNA);
    for (i = 0; i < RX_RING_DESCS - 4; i++) {
        g_assert_cmphex(qtest_readl(qts, ring + 8 * i) & RXD_USED, ==, 0);
    }

    /* The ring still works: a normal frame lands at the ring head. */
    build_frame(frame, 60, 0xab);
    send_frame(fd, frame, 60);
    status = wait_rx_used(qts, ring + 0);
    g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==, RXD_SOF | RXD_EOF);
    qtest_memread(qts, bufs, readback, 60);
    g_assert_cmpmem(readback, 60, frame, 60);

    close(fd);
    qtest_quit(qts);
}

static void wait_for_migration_complete(QTestState *qts)
{
    while (true) {
        QDict *response = qtest_qmp(qts,
                                    "{ 'execute': 'query-migrate' }");
        QDict *result = qdict_get_qdict(response, "return");
        const char *status = qdict_get_str(result, "status");
        bool complete = !strcmp(status, "completed");

        g_assert_cmpstr(status, !=, "failed");
        g_assert_cmpstr(status, !=, "cancelled");
        qobject_unref(response);
        if (complete) {
            return;
        }
        g_usleep(1000);
    }
}

/* Migrate mid-ring: the write position and ring state must continue. */
static void test_macb_rx_ring_migration(void)
{
    const uint32_t ring = macb_sdram + 0x158000;
    const uint32_t bufs = macb_sdram + 0x159000;
    g_autofree char *state_path = NULL;
    g_autofree char *uri = NULL;
    g_autofree char *dst_args = NULL;
    uint8_t frame[64], readback[64];
    QTestState *sqts, *dqts;
    uint32_t status;
    int sfd, dfd, state_fd;
    int i;

    state_fd = g_file_open_tmp("at91-macb-migration-XXXXXX",
                               &state_path, NULL);
    g_assert_cmpint(state_fd, >=, 0);
    close(state_fd);
    uri = g_strdup_printf("file:%s", state_path);

    sqts = macb_start(&sfd);
    macb_setup_rx_ring(sqts, ring, bufs);
    qtest_writel(sqts, macb_base + MACB_NCFGR,
                 NCFGR_CAF | NCFGR_RBOF_2);
    qtest_writel(sqts, macb_base + MACB_NCR, NCR_RE);
    for (i = 0; i < 3; i++) {
        build_frame(frame, 60, (uint8_t)i);
        send_frame(sfd, frame, 60);
        (void)wait_rx_used(sqts, ring + 8 * i);
    }

    qtest_qmp_assert_success(sqts,
        "{ 'execute': 'migrate', 'arguments': { 'uri': %s } }", uri);
    wait_for_migration_complete(sqts);
    qtest_quit(sqts);
    close(sfd);

    {
        int pair[2];

        g_assert_cmpint(socketpair(PF_UNIX, SOCK_STREAM, 0, pair), ==, 0);
        dst_args = g_strdup_printf(
            "-machine sam9m10g45ek -nic socket,fd=%d,model=at91-macb,"
            "mac=52:54:00:12:34:56 -S -incoming %s", pair[1], uri);
        dqts = qtest_init(dst_args);
        close(pair[1]);
        dfd = pair[0];
    }
    wait_for_migration_complete(dqts);
    qtest_qmp_assert_success(dqts, "{ 'execute': 'cont' }");

    /* Ring contents and position survived; delivery continues at 3. */
    for (i = 0; i < 3; i++) {
        g_assert_cmphex(qtest_readl(dqts, ring + 8 * i) & RXD_USED, ==,
                        RXD_USED);
    }
    build_frame(frame, 60, 0x44);
    send_frame(dfd, frame, 60);
    status = wait_rx_used(dqts, ring + 8 * 3);
    g_assert_cmphex(status & (RXD_SOF | RXD_EOF), ==, RXD_SOF | RXD_EOF);
    qtest_memread(dqts, bufs + RX_BUF_SIZE * 3 + 2, readback, 60);
    g_assert_cmpmem(readback, 60, frame, 60);

    close(dfd);
    qtest_quit(dqts);
    unlink(state_path);
}

/* The same TX ring persistence contract on the SAM9x5 board. */
static void test_g35_tx_ring_persist(void)
{
    macb_machine = "sam9g35ek";
    macb_base = 0xf802c000;
    macb_sdram = 0x20000000;
    test_macb_tx_ring_persist();
    macb_machine = "sam9m10g45ek";
    macb_base = G45_MACB_BASE;
    macb_sdram = G45_SDRAM_BASE;
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/at91-macb/tx-ring-persist", test_macb_tx_ring_persist);
    qtest_add_func("/at91-macb/g35/tx-ring-persist",
                   test_g35_tx_ring_persist);
    qtest_add_func("/at91-macb/rx-wrap-rbof-refill",
                   test_macb_rx_wrap_rbof_refill);
    qtest_add_func("/at91-macb/rx-multibuffer-frame",
                   test_macb_rx_multibuffer_frame);
    qtest_add_func("/at91-macb/rx-oversize-preflight-drop",
                   test_macb_rx_oversize_preflight_drop);
    qtest_add_func("/at91-macb/rx-ring-migration",
                   test_macb_rx_ring_migration);

    return g_test_run();
}
