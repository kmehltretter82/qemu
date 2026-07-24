/*
 * Network payload commands for the AT91SAM9G45 RT-Thread tests.  Built
 * only in lwip-enabled profiles; the baseline image compiles this file
 * to nothing.
 *
 * g45tcp <ip> <port> <bytes>: stream a seeded xorshift byte pattern to
 * a host-side echo server and verify every echoed byte.  Exercises
 * bidirectional segmented TCP through the MACB model.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <rtthread.h>

#ifdef RT_USING_LWIP

#include <lwip/sockets.h>
#include <stdlib.h>

#define G45NET_CHUNK 1472   /* max UDP payload at the 1500-byte MTU */

static rt_uint32_t g45net_prng(rt_uint32_t *state)
{
    rt_uint32_t value = *state;

    if (value == 0U) {
        value = 0x6d2b79f5U;
    }
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

static void g45net_fill(rt_uint8_t *buf, rt_uint32_t len, rt_uint32_t *state)
{
    rt_uint32_t i;

    for (i = 0; i < len; i++) {
        buf[i] = (rt_uint8_t)(g45net_prng(state) >> 24);
    }
}

/* Off the FinSH thread's small stack: overflowing it corrupts the
 * adjacent heap (seen as an rt_smem_free pool assertion). */
static rt_uint8_t txbuf[G45NET_CHUNK];
static rt_uint8_t rxbuf[G45NET_CHUNK];

static int g45tcp(int argc, char **argv)
{
    struct sockaddr_in addr;
    rt_uint32_t tx_state = 0x7cb1e523U, rx_state = 0x7cb1e523U;
    rt_uint32_t total, sent = 0, rcvd = 0, mismatch = 0;
    int sock, port;

    if (argc != 4) {
        rt_kprintf("usage: g45tcp <ip> <port> <bytes>\n");
        return -1;
    }
    port = atoi(argv[2]);
    total = (rt_uint32_t)atoi(argv[3]);

    sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        rt_kprintf("G45NET TCP FAIL socket\n");
        return -1;
    }
    rt_memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((rt_uint16_t)port);
    addr.sin_addr.s_addr = inet_addr(argv[1]);
    if (lwip_connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        rt_kprintf("G45NET TCP FAIL connect\n");
        lwip_close(sock);
        return -1;
    }

    /*
     * Strict lockstep: send one chunk, then blocking-receive the whole
     * echo of that chunk before the next.  At most one chunk is ever in
     * flight, which keeps both windows comfortable and needs no
     * non-blocking receives.
     */
    while (sent < total && mismatch == 0U) {
        rt_uint32_t n = total - sent;
        rt_uint32_t back = 0;

        if (n > G45NET_CHUNK) {
            n = G45NET_CHUNK;
        }
        g45net_fill(txbuf, n, &tx_state);
        if (lwip_send(sock, txbuf, n, 0) != (int)n) {
            rt_kprintf("G45NET TCP FAIL send at %u\n", sent);
            lwip_close(sock);
            return -1;
        }
        sent += n;
        while (back < n && mismatch == 0U) {
            int got = lwip_recv(sock, rxbuf, n - back, 0);
            rt_uint32_t i;

            if (got <= 0) {
                rt_kprintf("G45NET TCP FAIL recv at %u\n", rcvd);
                lwip_close(sock);
                return -1;
            }
            for (i = 0; i < (rt_uint32_t)got; i++) {
                if (rxbuf[i] !=
                    (rt_uint8_t)(g45net_prng(&rx_state) >> 24)) {
                    mismatch = rcvd + i + 1U;
                    break;
                }
            }
            back += (rt_uint32_t)got;
            rcvd += (rt_uint32_t)got;
        }
    }
    lwip_close(sock);

    if (mismatch != 0U || rcvd != total) {
        rt_kprintf("G45NET TCP FAIL rcvd=%u mismatch=%u\n", rcvd, mismatch);
        return -1;
    }
    rt_kprintf("G45NET TCP OK bytes=%u\n", rcvd);
    return 0;
}
MSH_CMD_EXPORT(g45tcp, stream seeded TCP payload to an echo server);

/*
 * g45udp <ip> <port> <size> <count>: echo <count> datagrams of exactly
 * <size> bytes and verify each one - a frame-size bisection tool for
 * the MACB multi-buffer RX path.
 */
static int g45udp(int argc, char **argv)
{
    struct sockaddr_in addr;
    rt_uint32_t state = 0x11a7b09dU, check;
    rt_uint32_t size, count, i, ok = 0;
    struct timeval tv;
    int sock;

    if (argc != 5) {
        rt_kprintf("usage: g45udp <ip> <port> <size> <count>\n");
        return -1;
    }
    size = (rt_uint32_t)atoi(argv[3]);
    count = (rt_uint32_t)atoi(argv[4]);
    if (size > G45NET_CHUNK) {
        size = G45NET_CHUNK;
    }

    sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        rt_kprintf("G45NET UDP FAIL socket\n");
        return -1;
    }
    tv.tv_sec = 3;
    tv.tv_usec = 0;
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    rt_memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((rt_uint16_t)atoi(argv[2]));
    addr.sin_addr.s_addr = inet_addr(argv[1]);

    for (i = 0; i < count; i++) {
        rt_uint32_t j;
        int n;

        check = state;
        g45net_fill(txbuf, size, &state);
        if (lwip_sendto(sock, txbuf, size, 0,
                        (struct sockaddr *)&addr, sizeof(addr))
            != (int)size) {
            break;
        }
        n = lwip_recv(sock, rxbuf, sizeof(rxbuf), 0);
        if (n != (int)size) {
            rt_kprintf("G45NET UDP FAIL dgram=%u n=%d\n", i, n);
            lwip_close(sock);
            return -1;
        }
        g45net_fill(txbuf, size, &check);   /* regenerate for compare */
        for (j = 0; j < size; j++) {
            if (rxbuf[j] != txbuf[j]) {
                rt_kprintf("G45NET UDP FAIL dgram=%u off=%u\n", i, j);
                lwip_close(sock);
                return -1;
            }
        }
        ok++;
    }
    lwip_close(sock);
    if (ok != count) {
        rt_kprintf("G45NET UDP FAIL ok=%u\n", ok);
        return -1;
    }
    rt_kprintf("G45NET UDP OK size=%u count=%u\n", size, count);
    return 0;
}
MSH_CMD_EXPORT(g45udp, echo seeded datagrams of an exact size);

/*
 * g45udpflood <ip> <port> <size> <count>: send every datagram
 * back-to-back, then collect the echoes.  Unlike the lockstep g45udp
 * this keeps transmit and receive genuinely concurrent through the
 * driver - the traffic shape TCP produces - without any TCP code.
 * Echo order is preserved (single slirp UDP flow), so contents are
 * still verified sequentially.
 */
static int g45udpflood(int argc, char **argv)
{
    struct sockaddr_in addr;
    rt_uint32_t state = 0x33cc55aaU, check = 0x33cc55aaU;
    rt_uint32_t size, count, i, got_total = 0;
    struct timeval tv;
    int sock;

    if (argc != 5) {
        rt_kprintf("usage: g45udpflood <ip> <port> <size> <count>\n");
        return -1;
    }
    size = (rt_uint32_t)atoi(argv[3]);
    count = (rt_uint32_t)atoi(argv[4]);
    if (size > G45NET_CHUNK) {
        size = G45NET_CHUNK;
    }

    sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        rt_kprintf("G45NET FLOOD FAIL socket\n");
        return -1;
    }
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    rt_memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((rt_uint16_t)atoi(argv[2]));
    addr.sin_addr.s_addr = inet_addr(argv[1]);

    for (i = 0; i < count; i++) {
        g45net_fill(txbuf, size, &state);
        if (lwip_sendto(sock, txbuf, size, 0,
                        (struct sockaddr *)&addr, sizeof(addr))
            != (int)size) {
            rt_kprintf("G45NET FLOOD FAIL send dgram=%u\n", i);
            lwip_close(sock);
            return -1;
        }
    }
    for (i = 0; i < count; i++) {
        rt_uint32_t j;
        int n = lwip_recv(sock, rxbuf, sizeof(rxbuf), 0);

        if (n != (int)size) {
            /* UDP may legitimately drop under flood; report and stop. */
            rt_kprintf("G45NET FLOOD SHORT got=%u of %u\n", got_total,
                       count);
            lwip_close(sock);
            return 0;
        }
        g45net_fill(txbuf, size, &check);
        for (j = 0; j < size; j++) {
            if (rxbuf[j] != txbuf[j]) {
                rt_kprintf("G45NET FLOOD FAIL dgram=%u off=%u\n", i, j);
                lwip_close(sock);
                return -1;
            }
        }
        got_total++;
    }
    lwip_close(sock);
    rt_kprintf("G45NET FLOOD OK size=%u count=%u\n", size, got_total);
    return 0;
}
MSH_CMD_EXPORT(g45udpflood, concurrent-TX/RX datagram flood);

/*
 * g45tcpself <bytes>: the same lockstep pattern as g45tcp but over the
 * lwip loopback interface with a guest-local listener - zero MACB
 * involvement.  If this crashes like g45tcp, the fault is in the lwip
 * port or BSP threading, and the QEMU model is exonerated.
 */
static int g45tcpself(int argc, char **argv)
{
    struct sockaddr_in addr;
    rt_uint32_t tx_state = 0x7cb1e523U, rx_state = 0x7cb1e523U;
    rt_uint32_t total, sent = 0, rcvd = 0, mismatch = 0;
    int lsock, csock, ssock;
    socklen_t alen = sizeof(addr);

    if (argc != 2) {
        rt_kprintf("usage: g45tcpself <bytes>\n");
        return -1;
    }
    total = (rt_uint32_t)atoi(argv[1]);

    lsock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    csock = lwip_socket(AF_INET, SOCK_STREAM, 0);
    rt_memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(47901);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (lsock < 0 || csock < 0 ||
        lwip_bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        lwip_listen(lsock, 1) < 0 ||
        lwip_connect(csock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        rt_kprintf("G45NET TCPSELF FAIL setup\n");
        return -1;
    }
    ssock = lwip_accept(lsock, (struct sockaddr *)&addr, &alen);
    if (ssock < 0) {
        rt_kprintf("G45NET TCPSELF FAIL accept\n");
        return -1;
    }

    while (sent < total && mismatch == 0U) {
        rt_uint32_t n = total - sent;
        rt_uint32_t back = 0;

        if (n > G45NET_CHUNK) {
            n = G45NET_CHUNK;
        }
        g45net_fill(txbuf, n, &tx_state);
        if (lwip_send(csock, txbuf, n, 0) != (int)n) {
            break;
        }
        sent += n;
        /* single-thread echo: pull from the server side, push back */
        while (back < n) {
            int got = lwip_recv(ssock, rxbuf, n - back, 0);

            if (got <= 0 || lwip_send(ssock, rxbuf, got, 0) != got) {
                mismatch = 0xffffffffU;
                break;
            }
            back += (rt_uint32_t)got;
        }
        back = 0;
        while (back < n && mismatch == 0U) {
            int got = lwip_recv(csock, rxbuf, n - back, 0);
            rt_uint32_t i;

            if (got <= 0) {
                mismatch = 0xfffffffeU;
                break;
            }
            for (i = 0; i < (rt_uint32_t)got; i++) {
                if (rxbuf[i] !=
                    (rt_uint8_t)(g45net_prng(&rx_state) >> 24)) {
                    mismatch = rcvd + i + 1U;
                    break;
                }
            }
            back += (rt_uint32_t)got;
            rcvd += (rt_uint32_t)got;
        }
    }
    lwip_close(csock);
    lwip_close(ssock);
    lwip_close(lsock);

    if (mismatch != 0U || rcvd != total) {
        rt_kprintf("G45NET TCPSELF FAIL rcvd=%u mismatch=0x%x\n",
                   rcvd, mismatch);
        return -1;
    }
    rt_kprintf("G45NET TCPSELF OK bytes=%u\n", rcvd);
    return 0;
}
MSH_CMD_EXPORT(g45tcpself, loopback TCP lockstep stream);

#endif /* RT_USING_LWIP */
