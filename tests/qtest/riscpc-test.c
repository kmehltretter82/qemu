/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Acorn RiscPC machine tests.
 *
 * Copyright (c) 2026 Karl Mehltretter
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "libqtest-single.h"

#define RISCPC_IOMD_BASE  0x03200000
#define IOMD_KART         (RISCPC_IOMD_BASE + 0x004)
#define IOMD_KCTRL        (RISCPC_IOMD_BASE + 0x008)
#define IOMD_IRQSTATB     (RISCPC_IOMD_BASE + 0x020)

#define KCTRL_RXPARITY    (1 << 2)
#define KCTRL_DATAIN      (1 << 1)
#define KCTRL_CLOCKIN     (1 << 0)
#define KCTRL_ENABLE      (1 << 3)
#define KCTRL_RXFULL      (1 << 5)
#define KCTRL_TXEMPTY     (1 << 7)
#define IRQB_KBDTX        (1 << 6)

static uint8_t wait_for_kart_data(void)
{
    uint8_t status;
    int i;

    for (i = 0; i < 20; i++) {
        status = readb(IOMD_KCTRL);
        if (status & KCTRL_RXFULL) {
            return status;
        }
    }
    g_assert_not_reached();
}

static void test_rom(void)
{
    static const uint8_t first[] = { 0x12, 0x34, 0x56, 0x78 };
    static const uint8_t last[] = { 0x9a, 0xbc, 0xde, 0xf0 };
    g_autofree char *path = NULL;
    g_autofree char *args = NULL;
    uint8_t actual[sizeof(first)];
    int fd;

    fd = g_file_open_tmp("riscpc-rom.XXXXXX", &path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 4 * MiB), ==, 0);
    g_assert_cmpint(pwrite(fd, first, sizeof(first), 0), ==, sizeof(first));
    g_assert_cmpint(pwrite(fd, last, sizeof(last), 4 * MiB - sizeof(last)),
                    ==, sizeof(last));
    close(fd);

    args = g_strdup_printf("-machine riscpc -bios %s", path);
    qtest_start(args);
    memread(0, actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), first, sizeof(first));
    memread(4 * MiB - sizeof(last), actual, sizeof(actual));
    g_assert_cmpmem(actual, sizeof(actual), last, sizeof(last));
    qtest_end();

    unlink(path);
}

static void test_kart(void)
{
    uint8_t status;

    qtest_start("-machine riscpc");

    writeb(IOMD_KCTRL, 0);
    writeb(IOMD_KCTRL, KCTRL_ENABLE);
    status = readb(IOMD_KCTRL);
    g_assert_cmphex(status & (KCTRL_ENABLE | KCTRL_TXEMPTY |
                              KCTRL_DATAIN | KCTRL_CLOCKIN), ==,
                    KCTRL_ENABLE | KCTRL_TXEMPTY |
                    KCTRL_DATAIN | KCTRL_CLOCKIN);
    g_assert_cmphex(readb(IOMD_IRQSTATB) & IRQB_KBDTX, ==, IRQB_KBDTX);

    /* PS/2 reset replies 0xfa, 0xaa; both carry a set odd-parity bit. */
    writeb(IOMD_KART, 0xff);
    status = wait_for_kart_data();
    g_assert_cmphex(status & (KCTRL_RXFULL | KCTRL_RXPARITY), ==,
                    KCTRL_RXFULL | KCTRL_RXPARITY);
    g_assert_cmphex(readb(IOMD_KART), ==, 0xfa);
    status = wait_for_kart_data();
    g_assert_cmphex(status & (KCTRL_RXFULL | KCTRL_RXPARITY), ==,
                    KCTRL_RXFULL | KCTRL_RXPARITY);
    g_assert_cmphex(readb(IOMD_KART), ==, 0xaa);
    g_assert_cmphex(readb(IOMD_KCTRL) & KCTRL_RXFULL, ==, 0);

    writeb(IOMD_KCTRL, 0);
    g_assert_cmphex(readb(IOMD_IRQSTATB) & IRQB_KBDTX, ==, 0);
    qtest_end();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/riscpc/rom", test_rom);
    qtest_add_func("/riscpc/kart", test_kart);

    return g_test_run();
}
