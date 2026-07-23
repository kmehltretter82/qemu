/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * RiscPC SuperIO controller tests.
 *
 * Copyright (c) 2026 Karl Mehltretter
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define RISCPC_FDC_BASE 0x03010fc0
#define RISCPC_IDE_BASE 0x030107c0
#define SUPERIO_INDEX   (RISCPC_FDC_BASE + (0 << 2))
#define SUPERIO_DATA    (RISCPC_FDC_BASE + (1 << 2))
#define FDC_REG_DOR     (RISCPC_FDC_BASE + (2 << 2))
#define FDC_REG_MSR     (RISCPC_FDC_BASE + (4 << 2))
#define FDC_REG_FIFO    (RISCPC_FDC_BASE + (5 << 2))

#define FD_MSR_CMDBUSY  0x10
#define FD_MSR_DIO      0x40
#define FD_MSR_RQM      0x80

#define FD_CMD_VERSION  0x10
#define FD_CMD_POWERDOWN 0x17
#define FD_CMD_PART_ID  0x18
#define FD_CMD_SAVE     0x2e
#define FD_CMD_OPTION   0x33
#define FD_CMD_RESTORE  0x4e
#define FD_CMD_DRIVESPEC 0x8e
#define FD_CMD_FORMAT_WRITE 0xcd

#define FD_SR0_INVCMD   0x80

#define IDE_REG_DATA    RISCPC_IDE_BASE
#define IDE_REG_STATUS  (RISCPC_IDE_BASE + (7 << 2))
#define IDE_REG_COMMAND IDE_REG_STATUS

#define IDE_STATUS_DRQ  0x08
#define IDE_STATUS_BUSY 0x80
#define IDE_CMD_IDENTIFY 0xec

#define SUPERIO_ENTER   0x55
#define SUPERIO_EXIT    0xaa
#define SUPERIO_CR0     0x00
#define SUPERIO_CR2     0x02
#define SUPERIO_CR5     0x05
#define SUPERIO_CRD     0x0d
#define SUPERIO_CRE     0x0e

static uint8_t superio_read_config(uint8_t reg)
{
    writeb(SUPERIO_INDEX, reg);
    return readb(SUPERIO_DATA);
}

static void test_superio_config(void)
{
    qtest_start("-machine riscpc");

    writeb(SUPERIO_INDEX, SUPERIO_ENTER);
    writeb(SUPERIO_INDEX, SUPERIO_ENTER);

    g_assert_cmphex(superio_read_config(SUPERIO_CRD), ==, 0x65);
    g_assert_cmphex(superio_read_config(SUPERIO_CRE), ==, 0x01);
    g_assert_cmphex(superio_read_config(SUPERIO_CR0), ==, 0x11);
    g_assert_cmphex(superio_read_config(SUPERIO_CR2), ==, 0x04);
    g_assert_cmphex(superio_read_config(SUPERIO_CR5), ==, 0x00);

    writeb(SUPERIO_INDEX, SUPERIO_EXIT);
    qtest_end();
}

static void test_ide_word_lanes(void)
{
    uint16_t identify[256];
    uint8_t status;
    size_t i;

    qtest_start("-machine riscpc "
                "-drive if=ide,file=null-co://,file.read-zeroes=on,"
                "format=raw,size=384M");

    writeb(IDE_REG_COMMAND, IDE_CMD_IDENTIFY);
    do {
        status = readb(IDE_REG_STATUS);
    } while (status & IDE_STATUS_BUSY);
    g_assert_cmphex(status & IDE_STATUS_DRQ, ==, IDE_STATUS_DRQ);

    /*
     * NetBSD/acorn32's insw16() uses two word loads to fetch each pair
     * of ATA words.  Qtest operates below the CPU, so the +2 access
     * observes the raw 16-bit device value before the pre-Armv6 CPU
     * aligns and rotates it into the upper half of the register.
     */
    for (i = 0; i < G_N_ELEMENTS(identify); i += 2) {
        uint32_t first = readl(IDE_REG_DATA + 2);
        uint32_t second = readl(IDE_REG_DATA);

        g_assert_cmphex(first & 0xffff0000, ==, 0);
        g_assert_cmphex(second & 0xffff0000, ==, 0);
        identify[i] = first;
        identify[i + 1] = second;
    }

    g_assert_cmpuint(identify[1], >, 0);       /* cylinders */
    g_assert_cmpuint(identify[3], ==, 16);     /* heads */
    g_assert_cmpuint(identify[6], ==, 63);     /* sectors per track */
    g_assert_cmphex(identify[49] & 0x0200, ==, 0x0200); /* LBA */
    g_assert_cmphex(((uint32_t)identify[61] << 16) | identify[60],
                    ==, 384 * 1024 * 2);

    qtest_end();
}

static void fdc_send(uint8_t value)
{
    uint8_t msr = readb(FDC_REG_MSR);

    g_assert((msr & FD_MSR_RQM) == FD_MSR_RQM);
    g_assert((msr & FD_MSR_DIO) == 0);
    writeb(FDC_REG_FIFO, value);
}

static uint8_t fdc_recv(void)
{
    uint8_t msr = readb(FDC_REG_MSR);

    g_assert((msr & (FD_MSR_RQM | FD_MSR_DIO)) ==
             (FD_MSR_RQM | FD_MSR_DIO));
    return readb(FDC_REG_FIFO);
}

static void test_fdc_identity(void)
{
    static const uint8_t commands_82078_only[] = {
        FD_CMD_POWERDOWN,
        FD_CMD_PART_ID,
        FD_CMD_SAVE,
        FD_CMD_OPTION,
        FD_CMD_RESTORE,
        FD_CMD_DRIVESPEC,
        FD_CMD_FORMAT_WRITE,
    };
    uint8_t msr;
    size_t i;

    qtest_start("-machine riscpc");

    /* Release the controller from reset and enable its interrupt output. */
    writeb(FDC_REG_DOR, 0x0c);

    fdc_send(FD_CMD_VERSION);
    g_assert(fdc_recv() == 0x90);

    /* The SMC core is 82077AA-compatible, not an Intel 82078. */
    for (i = 0; i < G_N_ELEMENTS(commands_82078_only); i++) {
        fdc_send(commands_82078_only[i]);
        g_assert(fdc_recv() == FD_SR0_INVCMD);
    }

    msr = readb(FDC_REG_MSR);
    g_assert((msr & FD_MSR_RQM) == FD_MSR_RQM);
    g_assert((msr & (FD_MSR_CMDBUSY | FD_MSR_DIO)) == 0);

    qtest_end();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/riscpc/superio/config", test_superio_config);
    qtest_add_func("/riscpc/superio/ide-word-lanes", test_ide_word_lanes);
    qtest_add_func("/riscpc/fdc/identity", test_fdc_identity);

    return g_test_run();
}
