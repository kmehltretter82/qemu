/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * RiscPC floppy controller tests.
 *
 * Copyright (c) 2026 Karl Mehltretter
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define RISCPC_FDC_BASE 0x03010fc0
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
    qtest_add_func("/riscpc/fdc/identity", test_fdc_identity);

    return g_test_run();
}
