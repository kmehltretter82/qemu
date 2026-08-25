/*
 * Nintendo 3DS / New 3DS (CTR/KTR) SoC definitions shared by the n3ds
 * machine and its device models.
 *
 * Register-level documentation: https://www.3dbrew.org and GBATEK
 * (https://problemkaputt.de/gbatek.htm), 3DS chapters.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_MISC_N3DS_H
#define HW_MISC_N3DS_H

#define TYPE_N3DS_SYSCTL "n3ds-sysctl"
#define TYPE_N3DS_TMIO   "n3ds-tmio"
#define TYPE_N3DS_I2C    "n3ds-i2c"
#define TYPE_N3DS_MCU    "n3ds-mcu"
#define TYPE_N3DS_HID    "n3ds-hid"
#define TYPE_N3DS_LCD    "n3ds-lcd"
#define TYPE_N3DS_PXI    "n3ds-pxi"
#define TYPE_N3DS_IRQ    "n3ds-irq"
#define TYPE_N3DS_TIMER  "n3ds-timer"
#define TYPE_N3DS_SPI    "n3ds-spi"
#define TYPE_N3DS_GPIO   "n3ds-gpio"

/* Physical memory map (both CPUs unless noted) */
#define N3DS_ITCM_BASE        0x00000000  /* ARM9, 32K mirrored */
#define N3DS_ITCM_SIZE        0x00008000
#define N3DS_ARM9_WRAM_BASE   0x08000000  /* ARM9 only */
#define N3DS_ARM9_WRAM_SIZE   0x00100000
#define N3DS_ARM9_WRAM_EXT    0x08100000  /* New 3DS, CFG9 gated */
#define N3DS_ARM9_WRAM_EXT_SIZE 0x00080000
#define N3DS_IO9_BASE         0x10000000  /* ARM9-only IO */
#define N3DS_IO_SHARED_BASE   0x10100000
#define N3DS_IO_SHARED_SIZE   0x00100000
#define N3DS_IO11_BASE        0x10200000  /* ARM11-only IO */
#define N3DS_GPU_BASE         0x10400000
#define N3DS_MPCORE_BASE      0x17E00000
#define N3DS_L2C_BASE         0x17E10000
#define N3DS_VRAM_BASE        0x18000000
#define N3DS_VRAM_SIZE        0x00600000
#define N3DS_QTM_BASE         0x1F000000  /* New 3DS extra RAM, ARM11 */
#define N3DS_QTM_SIZE         0x00400000
#define N3DS_DSP_BASE         0x1FF00000
#define N3DS_DSP_SIZE         0x00080000
#define N3DS_AXI_WRAM_BASE    0x1FF80000
#define N3DS_AXI_WRAM_SIZE    0x00080000
#define N3DS_FCRAM_BASE       0x20000000
#define N3DS_FCRAM_SIZE_CTR   0x08000000
#define N3DS_FCRAM_SIZE_KTR   0x10000000
#define N3DS_DTCM_BASE        0xFFF00000  /* ARM9 */
#define N3DS_DTCM_SIZE        0x00004000
#define N3DS_BOOTROM_BASE     0xFFFF0000
#define N3DS_BOOTROM_SIZE     0x00010000

/* IO blocks */
#define N3DS_CFG9_BASE        0x10000000
#define N3DS_IRQ9_BASE        0x10001000
#define N3DS_NDMA_BASE        0x10002000
#define N3DS_TIMER9_BASE      0x10003000
#define N3DS_CTRCARD_BASE     0x10004000
#define N3DS_SDMMC_BASE       0x10006000
#define N3DS_PXI9_BASE        0x10008000
#define N3DS_AES_BASE         0x10009000
#define N3DS_SHA_BASE         0x1000A000
#define N3DS_RSA_BASE         0x1000B000
#define N3DS_XDMA_BASE        0x1000C000
#define N3DS_SPICARD_BASE     0x1000D800
#define N3DS_CFG_BASE         0x10010000
#define N3DS_PRNG_BASE        0x10011000
#define N3DS_OTP_BASE         0x10012000
#define N3DS_SDIO3_BASE       0x10100000
#define N3DS_HASH_BASE        0x10101000
#define N3DS_CSND_BASE        0x10103000
#define N3DS_WIFI_BASE        0x10122000
#define N3DS_CFG11_BASE       0x10140000
#define N3DS_PDN_BASE         0x10141000
#define N3DS_SPI2_BASE        0x10142000
#define N3DS_SPI3_BASE        0x10143000
#define N3DS_I2C2_BASE        0x10144000
#define N3DS_I2S_BASE         0x10145000
#define N3DS_HID_BASE         0x10146000
#define N3DS_GPIO_BASE        0x10147000
#define N3DS_I2C3_BASE        0x10148000
#define N3DS_SPI1_BASE        0x10160000
#define N3DS_I2C1_BASE        0x10161000
#define N3DS_MIC_BASE         0x10162000
#define N3DS_PXI11_BASE       0x10163000
#define N3DS_NTRCARD_BASE     0x10164000
#define N3DS_CDMA_BASE        0x10200000
#define N3DS_FCRAMCFG_BASE    0x10201000
#define N3DS_LCD_BASE         0x10202000
#define N3DS_DSPREG_BASE      0x10203000
#define N3DS_CDMA2_BASE       0x10206000
#define N3DS_AXICFG_BASE      0x1020F000

/* ARM9 interrupt controller lines */
#define N3DS_IRQ9_TIMER0      8
#define N3DS_IRQ9_PXI_SYNC    12
#define N3DS_IRQ9_PXI_SEND    13
#define N3DS_IRQ9_PXI_RECV    14
#define N3DS_IRQ9_AES         15
#define N3DS_IRQ9_SDIO1       16
#define N3DS_IRQ9_SDIO1_ASYNC 17
#define N3DS_IRQ9_SDIO3       18
#define N3DS_IRQ9_SDIO3_ASYNC 19
#define N3DS_IRQ9_RSA         22
#define N3DS_IRQ9_SHA         30

/* ARM11 GIC interrupt IDs (SPI number = ID - 32) */
#define N3DS_GIC_SDIO3        0x40
#define N3DS_GIC_SDIO3_ASYNC  0x41
#define N3DS_GIC_PXI_SYNC     0x50
#define N3DS_GIC_PXI_SEND     0x52
#define N3DS_GIC_PXI_RECV     0x53
#define N3DS_GIC_I2C1         0x54
#define N3DS_GIC_I2C2         0x55
#define N3DS_GIC_SOCMODE      0x58
#define N3DS_GIC_I2C3         0x5C
#define N3DS_GIC_SPI(id)      ((id) - 32)

/* Luma3DS chainloader hand-off protocol (AXI WRAM top) */
#define N3DS_ARM11_ENTRY_MAILBOX 0x1FFFFFFC
#define N3DS_CORE1_ENTRY_MAILBOX 0x1FFFFFDC

#endif
