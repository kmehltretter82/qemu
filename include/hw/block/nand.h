/*
 * NAND flash memory chip emulation (hw/block/nand.c).
 *
 * The chip has no bus of its own: a flash-controller model drives it
 * through these helpers, mirroring the physical pin interface (CLE, ALE,
 * CE, WP, GND plus the eight I/O pins).
 *
 * Copyright (c) 2006 Openedhand Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_BLOCK_NAND_H
#define HW_BLOCK_NAND_H

#include "qemu/typedefs.h"

#define TYPE_NAND "nand"

DeviceState *nand_init(BlockBackend *blk, int manf_id, int chip_id);
void nand_setpins(DeviceState *dev, uint8_t cle, uint8_t ale,
                  uint8_t ce, uint8_t wp, uint8_t gnd);
void nand_getpins(DeviceState *dev, int *rb);
void nand_setio(DeviceState *dev, uint32_t value);
uint32_t nand_getio(DeviceState *dev);
uint32_t nand_getbuswidth(DeviceState *dev);

#define NAND_MFR_TOSHIBA        0x98
#define NAND_MFR_SAMSUNG        0xec
#define NAND_MFR_FUJITSU        0x04
#define NAND_MFR_NATIONAL       0x8f
#define NAND_MFR_RENESAS        0x07
#define NAND_MFR_STMICRO        0x20
#define NAND_MFR_HYNIX          0xad
#define NAND_MFR_MICRON         0x2c

#endif
