/*
 * Atmel AT91 ROM-code boot helpers.
 *
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_AT91_BOOTROM_H
#define HW_ARM_AT91_BOOTROM_H

#include "exec/hwaddr.h"
#include "qapi/error.h"
#include "qemu/typedefs.h"

bool at91_bootrom_load_sd(BlockBackend *blk, hwaddr sram_addr,
                          size_t sram_size, Error **errp);

#endif
