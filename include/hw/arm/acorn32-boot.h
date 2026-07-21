/*
 * NetBSD/acorn32 boot support for the Acorn RiscPC.
 *
 * Copyright (c) 2026 Karl Mehltretter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ARM_ACORN32_BOOT_H
#define HW_ARM_ACORN32_BOOT_H

#include "hw/core/boards.h"
#include "qapi/error.h"

/*
 * True if @filename is a NetBSD/acorn32 kernel, i.e. an ARM ELF linked at
 * the acorn32 KERNEL_BASE.  Linux is loaded as a raw zImage, so the two
 * boot paths never collide.
 */
bool acorn32_kernel_p(const char *filename);

/*
 * Load a NetBSD/acorn32 kernel and prepare the machine state its loader
 * is expected to provide: page tables with the MMU about to be enabled, a
 * struct bootconfig, and a stub to switch on translation.  On success
 * *entry is the physical address the CPU should start executing at.
 */
bool acorn32_load_netbsd(MachineState *machine, hwaddr ram_base,
                         hwaddr *entry, Error **errp);

#endif
