/*
 * qemu-exact: adversarially exact models for finding guest (Linux) bugs.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef TARGET_ARM_EXACT_H
#define TARGET_ARM_EXACT_H

#include "cpu.h"
#include "exec/cpu-common.h"
#include "mmuidx.h"

struct ARMCPRegInfo;

/* atlb.c */
extern bool arm_exact_tlb_enabled;
void arm_exact_tlb_init(void);
void arm_exact_tlb_leaf(CPUARMState *env, ARMMMUIdx mmu_idx,
                        ARMSecuritySpace space, uint64_t va, uint64_t desc_pa,
                        uint64_t desc_val, uint64_t oa, int level,
                        int lg_page_size, void *host);
void arm_exact_ptwatch_write(CPUState *cs, uint64_t ram_addr, unsigned size,
                             uintptr_t retaddr);
void arm_exact_tlb_table(CPUARMState *env, ARMMMUIdx mmu_idx,
                         ARMSecuritySpace space, uint64_t va, uint64_t desc_pa,
                         uint64_t desc_val, int level, int lg_cover,
                         void *host);
void arm_exact_tlb_tlbi(CPUARMState *env, const struct ARMCPRegInfo *ri,
                        uint64_t value);
void arm_exact_tlb_dump(void);

/* icache.c */
extern bool arm_exact_icache_enabled;
void arm_exact_icache_init(void);
void arm_exact_icache_fetch(CPUState *cs, uint64_t ram_addr, unsigned size);
void arm_exact_icache_store(CPUState *cs, uint64_t ram_addr, unsigned size);
void arm_exact_icache_maint(CPUState *cs, uint64_t ram_addr, bool all,
                            bool clean, bool invalidate);
void arm_exact_icache_dump(void);

#endif
