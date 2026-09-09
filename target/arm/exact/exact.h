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
void arm_exact_tlb_tlbi32(CPUARMState *env, const struct ARMCPRegInfo *ri,
                          uint64_t value);
void arm_exact_tlb_tlbi(CPUARMState *env, const struct ARMCPRegInfo *ri,
                        uint64_t value);
void arm_exact_tlb_dump(void);

/* icache.c */
extern bool arm_exact_icache_enabled;
extern bool arm_exact_icache_full;
void arm_exact_icache_init(void);
void arm_exact_icache_fetch(CPUState *cs, uint64_t ram_addr, unsigned size);
void arm_exact_icache_store(CPUState *cs, uint64_t ram_addr, unsigned size);
void arm_exact_icache_maint(CPUState *cs, uint64_t ram_addr, bool all,
                            bool clean, bool invalidate);
void arm_exact_icache_dump(void);

/* dcache.c */
extern bool arm_exact_dcache_enabled;
extern unsigned arm_exact_dcache_line;
void arm_exact_dcache_init(void);
void arm_exact_dcache_inflight(uint64_t ram_addr, uint64_t len, bool is_write,
                               bool inflight);
void arm_exact_dcache_dma(uint64_t ram_addr, uint64_t len, bool is_write,
                          const char *as_name);
void arm_exact_dcache_cpu(CPUState *cs, uint64_t ram_addr, unsigned size,
                          bool is_store);
void arm_exact_dcache_maint(CPUState *cs, uint64_t ram_addr, bool all,
                            char kind);
bool arm_exact_dcache_page_tracked(uint64_t ram_addr);
bool arm_exact_dcache_track_page(CPUState *cs, CPUTLBEntryFull *full,
                                 uint64_t ram_addr);
void arm_exact_dcache_track_access(CPUState *cs, CPUTLBEntryFull *full,
                                   uint64_t ram_addr, unsigned size,
                                   bool is_store);
void arm_exact_dcache_page_joined(uint64_t ram_addr);
void arm_exact_dcache_nc_skipped(void);
void arm_exact_dcache_dump(void);

/* exclusive.c */
extern bool arm_exact_exclusive_enabled;
extern unsigned arm_exact_exclusive_rate;
extern unsigned arm_exact_exclusive_maxrun;
extern uint64_t arm_exact_exclusive_seed;
void arm_exact_exclusive_init(void);
bool arm_exact_stxr_fail(CPUARMState *env, uint64_t addr);
void arm_exact_exclusive_exception(CPUARMState *env);
void arm_exact_exclusive_dump(void);
const char *arm_exact_llsc_forbidden_a64(uint32_t insn);
const char *arm_exact_llsc_forbidden_a32(uint32_t insn);
bool arm_exact_llsc_is_branch(const char *what);
void arm_exact_llsc_pair(uint64_t ldex_pc, uint64_t stex_pc, uint64_t bad_pc,
                         const char *what);
void arm_exact_llsc_dump(void);

#endif
