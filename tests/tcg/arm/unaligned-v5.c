/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Pre-v6 unaligned data access semantics.
 *
 * On an ARMv5 core with alignment checking disabled (SCTLR.A == 0) an
 * unaligned access is not byte-addressed: the memory interface
 * truncates the address, and a single-word *load* rotates the aligned
 * word right by 8 * addr[1:0]. DDI 0100I A2-40 and Table A2-10
 * ("U=0 A=0"). Nothing else rotates, and nothing faults -- in
 * particular LDM/STM, LDRD/STRD and SWP take no alignment fault at all
 * when A is 0 (DDI 0198E 3.5.1 conditions alignment faults on A).
 *
 * The values below were measured on real hardware, a Microchip
 * SAM9X75D2G (ARM926EJ-S r0p5), and cover the cases the architecture
 * leaves UNPREDICTABLE as well as the ones it defines.
 *
 * Must be run on a pre-v6 CPU; on v6+ these accesses are genuinely
 * byte-addressed and every expectation here is wrong.
 *
 * Freestanding, like hello-arm: this has to build at -march=armv5te,
 * which has no FPU, so it cannot link against the hard-float libc of
 * the arm-linux-gnueabihf toolchain that configure picks by default.
 */

typedef unsigned int u32;
typedef unsigned char u8;

static void xwrite(const char *buf, unsigned long len)
{
    register long r0 __asm__("r0") = 1;
    register long r1 __asm__("r1") = (long)buf;
    register long r2 __asm__("r2") = (long)len;
    register long r7 __asm__("r7") = 4;         /* __NR_write */
    __asm__ volatile("svc 0"
                     : "+r"(r0) : "r"(r1), "r"(r2), "r"(r7) : "memory");
}

static void xexit(int code)
{
    register long r0 __asm__("r0") = code;
    register long r7 __asm__("r7") = 248;       /* __NR_exit_group */
    __asm__ volatile("svc 0" : : "r"(r0), "r"(r7) : "memory");
    __builtin_unreachable();
}

static void puts_(const char *s)
{
    const char *p = s;
    unsigned long n = 0;

    while (*p++) {
        n++;
    }
    xwrite(s, n);
}

static void puthex(u32 v)
{
    static const char hex[] = "0123456789abcdef";
    char buf[8];
    int i;

    for (i = 0; i < 8; i++) {
        buf[7 - i] = hex[(v >> (i * 4)) & 0xf];
    }
    xwrite(buf, 8);
}

static u8 buf[32] __attribute__((aligned(8)));
static int failures;

static void fill(void)
{
    int i;

    for (i = 0; i < 32; i++) {
        buf[i] = 0x81 + i;
    }
}

static void check(const char *what, u32 got, u32 want)
{
    if (got != want) {
        puts_("FAIL ");
        puts_(what);
        puts_(" got 0x");
        puthex(got);
        puts_(" want 0x");
        puthex(want);
        puts_("\n");
        failures++;
    }
}

static void check_mem(const char *what, int off, const u8 *want, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        if (buf[off + i] != want[i]) {
            puts_("FAIL ");
            puts_(what);
            puts_(" byte ");
            puthex(off + i);
            puts_(" got 0x");
            puthex(buf[off + i]);
            puts_(" want 0x");
            puthex(want[i]);
            puts_("\n");
            failures++;
            return;
        }
    }
}

void _start(void)
{
    u32 v, ldm[4], ldrd[2];
    static const u8 str_want[]  = { 0xd4, 0xc3, 0xb2, 0xa1 };
    static const u8 strh_want[] = { 0xd4, 0xc3 };
    static const u8 stm_want[]  = { 0x02, 0x00, 0x00, 0x00,
                                    0x03, 0x00, 0x00, 0x00 };
    static const u32 ldr_want[4] = {
        0x8c8b8a89, 0x898c8b8a, 0x8a898c8b, 0x8b8a898c
    };
    static const char * const ldr_name[4] = {
        "ldr +8", "ldr +9", "ldr +10", "ldr +11"
    };
    int i;

    fill();

    /*
     * Word loads: the aligned word 0x8c8b8a89 at buf+8, rotated right
     * by 8 * addr[1:0].
     */
    for (i = 0; i < 4; i++) {
        __asm__ volatile("ldr %0, [%1]"
                         : "=r"(v) : "r"(buf + 8 + i) : "memory");
        check(ldr_name[i], v, ldr_want[i]);
    }

    /* Halfword load: address bit 0 dropped, and no rotation. */
    __asm__ volatile("ldrh %0, [%1]" : "=r"(v) : "r"(buf + 9) : "memory");
    check("ldrh +9", v, 0x00008a89);

    /*
     * LDM truncates the base and rotates nothing -- so a one-register
     * LDM and an LDR at the same unaligned address differ.
     */
    __asm__ volatile("ldmia %1, {r0-r3}\n\t"
                     "stmia %0, {r0-r3}"
                     : : "r"(ldm), "r"(buf + 9)
                     : "r0", "r1", "r2", "r3", "memory");
    check("ldmia +9 [0]", ldm[0], 0x8c8b8a89);
    check("ldmia +9 [1]", ldm[1], 0x908f8e8d);

    /* LDRD truncates to a word, not a doubleword, and does not fault. */
    __asm__ volatile("ldrd r0, r1, [%1]\n\t"
                     "stm %0, {r0, r1}"
                     : : "r"(ldrd), "r"(buf + 10)
                     : "r0", "r1", "memory");
    check("ldrd +10 [0]", ldrd[0], 0x8c8b8a89);
    check("ldrd +10 [1]", ldrd[1], 0x908f8e8d);

    /* Word store: truncated address, value not rotated. */
    fill();
    __asm__ volatile("str %0, [%1]"
                     : : "r"(0xa1b2c3d4), "r"(buf + 9) : "memory");
    check_mem("str +9", 8, str_want, 4);

    /* Halfword store: address bit 0 dropped. */
    fill();
    __asm__ volatile("strh %0, [%1]"
                     : : "r"(0xa1b2c3d4), "r"(buf + 9) : "memory");
    check_mem("strh +9", 8, strh_want, 2);

    /* STM: truncated base, no fault. */
    fill();
    __asm__ volatile("mov r0, #2\n\t"
                     "mov r1, #3\n\t"
                     "stmia %0, {r0, r1}"
                     : : "r"(buf + 9) : "r0", "r1", "memory");
    check_mem("stmia +9", 8, stm_want, 8);

    /*
     * SWP is an LDR and an STR at one address: load rotates, store
     * does not.
     */
    fill();
    __asm__ volatile("swp %0, %1, [%2]"
                     : "=&r"(v) : "r"(0xa1b2c3d4), "r"(buf + 9) : "memory");
    check("swp +9 loaded", v, 0x898c8b8a);
    check_mem("swp +9 stored", 8, str_want, 4);

    if (failures) {
        puts_("unaligned-v5: FAILED\n");
        xexit(1);
    }
    puts_("ok\n");
    xexit(0);
}
