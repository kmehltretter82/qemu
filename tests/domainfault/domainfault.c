/*
 * F1 reproducer: short-descriptor domain fault vs. L2-descriptor validity
 * ordering (QEMU-ARM926-REVIEW.md finding F1).
 *
 * Sets up a v5 (non-LPAE) page table by hand:
 *  - L1[0]       section, identity map of the low 1MB (our own code/data),
 *                domain 0.
 *  - L1[0x101]   section, identity map of versatilepb UART0 (0x101f1000),
 *                domain 0.
 *  - L1[4]       coarse (page-table) entry for VA 0x00400000, domain 1,
 *                pointing at an L2 table that is left entirely zeroed
 *                (every L2 entry therefore decodes as "invalid").
 *  - DACR: domain 0 = Client (viable), domain 1 = No Access.
 *
 * Then reads VA 0x00400000. Spec (DDI 0100I/E, Figure B4-2): the domain is
 * checked "if a valid second-level descriptor is returned" -- since our L2
 * entry is invalid, a spec-faithful walker must report a page Translation
 * Fault (DFSR[3:0]=0x7), independent of DACR[1]. QEMU (per F1) checks the
 * domain from the L1 descriptor before ever fetching L2, so with domain 1
 * set to No Access it reports a Domain Fault (DFSR[3:0]=0xB) instead --
 * the L2 descriptor is never even read.
 *
 * Build: see build.sh. Run:
 *   qemu-system-arm -M versatilepb -cpu arm926 -nographic -kernel domainfault.elf
 */
#include <stdint.h>

#define UART0_BASE 0x101f1000u
#define UARTDR   (*(volatile uint32_t *)(UART0_BASE + 0x00))
#define UARTFR   (*(volatile uint32_t *)(UART0_BASE + 0x18))
#define UARTIBRD (*(volatile uint32_t *)(UART0_BASE + 0x24))
#define UARTFBRD (*(volatile uint32_t *)(UART0_BASE + 0x28))
#define UARTLCRH (*(volatile uint32_t *)(UART0_BASE + 0x2c))
#define UARTCR   (*(volatile uint32_t *)(UART0_BASE + 0x30))

static void uart_init(void)
{
    UARTCR = 0;
    UARTIBRD = 1;
    UARTFBRD = 0;
    UARTLCRH = 0x60;
    UARTCR = 0x301;
}

static void putc_(char c)
{
    while (UARTFR & 0x20) {}
    UARTDR = c;
}

static void puts_(const char *s)
{
    while (*s) {
        putc_(*s++);
    }
}

static void puthex32(uint32_t v)
{
    static const char hex[] = "0123456789abcdef";
    char buf[8];
    for (int i = 7; i >= 0; i--) {
        buf[i] = hex[v & 0xf];
        v >>= 4;
    }
    for (int i = 0; i < 8; i++) {
        putc_(buf[i]);
    }
}

static void report(const char *label, uint32_t v)
{
    puts_(label);
    puthex32(v);
    puts_("\r\n");
}

/* L1: 4096 entries, 16KB, 16KB-aligned. L2: 256 entries, 1KB, 1KB-aligned. */
static uint32_t l1_table[4096] __attribute__((aligned(16384)));
static uint32_t l2_table[256] __attribute__((aligned(1024)));

#define VICTIM_VA 0x00400000u

void c_main(void)
{
    uint32_t i;
    uint32_t l2_phys = (uint32_t)(uintptr_t)l2_table;
    uint32_t sctlr;

    uart_init();
    puts_("=== F1 domain-fault-ordering test ===\r\n");

    for (i = 0; i < 4096; i++) {
        l1_table[i] = 0;              /* translation fault: unmapped */
    }
    for (i = 0; i < 256; i++) {
        l2_table[i] = 0;              /* invalid: every page in this 1MB */
    }

    /* L1[0]: identity section for our own code/data/stack, domain 0. */
    l1_table[0] = 0x00000000u | (3u << 10) | (0u << 5) | (1u << 4) | 0x2u;

    /* L1[0x101]: identity section for UART0 (0x101f1000), domain 0. */
    l1_table[0x101] = 0x10100000u | (3u << 10) | (0u << 5) | (1u << 4) | 0x2u;

    /* L1[4]: coarse entry for VICTIM_VA, domain 1, L2 all-invalid. */
    l1_table[VICTIM_VA >> 20] =
        (l2_phys & 0xfffffc00u) | (1u << 5) | (1u << 4) | 0x1u;

    report("L1 table  @ 0x", (uint32_t)(uintptr_t)l1_table);
    report("L2 table  @ 0x", l2_phys);
    report("L1[victim]  = 0x", l1_table[VICTIM_VA >> 20]);

    /* DACR: domain0 = Client (0b01), domain1 = No Access (0b00). */
    __asm__ volatile ("mcr p15, 0, %0, c3, c0, 0" : : "r"(0x00000001u));

    /* TTBR0 (single table on v5, no TTBR split). */
    __asm__ volatile ("mcr p15, 0, %0, c2, c0, 0"
                       : : "r"((uint32_t)(uintptr_t)l1_table));

    /* Invalidate unified TLB. */
    __asm__ volatile ("mcr p15, 0, %0, c8, c7, 0" : : "r"(0));

    __asm__ volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));
    report("SCTLR before MMU on = 0x", sctlr);
    sctlr |= 1u; /* M bit */
    __asm__ volatile (
        "mcr p15, 0, %0, c1, c0, 0\n"
        "nop\n\tnop\n\tnop\n\tnop\n"
        : : "r"(sctlr));

    puts_("MMU enabled, reading victim VA (expect abort)...\r\n");
    {
        volatile uint32_t *victim = (volatile uint32_t *)VICTIM_VA;
        uint32_t v = *victim;
        /* Unreachable if the fault fires as expected. */
        report("UNEXPECTED: read succeeded, value = 0x", v);
    }
    for (;;) {}
}

void report_fault(uint32_t dfsr, uint32_t dfar)
{
    report("DFAR = 0x", dfar);
    report("DFSR = 0x", dfsr);
    report("DFSR[3:0] (FS low nibble) = 0x", dfsr & 0xfu);
    puts_("Expect 0x7 (Translation, spec-faithful) vs 0xb "
          "(Domain, current QEMU F1 bug)\r\n");
}
