/*
 * Bare-metal ARM926EJ-S CP15 ID-register probe.
 * Reads SCTLR at the very first C instruction (before anything else can
 * touch it) so it reflects the true post-reset value, then dumps MIDR and
 * the c0,c0,{0,3-7} "ID code" encodings over versatilepb's PL011 UART0.
 *
 * Build: see build.sh in this directory.
 * Run:   qemu-system-arm -M versatilepb -cpu arm926 -nographic -kernel idregs.elf
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
    UARTLCRH = 0x60;  /* WLEN=8, no FIFO */
    UARTCR = 0x301;   /* UARTEN | TXE | RXE */
}

static void putc_(char c)
{
    while (UARTFR & 0x20) {} /* TXFF */
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

static uint32_t mrc_c0_c0(int opc2)
{
    uint32_t v = 0xffffffffu;
    switch (opc2) {
    case 0: __asm__ volatile ("mrc p15, 0, %0, c0, c0, 0" : "=r"(v)); break;
    case 3: __asm__ volatile ("mrc p15, 0, %0, c0, c0, 3" : "=r"(v)); break;
    case 4: __asm__ volatile ("mrc p15, 0, %0, c0, c0, 4" : "=r"(v)); break;
    case 5: __asm__ volatile ("mrc p15, 0, %0, c0, c0, 5" : "=r"(v)); break;
    case 6: __asm__ volatile ("mrc p15, 0, %0, c0, c0, 6" : "=r"(v)); break;
    case 7: __asm__ volatile ("mrc p15, 0, %0, c0, c0, 7" : "=r"(v)); break;
    }
    return v;
}

static void report(const char *label, uint32_t v)
{
    puts_(label);
    puthex32(v);
    puts_("\r\n");
}

void c_main(void)
{
    uint32_t sctlr;

    /* Must be the very first CP15 access: captures the post-reset value. */
    __asm__ volatile ("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));

    uart_init();

    report("c1,c0,0 (SCTLR) = 0x", sctlr);
    report("c0,c0,0 (MIDR)  = 0x", mrc_c0_c0(0));
    report("c0,c0,3         = 0x", mrc_c0_c0(3));
    report("c0,c0,4         = 0x", mrc_c0_c0(4));
    report("c0,c0,5         = 0x", mrc_c0_c0(5));
    report("c0,c0,6         = 0x", mrc_c0_c0(6));
    report("c0,c0,7         = 0x", mrc_c0_c0(7));

    for (;;) {
        /* ARMv5: WFI is CP15 c7,c0,4, not a dedicated mnemonic. */
        __asm__ volatile ("mcr p15, 0, %0, c7, c0, 4" : : "r"(0));
    }
}
