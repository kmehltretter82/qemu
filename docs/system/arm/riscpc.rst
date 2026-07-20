Acorn RiscPC (``riscpc``)
=========================

The RiscPC is Acorn's 1994 desktop - the machine 32-bit ARM Linux was
originally developed on. This model emulates the common Linux-era
configuration: a StrongARM SA-110 processor card with the IOMD I/O
controller and the SuperIO's 16550 serial console.

Emulated hardware:

- StrongARM SA-110 CPU (ARMv4)
- Acorn IOMD: interrupt controller (banks A/B, DMA requests, FIQ),
  two 2 MHz IOC-style 16-bit timers, stubbed KART keyboard link
- SuperIO 16550 at 0x03010fe0 (``ttyS0``, IOMD bank B bit 2)
- onboard SuperIO IDE at 0x030107c0 (Linux ``pata_platform``;
  attach disks with ``-drive ...,if=ide``)
- RAM at physical 0x10000000 (the RiscPC's non-zero RAM base)
- I/O and EASI podule spaces read as a floating bus (0xff), so the
  kernel's expansion-card probe correctly finds empty slots

Booting a kernel built from ``rpc_defconfig``::

  qemu-system-arm -M riscpc -kernel zImage -initrd rootfs.cpio.gz \
      -append 'console=ttyS0 rdinit=/init' \
      -display none -serial stdio

Note that mainline restricts ``ARCH_RPC`` to GCC 6 through 8: newer
compilers emit ``strh`` instructions that the real RiscPC's bus
cannot execute, so kernels must be built with such a toolchain.

Vendor-era kernels booted by the RISC OS loader expect a
``param_struct`` parameter block; use ``-M riscpc,old-param=on``.

Not yet modelled: VIDC20 video, the KART keyboard/quadrature mouse,
podules (including ICS IDE expansion cards), floppy and sound DMA.
