Acorn RiscPC (``riscpc``)
=========================

The RiscPC is Acorn's 1994 desktop - the machine 32-bit ARM Linux was
originally developed on. This model emulates the common Linux-era
configuration: a StrongARM SA-110 processor card with the IOMD I/O
controller, VIDC20 video, and the onboard SuperIO devices.

Emulated hardware:

- StrongARM SA-110 CPU (ARMv4)
- original ARM610 processor-card option (ARMv3; select with
  ``-cpu arm610``)
- Acorn IOMD: interrupt controller (banks A/B, DMA requests, FIQ),
  two 2 MHz IOC-style 16-bit timers, KART PS/2 keyboard link,
  quadrature mouse and video DMA
- VIDC20 video controller with guest-programmable scanout from main RAM
- SuperIO 16550 at 0x03010fe0 (``ttyS0``, IOMD bank B bit 2)
- onboard SuperIO floppy controller at 0x03010fc0, including the
  IOMD FIQ pseudo-DMA path used by Linux (attach raw images with
  ``-drive ...,if=floppy``)
- onboard SuperIO IDE at 0x030107c0 (Linux ``pata_platform``;
  attach disks with ``-drive ...,if=ide``)
- RAM at physical 0x10000000 (the RiscPC's non-zero RAM base)
- I/O and EASI podule spaces read as a floating bus (0xff), so the
  kernel's expansion-card probe correctly finds empty slots

Booting a kernel built from ``rpc_defconfig``::

  qemu-system-arm -M riscpc -kernel zImage -initrd rootfs.cpio.gz \
      -append 'console=ttyS0 rdinit=/init' \
      -display none -serial stdio

Booting a RISC OS ROM image::

  qemu-system-arm -M riscpc -m 128M -bios riscos \
      -display gtk -serial none

Raw 2, 4, 6 and 8 MiB ROM images are accepted.  A ROM image and a
directly loaded kernel cannot be used together.

The default CPU is a StrongARM SA-110.  ``-cpu arm610`` selects the original
ARM610 RiscPC processor card.  It implements ARMv3, so ARMv4 halfword and
signed transfers and ARMv4T ``BX`` instructions take an undefined-instruction
exception as they would on the original processor.

The ARM610 model also preserves its early writeback state when a writeback
``LDM`` or ``STM`` takes a data abort.  This is needed by the historical ARM610
Linux data-abort handler when it retries a faulting copy-on-write stack access.
Its CP15 c1 control register is write-only, as on ARMv3 hardware; reads return
zero rather than the value most recently written.

RiscPC StrongARM processor cards have a separate defect in ARMv4 ``LDRH`` and
``STRH`` transfers.  QEMU preserves their address-fault behaviour but supplies
an unpredictable data value and logs each execution with ``-d guest_errors``.
This is RiscPC-only: an SA-110 in another machine such as the NetWinder keeps
normal ARMv4 halfword transfers.  The quirk is enabled by default and can be
temporarily disabled for software diagnosis with ``-M riscpc,broken-halfword=off``.

Note that mainline restricts ``ARCH_RPC`` to GCC 6 through 8: newer
compilers emit ``strh`` instructions that the real RiscPC's bus
cannot execute, so kernels must be built with such a toolchain.

Vendor-era kernels booted by the RISC OS loader expect a
``param_struct`` parameter block; use ``-M riscpc,old-param=on``.

The onboard floppy controller is enabled by default.  Use
``-M riscpc,floppy=off`` to reproduce the no-controller path in older
Linux kernels.

Not yet modelled: podules (including ICS IDE expansion cards) and sound DMA.
