Rebel NetWinder (``netwinder``)
===============================

The NetWinder is a 1998 network appliance built by Rebel.com around a
StrongARM SA-110 with the DC21285 "Footbridge" core logic and a
SuperIO chip on the 21285's X-Bus.

Emulated hardware:

- StrongARM SA-110 CPU (ARMv4)
- DC21285 CSR block: IRQ/FIQ controller, four 24-bit timers, the
  internal UART (Linux ``ttyFB``, also the DEBUG_LL port)
- ISA world behind the PCI I/O window at 0x7c000000: i8259 interrupt
  controller pair (cascaded into 21285 IN3, acknowledged through the
  PCI IACK region), two 16550 UARTs (``ttyS0``/``ttyS1``), i8254 PIT
  (the machine's timekeeping source)
- 4 MiB flash at 0x41000000 (NeTTrom firmware home)
- DC21285 PCI host bridge with the onboard DC21143 Tulip ethernet in
  slot 10; use ``-nic user,model=tulip``

Serial ports: ``-serial`` 0 and 1 are the SuperIO 16550s (``ttyS0``
is the console), 2 is the 21285 internal UART.

Booting a current kernel built from ``netwinder_defconfig``::

  qemu-system-arm -M netwinder -kernel zImage -initrd rootfs.cpio.gz \
      -append 'console=ttyS0 rdinit=/init' \
      -display none -serial stdio

Vendor-era kernels (Rebel 2.2.x, Debian 2.4.x) expect NeTTrom boot
conventions: a load address of 0x8000 (always honoured) and a
``param_struct`` parameter block instead of ATAGs. Enable the latter
with the ``old-param`` machine property::

  qemu-system-arm -M netwinder,old-param=on \
      -kernel vmlinuz-2.4.27-netwinder -initrd initrd.gz \
      -append 'console=ttyS0 root=/dev/ram0' -serial stdio

Not yet modelled: SuperIO IDE, GPIO/LEDs and the DS1620
thermometer.
