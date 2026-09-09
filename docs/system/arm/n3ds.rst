Nintendo New 3DS / New 2DS XL (``n3ds``)
========================================

The ``n3ds`` machine models the New Nintendo 3DS family ("KTR" units,
SoC "LGR": New 3DS, New 3DS XL, New 2DS XL). It is a bare-metal and
Linux development target, not a game emulator: the PICA200 GPU, the
DSP, cameras and the game-card slot are not modelled.

Modelled hardware
-----------------

- 4x ARM11 MPCore with the MPCore private peripherals (SCU, GIC, TWD
  timers/watchdogs) at ``0x17E00000`` and an L2C-310 at ``0x17E10000``
- ARM946E-S "ARM9" security processor with its own address space
  (ITCM mirrors, 1 MiB + 512 KiB WRAM, DTCM)
- 256 MiB FCRAM (``-m 128M`` selects the original 3DS size), 6 MiB
  VRAM, 512 KiB AXI WRAM, 512 KiB DSP RAM, 4 MiB New 3DS extra RAM
- CONFIG9 / CONFIG11 / PDN system control, including the New 3DS
  core 2/3 release protocol and the clock-mode acknowledge interrupt
- TMIO SD/MMC host (``0x10006000``) with the SD card on port 0 and the
  eMMC "NAND" on port 1
- Three I2C controllers and the power-management MCU (power off,
  reboot, RTC, battery, and the 200 bytes of free storage behind
  registers ``0x60``/``0x61`` that Luma3DS keeps its boot configuration
  in; ``-global n3ds-mcu.free-regs=FILE`` backs them with a file so they
  persist across runs like the battery-backed originals, and Luma3DS
  then boots without its first-boot configuration menu)
- HID pad register, driven from the keyboard: A=X, B=Z, X=S, Y=A,
  L=Q, R=W, Start=Enter, Select=Backspace, D-pad=cursor keys
- LCD/PDC framebuffer controllers with a two-screen console (top
  400x240 above bottom 320x240), PSC memory-fill engines
- PXI ARM9<->ARM11 mailbox and FIFOs, ARM9 interrupt controller and
  timers, two PL330 CDMAs

Booting
-------

The machine boots a FIRM image the way the Luma3DS chainloader does:
sections are loaded to their physical addresses, the ARM9 starts at the
ARM9 entry point with Luma's ``argc``/``argv`` (the SD path of the
payload and the framebuffer descriptors) and the magic ``0x4BEEF`` in
``r2``, and the ARM11 entry point is published at ``0x1FFFFFFC`` where a
small boot11 stub at physical address 0 picks it up on core 0. Cores 1-3
park like the boot ROM leaves them: core 1 waits for an entry point at
``0x1FFFFFDC`` and an SGI, cores 2 and 3 are powered up through PDN.
No boot ROM dump is needed.

.. code-block:: bash

  $ qemu-system-arm -M n3ds -kernel payload.firm \
      -drive if=sd,index=0,format=raw,file=sd.img \
      -drive if=sd,index=1,format=raw,file=nand.img

``sd.img`` is a FAT32 image (``mformat -i sd.img -F ::``); ``nand.img``
backs the eMMC and can be an empty file, but must be present for
firmware that initialises the eMMC.

Instruction-count scheduling
----------------------------

When QEMU is run with ``-icount``, its round-robin TCG scheduler gives each
ARM11 CPU twice the instruction budget of the ARM9 CPU. This matches the
hardware's default 268 MHz to 134 MHz clock ratio. The relative budgets can
be changed for scheduling experiments without rebuilding QEMU:

.. code-block:: bash

  $ qemu-system-arm -M n3ds,arm11-icount-weight=3,arm9-icount-weight=1 \
      -icount shift=0,sleep=off ...

Both weights accept values from 1 through 1024 and have no effect unless
``-icount`` is enabled.

Known gaps
----------

WiFi SDIO, SPI (touchscreen, NVRAM), GPIO blocks, NDMA/XDMA, the
AES/SHA/RSA engines and OTP are logged as unimplemented accesses.
