# Linux boot ladder for the AT91 machines

`run-boot-ladder.py` turns the ad-hoc Linux boots used while bringing up
`-M sam9m10g45ek` and `-M sam9g35ek` into a repeatable acceptance run. It is
the guest-level counterpart to the `at91-*` qtests: the qtests pin register
contracts, this pins that a real driver stack still probes the board's wiring
and reaches userspace.

Two rows per board:

| row | what it proves |
| --- | --- |
| `mainline` | mainline `at91_dt` kernel + diagnostics initramfs: every wired peripheral probes under its real driver, userspace is reached, and the guest powers itself down in one boot cycle |
| `migrate` | the same boot handed to a second QEMU mid-flight (once the RTC has registered): the destination finishes the boot it inherited — the diagnostics and power-down markers appear there and not on the source — without starting a boot of its own |
| `openwrt` | OpenWrt 23.05.5 with root on SD: console activation, a proof file written to the rw ext4 root, clean `poweroff`, qemu exit 0 |

The `mainline` row checks a per-board marker list, so it fails on wiring
regressions a "did it boot?" check would miss. The G35 list pins the SAM9x5
facts that differ from the G45: two DMA controllers, HSMCI0 requests on DMAC0
and HSMCI1 requests on DMAC1, TWI0 on DMAC0, and the SAM9x5 UDPHS register and
FIFO windows. The G45 list additionally asserts that the second controller is
*absent*. Both rows reject kernel faults and any second boot cycle.

## Assets

Everything lives in the git-ignored `images/` tree:

```text
images/zImage-at91_dt                  mainline at91_dt kernel
images/initramfs-armv5.cpio.gz         diagnostics initramfs (/init powers down)
images/at91sam9m10g45ek.dtb            G45 device tree
images/at91sam9g35ek.dtb               G35 device tree
images/openwrt-23.05.5/                OpenWrt kernels, SD card image, README
```

A row whose assets are missing is reported `SKIP`, not `FAIL`; the exit status
is 0 when everything selected passed, 1 on failure and 2 when a row was
skipped. The OpenWrt row boots a private copy of the card image, so the
canonical `openwrt-sd.img` is never mutated; the copy, and the `migrate` row's
saved state, are kept only when the row fails, for post-mortem.

## Running

```sh
python3 tests/guest/at91/linux/run-boot-ladder.py                  # all rows
python3 tests/guest/at91/linux/run-boot-ladder.py --board g35
python3 tests/guest/at91/linux/run-boot-ladder.py --row mainline
python3 tests/guest/at91/linux/run-boot-ladder.py -- -trace 'at91_wdt*'
```

Logs land in `build/at91-boot-ladder/<UTC stamp>/`.

## Notes

- The `mainline` row captures the console with `-serial file:`, not by
  redirecting `-nographic` stdout: libc block-buffers the latter, so a killed
  run leaves an empty-looking log even though the boot was fine.
- The OpenWrt row passes `-global at91-wdt.disabled-at-boot=on`, which models
  the write-once `WDT_MR` disable a real bootstrap performs. Without it the
  boot reset-loops racily under TCG time skew: the `at91sam9_wdt` pinger runs
  on guest jiffies while the model's 16 s deadline runs on wall-driven virtual
  time. The armed-watchdog path stays covered by the `mainline` row, which
  asserts the `at91sam9_wdt: enabled` marker.
- The OpenWrt row's root is `PARTUUID=`, not `/dev/mmcblkN`: the 5.15 at91
  dtsi has no mmc aliases, so the two HSMCI controllers race for `mmc0`.
- The `migrate` row passes the same watchdog flag, for a related reason: the
  seconds spent moving half a gigabyte of guest RAM are wall time the
  jiffies-driven pinger does not see, so an armed watchdog expires and the
  destination resets instead of finishing the boot it inherited.
- The vendor Linux4SAM NAND/UBI flow keeps its own harness next to its assets
  (`images/linux4sam-4.2/l4s-login.py`); it is a G45-only vendor-kernel flow.
