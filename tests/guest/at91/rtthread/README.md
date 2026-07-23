# RT-Thread tests for the AT91SAM9G45 machine

This directory contains a reproducible, externally built guest suite for
`-M sam9m10g45ek`. Its purpose is to find AT91 model defects with a driver stack
independent of Linux, with DMA as the first priority. Generated RT-Thread source
trees, binaries, migration streams and logs stay under the git-ignored `build/`
tree; third-party release archives stay under the git-ignored
`images/other-os/` tree.

RT-Thread v5.2.2 is the primary target and already contains the exact
`bsp/at91/at91sam9g45` BSP; current upstream `master` retains it as well. The
builder extracts that BSP into a disposable tree and applies tracked fixes plus
the test overlay. v4.1.1 remains available as an archival differential with
`--version 4.1.1`. This BSP hardcodes the board/register map; it does not use a
Linux-style DTB.

## Prerequisites and source artifacts

Required host tools are SCons, Kconfiglib, and the GNU Arm Embedded bare-metal
toolchain (`arm-none-eabi-gcc`, `objcopy`, and `size`). On Ubuntu, the relevant
packages include:

```sh
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi \
  libnewlib-dev python3-kconfiglib scons
```

Place pristine release archives at:

```text
images/other-os/rt-thread/rt-thread-v5.2.2.tar.gz
images/other-os/rt-thread/rt-thread-v4.1.1.tar.gz
```

The builder rejects archives whose SHA-256 does not match:

```text
4449e87c59a7337a803625ad0f047ef8f934ad81e4c7c669adabd1289cf9813e  rt-thread-v5.2.2.tar.gz
c4af708ca5e30937ceaa0c5435bd363dba883dc2c5ba81362ecd112f2e862184  rt-thread-v4.1.1.tar.gz
```

Do not patch or replace the pristine archives. `build-rtthread-g45.sh`
extracts a disposable tree, applies `patches/`, installs `overlay/*.[ch]`,
resolves one of the `profiles/*.conf` fragments, builds, and records tool and
content hashes.

## Build and run

Build the primary baseline image:

```sh
tests/guest/at91/rtthread/build-rtthread-g45.sh --profile baseline
```

The ELF is written to:

```text
build/rtthread-g45/5.2.2/artifacts/baseline/rtthread-at91sam9g45.elf
```

Run the default D0, D1, and ten-second scheduler suites, then migrate the live
guest to a new QEMU process and rerun deterministic D0/D1 cases:

```sh
python3 tests/guest/at91/rtthread/run-g45-tests.py
```

Useful focused variants are:

```sh
python3 tests/guest/at91/rtthread/run-g45-tests.py \
  --suite d1 --no-migration
python3 tests/guest/at91/rtthread/run-g45-tests.py \
  --seed 0x12345678 --artifacts build/rtthread-g45/manual-run
```

The runner uses serial and QMP Unix sockets, gives every phase a bounded
timeout, records the exact QEMU/ELF hashes and command lines, and treats a guest
hang, reset, malformed total or missing interrupt as failure. Results include
`metadata.json`, `summary.json`, serial transcripts, QEMU logs, commands and the
migration state.

## Guest protocol

The FinSH command is `g45test`:

```text
g45test list
g45test status
g45test run <suite-or-case> [seed]
g45test all [seed]
```

Stable result records are suitable for a host parser and a physical-board log
collector:

```text
G45TEST READY protocol=1 tick_hz=1000 watchdog_disabled=1
G45TEST BEGIN d1.mem2mem seed=0x45d0a11c phase=D1
G45TEST PASS d1.mem2mem checks=52296 elapsed_ticks=4
G45TEST FAIL <case> check=<n> expected=<hex> actual=<hex> offset=<n> ...
G45TEST END suite=d1 passed=5 failed=0 skipped=0 seed=0x45d0a11c
```

The default seed is `0x45d0a11c`. Every failure reports its case, deterministic
check number, expected and actual values, and byte/register offset.

## Current coverage

The registry currently contains 21 cases:

- D0: guarded patterns, boundary/alignment copies, canary self-tests,
  deterministic PRNG/CRC32, and the ARM926 drain-write-buffer barrier.
- D1: HDMAC reset/global-enable contract; 24 mem2mem width/address/chunk
  vectors; all eight channels concurrently; four linked descriptors with
  live-register and DONE writeback checks; stop-on-DONE and selective
  source/destination descriptor reload; SUSP/RES; source, destination,
  pre-enable and mixed-order SREQ/CREQ software pacing; peripheral-controlled
  LAST; both documented WORD encodings; source and destination
  Picture-in-Picture row-hole addressing; source/destination/descriptor bus
  errors and recovery; BTC/CBTC, interrupt masking and AIC source 21.
- Core: the 1 kHz scheduler runs and switches tasks for at least ten seconds.

The D1 suite performs 54,564 checks. It first found three deterministic bugs in
the local QEMU HDMAC model: missing CHSR EMPTY reset bits, missing global-enable
gating, and missing linked-list DONE/writeback semantics. The adversarial
extension found eight more: inert suspend/resume, absent SREQ/CREQ, absent LAST,
ignored AHB transaction errors, an eight-byte interpretation of the `11b` WORD
encoding, failure to reconstruct a pending IRQ after migration, ignored
stop-on-DONE, and unconditional source/destination reload between linked
descriptors. A ninth adversarial defect was ignored SPIP/DPIP
Picture-in-Picture addressing: transfers silently used contiguous addresses
instead of inserting the programmed source and destination row holes. The
corrected model passes the unrelaxed guest suite and 24 focused
qtests, including migration with a byte held in the conversion FIFO and a queued
request, with an enabled pending interrupt, and in the middle of a PiP row.
Together with the independent USART backpressure defect, this campaign has
found 13 local-QEMU defects: 12 HDMAC and one USART. None is a released
upstream-QEMU regression because these
AT91 device models have not been submitted upstream.

The default migration run also proves that the shell and tick continue in the
destination and reruns `d0.prng` and `d1.mem2mem` there. The machine-readable
inventory and incomplete paths are in `manifests/dma-coverage.json`. The latest
complete evidence is
`build/rtthread-g45/5.2.2/results/d0-d1-pip-migration-final/`: all 21 cases
pass, and the restored guest advances from tick 10,105 to 10,245.

## Next DMA work

D0 still needs cache-alias negative tests and placement across DDR, SRAM and
TCM boundaries. D1 still needs priority/arbitration, maximum and overlapping
transfers, malformed/circular descriptors, plus the remaining read-only,
writeback, alignment and peripheral-abort error/residue cases. D2 will add beat-level peripheral
request/backpressure testing; D3 will exercise DBGU/USART, SSC and AC97 PDC
current/next-buffer state machines. Later phases cover storage, rings,
continuous fetch, cyclic streams, Linux companion tests, errors and soak.

Run the same payload on a physical SAM9M10-G45-EK when available. A QEMU pass
is not evidence that cache maintenance or ordering is correct on non-coherent
ARM926 hardware. Suspected Linux bugs require a datasheet-backed minimal
reproducer and physical-board evidence; QEMU-only behavior is never enough.

## Directory map

```text
build-rtthread-g45.sh  verified disposable build and artifact hashing
configure-profile.py  deterministic Kconfig fragment resolver
run-g45-tests.py       serial/QMP test and active-migration harness
overlay/               guest registry, D0 oracle and D1 tests
patches/               auditable G45 BSP compatibility corrections
profiles/              baseline and later peripheral feature selections
manifests/             DMA inventory, current evidence and missing paths
```

The harness and overlay are QEMU test code. RT-Thread itself remains subject to
its own Apache-2.0 license and is neither vendored nor redistributed here.
