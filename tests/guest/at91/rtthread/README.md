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

The registry currently contains 27 cases:

- D0: guarded patterns, boundary/alignment copies, canary self-tests,
  deterministic PRNG/CRC32, and the ARM926 drain-write-buffer barrier.
- D1: HDMAC reset/global-enable contract; 24 mem2mem width/address/chunk
  vectors; all eight channels concurrently; four linked descriptors with
  live-register and DONE writeback checks; stop-on-DONE and selective
  source/destination descriptor reload; SUSP/RES; source, destination,
  pre-enable and mixed-order SREQ/CREQ software pacing; peripheral-controlled
  LAST; both documented WORD encodings; source and destination
  Picture-in-Picture row-hole addressing; source/destination/descriptor bus
  errors and recovery; non-descriptor AUTO replay with source/destination
  reload, BTC-boundary STALLED and KEEPON; descriptor-coupled AUTO rows 7/8
  with replayed BTSIZE and LLI control/address selection; a cross-page LLI;
  live mutation of a future descriptor while its predecessor is active;
  Row-1 termination despite a poisoned nonzero next pointer; read-only
  destination, peripheral-access and descriptor-writeback aborts;
  fixed versus modified-round-robin one-transaction contention order; a full
  `0xffff`-word transfer crossing 4 KiB, 64 KiB and 1 MiB boundaries; exact
  DDR-end completion; safe forward and reverse 8 KiB overlaps; BTC/CBTC,
  interrupt masking and AIC source 21.
- D2: sub-buffer arbitration between two concurrently pending unpaced
  channels writing one shared fixed word - modified round-robin interleaves
  at chunk granularity so the long channel's tail lands last, while fixed
  priority drains the lower channel first so the short channel's word lands
  last, independent of the starting cursor.
- Core: the 1 kHz scheduler runs and switches tasks for at least ten seconds.

The D1 suite performs 340,206 checks. It first found three deterministic bugs in
the local QEMU HDMAC model: missing CHSR EMPTY reset bits, missing global-enable
gating, and missing linked-list DONE/writeback semantics. The adversarial
extension found eight more: inert suspend/resume, absent SREQ/CREQ, absent LAST,
ignored AHB transaction errors, an eight-byte interpretation of the `11b` WORD
encoding, failure to reconstruct a pending IRQ after migration, ignored
stop-on-DONE, and unconditional source/destination reload between linked
descriptors. A ninth adversarial defect was ignored SPIP/DPIP
Picture-in-Picture addressing: transfers silently used contiguous addresses
instead of inserting the programmed source and destination row holes. The next
oracle exposed ignored AUTO/SRC_REP/DST_REP semantics: a completed replay
buffer disabled the channel instead of reloading it, asserting STALLED at an
unmasked BTC boundary, and waiting for KEEPON. A later contention oracle found
that `GCFG.ARB_CFG` was stored but ignored: both modes always granted channels
in ascending order. The boundary/overlap oracle did not expose another model
defect; it defines the supported overlap policy as incrementing when the
destination is below the source and decrementing in the reverse arrangement.
Unsafe overlap ordering is deliberately unspecified because later beat/burst
pacing will change transaction granularity. The descriptor adversarial oracle
then exposed two more defects: rows 7/8 loaded the descriptor's poisoned
BTSIZE instead of replaying the channel's initial value, and the cyclic-list
pre-scan recognized only loops back to the head, allowing a tail cycle to
execute 1,024 buffers before a synthetic error. The corrected model merges
only replayed BTSIZE with LLI control fields and parks any reachable cyclic
graph. The final adversarial pass found that Row 1 still followed a nonzero
next pointer even though the hardware samples the Table 40-2 row at each
buffer boundary. Row 1 now terminates from its control state, while a focused
test poisons the normally-zero next pointer. The first D2 increment then made
hardware handshakes chunk-scoped: one request edge grants one SCSIZE/DCSIZE
chunk rather than permission to drain the descriptor, source and destination
requests act independently through the conversion FIFO, and an unpaced memory
side legitimately prefetches ahead of a paced peripheral side. Bringing that
up exposed one more model hazard, caught by the Linux differential before it
was ever committed: request edges were banked even while no armed channel
listened, so the request-line toggling that every HSMCI driver-PIO transfer
performs banked a stale grant, and the first Linux DMA transfer (the ACMD13
SD-Status read) consumed one beat before data existed, leaving the card
wedged mid-transfer with no XFRDONE ("mmc0: problem reading SD Status
register", no `mmcblk0`). Edges are now visible only to armed channels and
CHER samples the live request level; the at_hdmac-shaped
`acmd13-completion` qtest holds the regression. The second D2 increment
extends this: the HSMCI honors `HSMCI_DMA.DMAEN` exactly as the driver
programs it (set for DMA transfers, cleared for CPU transfers), so a PIO
transfer never toggles the request line at all; a banked edge whose only
armed consumer is disabled is forgotten; concurrently pending unpaced
channels interleave at SCSIZE/DCSIZE chunk granularity under GCFG.ARB_CFG
instead of running whole buffers back to back; and descriptor-versus-
transaction length mismatches have deterministic outcomes - a short
descriptor stops at BTSIZE with BTC while the card transaction stays in
progress until STOP, and a long descriptor waits with exact residue after
XFRDONE until the driver disables the channel, with byte-exact reuse
afterward. The third D2 increment models the HSMCI DTOR data timeout:
DTOCYC scaled by DTOMUL counts MCK cycles between two data accesses, an
access restarts the counter, and expiry raises read-to-clear DTOE with the
data phase ended - exactly the -ETIMEDOUT the atmel-mci driver reports for
a stuck transfer. Both mismatch boundaries and a mid-countdown timeout
migrate exactly. The fourth D2 increment wires the SPI0/SPI1 hardware
request routes (Table 40-1 ids 1..4): the SPI TX request follows TDRE with
a per-write rearm edge held until RDR is drained, RX follows RDRF, and a
byte-width full-duplex qtest reads the board flash's JEDEC id through both
SPI0 routes exactly as spi-atmel programs at_hdmac. Bringing that up with
a dmas-patched DTB (mainline G45 DTs never declare SPI dmas) caught a real
contract bug the word-width qtests missed: the SPI data registers only
accepted 4-byte MMIO accesses, so the driver's 8-bit DMA beats faulted -
the same 1..4-byte data-register contract the TWI needed. A 1 MiB
/dev/mtd0 DMA read now md5-matches the host pattern. The fifth increment
wires the SSC0/SSC1 routes (ids 5..8) with the same TX-rearm/RX-natural
contract and round-trips loopback patterns through both controllers at
halfword and byte widths; the AC97 route (ids 9/10) is documented
not-applicable - the model's AC97C data path is its embedded PDC (as is
the mainline driver's) and CARHR/CATHR are register stubs. The model
passes the
unrelaxed guest suite and 51 focused DMAC qtests plus 9 HSMCI qtests,
including migration with a byte
held in the conversion FIFO and a queued request, with an enabled pending
interrupt, in the middle of a PiP row, at an AUTO replay stalled boundary, and
after a round-robin grant.
Together with the independent USART backpressure defect, the DMA/RT-Thread
phase has found 19 local-QEMU defects: 18 HDMAC and one USART. The interleaved
PIO qtests subsequently made three previously inert behaviors executable
(mux handoff, multi-drive resolution and the MCK glitch filter), bringing the
whole local campaign total to 22. None is a released upstream-QEMU regression
because these AT91 device models have not been submitted upstream.

The default migration run also proves that the shell and tick continue in the
destination and reruns `d0.prng` and `d1.mem2mem` there. The machine-readable
inventory and incomplete paths are in `manifests/dma-coverage.json`. The latest
complete evidence is
`build/rtthread-g45/5.2.2/results/d0-d2-subbuffer-final/`: all 27 cases pass
(D2 adds 134 checks), and the restored guest advances from tick 10,146 to
10,294. This run uses the PIO mux/filter/open-drain, LCDC palette/EOF/timing,
HSMCI timing/reentrancy/DMAEN and HDMAC AUTO/arbitration/boundary/descriptor/
error coverage plus chunk-scoped, level-sampled request pacing with
sub-buffer interleave, so it is the current whole-machine migration baseline.

## Next DMA work

D0 still needs cache-alias negative tests and placement across DDR, SRAM and
TCM boundaries. D1 is complete: its final pass covers live and disabled-next
descriptor behavior plus deterministic read-only, peripheral and writeback
faults without test-only model hooks. Unaligned RAM is not treated as an error
oracle because the DMAC datasheet does not require it to produce an AHB error
response. D2 now covers chunk-scoped hardware handshakes with independent
source/destination pacing, live level sampling at CHER and on channel
disable, HSMCI_DMA.DMAEN gating, sub-buffer channel interleave under both
arbitration modes, descriptor-versus-transaction length mismatches with
driver-shaped recovery and migration at those boundaries, and the DTOR
data timeout with a migrated countdown, and every applicable Table 40-1
request route (HSMCI 0/13, SPI 1..4, SSC 5..8; AC97 9/10 is
not-applicable to this model's PDC-based AC97C). Still open in D2 are
beat granularity inside a chunk, FIFO_CFG thresholds and physical card
removal; D3 will exercise DBGU/USART, SSC and AC97 PDC
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
