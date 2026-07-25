#!/usr/bin/env python3
"""Run the AT91 Linux boot ladder for the SAM9M10-G45-EK and SAM9G35-EK.

Two rows per board:

  mainline  mainline at91_dt kernel + the diagnostics initramfs, booted
            non-interactively.  Passes when the board's peripherals probe
            under the real drivers, userspace is reached and the guest
            powers down by itself in a single boot cycle.
  migrate   the same boot, live-migrated to a second qemu part-way through
            (once the RTC has registered, with the USB probe still ahead).
            Passes when the destination finishes the boot it inherited
            rather than starting one of its own.
  openwrt   OpenWrt 23.05.5 with root on an SD card, driven over a pty:
            activate the console, prove the ext4 root is writable, then
            poweroff.

Both rows read their (git-ignored) assets from images/; a row whose assets
are missing is reported as SKIP rather than a failure, so the script stays
useful on a tree that has not downloaded them.

SPDX-License-Identifier: GPL-2.0-or-later
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pty
import re
import select
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]

ZIMAGE = "images/zImage-at91_dt"
INITRAMFS = "images/initramfs-armv5.cpio.gz"
OPENWRT_DIR = "images/openwrt-23.05.5"
OPENWRT_SD = f"{OPENWRT_DIR}/openwrt-sd.img"

# Faults that must not appear in any boot log.
FAULT_RE = re.compile(
    r"Unable to handle kernel|Internal error|Kernel panic|Oops"
    r"|Unhandled fault|Unhandled prefetch abort")

# Where the migrate row cuts the boot in half: the RTC has registered, the
# long USB probe and all of userspace are still ahead of the guest.
MIGRATE_TRIGGER = "registered as rtc0"

# The initramfs /init prints these and then powers the machine down.  The
# migrate row can only require these of its destination: everything earlier
# may have been printed by the source before the state was handed over.
USERSPACE_MARKERS = [
    ("userspace", r"=== AT91 test initramfs: reached userspace ==="),
    ("diagnostics", r"=== DIAGNOSTICS DONE ==="),
    ("power-down", r"reboot: Power down"),
]

COMMON_MAINLINE_MARKERS = USERSPACE_MARKERS + [
    ("watchdog armed", r"at91sam9_wdt: enabled"),
]


@dataclass
class Board:
    machine: str
    dtb: str
    model: str
    openwrt_kernel: str
    # (label, regex) pairs that must appear in the mainline boot log.
    markers: list[tuple[str, str]]
    # (label, regex) pairs that must NOT appear in the mainline boot log.
    absent: list[tuple[str, str]] = field(default_factory=list)


BOARDS = {
    "g45": Board(
        machine="sam9m10g45ek",
        dtb="images/at91sam9m10g45ek.dtb",
        model="Atmel AT91SAM9M10G45-EK",
        openwrt_kernel=f"{OPENWRT_DIR}/zImage-openwrt-5.15.167",
        markers=[
            ("dmac", r"at_hdmac ffffec00\.dma-controller: Atmel AHB DMA"),
            ("hsmci0 on dmac", r"atmel_mci fff80000\.mmc: using dma0chan"),
            ("hsmci1 on dmac", r"atmel_mci fffd0000\.mmc: using dma0chan"),
            ("spi0", r"atmel_spi fffa4000\.spi: Atmel SPI Controller version"),
            ("twi0", r"at91_i2c fff84000\.i2c: AT91 i2c bus driver"),
            ("twi1", r"at91_i2c fff88000\.i2c: AT91 i2c bus driver"),
            ("macb", r"macb fffbc000\.ethernet eth0: Cadence MACB"),
            ("rtc", r"at91_rtc fffffdb0\.rtc: registered as rtc0"),
            ("udphs regs",
             r"atmel_usba_udc 600000\.gadget: MMIO registers at "
             r"\[mem 0xfff78000-0xfff783ff\]"),
            ("udphs fifo",
             r"atmel_usba_udc 600000\.gadget: FIFO at "
             r"\[mem 0x00600000-0x0067ffff\]"),
        ],
        # The G45 has a single DMA controller; the second sam9x5 one must
        # not appear here.
        absent=[("second dmac", r"ffffee00\.dma-controller")],
    ),
    "g35": Board(
        machine="sam9g35ek",
        dtb="images/at91sam9g35ek.dtb",
        model="Atmel AT91SAM9G35-EK",
        openwrt_kernel=f"{OPENWRT_DIR}/zImage-openwrt-g35-5.15.167",
        markers=[
            ("dmac0", r"at_hdmac ffffec00\.dma-controller: Atmel AHB DMA"),
            ("dmac1", r"at_hdmac ffffee00\.dma-controller: Atmel AHB DMA"),
            # The sam9x5 peripheral DMA map splits across both controllers:
            # HSMCI0 requests land on DMAC0, HSMCI1 requests on DMAC1.
            ("hsmci0 on dmac0", r"atmel_mci f0008000\.mmc: using dma0chan"),
            ("hsmci1 on dmac1", r"atmel_mci f000c000\.mmc: using dma1chan"),
            ("twi0 on dmac0", r"at91_i2c f8010000\.i2c: using dma0chan"),
            ("ssc", r"ssc f0010000\.ssc: Atmel SSC device"),
            ("macb", r"macb f802c000\.ethernet eth0: Cadence MACB"),
            ("rtc", r"at91_rtc fffffeb0\.rtc: registered as rtc0"),
            ("udphs regs",
             r"atmel_usba_udc 500000\.gadget: MMIO registers at "
             r"\[mem 0xf803c000-0xf803c3ff\]"),
            ("udphs fifo",
             r"atmel_usba_udc 500000\.gadget: FIFO at "
             r"\[mem 0x00500000-0x0057ffff\]"),
        ],
    ),
}

# Console dialogue for the OpenWrt row: (expect, send-on-match).
OPENWRT_STEPS = [
    (b"Please press Enter to activate this console.", b"\n"),
    (b"root@OpenWrt:/#", b"grep ' / ' /proc/mounts && "
                        b"echo proof-$((6*7)) > /root/sd-proof && "
                        b"cat /root/sd-proof\n"),
    (b"proof-42", None),
    (b"root@OpenWrt:/#", b"poweroff\n"),
]


class Skip(Exception):
    """A row whose assets are not present in this tree."""


class Qmp:
    """Just enough QMP to drive a migration."""

    def __init__(self, path: Path, proc: subprocess.Popen, timeout: float):
        deadline = time.time() + timeout
        while True:
            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(str(path))
                break
            except (FileNotFoundError, ConnectionRefusedError):
                self.sock.close()
                if proc.poll() is not None:
                    raise RuntimeError("qemu exited before QMP came up")
                if time.time() > deadline:
                    raise RuntimeError("QMP socket never appeared")
                time.sleep(0.1)
        self.buffer = bytearray()
        self.recv()                      # greeting
        self.command("qmp_capabilities")

    def recv(self) -> dict:
        while b"\n" not in self.buffer:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("QMP connection closed")
            self.buffer += chunk
        line, _, rest = bytes(self.buffer).partition(b"\n")
        self.buffer = bytearray(rest)
        return json.loads(line)

    def command(self, name: str, **arguments) -> dict:
        request = {"execute": name}
        if arguments:
            request["arguments"] = arguments
        self.sock.sendall(json.dumps(request).encode() + b"\n")
        while True:
            message = self.recv()
            if "error" in message:
                raise RuntimeError(f"{name}: {message['error']}")
            if "return" in message:
                return message["return"]
            # Anything else is an asynchronous event; keep reading.

    def wait_migration(self, deadline: float) -> None:
        while True:
            status = self.command("query-migrate").get("status")
            if status == "completed":
                return
            if status in ("failed", "cancelled"):
                raise RuntimeError(f"migration {status}")
            if time.time() > deadline:
                raise RuntimeError(f"migration stuck in {status}")
            time.sleep(0.2)

    def close(self) -> None:
        self.sock.close()


def need(*paths: str) -> list[Path]:
    resolved = []
    for rel in paths:
        path = REPO_ROOT / rel
        if not path.exists():
            raise Skip(f"missing {rel}")
        resolved.append(path)
    return resolved


def check_log(text: str, board: Board, markers: list[tuple[str, str]],
              absent: list[tuple[str, str]]) -> list[str]:
    problems = []
    cycles = text.count("Booting Linux on physical CPU")
    if cycles != 1:
        problems.append(f"expected exactly one boot cycle, saw {cycles}")
    if f"Machine model: {board.model}" not in text:
        problems.append(f"machine model is not {board.model!r}")
    for label, pattern in markers:
        if not re.search(pattern, text):
            problems.append(f"missing marker: {label}")
    for label, pattern in absent:
        if re.search(pattern, text):
            problems.append(f"unexpected marker: {label}")
    fault = FAULT_RE.search(text)
    if fault:
        line = text[:fault.start()].count("\n") + 1
        problems.append(f"kernel fault at line {line}: {fault.group(0)}")
    return problems


def mainline_command(board: Board, qemu: str, log_path: Path,
                     extra: list[str]) -> list[str]:
    kernel, dtb, initrd = need(ZIMAGE, board.dtb, INITRAMFS)
    return [
        qemu, "-M", board.machine,
        "-kernel", str(kernel), "-dtb", str(dtb), "-initrd", str(initrd),
        "-append", "console=ttyS0,115200 rdinit=/init",
        # A file chardev is unbuffered; redirecting -nographic stdout to a
        # file loses the tail when the run is killed.
        "-serial", f"file:{log_path}",
        "-display", "none", "-monitor", "none",
    ] + extra


def run_mainline(board: Board, qemu: str, log_path: Path, timeout: float,
                 extra: list[str], results: Path) -> list[str]:
    cmd = mainline_command(board, qemu, log_path, extra)
    with open(os.devnull) as devnull:
        proc = subprocess.Popen(cmd, cwd=REPO_ROOT, stdin=devnull,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE)
        try:
            _, err = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            return [f"timed out after {timeout:.0f}s"]

    problems = []
    if proc.returncode != 0:
        problems.append(f"qemu exit {proc.returncode}: "
                        f"{err.decode(errors='replace').strip()}")
    text = log_path.read_text(errors="replace")
    problems += check_log(text, board, board.markers + COMMON_MAINLINE_MARKERS,
                          board.absent)
    return problems


def wait_for_marker(log_path: Path, marker: str, proc: subprocess.Popen,
                    deadline: float) -> None:
    while True:
        if log_path.exists() and marker in log_path.read_text(errors="replace"):
            return
        if proc.poll() is not None:
            raise RuntimeError(f"qemu exited before {marker!r}")
        if time.time() > deadline:
            raise RuntimeError(f"never reached {marker!r}")
        time.sleep(0.2)


def run_migrate(board: Board, qemu: str, log_path: Path, timeout: float,
                extra: list[str], results: Path) -> list[str]:
    src_log = log_path.with_name(f"{board.machine}-migrate-src.log")
    dst_log = log_path.with_name(f"{board.machine}-migrate-dst.log")
    state = results / f"{board.machine}-migrate.state"
    sock_path = results / f"{board.machine}-migrate.sock"
    deadline = time.time() + timeout
    problems = []

    def spawn(log: Path, incoming: list[str]) -> subprocess.Popen:
        cmd = (mainline_command(board, qemu, log, extra)
               # The power-on-armed watchdog does not survive a handover:
               # its deadline runs on wall-driven virtual time while the
               # at91sam9_wdt pinger runs on guest jiffies, so the seconds
               # spent moving a half-gigabyte of guest RAM expire it and the
               # destination resets instead of finishing the boot.  Model the
               # bootstrap-disabled watchdog a real board would have here;
               # the armed path stays covered by the mainline row.
               + ["-global", "at91-wdt.disabled-at-boot=on"]
               + ["-qmp", f"unix:{sock_path},server=on,wait=off"] + incoming)
        return subprocess.Popen(cmd, cwd=REPO_ROOT,
                                stdin=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE)

    src = spawn(src_log, [])
    try:
        qmp = Qmp(sock_path, src, 30.0)
        wait_for_marker(src_log, MIGRATE_TRIGGER, src, deadline)
        qmp.command("migrate", uri=f"file:{state}")
        qmp.wait_migration(deadline)
        qmp.command("quit")
        qmp.close()
        src.wait(timeout=30)
    except (RuntimeError, subprocess.TimeoutExpired) as failure:
        src.kill()
        src.wait()
        return [f"source: {failure}"]
    finally:
        sock_path.unlink(missing_ok=True)

    dst = spawn(dst_log, ["-incoming", f"file:{state}"])
    try:
        dst.communicate(timeout=max(30.0, deadline - time.time()))
    except subprocess.TimeoutExpired:
        dst.kill()
        dst.communicate()
        problems.append(f"destination timed out after {timeout:.0f}s")
    finally:
        sock_path.unlink(missing_ok=True)

    if dst.returncode not in (0, None) and not problems:
        problems.append(f"destination qemu exit {dst.returncode}")

    src_text = src_log.read_text(errors="replace")
    dst_text = dst_log.read_text(errors="replace")
    if MIGRATE_TRIGGER not in src_text:
        problems.append("source never reached the migration point")
    # The destination inherits a half-booted kernel: it must finish that
    # boot without starting one of its own.  How far the source got past
    # the trigger varies - the boot outruns the log poll - so the contract
    # is only that the finish happened on the destination.
    if "Booting Linux on physical CPU" in dst_text:
        problems.append("destination started its own boot")
    for label, pattern in USERSPACE_MARKERS[1:]:
        if not re.search(pattern, dst_text):
            problems.append(f"destination missing marker: {label}")
        if re.search(pattern, src_text):
            problems.append(f"source reached {label} before the handover")
    for text, side in ((src_text, "source"), (dst_text, "destination")):
        fault = FAULT_RE.search(text)
        if fault:
            problems.append(f"{side} kernel fault: {fault.group(0)}")
    if not problems:
        state.unlink(missing_ok=True)
    return problems


def run_openwrt(board: Board, qemu: str, log_path: Path, timeout: float,
                extra: list[str], results: Path) -> list[str]:
    kernel, dtb, sd_master = need(board.openwrt_kernel, board.dtb, OPENWRT_SD)

    # Boot a private copy: the row writes a proof file into the root
    # filesystem, and the canonical card image stays untouched.
    sd = results / f"openwrt-sd-{board.machine}.img"
    shutil.copyfile(sd_master, sd)

    cmd = [
        qemu, "-M", board.machine,
        # Real boards reach Linux with the watchdog bootstrap-disabled
        # (write-once WDT_MR).  Booting -kernel with the power-on-armed
        # watchdog also races TCG time skew: the at91sam9_wdt pinger runs on
        # guest jiffies while the model deadline runs on wall-driven virtual
        # time, so host load can stretch the ping past the 16 s budget.
        "-global", "at91-wdt.disabled-at-boot=on",
        "-kernel", str(kernel), "-dtb", str(dtb),
        "-drive", f"if=sd,format=raw,file={sd}",
        # PARTUUID sidesteps the mmc0/mmc1 probe-order race: the 5.15 at91
        # dtsi has no mmc aliases, so /dev/mmcblkN numbering is not stable.
        "-append", "console=ttyS0,115200 root=PARTUUID=4f575254-01 rootwait",
        "-nographic",
    ] + extra

    mfd, sfd = pty.openpty()
    proc = subprocess.Popen(cmd, cwd=REPO_ROOT, stdin=sfd, stdout=sfd,
                            stderr=sfd, close_fds=True)
    os.close(sfd)
    deadline = time.time() + timeout
    transcript = bytearray()
    problems = []
    buf = b""
    step = 0
    last_send = 0.0
    last_recv = time.time()

    with open(log_path, "wb") as log:
        try:
            while proc.poll() is None:
                if time.time() > deadline:
                    problems.append(f"timed out after {timeout:.0f}s "
                                    f"at step {step}/{len(OPENWRT_STEPS)}")
                    proc.kill()
                    break
                readable, _, _ = select.select([mfd], [], [], 1.0)
                if not readable:
                    # The console-activation Enter can be swallowed by the
                    # procd service storm.  Re-nudge only on genuine silence:
                    # under host load the guest echo round-trip takes tens of
                    # seconds, and nudging while output still trickles in
                    # grows the tty input queue faster than it drains.
                    now = time.time()
                    if (step == 1 and now - last_recv > 15
                            and now - last_send > 15):
                        os.write(mfd, b"\n")
                        last_send = now
                    continue
                try:
                    chunk = os.read(mfd, 4096)
                except OSError:
                    break
                if not chunk:
                    break
                last_recv = time.time()
                log.write(chunk)
                log.flush()
                buf += chunk
                transcript += chunk
                if transcript.count(b"Booting Linux on physical CPU") > 1:
                    problems.append("unexpected reboot")
                    proc.kill()
                    break
                if step < len(OPENWRT_STEPS) and OPENWRT_STEPS[step][0] in buf:
                    pattern, response = OPENWRT_STEPS[step]
                    buf = buf[buf.index(pattern) + len(pattern):]
                    if response:
                        time.sleep(0.3)
                        os.write(mfd, response)
                        last_send = time.time()
                    step += 1
            returncode = proc.wait()
        finally:
            os.close(mfd)

    if step != len(OPENWRT_STEPS):
        problems.append(f"console steps {step}/{len(OPENWRT_STEPS)}")
    if returncode != 0:
        problems.append(f"qemu exit {returncode}")
    text = transcript.decode(errors="replace")
    fault = FAULT_RE.search(text)
    if fault:
        problems.append(f"kernel fault: {fault.group(0)}")
    if f"Machine model: {board.model}" not in text:
        problems.append(f"machine model is not {board.model!r}")
    if not problems:
        sd.unlink()
    return problems


ROWS = {
    "mainline": run_mainline,
    "migrate": run_migrate,
    "openwrt": run_openwrt,
}
ROW_ORDER = ["mainline", "migrate", "openwrt"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--board", action="append", choices=sorted(BOARDS),
                        help="board to run (default: all)")
    parser.add_argument("--row", action="append", choices=sorted(ROWS),
                        help="ladder row to run (default: all)")
    parser.add_argument("--qemu",
                        default=str(REPO_ROOT / "build/qemu-system-arm"))
    parser.add_argument("--results", type=Path, default=None,
                        help="results directory "
                             "(default: build/at91-boot-ladder/<stamp>)")
    parser.add_argument("--timeout", type=float, default=600.0,
                        help="per-row wall-clock budget in seconds")
    parser.add_argument("qemu_args", nargs="*",
                        help="extra qemu arguments, e.g. -trace at91_wdt_*")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    boards = args.board or sorted(BOARDS)
    rows = [row for row in ROW_ORDER if not args.row or row in args.row]
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    results = args.results or REPO_ROOT / "build/at91-boot-ladder" / stamp
    results.mkdir(parents=True, exist_ok=True)

    if not Path(args.qemu).exists():
        print(f"no qemu binary at {args.qemu}", file=sys.stderr)
        return 1

    outcomes = []
    for name in boards:
        board = BOARDS[name]
        for row in rows:
            log_path = results / f"{name}-{row}.log"
            started = time.time()
            print(f"== {name}/{row} ...", flush=True)
            try:
                problems = ROWS[row](board, args.qemu, log_path, args.timeout,
                                     args.qemu_args, results)
                verdict = "PASS" if not problems else "FAIL"
            except Skip as skip:
                problems = [str(skip)]
                verdict = "SKIP"
            elapsed = time.time() - started
            outcomes.append((name, row, verdict, problems, elapsed))
            detail = "" if not problems else " - " + "; ".join(problems)
            print(f"   {verdict} in {elapsed:.0f}s{detail}", flush=True)

    print(f"\nlogs: {results}")
    failed = [o for o in outcomes if o[2] == "FAIL"]
    skipped = [o for o in outcomes if o[2] == "SKIP"]
    passed = [o for o in outcomes if o[2] == "PASS"]
    print(f"{len(passed)} passed, {len(failed)} failed, {len(skipped)} skipped")
    if failed:
        return 1
    return 2 if skipped else 0


if __name__ == "__main__":
    sys.exit(main())
