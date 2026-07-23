#!/usr/bin/env python3
"""Run RT-Thread SAM9G45 differential suites and an active migration.

SPDX-License-Identifier: GPL-2.0-or-later
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from typing import Any


PROTOCOL_VERSION = 1
DEFAULT_SEED = 0x45D0A11C
PROMPT_RE = re.compile(rb"msh />")
DEFAULT_SUITES = ("d0", "d1", "core.scheduler")
EXPECTED_PASSES = {
    "d0": 5,
    "d1": 15,
    "core.scheduler": 1,
}


class TestFailure(RuntimeError):
    """A deterministic guest or host-side test failure."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class QMPClient:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buffer = bytearray()
        self.command_id = 0

    @classmethod
    def connect(cls, path: Path, process: subprocess.Popen[bytes],
                timeout: float) -> "QMPClient":
        deadline = time.monotonic() + timeout
        last_error: OSError | None = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise TestFailure(
                    f"QEMU exited with status {process.returncode} before QMP"
                )
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                sock.connect(str(path))
                sock.settimeout(0.5)
                client = cls(sock)
                greeting = client._receive(deadline)
                if "QMP" not in greeting:
                    raise TestFailure(f"invalid QMP greeting: {greeting!r}")
                client.execute("qmp_capabilities", timeout=timeout)
                return client
            except OSError as error:
                last_error = error
                sock.close()
                time.sleep(0.05)
        raise TestFailure(f"timed out connecting QMP at {path}: {last_error}")

    def _receive(self, deadline: float) -> dict[str, Any]:
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                raw = bytes(self.buffer[:newline])
                del self.buffer[:newline + 1]
                if raw.strip():
                    return json.loads(raw)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TestFailure("QMP response timeout")
            self.sock.settimeout(min(0.5, remaining))
            try:
                data = self.sock.recv(65536)
            except TimeoutError:
                continue
            if not data:
                raise TestFailure("QMP disconnected")
            self.buffer.extend(data)

    def execute(self, command: str, arguments: dict[str, Any] | None = None,
                timeout: float = 10.0) -> Any:
        self.command_id += 1
        message: dict[str, Any] = {
            "execute": command,
            "id": self.command_id,
        }
        if arguments is not None:
            message["arguments"] = arguments
        self.sock.sendall(json.dumps(message).encode("utf-8") + b"\n")
        deadline = time.monotonic() + timeout
        while True:
            response = self._receive(deadline)
            if response.get("id") != self.command_id:
                continue
            if "error" in response:
                raise TestFailure(
                    f"QMP {command} failed: {response['error']}"
                )
            return response.get("return")

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class SerialStream:
    def __init__(self, sock: socket.socket, transcript: Path,
                 process: subprocess.Popen[bytes]):
        self.sock = sock
        self.process = process
        self.buffer = bytearray()
        self.transcript = transcript.open("wb")

    @classmethod
    def connect(cls, path: Path, transcript: Path,
                process: subprocess.Popen[bytes], timeout: float) -> "SerialStream":
        deadline = time.monotonic() + timeout
        last_error: OSError | None = None
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise TestFailure(
                    f"QEMU exited with status {process.returncode} before serial"
                )
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                sock.connect(str(path))
                sock.settimeout(0.25)
                return cls(sock, transcript, process)
            except OSError as error:
                last_error = error
                sock.close()
                time.sleep(0.05)
        raise TestFailure(
            f"timed out connecting serial socket at {path}: {last_error}"
        )

    def mark(self) -> int:
        return len(self.buffer)

    def send_line(self, line: str) -> None:
        self.sock.sendall(line.encode("ascii") + b"\n")

    def _pump(self) -> None:
        try:
            data = self.sock.recv(65536)
        except TimeoutError:
            return
        if not data:
            raise TestFailure("guest serial socket disconnected")
        self.buffer.extend(data)
        self.transcript.write(data)
        self.transcript.flush()

    def wait_regex(self, pattern: re.Pattern[bytes], start: int,
                   timeout: float) -> tuple[re.Match[bytes], int]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            match = pattern.search(self.buffer, start)
            if match is not None:
                return match, match.end()
            if self.process.poll() is not None:
                tail = bytes(self.buffer[-1000:]).decode("utf-8", "replace")
                raise TestFailure(
                    f"QEMU exited with status {self.process.returncode}; "
                    f"serial tail:\n{tail}"
                )
            self._pump()
        tail = bytes(self.buffer[max(start, len(self.buffer) - 2000):])
        raise TestFailure(
            f"timeout waiting for {pattern.pattern!r}; serial tail:\n"
            f"{tail.decode('utf-8', 'replace')}"
        )

    def wait_prompt(self, start: int, timeout: float = 10.0) -> int:
        _, end = self.wait_regex(PROMPT_RE, start, timeout)
        return end

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
        self.transcript.close()


class QemuVM:
    def __init__(self, qemu: Path, elf: Path, machine: str, tag: str,
                 socket_dir: Path, artifacts: Path,
                 incoming: Path | None = None, nic: str | None = None):
        self.qemu_binary = qemu
        self.elf = elf
        self.machine = machine
        self.tag = tag
        self.socket_dir = socket_dir
        self.artifacts = artifacts
        self.incoming = incoming
        self.serial_path = socket_dir / f"{tag}-serial.sock"
        self.qmp_path = socket_dir / f"{tag}-qmp.sock"
        self.log_path = artifacts / f"{tag}-qemu.log"
        self.transcript_path = artifacts / f"{tag}-serial.log"
        self.process: subprocess.Popen[bytes] | None = None
        self.qmp: QMPClient | None = None
        self.serial: SerialStream | None = None
        self.log_stream: Any = None
        self.command = [
            str(qemu),
            "-M", machine,
            "-device", f"loader,file={elf},cpu-num=0",
            "-chardev",
            f"socket,id=uart0,path={self.serial_path},server=on,wait=off",
            "-serial", "chardev:uart0",
            "-qmp", f"unix:{self.qmp_path},server=on,wait=off",
            "-display", "none",
            "-monitor", "none",
            "-no-reboot",
            "-S",
        ]
        if incoming is not None:
            self.command.extend(["-incoming", f"file:{incoming}"])
        if nic is not None:
            self.command.extend(["-nic", nic])

    def start(self, timeout: float = 30.0) -> None:
        for path in (self.serial_path, self.qmp_path):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        self.log_stream = self.log_path.open("wb")
        self.process = subprocess.Popen(
            self.command,
            stdin=subprocess.DEVNULL,
            stdout=self.log_stream,
            stderr=subprocess.STDOUT,
        )
        self.qmp = QMPClient.connect(self.qmp_path, self.process, timeout)
        self.serial = SerialStream.connect(
            self.serial_path, self.transcript_path, self.process, timeout
        )

    def stop(self, qmp_quit: bool = True) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None
        if self.qmp is not None:
            if qmp_quit and self.process is not None and self.process.poll() is None:
                try:
                    self.qmp.execute("quit", timeout=3.0)
                except (OSError, TestFailure):
                    pass
            self.qmp.close()
            self.qmp = None
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5.0)
        if self.log_stream is not None:
            self.log_stream.close()
            self.log_stream = None


def wait_for_running(vm: QemuVM, timeout: float) -> None:
    assert vm.qmp is not None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = vm.qmp.execute("query-status")
        state = status["status"]
        if state == "running":
            return
        if state in ("paused", "prelaunch"):
            vm.qmp.execute("cont")
        elif state not in ("inmigrate", "postmigrate"):
            raise TestFailure(
                f"{vm.tag} VM entered unexpected state {state!r}"
            )
        time.sleep(0.05)
    raise TestFailure(f"{vm.tag} VM did not enter running state")


def wait_for_migration(vm: QemuVM, state_file: Path, timeout: float) -> None:
    assert vm.qmp is not None
    vm.qmp.execute("migrate", {"uri": f"file:{state_file}"})
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = vm.qmp.execute("query-migrate")
        migration_state = status.get("status")
        if migration_state == "completed":
            return
        if migration_state in ("failed", "cancelled"):
            raise TestFailure(f"migration {migration_state}: {status}")
        time.sleep(0.05)
    raise TestFailure("migration did not complete")


def wait_for_boot(vm: QemuVM, timeout: float) -> None:
    assert vm.serial is not None
    _, ready_end = vm.serial.wait_regex(
        re.compile(
            rb"G45TEST READY protocol=(\d+) tick_hz=(\d+) "
            rb"watchdog_disabled=(\d+)\r?\n"
        ),
        0,
        timeout,
    )
    ready = re.search(
        rb"G45TEST READY protocol=(\d+) tick_hz=(\d+) "
        rb"watchdog_disabled=(\d+)",
        vm.serial.buffer,
    )
    assert ready is not None
    if int(ready.group(1)) != PROTOCOL_VERSION:
        raise TestFailure(f"guest protocol is {int(ready.group(1))}, expected 1")
    if int(ready.group(2)) != 1000:
        raise TestFailure(f"guest tick rate is {int(ready.group(2))}, expected 1000")
    if int(ready.group(3)) != 1:
        raise TestFailure("guest watchdog was not disabled before the test run")
    vm.serial.wait_prompt(ready_end, timeout)


def guest_list(vm: QemuVM) -> int:
    assert vm.serial is not None
    start = vm.serial.mark()
    vm.serial.send_line("g45test list")
    match, end = vm.serial.wait_regex(
        re.compile(rb"G45TEST LIST count=(\d+) protocol=(\d+)\r?\n"),
        start,
        10.0,
    )
    count = int(match.group(1))
    if count < 11 or int(match.group(2)) != PROTOCOL_VERSION:
        raise TestFailure(f"unexpected guest test registry: {match.group(0)!r}")
    vm.serial.wait_prompt(end, 10.0)
    return count


def guest_status(vm: QemuVM) -> int:
    assert vm.serial is not None
    start = vm.serial.mark()
    vm.serial.send_line("g45test status")
    match, end = vm.serial.wait_regex(
        re.compile(
            rb"G45TEST STATUS protocol=(\d+) tick=(\d+) tick_hz=(\d+) "
            rb"watchdog_disabled=(\d+) watchdog_mr=0x[0-9a-fA-F]+\r?\n"
        ),
        start,
        10.0,
    )
    protocol, tick, tick_hz, watchdog = map(int, match.groups())
    if protocol != PROTOCOL_VERSION or tick_hz != 1000 or watchdog != 1:
        raise TestFailure(f"bad guest status: {match.group(0)!r}")
    vm.serial.wait_prompt(end, 10.0)
    return tick


def guest_run(vm: QemuVM, selection: str, seed: int, expected_passes: int,
              timeout: float) -> bytes:
    assert vm.serial is not None
    start = vm.serial.mark()
    vm.serial.send_line(f"g45test run {selection} 0x{seed:08x}")
    end_pattern = re.compile(
        rb"G45TEST END suite=" + re.escape(selection.encode("ascii")) +
        rb" passed=(\d+) failed=(\d+) skipped=(\d+) "
        rb"seed=0x([0-9a-fA-F]+)\r?\n"
    )
    match, end = vm.serial.wait_regex(end_pattern, start, timeout)
    region = bytes(vm.serial.buffer[start:end])
    passed, failed, skipped = map(int, match.groups()[:3])
    returned_seed = int(match.group(4), 16)
    if b"G45TEST FAIL " in region:
        raise TestFailure(
            f"guest reported a failure in {selection}:\n"
            f"{region.decode('utf-8', 'replace')}"
        )
    if (passed, failed, skipped) != (expected_passes, 0, 0):
        raise TestFailure(
            f"unexpected {selection} totals: "
            f"passed={passed} failed={failed} skipped={skipped}"
        )
    if returned_seed != seed:
        raise TestFailure(
            f"guest returned seed 0x{returned_seed:08x}, expected 0x{seed:08x}"
        )
    vm.serial.wait_prompt(end, 10.0)
    return region


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(
        description="Run RT-Thread SAM9G45 differential and migration tests"
    )
    parser.add_argument(
        "--qemu", type=Path,
        default=repo_root / "build/qemu-system-arm",
        help="qemu-system-arm binary",
    )
    parser.add_argument(
        "--elf", type=Path,
        default=repo_root /
        "build/rtthread-g45/5.2.2/artifacts/baseline/"
        "rtthread-at91sam9g45.elf",
        help="RT-Thread test ELF",
    )
    parser.add_argument("--machine", default="sam9m10g45ek")
    parser.add_argument(
        "--nic", help="QEMU -nic argument, for example 'user'"
    )
    parser.add_argument(
        "--seed", type=lambda value: int(value, 0), default=DEFAULT_SEED
    )
    parser.add_argument(
        "--suite", action="append", choices=tuple(EXPECTED_PASSES),
        help="suite to run (repeatable; default: d0, d1, core.scheduler)",
    )
    parser.add_argument(
        "--artifacts", type=Path,
        default=repo_root / "build/rtthread-g45/5.2.2/results" /
        dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ"),
    )
    parser.add_argument("--no-migration", action="store_true")
    parser.add_argument("--boot-timeout", type=float, default=45.0)
    return parser.parse_args()


def command_output(command: list[str], cwd: Path | None = None) -> str:
    try:
        return subprocess.check_output(
            command, cwd=cwd, text=True, stderr=subprocess.STDOUT
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[4]
    qemu = args.qemu.resolve()
    elf = args.elf.resolve()
    artifacts = args.artifacts.resolve()
    if not qemu.is_file() or not os.access(qemu, os.X_OK):
        raise TestFailure(f"QEMU binary is missing or not executable: {qemu}")
    if not elf.is_file():
        raise TestFailure(f"guest ELF is missing: {elf}")
    if not 0 <= args.seed <= 0xFFFFFFFF:
        raise TestFailure("seed must fit in 32 bits")

    artifacts.mkdir(parents=True, exist_ok=False)
    socket_dir = Path(tempfile.mkdtemp(prefix="g45rt-"))
    source: QemuVM | None = None
    destination: QemuVM | None = None
    summary: dict[str, Any] = {
        "result": "FAIL",
        "seed": f"0x{args.seed:08x}",
        "protocol": PROTOCOL_VERSION,
        "artifacts": str(artifacts),
    }
    commands: list[list[str]] = []

    metadata = {
        "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "qemu": str(qemu),
        "qemu_sha256": sha256_file(qemu),
        "qemu_version": command_output([str(qemu), "--version"]).splitlines()[0],
        "qemu_git_commit": command_output(["git", "rev-parse", "HEAD"], repo_root),
        "elf": str(elf),
        "elf_sha256": sha256_file(elf),
        "machine": args.machine,
        "seed": f"0x{args.seed:08x}",
        "migration": not args.no_migration,
        "nic": args.nic,
        "suites": args.suite or list(DEFAULT_SUITES),
    }
    (artifacts / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )

    try:
        source = QemuVM(
            qemu, elf, args.machine, "source", socket_dir, artifacts,
            nic=args.nic
        )
        commands.append(source.command)
        source.start(timeout=args.boot_timeout)
        wait_for_running(source, 10.0)
        wait_for_boot(source, args.boot_timeout)
        summary["registry_count"] = guest_list(source)
        summary["initial_tick"] = guest_status(source)

        for suite in args.suite or DEFAULT_SUITES:
            suite_timeout = 90.0 if suite == "d1" else 60.0
            output = guest_run(
                source, suite, args.seed, EXPECTED_PASSES[suite], suite_timeout
            )
            if suite == "core.scheduler":
                scheduler_match = re.search(
                    rb"G45TEST PASS core\.scheduler checks=\d+ "
                    rb"elapsed_ticks=(\d+)", output,
                )
                if (scheduler_match is None or
                        int(scheduler_match.group(1)) < 10000):
                    raise TestFailure(
                        "scheduler case did not span 10,000 guest ticks"
                    )
                summary["scheduler_elapsed_ticks"] = int(
                    scheduler_match.group(1)
                )

        before_migration = guest_status(source)
        summary["pre_migration_tick"] = before_migration
        if not args.no_migration:
            state_file = artifacts / "migration.state"
            wait_for_migration(source, state_file, 30.0)
            summary["migration_state_bytes"] = state_file.stat().st_size
            source.stop()
            source = None

            destination = QemuVM(
                qemu, elf, args.machine, "destination", socket_dir,
                artifacts, incoming=state_file, nic=args.nic
            )
            commands.append(destination.command)
            destination.start(timeout=30.0)
            wait_for_running(destination, 30.0)
            assert destination.serial is not None
            serial_mark = destination.serial.mark()
            destination.serial.send_line("")
            destination.serial.wait_prompt(serial_mark, 10.0)
            after_migration = guest_status(destination)
            if after_migration <= before_migration:
                time.sleep(0.1)
                after_migration = guest_status(destination)
            if after_migration <= before_migration:
                raise TestFailure(
                    f"guest tick did not advance across migration: "
                    f"before={before_migration} after={after_migration}"
                )
            summary["post_migration_tick"] = after_migration
            guest_run(destination, "d0.prng", args.seed, 1, 15.0)
            guest_run(destination, "d1.mem2mem", args.seed, 1, 60.0)
        summary["result"] = "PASS"
        return 0
    finally:
        if destination is not None:
            destination.stop()
        if source is not None:
            source.stop()
        shutil.rmtree(socket_dir, ignore_errors=True)
        (artifacts / "commands.json").write_text(
            json.dumps(commands, indent=2) + "\n", encoding="utf-8"
        )
        summary["finished_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        (artifacts / "summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        if summary["result"] == "PASS":
            print(f"G45 host test PASS: {artifacts}")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except TestFailure as error:
        print(f"G45 host test FAIL: {error}", file=sys.stderr)
        sys.exit(1)
