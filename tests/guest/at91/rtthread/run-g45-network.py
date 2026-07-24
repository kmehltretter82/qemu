#!/usr/bin/env python3
"""Network smoke for the sam9m10g45ek MACB model: boot the RT-Thread
network-lwip-threadsafe profile against a slirp backend and prove the
full driver+stack path with ifconfig and ICMP echo.

SPDX-License-Identifier: GPL-2.0-or-later
"""

import json
import os
import pty
import re
import select
import socket
import subprocess
import sys
import tempfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
ELF = os.path.join(REPO, "build/rtthread-g45/5.2.2/artifacts/"
                   "network-lwip-threadsafe/rtthread-at91sam9g45.elf")
QEMU = os.path.join(REPO, "build/qemu-system-arm")

QMP_PATH = os.path.join(tempfile.mkdtemp(prefix="g45net-"), "qmp.sock")

CMD = [
    QEMU, "-M", "sam9m10g45ek",
    "-kernel", ELF,
    "-nic", "user,id=n0,net=192.168.1.0/24,host=192.168.1.1",
    "-qmp", f"unix:{QMP_PATH},server=on,wait=off",
    "-nographic",
]


def qmp_set_link(up):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(QMP_PATH)
    stream = sock.makefile("rw")
    stream.readline()                       # greeting
    for cmd in ({"execute": "qmp_capabilities"},
                {"execute": "set_link",
                 "arguments": {"name": "n0", "up": up}}):
        stream.write(json.dumps(cmd) + "\n")
        stream.flush()
        while True:
            reply = json.loads(stream.readline())
            if "return" in reply or "error" in reply:
                assert "error" not in reply, reply
                break
    sock.close()


STEPS = [
    (rb"msh />", b"ifconfig\n"),
    # DHCP is enabled in this profile: the lease arrives through the
    # MACB model (broadcast RX included).  "poll" re-sends the command
    # until the pattern appears.
    (rb"ip address: 192\.168\.1\.(?!0\.)\d?[1-9]\d*", b"ifconfig\n"),
    (rb"msh />", b"ping 192.168.1.1\n"),
    (rb"\d+ bytes from 192\.168\.1\.1", None),
    # Link-change torture: drop the backend link, watch the PHY and
    # driver notice, restore it and prove traffic recovers.
    (rb"msh />", lambda: qmp_set_link(False)),
    (rb"link down", lambda: qmp_set_link(True)),
    (rb"link up", b"ping 192.168.1.1\n"),
    (rb"\d+ bytes from 192\.168\.1\.1", None),
]
POLL_STEP = 1

DEADLINE = time.time() + 240


def main():
    mfd, sfd = pty.openpty()
    proc = subprocess.Popen(CMD, stdin=sfd, stdout=sfd, stderr=sfd,
                            close_fds=True)
    os.close(sfd)
    buf = b""
    step = 0
    last_poll = 0.0
    try:
        while proc.poll() is None and step < len(STEPS):
            if time.time() > DEADLINE:
                print(f"TIMEOUT at step {step}; tail: "
                      f"{buf[-400:].decode('utf-8', 'replace')}", flush=True)
                proc.kill()
                return 1
            r, _, _ = select.select([mfd], [], [], 1.0)
            if step == POLL_STEP and time.time() - last_poll > 3.0:
                os.write(mfd, STEPS[POLL_STEP][1])
                last_poll = time.time()
            if not r:
                continue
            try:
                chunk = os.read(mfd, 4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            pat, resp = STEPS[step]
            match = re.search(pat, buf)
            if match:
                print(f"matched: {pat.decode()}", flush=True)
                buf = buf[match.end():]
                if resp is not None and step != POLL_STEP:
                    time.sleep(0.3)
                    if callable(resp):
                        resp()
                    else:
                        os.write(mfd, resp)
                step += 1
        proc.kill()
        proc.wait()
        print(f"steps completed: {step}/{len(STEPS)}", flush=True)
        return 0 if step == len(STEPS) else 1
    finally:
        pass


if __name__ == "__main__":
    sys.exit(main())
