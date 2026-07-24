#!/usr/bin/env python3
"""Network smoke for the sam9m10g45ek MACB model: boot the RT-Thread
network-lwip-threadsafe profile against a slirp backend and prove the
full driver+stack path with ifconfig and ICMP echo.

SPDX-License-Identifier: GPL-2.0-or-later
"""

import os
import pty
import re
import select
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
ELF = os.path.join(REPO, "build/rtthread-g45/5.2.2/artifacts/"
                   "network-lwip-threadsafe/rtthread-at91sam9g45.elf")
QEMU = os.path.join(REPO, "build/qemu-system-arm")

CMD = [
    QEMU, "-M", "sam9m10g45ek",
    "-kernel", ELF,
    "-nic", "user,net=192.168.1.0/24,host=192.168.1.1",
    "-nographic",
]

STEPS = [
    (rb"msh />", b"ifconfig\n"),
    # DHCP is enabled in this profile: the lease arrives through the
    # MACB model (broadcast RX included).  "poll" re-sends the command
    # until the pattern appears.
    (rb"ip address: 192\.168\.1\.(?!0\.)\d?[1-9]\d*", b"ifconfig\n"),
    (rb"msh />", b"ping 192.168.1.1\n"),
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
                if resp and step != POLL_STEP:
                    time.sleep(0.3)
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
