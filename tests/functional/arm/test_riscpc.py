#!/usr/bin/env python3
#
# Functional test that boots Linux on an Acorn RiscPC
#
# Copyright (c) 2026 Karl Mehltretter
#
# SPDX-License-Identifier: GPL-2.0-or-later

from qemu_test import Asset, LinuxKernelTest
from qemu_test import exec_command_and_wait_for_pattern


class RiscPCTest(LinuxKernelTest):

    ASSET_KERNEL = Asset(
        ('https://raw.githubusercontent.com/kmehltretter82/'
         'risc_pc_linux_emu/8f5a4bf1c9954162278f353c59a0a3e8a5bc0cbb/'
         'assets/zImage'),
        '6e60cd1b036a38e21a1d2a746e922a01bed05f64c6b116fc0ceaa987d66db3bf')

    ASSET_INITRD = Asset(
        ('https://raw.githubusercontent.com/kmehltretter82/'
         'risc_pc_linux_emu/8f5a4bf1c9954162278f353c59a0a3e8a5bc0cbb/'
         'assets/initramfs-busybox.cpio.gz'),
        '46d8eeecb385d6a1eda924c7d706d03947b5193ebd988a5f447c1a1a88d885d0')

    def test_riscpc(self):
        self.set_machine('riscpc')
        self.vm.add_args('-append', 'console=ttyS0 rdinit=/init')

        self.launch_kernel(self.ASSET_KERNEL.fetch(),
                           initrd=self.ASSET_INITRD.fetch(),
                           wait_for='Machine: Acorn-RiscPC')
        self.wait_for_console_pattern('BusyBox on ARMv4')
        self.wait_for_console_pattern('~ #')
        exec_command_and_wait_for_pattern(self, 'uname -m', 'armv4l')


if __name__ == '__main__':
    LinuxKernelTest.main()
