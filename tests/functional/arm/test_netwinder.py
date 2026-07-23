#!/usr/bin/env python3
#
# Functional test that boots Linux on a Rebel NetWinder
#
# Copyright (c) 2026 Karl Mehltretter
#
# SPDX-License-Identifier: GPL-2.0-or-later

from qemu_test import Asset, LinuxKernelTest
from qemu_test import exec_command_and_wait_for_pattern


class NetWinderTest(LinuxKernelTest):

    ASSET_KERNEL = Asset(
        ('https://raw.githubusercontent.com/kmehltretter82/'
         'risc_pc_linux_emu/fc5dc0490ade0d5ea622be4d5793a68084b55756/'
         'assets/zImage-netwinder'),
        '56171c595b55c6cb5ca71f8450213bb9a3604b9021e78ab95753c02e12890a9d')

    ASSET_INITRD = Asset(
        ('https://raw.githubusercontent.com/kmehltretter82/'
         'risc_pc_linux_emu/8f5a4bf1c9954162278f353c59a0a3e8a5bc0cbb/'
         'assets/initramfs-busybox.cpio.gz'),
        '46d8eeecb385d6a1eda924c7d706d03947b5193ebd988a5f447c1a1a88d885d0')

    def test_netwinder(self):
        self.set_machine('netwinder')
        self.vm.add_args('-append', 'console=ttyS0 rdinit=/init',
                         '-serial', 'null', '-serial', 'null',
                         '-nic', 'user,model=tulip')

        self.launch_kernel(self.ASSET_KERNEL.fetch(),
                           initrd=self.ASSET_INITRD.fetch(),
                           wait_for='PCI: DC21285 footbridge')
        self.wait_for_console_pattern('pata_sl82c105')
        self.wait_for_console_pattern('Digital DS21142/43 Tulip')
        self.wait_for_console_pattern('BusyBox on ARMv4')
        self.wait_for_console_pattern('~ #')
        exec_command_and_wait_for_pattern(self, 'uname -m', 'armv4l')


if __name__ == '__main__':
    LinuxKernelTest.main()
