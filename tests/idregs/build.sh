#!/bin/sh
# Bare-metal build, no libc, no toolchain exotica: brew clang/lld only.
set -e
cd "$(dirname "$0")"
CFLAGS="--target=armv5te-none-eabi -march=armv5te -mfloat-abi=soft -ffreestanding -fno-builtin -Wall -Wextra -O2"
clang $CFLAGS -c start.S -o start.o
clang $CFLAGS -c idregs.c -o idregs.o
ld.lld -T idregs.ld start.o idregs.o -o idregs.elf
llvm-objdump -d idregs.elf > idregs.dis
echo "built idregs.elf"
