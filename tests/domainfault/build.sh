#!/bin/sh
set -e
cd "$(dirname "$0")"
CFLAGS="--target=armv5te-none-eabi -march=armv5te -mfloat-abi=soft -ffreestanding -fno-builtin -Wall -Wextra -O1"
clang $CFLAGS -c start.S -o start.o
clang $CFLAGS -c domainfault.c -o domainfault.o
ld.lld -T domainfault.ld start.o domainfault.o -o domainfault.elf
llvm-objdump -d domainfault.elf > domainfault.dis
echo "built domainfault.elf"
