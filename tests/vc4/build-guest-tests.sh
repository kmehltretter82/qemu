#!/bin/sh
# Cross-compile the vc4 guest tests to static ARM EABI binaries.
#
# The host is normally macOS with no cross toolchain, so this builds in Docker
# with the same gcc-arm the guest kernel is built with.  Set VC4_NATIVE_CC to a
# cross compiler to skip Docker (e.g. VC4_NATIVE_CC=arm-linux-gnueabi-gcc).
#
#   usage: sh build-guest-tests.sh [name ...]     (default: all of them)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
TESTS=${*:-"vc4_p1_draw vc4_p5_clear"}
CFLAGS="-static -O2 -Wall -Wextra -I$HERE/guest/uapi -D__user="

if [ -n "$VC4_NATIVE_CC" ]; then
    for t in $TESTS; do
        # shellcheck disable=SC2086
        "$VC4_NATIVE_CC" $CFLAGS "$HERE/guest/$t.c" -o "$HERE/guest/$t"
        echo "built $HERE/guest/$t"
    done
    exit 0
fi

docker run --rm -v "$HERE/guest":/guest -w /guest -e TESTS="$TESTS" \
    ubuntu:24.04 bash -eux -c '
        export DEBIAN_FRONTEND=noninteractive
        apt-get update -qq
        apt-get install -y -qq gcc-arm-linux-gnueabi libc6-dev-armel-cross >/dev/null
        for t in $TESTS; do
            arm-linux-gnueabi-gcc -static -O2 -Wall -Wextra \
                -I/guest/uapi -D__user= "$t.c" -o "$t"
            arm-linux-gnueabi-strip "$t" || true
        done
    '
for t in $TESTS; do
    echo "built $HERE/guest/$t"
done
