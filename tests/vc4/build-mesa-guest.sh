#!/bin/sh
# Cross-build Mesa's vc4 driver for the ARMv6 guest and stage its runtime.
#
# This is what lets the tests drive the emulated GPU through the *real* OpenGL
# ES stack rather than hand-assembled control lists: Mesa compiles the shaders,
# builds the control lists and talks to the kernel's vc4 driver exactly as it
# would on a Raspberry Pi.
#
# Everything happens in Docker, because the usual host here is macOS with no
# ARM cross toolchain.  The result is staged in out/mesa-root/, which
# run-gles-test.sh turns into an initramfs.
#
#   usage: sh build-mesa-guest.sh [mesa-version]     (default: 23.1.9)
#
# Takes a while the first time (a full Mesa build); the unpacked tree and the
# build directory are cached in out/ and reused.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
MESA_VERSION=${1:-23.1.9}
OUT=${VC4_OUT:-$HERE/out}
WORK=$OUT/mesa
STAGE=$OUT/mesa-root
IMAGE=vc4-mesa-cross

mkdir -p "$WORK"

# The guest is ARMv6 soft-float (Pi 1 / Zero), so armel is the right target:
# Debian armhf assumes ARMv7 with NEON, which an ARM1176 does not have.
cat > "$WORK/Dockerfile" <<'EOF'
FROM debian:bookworm
RUN dpkg --add-architecture armel && apt-get update -qq && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends \
      crossbuild-essential-armel meson ninja-build pkg-config \
      python3 python3-mako python3-yaml python3-setuptools \
      bison flex git ca-certificates xz-utils \
      libdrm-dev:armel libexpat1-dev:armel zlib1g-dev:armel \
      libdrm2:armel libexpat1:armel zlib1g:armel libc6:armel \
    && rm -rf /var/lib/apt/lists/*
RUN printf '%s\n' \
  '[binaries]' \
  "c = 'arm-linux-gnueabi-gcc'" \
  "cpp = 'arm-linux-gnueabi-g++'" \
  "ar = 'arm-linux-gnueabi-ar'" \
  "strip = 'arm-linux-gnueabi-strip'" \
  "pkgconfig = 'pkg-config'" \
  '' \
  '[host_machine]' \
  "system = 'linux'" \
  "cpu_family = 'arm'" \
  "cpu = 'armv6'" \
  "endian = 'little'" \
  > /cross-armel.txt
EOF
echo "building the cross toolchain image ..."
docker build -q -t "$IMAGE" "$WORK" > /dev/null

TARBALL=$WORK/mesa-$MESA_VERSION.tar.xz
if [ ! -f "$TARBALL" ]; then
    echo "fetching mesa $MESA_VERSION ..."
    curl -sSL -o "$TARBALL" "https://archive.mesa3d.org/mesa-$MESA_VERSION.tar.xz"
fi

SRC=$WORK/mesa-$MESA_VERSION
if [ ! -d "$SRC" ]; then
    tar xf "$TARBALL" -C "$WORK"
    # ARMv6 has no NEON; see the patch header for why Mesa needs telling.
    (cd "$SRC" && patch -p1 < "$HERE/mesa-armv6-neon.patch")
fi

echo "configuring and building mesa (vc4 only, no LLVM) ..."
docker run --rm -v "$WORK":/work -v "$HERE":/vc4 "$IMAGE" bash -eu -c "
    cd /work/mesa-$MESA_VERSION
    export PKG_CONFIG_LIBDIR=/usr/lib/arm-linux-gnueabi/pkgconfig
    if [ ! -d build-armel ]; then
        meson setup build-armel --cross-file /cross-armel.txt \
            -Dgallium-drivers=vc4 -Dvulkan-drivers= -Dplatforms= \
            -Dglx=disabled -Degl=enabled -Dgbm=enabled \
            -Dgles1=disabled -Dgles2=enabled -Dopengl=false \
            -Dllvm=disabled -Dshared-glapi=enabled -Dbuildtype=release \
            -Dgallium-vdpau=disabled -Dgallium-va=disabled -Dgallium-xa=disabled \
            > /dev/null
    fi
    ninja -C build-armel

    echo 'building the GLES test ...'
    arm-linux-gnueabi-gcc -O2 -Wall -Wextra -I/work/mesa-$MESA_VERSION/include \
        /vc4/guest/gles_vc4_test.c -o /work/gles_vc4_test \
        -Lbuild-armel/src/egl -Lbuild-armel/src/mapi/es2api -lEGL -lGLESv2 \
        -Wl,-rpath-link,build-armel/src/egl \
        -Wl,-rpath-link,build-armel/src/mapi/es2api \
        -Wl,-rpath-link,build-armel/src/mapi/shared-glapi \
        -Wl,-rpath-link,build-armel/src/gbm
"

echo "staging the guest runtime ..."
docker run --rm -v "$WORK":/work -v "$STAGE":/stage "$IMAGE" bash -eu -c "
    M=/work/mesa-$MESA_VERSION/build-armel
    S=/stage
    rm -rf \$S/lib \$S/usr \$S/gles_vc4_test
    mkdir -p \$S/lib \$S/usr/lib/dri
    cp \$M/src/egl/libEGL.so.1.0.0                 \$S/lib/libEGL.so.1
    cp \$M/src/mapi/es2api/libGLESv2.so.2.0.0      \$S/lib/libGLESv2.so.2
    cp \$M/src/mapi/shared-glapi/libglapi.so.0.0.0 \$S/lib/libglapi.so.0
    cp \$M/src/gbm/libgbm.so.1.0.0                 \$S/lib/libgbm.so.1
    cp \$M/src/gallium/targets/dri/vc4_dri.so      \$S/usr/lib/dri/
    cp /work/gles_vc4_test                         \$S/

    # Collect the shared-library closure out of the armel sysroot.  The cross
    # toolchain keeps libstdc++ in its own prefix, hence the third directory.
    SYSDIRS='/lib/arm-linux-gnueabi /usr/lib/arm-linux-gnueabi /usr/arm-linux-gnueabi/lib'
    MISSING=''
    resolve() {
        for so in \$(arm-linux-gnueabi-readelf -d \"\$1\" 2>/dev/null | \
                     sed -n 's/.*Shared library: \[\(.*\)\]/\1/p'); do
            [ -e \"\$S/lib/\$so\" ] && continue
            found=''
            for d in \$SYSDIRS; do
                if [ -e \"\$d/\$so\" ]; then
                    cp -L \"\$d/\$so\" \"\$S/lib/\$so\"; found=1; resolve \"\$S/lib/\$so\"; break
                fi
            done
            [ -n \"\$found\" ] || MISSING=\"\$MISSING \$so\"
        done
    }
    for f in \$S/lib/*.so.* \$S/usr/lib/dri/vc4_dri.so \$S/gles_vc4_test; do resolve \"\$f\"; done
    cp -L /lib/arm-linux-gnueabi/ld-linux.so.3 \$S/lib/
    [ -z \"\$MISSING\" ] || { echo \"unresolved libraries:\$MISSING\" >&2; exit 1; }
"

echo
echo "staged $STAGE ($(du -sh "$STAGE" | cut -f1))"
echo "now run: sh run-gles-test.sh"
