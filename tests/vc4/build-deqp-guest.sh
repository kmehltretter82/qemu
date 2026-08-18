#!/bin/sh
# Cross-build dEQP-GLES2 (the Khronos GLES2 conformance module) for the guest.
#
# This is the strictest workload available for the model: dEQP renders with the
# real driver and compares against its own reference renderer.  Running the whole
# module is not practical here -- it is roughly 19000 tests and a single guest
# boot takes minutes under TCG with soft-float Mesa -- but individual groups are,
# and they are a far harder oracle than anything hand-written.
#
#   usage: sh build-deqp-guest.sh
#
# Needs Docker, and build-mesa-guest.sh to have run first: dEQP links against
# that Mesa.  Expect a long first run; the clone alone is ~2 GB.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${VC4_OUT:-$HERE/out}
WORK=$OUT/deqp
MESA=$OUT/mesa
STAGE=$OUT/deqp-root
PREFIX=$OUT/mesa-prefix
IMAGE=vc4-deqp-cross

[ -d "$MESA" ] || { echo "run build-mesa-guest.sh first" >&2; exit 2; }
mkdir -p "$WORK" "$PREFIX"

cat > "$WORK/Dockerfile" <<'EOF'
FROM vc4-mesa-cross
RUN apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
      --no-install-recommends cmake ninja-build git ca-certificates python3 \
    && rm -rf /var/lib/apt/lists/*
EOF
echo "building the dEQP toolchain image ..."
docker build -q -t "$IMAGE" "$WORK" > /dev/null

# dEQP needs Mesa as an installed prefix: its surfaceless target looks for
# egl.pc, and the linker needs somewhere to find -lEGL.
if [ ! -f "$PREFIX/usr/local/lib/pkgconfig/egl.pc" ]; then
    echo "installing mesa into a prefix ..."
    # The Mesa build directory has its build path baked in, so mount it back
    # where it was configured.
    docker run --rm -v "$MESA":/work -v "$PREFIX":/prefix "$IMAGE" \
        sh -c 'cd /work/mesa-*/ && DESTDIR=/prefix meson install -C build-armel' \
        > /dev/null
fi

if [ ! -d "$WORK/VK-GL-CTS" ]; then
    echo "cloning VK-GL-CTS (large) ..."
    docker run --rm -v "$WORK":/w -w /w "$IMAGE" \
        git clone --depth 1 https://github.com/KhronosGroup/VK-GL-CTS.git
    echo "fetching its external sources ..."
    docker run --rm -v "$WORK":/w -w /w/VK-GL-CTS "$IMAGE" \
        python3 external/fetch_sources.py
fi

cat > "$WORK/toolchain-armel.cmake" <<'EOF'
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER   arm-linux-gnueabi-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabi-g++)
set(CMAKE_FIND_ROOT_PATH /prefix/usr/local /usr/arm-linux-gnueabi)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
EOF

echo "configuring and building deqp-gles2 ..."
docker run --rm -v "$WORK":/w -v "$PREFIX":/prefix -w /w "$IMAGE" bash -euc '
    # egl.pc pulls in libdrm, so both pkgconfig directories are needed.
    export PKG_CONFIG_LIBDIR=/prefix/usr/local/lib/pkgconfig:/usr/lib/arm-linux-gnueabi/pkgconfig
    cmake -S /w/VK-GL-CTS -B /w/build-gles2 -G Ninja \
        -DCMAKE_TOOLCHAIN_FILE=/w/toolchain-armel.cmake \
        -DDEQP_TARGET=surfaceless \
        -DCMAKE_BUILD_TYPE=Release \
        -DDE_CPU=DE_CPU_ARM \
        -DDE_CPU_IS_ARM_NEON=0 \
        -DCMAKE_EXE_LINKER_FLAGS=-L/prefix/usr/local/lib > /dev/null
    ninja -C /w/build-gles2 deqp-gles2
    arm-linux-gnueabi-strip /w/build-gles2/modules/gles2/deqp-gles2
'

echo "staging ..."
rm -rf "$STAGE"
mkdir -p "$STAGE/deqp"
cp "$WORK/build-gles2/modules/gles2/deqp-gles2" "$STAGE/deqp/"
cp -r "$WORK/VK-GL-CTS/data" "$STAGE/deqp/"

echo
echo "staged $STAGE ($(du -sh "$STAGE" | cut -f1))"
echo "now run: sh run-deqp.sh [case-pattern]"
