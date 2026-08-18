#!/bin/sh
# Run groups of dEQP-GLES2 conformance tests against the model.
#
#   usage: sh run-deqp.sh ['dEQP-GLES2.functional.prerequisite.*' ...]
#
# Each argument is a --deqp-case pattern and gets its own dEQP process inside a
# single guest boot.  That split is not cosmetic: dEQP builds one EGL context
# per process and reuses it for every case in the run, so a single test that
# leaves the framebuffer incomplete makes every later test fail at its first
# checked GL call.  One process per group bounds that blast radius to the group
# and gives a separate tally for each.
#
# Prerequisites: build-mesa-guest.sh, then build-deqp-guest.sh.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
QEMU_SRC=$(cd "$HERE/../.." && pwd)
# "all" walks the whole functional module one top-level group at a time.  The
# order is deliberate: cheap, high-signal groups first, the two enormous ones
# (texture and shaders, together most of the module's ~19000 cases) last, so a
# run that is cut short still leaves a usable spread of results behind.
ALL_GROUPS='
dEQP-GLES2.functional.prerequisite.*
dEQP-GLES2.functional.implementation_limits.*
dEQP-GLES2.functional.color_clear.*
dEQP-GLES2.functional.depth_stencil_clear.*
dEQP-GLES2.functional.buffer.*
dEQP-GLES2.functional.read_pixels.*
dEQP-GLES2.functional.depth_range.*
dEQP-GLES2.functional.dither.*
dEQP-GLES2.functional.polygon_offset.*
dEQP-GLES2.functional.clipping.*
dEQP-GLES2.functional.rasterization.*
dEQP-GLES2.functional.vertex_arrays.*
dEQP-GLES2.functional.attribute_location.*
dEQP-GLES2.functional.draw.*
dEQP-GLES2.functional.fragment_ops.*
dEQP-GLES2.functional.state_query.*
dEQP-GLES2.functional.negative_api.*
dEQP-GLES2.functional.lifetime.*
dEQP-GLES2.functional.fbo.*
dEQP-GLES2.functional.shader_api.*
dEQP-GLES2.functional.uniform_api.*
dEQP-GLES2.functional.texture.*
dEQP-GLES2.functional.shaders.*
'
if [ "$#" -eq 1 ] && [ "$1" = "all" ]; then
    GROUPS=$ALL_GROUPS
elif [ "$#" -gt 0 ]; then
    GROUPS="$*"
else
    GROUPS='dEQP-GLES2.functional.prerequisite.*'
fi

QEMU_BUILD=${QEMU_BUILD:-$QEMU_SRC/build}
VC4_ASSETS=${VC4_ASSETS:-$(cd "$QEMU_SRC/.." && pwd)/guest-assets}
GEN_INIT_CPIO=${GEN_INIT_CPIO:-$(cd "$QEMU_SRC/../.." && pwd)/toolchain/gen_init_cpio}
BUSYBOX=${BUSYBOX:-$(cd "$QEMU_SRC/../.." && pwd)/images/armv5l/busybox}

QEMU=$QEMU_BUILD/qemu-system-arm
ZIMAGE=${VC4_ZIMAGE:-$VC4_ASSETS/zImage}
DTB=${VC4_DTB:-$VC4_ASSETS/bcm2835-rpi-a-plus-v3donly.dtb}
OUT=${VC4_OUT:-$HERE/out}
MESA_STAGE=$OUT/mesa-root
DEQP_STAGE=$OUT/deqp-root
SERIAL=$OUT/deqp-serial.log
TRACE=$OUT/deqp-trace.log
CPIO=$OUT/initramfs-deqp.cpio.gz
SPEC=$OUT/initramfs-deqp.spec
INIT=$OUT/init-deqp.sh
# dEQP's default config asks for a packed depth/stencil this driver does not
# offer, and its rasterization group needs at least 256x256.
CONFIG=${DEQP_CONFIG:-rgba8888d0s0ms0}
SIZE=${DEQP_SIZE:-256}
# Seconds to let the guest run.  A group of a few tests needs minutes; a whole
# functional sweep needs many hours, so raise it for those.
TIMEOUT=${DEQP_TIMEOUT:-1800}
# A long run should not be disturbed by a rebuild of the emulator underneath it,
# so it can be pointed at a snapshot of the binary.
QEMU_BIN=${DEQP_QEMU:-$QEMU}

need() { [ -e "$2" ] || { echo "missing $1: $2" >&2; exit 2; }; }
need "qemu-system-arm" "$QEMU_BIN"
need "guest kernel" "$ZIMAGE"
need "the Mesa runtime (build-mesa-guest.sh)" "$MESA_STAGE/lib"
need "deqp-gles2 (build-deqp-guest.sh)" "$DEQP_STAGE/deqp/deqp-gles2"
need "gen_init_cpio" "$GEN_INIT_CPIO"

mkdir -p "$OUT"
rm -f "$SERIAL" "$TRACE"

cat > "$INIT" <<EOS
#!/bin/busybox sh
export PATH=/bin:/sbin:/usr/bin:/usr/sbin
/bin/busybox --install -s /bin
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || mdev -s
echo "=== initramfs alive: \$(uname -m) ==="
export LD_LIBRARY_PATH=/lib
export LIBGL_DRIVERS_PATH=/usr/lib/dri
export MESA_SHADER_CACHE_DISABLE=true
export EGL_PLATFORM=surfaceless
cd /deqp
# The patterns are globs meant for dEQP; the shell must not expand them itself.
set -f
for g in $GROUPS; do
    echo "=== DEQP BEGIN (\$g) ==="
    ./deqp-gles2 --deqp-case="\$g" --deqp-archive-dir=/deqp/data --deqp-surface-type=fbo \\
                 --deqp-gl-config-name=$CONFIG \\
                 --deqp-surface-width=$SIZE --deqp-surface-height=$SIZE \\
                 --deqp-log-images=disable --deqp-log-shader-sources=disable \\
                 --deqp-log-filename=/tmp/deqp.qpa > /tmp/deqp.txt 2>&1
    # The full per-case chatter is thousands of lines for a big group, so keep
    # the part that carries information: which cases failed and why, deduped by
    # reason, plus dEQP's own tally.
    echo "--- failures in \$g ---"
    grep -B1 "^  \\(Fail\\|InternalError\\|ResourceError\\|Crash\\)" /tmp/deqp.txt \\
        | grep "^  " | sort | uniq -c | sort -rn | head -40
    echo "--- tally for \$g ---"
    grep -A8 "Test run totals" /tmp/deqp.txt | head -9
    echo "=== DEQP END (\$g) ==="
done
echo "=== ALL DONE ==="
poweroff -f 2>/dev/null
exec sh
EOS
chmod +x "$INIT"

{
    echo "dir /dev 0755 0 0"; echo "nod /dev/console 0600 0 0 c 5 1"
    echo "dir /proc 0755 0 0"; echo "dir /sys 0755 0 0"; echo "dir /tmp 1777 0 0"
    echo "dir /bin 0755 0 0"; echo "dir /sbin 0755 0 0"; echo "dir /lib 0755 0 0"
    echo "dir /usr 0755 0 0"; echo "dir /usr/lib 0755 0 0"
    echo "dir /usr/lib/dri 0755 0 0"
    echo "file /bin/busybox $BUSYBOX 0755 0 0"
    echo "slink /bin/sh busybox 0777 0 0"
    echo "file /init $INIT 0755 0 0"
    for f in "$MESA_STAGE"/lib/*; do
        echo "file /lib/$(basename "$f") $f 0755 0 0"
    done
    # dEQP dlopens the unversioned names.
    echo "slink /lib/libEGL.so libEGL.so.1 0777 0 0"
    echo "slink /lib/libGLESv2.so libGLESv2.so.2 0777 0 0"
    echo "file /usr/lib/dri/vc4_dri.so $MESA_STAGE/usr/lib/dri/vc4_dri.so 0755 0 0"
    echo "dir /deqp 0755 0 0"
    echo "file /deqp/deqp-gles2 $DEQP_STAGE/deqp/deqp-gles2 0755 0 0"
    ( cd "$DEQP_STAGE" && find deqp/data -type d ) | sed 's|^|dir /|; s|$| 0755 0 0|'
    ( cd "$DEQP_STAGE" && find deqp/data -type f ) | while read -r f; do
        echo "file /$f $DEQP_STAGE/$f 0644 0 0"
    done
} > "$SPEC"
"$GEN_INIT_CPIO" "$SPEC" | gzip -9 > "$CPIO"

echo "groups:    $GROUPS"
echo "surface:   ${SIZE}x${SIZE} $CONFIG"
echo "initramfs: $CPIO ($(du -h "$CPIO" | cut -f1))"
echo "----------------------------------------------------------------"

"$QEMU_BIN" -M raspi1ap \
    -kernel "$ZIMAGE" -dtb "$DTB" -initrd "$CPIO" \
    -append "console=ttyAMA0 earlycon=pl011,0x20201000 rdinit=/init drm_kms_helper.fbdev_emulation=0 loglevel=4" \
    -serial stdio -display none -no-reboot \
    -d 'trace:bcm2835_v3d_cl_end,guest_errors' -D "$TRACE" > "$SERIAL" 2>&1 &
QPID=$!

for _ in $(seq 1 "$TIMEOUT"); do
    kill -0 "$QPID" 2>/dev/null || break
    if grep -q "=== ALL DONE ===" "$SERIAL" 2>/dev/null; then sleep 1; break; fi
    sleep 1
done
kill "$QPID" 2>/dev/null || true
wait "$QPID" 2>/dev/null || true

sed -n '/DEQP BEGIN/,/DEQP END/p' "$SERIAL" || true

echo "---- anything the model declined ----"
grep -oE "(bcm2835-v3d|vc4-qpu): [^(]*" "$TRACE" 2>/dev/null | sort | uniq -c || true
echo "(nothing above means the model was never asked for something it lacks)"

# dEQP's own tally is the verdict.  It throws on the way out on Linux
# ("Device fault tests execution not supported"), after reporting, so the exit
# status is not usable -- and with one process per group there is no single
# status to read anyway.  Count the groups that reported a clean tally.
echo "---- verdict ----"
ran=$(grep -c "^--- tally for " "$SERIAL" || true)
clean=$(grep -c "Failed:        0/" "$SERIAL" || true)
want=$(printf '%s\n' $GROUPS | wc -l | tr -d ' ')
echo "groups requested: $want   reported a tally: $ran   with no failures: $clean"
[ "$ran" = "$want" ] || echo "NOTE: $((want - ran)) group(s) produced no tally -- crashed, hung, or timed out"
[ "$ran" = "$want" ] && [ "$clean" = "$want" ]
