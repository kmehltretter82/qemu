#!/bin/sh
# Drive the emulated VideoCore IV through the real OpenGL ES stack.
#
# Boots a guest whose initramfs carries Mesa's vc4 driver (built by
# build-mesa-guest.sh) and runs guest/gles_vc4_test.c: EGL surfaceless, a GLES2
# context, an off-screen framebuffer, a glClear and a real compiled GLSL
# triangle -- all read back with glReadPixels.
#
# Unlike run-guest-tests.sh, which submits hand-assembled control lists, this
# exercises what a real application produces.  The triangle is expected to fail
# its pixel check until the model has a rasterizer; the point of running it is
# the trace of genuine shaders, control lists and VPM traffic it produces.
#
#   usage: sh run-gles-test.sh
#
# Prerequisites: see README.md, plus
#   sh build-mesa-guest.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
QEMU_SRC=$(cd "$HERE/../.." && pwd)

QEMU_BUILD=${QEMU_BUILD:-$QEMU_SRC/build}
VC4_ASSETS=${VC4_ASSETS:-$(cd "$QEMU_SRC/.." && pwd)/guest-assets}
GEN_INIT_CPIO=${GEN_INIT_CPIO:-$(cd "$QEMU_SRC/../.." && pwd)/toolchain/gen_init_cpio}
BUSYBOX=${BUSYBOX:-$(cd "$QEMU_SRC/../.." && pwd)/images/armv5l/busybox}

QEMU=$QEMU_BUILD/qemu-system-arm
ZIMAGE=${VC4_ZIMAGE:-$VC4_ASSETS/zImage}
DTB=${VC4_DTB:-$VC4_ASSETS/bcm2835-rpi-a-plus-v3donly.dtb}
OUT=${VC4_OUT:-$HERE/out}
STAGE=$OUT/mesa-root
SERIAL=$OUT/gles-serial.log
TRACE=$OUT/gles-trace.log
CPIO=$OUT/initramfs-gles.cpio.gz
SPEC=$OUT/initramfs-gles.spec
INIT=$OUT/init-gles.sh

need() { [ -e "$2" ] || { echo "missing $1: $2" >&2; exit 2; }; }
need "qemu-system-arm" "$QEMU"
need "guest kernel" "$ZIMAGE"
need "stripped DTB" "$DTB"
need "busybox" "$BUSYBOX"
need "gen_init_cpio" "$GEN_INIT_CPIO"
need "the staged Mesa runtime (run 'sh build-mesa-guest.sh')" "$STAGE/gles_vc4_test"

mkdir -p "$OUT"
rm -f "$SERIAL" "$TRACE"

cat > "$INIT" <<'EOS'
#!/bin/busybox sh
export PATH=/bin:/sbin:/usr/bin:/usr/sbin
/bin/busybox --install -s /bin
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || mdev -s

echo
echo "=== initramfs alive: $(uname -m) ==="
ls -l /dev/dri 2>/dev/null || echo "(no /dev/dri)"

export LD_LIBRARY_PATH=/lib
export LIBGL_DRIVERS_PATH=/usr/lib/dri
export MESA_SHADER_CACHE_DISABLE=true

echo "=== GLES BEGIN ==="
/gles_vc4_test
echo "=== GLES END (exit $?) ==="

echo "=== ALL DONE ==="
poweroff -f 2>/dev/null
exec sh
EOS
chmod +x "$INIT"

{
    echo "dir /dev 0755 0 0"
    echo "nod /dev/console 0600 0 0 c 5 1"
    echo "dir /proc 0755 0 0"; echo "dir /sys 0755 0 0"; echo "dir /tmp 1777 0 0"
    echo "dir /bin 0755 0 0"; echo "dir /sbin 0755 0 0"; echo "dir /lib 0755 0 0"
    echo "dir /usr 0755 0 0"; echo "dir /usr/lib 0755 0 0"; echo "dir /usr/lib/dri 0755 0 0"
    echo "file /bin/busybox $BUSYBOX 0755 0 0"
    echo "slink /bin/sh busybox 0777 0 0"
    echo "file /init $INIT 0755 0 0"
    echo "file /gles_vc4_test $STAGE/gles_vc4_test 0755 0 0"
    for f in "$STAGE"/lib/*; do
        echo "file /lib/$(basename "$f") $f 0755 0 0"
    done
    echo "file /usr/lib/dri/vc4_dri.so $STAGE/usr/lib/dri/vc4_dri.so 0755 0 0"
} > "$SPEC"
"$GEN_INIT_CPIO" "$SPEC" | gzip -9 > "$CPIO"

echo "QEMU:      $QEMU"
echo "initramfs: $CPIO ($(du -h "$CPIO" | cut -f1), Mesa included)"
echo "logs:      $SERIAL, $TRACE"
echo "----------------------------------------------------------------"

"$QEMU" -M raspi1ap \
    -kernel "$ZIMAGE" -dtb "$DTB" -initrd "$CPIO" \
    -append "console=ttyAMA0 earlycon=pl011,0x20201000 rdinit=/init drm_kms_helper.fbdev_emulation=0 loglevel=7" \
    -serial stdio -display none -no-reboot \
    -d 'trace:bcm2835_v3d_*,guest_errors' -D "$TRACE" > "$SERIAL" 2>&1 &
QPID=$!

# Mesa compiling shaders under TCG emulation is not quick; allow plenty.
for _ in $(seq 1 600); do
    kill -0 "$QPID" 2>/dev/null || break
    if grep -q "=== ALL DONE ===" "$SERIAL" 2>/dev/null; then sleep 1; break; fi
    sleep 1
done
kill "$QPID" 2>/dev/null || true
wait "$QPID" 2>/dev/null || true

echo "---- the real OpenGL ES stack ----"
sed -n '/GLES BEGIN/,/GLES END/p' "$SERIAL" || true

echo "---- what Mesa asked the model to do ----"
grep -cE "bcm2835_v3d_cl_packet" "$TRACE" | sed 's/^/  control-list packets decoded: /'
grep -E "bcm2835_v3d_shader_probe" "$TRACE" | sed 's/^/  /' | sort -u

echo "---- checks ----"
fails=0
check_serial() {
    if grep -Fq "$2" "$SERIAL"; then
        echo "  ok   : $1"
    else
        echo "  FAIL : $1   (guest never printed \"$2\")"
        fails=$((fails + 1))
    fi
}
check_serial "Mesa bound to the emulated V3D" "GL_RENDERER = VC4 V3D 2.1"
check_serial "GLES2 context is current"       "context on vc4         PASS"
check_serial "a real glClear reaches memory"  "glClear readback       PASS"
check_serial "GLSL compiles"                  "glsl compile           PASS"
check_serial "GLSL links"                     "glsl link              PASS"
check_serial "a real draw is submitted"       "draw submitted         PASS"

if grep -q "control list aborted" "$TRACE"; then
    echo "  FAIL : a control list was aborted"
    fails=$((fails + 1))
else
    echo "  ok   : no control list aborted"
fi

echo "---- verdict ----"
if [ "$fails" -eq 0 ]; then
    echo "GLES OK: real Mesa drives the model; clears produce correct pixels."
    echo "         The triangle still needs a rasterizer -- that is the next phase,"
    echo "         and the trace above says what it must supply."
else
    echo "GLES FAILED: $fails check(s)"
fi
exit "$fails"
