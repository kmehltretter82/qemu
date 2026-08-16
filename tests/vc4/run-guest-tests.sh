#!/bin/sh
# Boot a vc4 guest kernel on the QEMU bcm2835-v3d model and run the guest test
# suite in one boot, then check both the interpreter trace and the pixels the
# model produced.
#
#   P1  vc4_p1_draw array / indexed -- the control-list interpreter: does the
#       model decode the bin and render lists and fetch the vertex data?
#   P2  (checked in passing)        -- do the guest's shaders decode?
#   P5  vc4_p5_clear                -- the tile buffer and tile store: does a
#       clear actually land in the framebuffer, pixel for pixel?
#
#   usage: sh run-guest-tests.sh
#
# Prerequisites (see README.md):
#   ninja -C ../../build qemu-system-arm
#   sh build-guest-tests.sh
#   a guest kernel, DTB and busybox in $VC4_ASSETS
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
QEMU_SRC=$(cd "$HERE/../.." && pwd)

# Everything below can be pointed elsewhere; the defaults match the layout
# described in README.md.
QEMU_BUILD=${QEMU_BUILD:-$QEMU_SRC/build}
VC4_ASSETS=${VC4_ASSETS:-$(cd "$QEMU_SRC/.." && pwd)/guest-assets}
GEN_INIT_CPIO=${GEN_INIT_CPIO:-$(cd "$QEMU_SRC/../.." && pwd)/toolchain/gen_init_cpio}
BUSYBOX=${BUSYBOX:-$(cd "$QEMU_SRC/../.." && pwd)/images/armv5l/busybox}

QEMU=$QEMU_BUILD/qemu-system-arm
ZIMAGE=${VC4_ZIMAGE:-$VC4_ASSETS/zImage}
DTB=${VC4_DTB:-$VC4_ASSETS/bcm2835-rpi-a-plus-v3donly.dtb}
OUT=${VC4_OUT:-$HERE/out}
SERIAL=$OUT/serial.log
TRACE=$OUT/trace.log
CPIO=$OUT/initramfs.cpio.gz
SPEC=$OUT/initramfs.spec
INIT=$OUT/init.sh

need() {
    [ -e "$2" ] || { echo "missing $1: $2" >&2; exit 2; }
}
need "qemu-system-arm (ninja -C $QEMU_BUILD qemu-system-arm)" "$QEMU"
need "guest kernel (set VC4_ZIMAGE)" "$ZIMAGE"
need "stripped DTB (build it from $HERE/bcm2835-rpi-a-plus-v3donly.dts)" "$DTB"
need "busybox (set BUSYBOX)" "$BUSYBOX"
need "gen_init_cpio (set GEN_INIT_CPIO)" "$GEN_INIT_CPIO"
for t in vc4_p1_draw vc4_p5_clear; do
    need "guest binary (run 'sh build-guest-tests.sh')" "$HERE/guest/$t"
done

mkdir -p "$OUT"
# Stale logs would let the reaper below match a previous run's "ALL DONE".
rm -f "$SERIAL" "$TRACE"

cat > "$INIT" <<'EOS'
#!/bin/busybox sh
export PATH=/bin:/sbin:/usr/bin:/usr/sbin
/bin/busybox --install -s /bin
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || mdev -s

echo
echo "=== initramfs alive: $(uname -a) ==="
echo "=== /dev/dri ==="; ls -l /dev/dri 2>/dev/null || echo "(none)"

for mode in array indexed; do
    echo "=== P1 BEGIN ($mode) ==="
    /vc4_p1_draw "$mode"
    echo "=== P1 END ($mode) (exit $?) ==="
done

echo "=== P5 BEGIN (clear) ==="
/vc4_p5_clear
echo "=== P5 END (clear) (exit $?) ==="

echo "=== ALL DONE ==="
poweroff -f 2>/dev/null
exec sh
EOS
chmod +x "$INIT"

# gen_init_cpio needs absolute paths, so the spec is generated rather than
# checked in with someone's home directory baked into it.
cat > "$SPEC" <<EOF
dir /dev 0755 0 0
nod /dev/console 0600 0 0 c 5 1
dir /proc 0755 0 0
dir /sys 0755 0 0
dir /tmp 1777 0 0
dir /bin 0755 0 0
dir /sbin 0755 0 0
file /bin/busybox $BUSYBOX 0755 0 0
slink /bin/sh busybox 0777 0 0
file /init $INIT 0755 0 0
file /vc4_p1_draw $HERE/guest/vc4_p1_draw 0755 0 0
file /vc4_p5_clear $HERE/guest/vc4_p5_clear 0755 0 0
EOF
"$GEN_INIT_CPIO" "$SPEC" | gzip -9 > "$CPIO"

echo "QEMU:   $QEMU"
echo "kernel: $ZIMAGE"
echo "dtb:    $DTB"
echo "logs:   $SERIAL, $TRACE"
echo "----------------------------------------------------------------"

# Boot notes: QEMU models neither the vc4 display blocks nor the bcm2835 PM
# controller, so the DTB disables them and the guest cannot power QEMU off --
# hence the reaper below.  fbdev_emulation=0 skips a modeset that has no CRTC.
"$QEMU" -M raspi1ap \
    -kernel "$ZIMAGE" \
    -dtb "$DTB" \
    -initrd "$CPIO" \
    -append "console=ttyAMA0 earlycon=pl011,0x20201000 rdinit=/init drm_kms_helper.fbdev_emulation=0 loglevel=7" \
    -serial stdio -display none -no-reboot \
    -d 'trace:bcm2835_v3d_*,guest_errors' -D "$TRACE" > "$SERIAL" 2>&1 &
QPID=$!

for _ in $(seq 1 240); do
    kill -0 "$QPID" 2>/dev/null || break
    if grep -q "=== ALL DONE ===" "$SERIAL" 2>/dev/null; then
        sleep 1
        break
    fi
    sleep 1
done
kill "$QPID" 2>/dev/null || true
wait "$QPID" 2>/dev/null || true

echo "---- guest ----"
grep -E "^P1:|^P5:|=== P[15]" "$SERIAL" || true

echo "---- checks ----"
fails=0
check() {  # check <description> <extended regex over the trace>
    if grep -Eq "$2" "$TRACE"; then
        echo "  ok   : $1"
    else
        echo "  FAIL : $1   (no match for /$2/)"
        fails=$((fails + 1))
    fi
}
check_serial() {  # check_serial <description> <fixed string the guest printed>
    if grep -Fq "$2" "$SERIAL"; then
        echo "  ok   : $1"
    else
        echo "  FAIL : $1   (guest never printed \"$2\")"
        fails=$((fails + 1))
    fi
}

echo "  -- P1: control-list interpreter --"
check "CT0 bin list walked"          'bcm2835_v3d_cl_begin CT0 bin list'
check "TILE_BINNING_MODE_CONFIG"     'bcm2835_v3d_bin_config .*1x1 tiles'
check "GL_SHADER_STATE, 1 attribute" 'bcm2835_v3d_shader_state .*, 1 attributes'
check "shader record relocated"      'bcm2835_v3d_shader_rec flags 0x0001 fs 0x[0-9a-f]{8}/unif'
check "attribute 0 size 12 stride 12" 'bcm2835_v3d_shader_attr attribute 0: 0x[0-9a-f]+ size 12 stride 12'
check "GL_ARRAY_PRIMITIVE decoded"   'bcm2835_v3d_array_prim mode 0x04, 3 vertices from index 0'
check "GL_INDEXED_PRIMITIVE decoded" 'bcm2835_v3d_indexed_prim mode 0x14, 3 indices at 0x[0-9a-f]+ \(max index 2, 2-byte\)'
check "VCD fetched 3 vertices"       'bcm2835_v3d_vcd_fetch VCD fetched 3 vertices x 1 attributes'
check "tile lists capped"            'bcm2835_v3d_tile_lists capped 1 tile lists at 0x[0-9a-f]+, 32-byte blocks'
check "CT0 ran to the end"           'bcm2835_v3d_cl_end CT0 .*\(end of list\)'
check "CT1 render list walked"       'bcm2835_v3d_cl_begin CT1 render list'
check "CLEAR_COLORS decoded"         'bcm2835_v3d_clear_colors color 0xff00ff00'
check "render config 16x16"          'bcm2835_v3d_render_config framebuffer 0x[0-9a-f]+ 16x16 bits 0x0004'
check "sub-list branch followed"     'bcm2835_v3d_cl_branch CT1 .*\(sub-list\)'
check "returned from the tile list"  'bcm2835_v3d_cl_return CT1'
check "CT1 ended at the frame end"   'bcm2835_v3d_cl_end CT1 .*\(end of frame\)'
check_serial "guest reported PASS (array)"   "P1: PASS (array)"
check_serial "guest reported PASS (indexed)" "P1: PASS (indexed)"

echo "  -- P2: shaders decoded --"
check "shaders decoded to completion" 'bcm2835_v3d_shader_probe cs shader .* -> done'

echo "  -- P5: tile buffer reaches memory --"
check "clear stored to the framebuffer" 'bcm2835_v3d_tlb_transfer tile buffer store at 0x[0-9a-f]+, tile 0,0: 256 pixels'
check_serial "guest saw the cleared pixels" "P5: PASS: all 256 pixels are 0xdeadbeef"

echo "  -- no faults --"
if grep -q "control list aborted" "$TRACE"; then
    echo "  FAIL : a control list was aborted"
    grep "control list aborted" "$TRACE"
    fails=$((fails + 1))
else
    echo "  ok   : no control list aborted"
fi

echo "---- verdict ----"
if [ "$fails" -eq 0 ]; then
    echo "GUEST TESTS OK: lists decoded, vertex data fetched, pixels landed"
else
    echo "GUEST TESTS FAILED: $fails check(s)"
fi
exit "$fails"
