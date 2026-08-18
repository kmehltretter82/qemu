#!/bin/sh
# Resumable dEQP-GLES2 sweep over the whole module.
#
#   usage: sh run-deqp-sweep.sh          # start or continue the sweep
#          sh run-deqp-sweep.sh status   # report progress, run nothing
#          sh run-deqp-sweep.sh reset    # forget all progress
#
# run-deqp.sh runs one group in one go, which is right for a targeted question.
# It is the wrong shape for the whole module: that is ~19000 cases, every one of
# them running Mesa's GLSL compiler on an emulated ARM1176, so the run takes days
# and any interruption -- a crash, a timeout, a reboot -- throws all of it away.
#
# This runner splits the module into fixed-size chunks and checkpoints each one
# as it completes, so a re-run continues where the last one stopped.  Two
# properties matter and both are deliberate:
#
#   * A chunk is a separate dEQP process.  dEQP builds one EGL context per
#     process and reuses it for every case, so one test that leaves the
#     framebuffer incomplete makes every later case in that process fail at its
#     first checked GL call.  Chunking bounds that blast radius to one chunk.
#   * A chunk that starts but never reports is retried once, then recorded as
#     crashed and skipped, so a case that kills the guest cannot wedge the sweep.
#
# Prerequisites: build-mesa-guest.sh, then build-deqp-guest.sh.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
QEMU_SRC=$(cd "$HERE/../.." && pwd)
MODE=${1:-run}

QEMU_BUILD=${QEMU_BUILD:-$QEMU_SRC/build}
VC4_ASSETS=${VC4_ASSETS:-$(cd "$QEMU_SRC/.." && pwd)/guest-assets}
GEN_INIT_CPIO=${GEN_INIT_CPIO:-$(cd "$QEMU_SRC/../.." && pwd)/toolchain/gen_init_cpio}
BUSYBOX=${BUSYBOX:-$(cd "$QEMU_SRC/../.." && pwd)/images/armv5l/busybox}

ZIMAGE=${VC4_ZIMAGE:-$VC4_ASSETS/zImage}
DTB=${VC4_DTB:-$VC4_ASSETS/bcm2835-rpi-a-plus-v3donly.dtb}
OUT=${VC4_OUT:-$HERE/out}
MESA_STAGE=$OUT/mesa-root
DEQP_STAGE=$OUT/deqp-root
# A long sweep must not be disturbed by a rebuild of the emulator underneath it,
# so it can be pointed at a snapshot of the binary.
QEMU_BIN=${DEQP_QEMU:-$QEMU_BUILD/qemu-system-arm}

# Everything the sweep remembers lives here, and only here.
STATE=$OUT/sweep
CASELIST=$STATE/caselist.txt
CHUNKDIR=$STATE/chunks
DONE=$STATE/done.txt
ATTEMPTS=$STATE/attempts.txt
RESULTS=$STATE/results.txt
SERIAL=$STATE/serial.log
TRACE=$STATE/trace.log

CHUNK=${DEQP_CHUNK:-200}         # cases per dEQP process
CONFIG=${DEQP_CONFIG:-rgba8888d0s0ms0}
SIZE=${DEQP_SIZE:-256}
TIMEOUT=${DEQP_TIMEOUT:-36000}   # seconds of wall clock for one boot
MODULE=${DEQP_MODULE:-dEQP-GLES2}

need() { [ -e "$2" ] || { echo "missing $1: $2" >&2; exit 2; }; }

# Boot the guest once, running the given guest-side shell body, and leave the
# serial output in $SERIAL.  The reaper is needed because with the bcm2835 PM
# block disabled the guest cannot make QEMU exit by itself.
boot_guest() {
    body=$1
    phase=${2:-run}
    spec=$STATE/initramfs.spec
    cpio=$STATE/initramfs.cpio.gz
    init=$STATE/init.sh

    {
        echo "#!/bin/busybox sh"
        echo "export PATH=/bin:/sbin:/usr/bin:/usr/sbin"
        echo "/bin/busybox --install -s /bin"
        echo "mount -t proc proc /proc"
        echo "mount -t sysfs sysfs /sys"
        echo "mount -t devtmpfs devtmpfs /dev 2>/dev/null || mdev -s"
        echo "export LD_LIBRARY_PATH=/lib"
        echo "export LIBGL_DRIVERS_PATH=/usr/lib/dri"
        echo "export MESA_SHADER_CACHE_DISABLE=true"
        echo "export EGL_PLATFORM=surfaceless"
        echo "cd /deqp"
        cat "$body"
        echo "echo '=== ALL DONE ==='"
        echo "poweroff -f 2>/dev/null"
        echo "exec sh"
    } > "$init"
    chmod +x "$init"

    {
        echo "dir /dev 0755 0 0"; echo "nod /dev/console 0600 0 0 c 5 1"
        echo "dir /proc 0755 0 0"; echo "dir /sys 0755 0 0"; echo "dir /tmp 1777 0 0"
        echo "dir /bin 0755 0 0"; echo "dir /sbin 0755 0 0"; echo "dir /lib 0755 0 0"
        echo "dir /usr 0755 0 0"; echo "dir /usr/lib 0755 0 0"
        echo "dir /usr/lib/dri 0755 0 0"
        echo "file /bin/busybox $BUSYBOX 0755 0 0"
        echo "slink /bin/sh busybox 0777 0 0"
        echo "file /init $init 0755 0 0"
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
        if [ -d "$CHUNKDIR" ] && [ -n "$(ls -A "$CHUNKDIR" 2>/dev/null)" ]; then
            echo "dir /chunks 0755 0 0"
            for f in "$CHUNKDIR"/*.trie; do
                echo "file /chunks/$(basename "$f") $f 0644 0 0"
            done
        fi
    } > "$spec"
    "$GEN_INIT_CPIO" "$spec" | gzip -9 > "$cpio"

    rm -f "$SERIAL" "$TRACE"
    "$QEMU_BIN" -M raspi1ap \
        -kernel "$ZIMAGE" -dtb "$DTB" -initrd "$cpio" \
        -append "console=ttyAMA0 earlycon=pl011,0x20201000 rdinit=/init drm_kms_helper.fbdev_emulation=0 loglevel=4" \
        -serial stdio -display none -no-reboot \
        -d 'trace:bcm2835_v3d_cl_end,guest_errors' -D "$TRACE" > "$SERIAL" 2>&1 &
    qpid=$!
    i=0
    while [ "$i" -lt "$TIMEOUT" ]; do
        kill -0 "$qpid" 2>/dev/null || break
        if grep -q "=== ALL DONE ===" "$SERIAL" 2>/dev/null; then sleep 2; break; fi
        sleep 1
        i=$((i + 1))
    done
    kill "$qpid" 2>/dev/null || true
    wait "$qpid" 2>/dev/null || true
    # Each phase keeps its own copy; the next boot truncates $SERIAL, and the
    # evidence for why a phase went wrong is in the phase's own log.
    cp "$SERIAL" "$STATE/serial-$phase.log" 2>/dev/null || true
}

# --- progress reporting -----------------------------------------------------

report() {
    total=$(ls "$CHUNKDIR"/*.trie 2>/dev/null | wc -l | tr -d ' ')
    [ -f "$DONE" ] || : > "$DONE"
    done_n=$(sort -u "$DONE" | wc -l | tr -d ' ')
    cases=$(grep -c "^TEST: " "$CASELIST" 2>/dev/null || echo 0)
    echo "chunks: $done_n done of $((total + done_n)) planned   (~$CHUNK cases each, $cases in the module)"
    if [ -s "$RESULTS" ]; then
        # dEQP prints "  Passed:        3/3 (100.0%)"; take the numerator.
        awk '
            function num(s,   n) { match(s, /[0-9]+\//)
                                   return substr(s, RSTART, RLENGTH - 1) + 0 }
            /Passed:/        { p += num($0) }
            /Failed:/        { f += num($0) }
            /Not supported:/ { n += num($0) }
            /Warnings:/      { w += num($0) }
            END { printf "cases:  %d passed, %d failed, %d not supported, %d warnings\n",
                         p, f, n, w }' "$RESULTS"
    fi
    if [ -s "$STATE/crashed.txt" ]; then
        echo "crashed chunks (skipped after a retry):"
        sed 's/^/  /' "$STATE/crashed.txt"
    fi
}

case "$MODE" in
status)
    [ -d "$STATE" ] || { echo "no sweep started yet"; exit 0; }
    report
    exit 0
    ;;
reset)
    rm -rf "$STATE"
    echo "sweep state cleared"
    exit 0
    ;;
run) ;;
*)  echo "usage: sh run-deqp-sweep.sh [run|status|reset]" >&2; exit 2 ;;
esac

need "qemu-system-arm" "$QEMU_BIN"
need "guest kernel" "$ZIMAGE"
need "the Mesa runtime (build-mesa-guest.sh)" "$MESA_STAGE/lib"
need "deqp-gles2 (build-deqp-guest.sh)" "$DEQP_STAGE/deqp/deqp-gles2"
need "gen_init_cpio" "$GEN_INIT_CPIO"
mkdir -p "$STATE" "$CHUNKDIR"
: > "$STATE/body.sh"

# --- phase 1: learn the case list (once) ------------------------------------

if [ ! -s "$CASELIST" ]; then
    echo "== enumerating $MODULE (one boot, no rendering) =="
    # Even in list-only mode dEQP initialises the platform, and its default
    # surface type is a window the surfaceless platform cannot provide.
    cat > "$STATE/body.sh" <<EOS
echo "=== CASELIST BEGIN ==="
./deqp-gles2 --deqp-runmode=stdout-caselist --deqp-archive-dir=/deqp/data --deqp-surface-type=fbo \\
             --deqp-gl-config-name=$CONFIG \\
             --deqp-surface-width=$SIZE --deqp-surface-height=$SIZE
echo "=== CASELIST END ==="
EOS
    boot_guest "$STATE/body.sh" list
    sed -n '/=== CASELIST BEGIN ===/,/=== CASELIST END ===/p' "$SERIAL" \
        | grep "^TEST: " | tr -d '\r' > "$CASELIST" || true
    if [ ! -s "$CASELIST" ]; then
        echo "the guest produced no case list -- see $SERIAL" >&2
        exit 1
    fi
    echo "$(grep -c '^TEST: ' "$CASELIST") cases"
fi

# --- phase 2: plan the chunks (once) ----------------------------------------

if [ -z "$(ls -A "$CHUNKDIR" 2>/dev/null)" ] && [ ! -s "$DONE" ]; then
    echo "== planning chunks of $CHUNK cases =="
    CHUNKDIR="$CHUNKDIR" CHUNK="$CHUNK" CASELIST="$CASELIST" python3 - <<'PY'
import os

chunkdir = os.environ["CHUNKDIR"]
size = int(os.environ["CHUNK"])
names = [l[len("TEST: "):].strip()
         for l in open(os.environ["CASELIST"]) if l.startswith("TEST: ")]

def trie(names):
    """dEQP's case-list file format: {group{sub{case,case}}}."""
    root = {}
    for n in names:
        node = root
        for part in n.split('.'):
            node = node.setdefault(part, {})
    def emit(node):
        out = []
        for k, v in node.items():
            out.append(k + ("{" + emit(v) + "}" if v else ""))
        return ",".join(out)
    return "{" + emit(root) + "}"

for i in range(0, len(names), size):
    with open(f"{chunkdir}/chunk-{i // size:04d}.trie", "w") as f:
        f.write(trie(names[i:i + size]) + "\n")
print(f"{(len(names) + size - 1) // size} chunks")
PY
fi

# --- phase 3: run the pending chunks ----------------------------------------

[ -f "$DONE" ] || : > "$DONE"
[ -f "$ATTEMPTS" ] || : > "$ATTEMPTS"

# A chunk that has been started twice without ever reporting is taken to kill
# the process or the guest; record it and stop offering it.
for f in "$CHUNKDIR"/*.trie; do
    [ -e "$f" ] || continue
    id=$(basename "$f" .trie)
    if [ "$(grep -c "^$id\$" "$ATTEMPTS")" -ge 2 ]; then
        echo "$id" >> "$STATE/crashed.txt"
        rm -f "$f"
    fi
done
sort -u -o "$STATE/crashed.txt" "$STATE/crashed.txt" 2>/dev/null || true

pending=$(ls "$CHUNKDIR"/*.trie 2>/dev/null | wc -l | tr -d ' ')
if [ "$pending" -eq 0 ]; then
    echo "== sweep complete =="
    report
    exit 0
fi

echo "== running $pending pending chunk(s), ${TIMEOUT}s budget =="
report
{
    echo 'for c in /chunks/*.trie; do'
    echo '    id=$(basename "$c" .trie)'
    echo '    echo "=== CHUNK $id BEGIN ==="'
    echo "    ./deqp-gles2 --deqp-caselist-file=\"\$c\" --deqp-archive-dir=/deqp/data --deqp-surface-type=fbo \\"
    echo "                 --deqp-gl-config-name=$CONFIG \\"
    echo "                 --deqp-surface-width=$SIZE --deqp-surface-height=$SIZE \\"
    echo "                 --deqp-log-images=disable --deqp-log-shader-sources=disable \\"
    echo '                 --deqp-log-filename=/tmp/deqp.qpa > /tmp/out.txt 2>&1'
    # Keep what carries information: which cases failed and why, deduped by
    # reason, plus dEQP's own tally.  The full chatter is thousands of lines.
    echo '    grep -B1 "^  \(Fail\|InternalError\|ResourceError\|Crash\) (" /tmp/out.txt \'
    echo '        | grep "^  " | sort | uniq -c | sort -rn | head -25'
    echo '    grep -A8 "Test run totals" /tmp/out.txt | head -9'
    echo '    echo "=== CHUNK $id END ==="'
    echo 'done'
} > "$STATE/body.sh"
boot_guest "$STATE/body.sh"

# --- phase 4: harvest -------------------------------------------------------

tr -d '\r' < "$SERIAL" | awk -v state="$STATE" '
    /^=== CHUNK .* BEGIN ===$/ { id = $3; started[id] = 1; buf = ""; next }
    /^=== CHUNK .* END ===$/   { finished[id] = buf; id = ""; next }
    id != ""                   { buf = buf $0 "\n" }
    END {
        for (i in started)  print i > (state "/started.tmp")
        for (i in finished) {
            print i > (state "/finished.tmp")
            printf "%s", "=== " i " ===\n" finished[i] >> (state "/results.txt")
        }
    }'
touch "$STATE/started.tmp" "$STATE/finished.tmp"
# A chunk counts as done only if dEQP got far enough to print its END marker,
# so a boot cut short mid-chunk leaves that chunk pending rather than lost.
while read -r id; do
    [ -n "$id" ] || continue
    echo "$id" >> "$DONE"
    rm -f "$CHUNKDIR/$id.trie"
done < "$STATE/finished.tmp"
cat "$STATE/started.tmp" >> "$ATTEMPTS"
sort -u -o "$DONE" "$DONE"
rm -f "$STATE/started.tmp" "$STATE/finished.tmp"

echo "---- anything the model declined ----"
grep -oE "(bcm2835-v3d|vc4-qpu): [^(]*" "$TRACE" 2>/dev/null | sort | uniq -c || true
echo "(nothing above means the model was never asked for something it lacks)"
echo "---- progress ----"
report
echo "re-run this script to continue; 'sh run-deqp-sweep.sh status' just reports"
