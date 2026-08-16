# bcm2835-v3d (VideoCore IV) guest tests

End-to-end tests for the `hw/display/bcm2835_v3d.c` GPU model: boot a Linux
guest whose mainline `vc4` DRM driver binds to the emulated V3D, submit real
control lists through the `SUBMIT_CL` ioctl, and check both what the model
decoded (its trace) and what it produced (the pixels in the framebuffer).

The device's pure logic is unit tested separately and needs none of this:

    ninja -C ../../build tests/unit/test-vc4-qpu tests/unit/test-vc4-tiling
    ../../build/tests/unit/test-vc4-qpu       # the QPU instruction interpreter
    ../../build/tests/unit/test-vc4-tiling    # surface layout arithmetic

## Running

```sh
ninja -C ../../build qemu-system-arm
sh build-guest-tests.sh      # cross-compiles guest/*.c (Docker, see below)
sh run-guest-tests.sh
```

A clean run ends with:

```
GUEST TESTS OK: lists decoded, vertex data fetched, pixels landed
```

Logs and the generated initramfs land in `out/`.

## What each test covers

| Test | Checks |
|------|--------|
| `guest/vc4_p1_draw.c` | A complete single-triangle submission, once with `GL_ARRAY_PRIMITIVE` and once with `GL_INDEXED_PRIMITIVE`: full state block, a shader record with one vertex attribute over a real vertex BO, and a colour surface so the kernel generates a real render list. Verifies the model decodes both control lists and fetches exactly the vertex and index data the driver programmed. |
| `guest/vc4_p5_clear.c` | A render-only submit that clears a surface to `0xdeadbeef`, then maps the BO and checks every pixel. It poisons the buffer first, so a store that never happened cannot pass. This is the test that proves pixels, not just tracing. |

`run-guest-tests.sh` runs both in a single boot and asserts 22 things about the
resulting trace and guest output.

## Assets this needs, and why they are not in the tree

The tests need a guest kernel, a device tree and a busybox, which together are
tens of megabytes of build output — too big for a source tree, and rebuildable.
They are looked up in `$VC4_ASSETS` (default: `../../../guest-assets` relative
to the QEMU source root), and every path can be overridden:

| Variable | Default | What |
|----------|---------|------|
| `VC4_ASSETS` | `<qemu>/../guest-assets` | where the rest are looked up |
| `VC4_ZIMAGE` | `$VC4_ASSETS/zImage` | arm32 kernel with `CONFIG_DRM_VC4=y` (`bcm2835_defconfig` has it) |
| `VC4_DTB` | `$VC4_ASSETS/bcm2835-rpi-a-plus-v3donly.dtb` | built from the `.dts` here |
| `BUSYBOX` | `<qemu>/../../images/armv5l/busybox` | static soft-float arm binary |
| `GEN_INIT_CPIO` | `<qemu>/../../toolchain/gen_init_cpio` | the kernel's, so `/dev/console` needs no root |
| `QEMU_BUILD` | `<qemu>/build` | the QEMU build directory |
| `VC4_OUT` | `./out` | logs and generated initramfs |

`bcm2835-rpi-a-plus-v3donly.dts` is checked in because it is the non-obvious
part of the recipe. Build it with:

    dtc -I dts -O dtb -o $VC4_ASSETS/bcm2835-rpi-a-plus-v3donly.dtb \
        bcm2835-rpi-a-plus-v3donly.dts

## Why the guest boots the way it does

Three things in `run-guest-tests.sh` are not obvious and were each found the
hard way:

- **The stripped DTB.** QEMU models neither the vc4 display blocks (HVS,
  pixelvalve, HDMI) nor the bcm2835 PM/power controller, and both hang or crash
  the guest. The DTS here disables them and reduces the v3d node to the minimal
  contract, so the DRM master binds render-only against V3D. It also drops
  `clocks`/`power-domains` from the v3d node — with the PM block disabled, v3d
  would otherwise defer forever.
- **`drm_kms_helper.fbdev_emulation=0`.** The initial framebuffer modeset
  NULL-derefs in `vc4_atomic_commit_tail` in a render-only setup with no CRTC.
- **The reaper loop.** With no PM block the guest cannot make QEMU exit, so the
  script waits for the guest to print `=== ALL DONE ===` and kills it, with a
  timeout as a backstop.

## Cross-compiling the guest binaries

The usual host here is macOS with no arm cross toolchain, so
`build-guest-tests.sh` builds them in an `ubuntu:24.04` container with
`gcc-arm-linux-gnueabi` (Docker must be running). If you have a cross compiler,
skip Docker entirely:

    VC4_NATIVE_CC=arm-linux-gnueabi-gcc sh build-guest-tests.sh

`guest/uapi/` holds the DRM uapi headers the tests compile against, copied from
the kernel tree: Ubuntu's `linux-libc-dev` is too old for `vc4_drm.h` and lacks
`linux/bits.h`.
