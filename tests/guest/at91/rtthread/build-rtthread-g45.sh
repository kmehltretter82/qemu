#!/usr/bin/env bash
# Build the tracked SAM9G45 test overlay against a pristine RT-Thread release.
# SPDX-License-Identifier: GPL-2.0-or-later

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/../../../.." && pwd)"
jobs="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"

usage()
{
    echo "usage: $0 [--clean] [--profile PROFILE] [--version VERSION]" >&2
    echo "" >&2
    echo "Environment overrides:" >&2
    echo "  RTTHREAD_VERSION     release to build (default: 5.2.2)" >&2
    echo "  RTTHREAD_ARCHIVE     pristine release tar.gz" >&2
    echo "  RTTHREAD_BUILD_ROOT  disposable build/output directory" >&2
    echo "  SCONS                SCons executable" >&2
    echo "  JOBS                 parallel build jobs" >&2
}

clean=0
profile="baseline"
version="${RTTHREAD_VERSION:-5.2.2}"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)
            clean=1
            ;;
        --profile)
            if [[ $# -lt 2 ]]; then
                usage
                exit 2
            fi
            profile="$2"
            shift
            ;;
        --version)
            if [[ $# -lt 2 ]]; then
                usage
                exit 2
            fi
            version="$2"
            shift
            ;;
        *)
            usage
            exit 2
            ;;
    esac
    shift
done

case "${version}" in
    5.2.2)
        archive_sha256="4449e87c59a7337a803625ad0f047ef8f934ad81e4c7c669adabd1289cf9813e"
        ;;
    4.1.1)
        archive_sha256="c4af708ca5e30937ceaa0c5435bd363dba883dc2c5ba81362ecd112f2e862184"
        ;;
    *)
        echo "unsupported RT-Thread release: ${version}" >&2
        exit 2
        ;;
esac

archive="${RTTHREAD_ARCHIVE:-${repo_root}/images/other-os/rt-thread/rt-thread-v${version}.tar.gz}"
build_root="${RTTHREAD_BUILD_ROOT:-${repo_root}/build/rtthread-g45/${version}}"
source_root="${build_root}/source"
source_dir="${source_root}/rt-thread-${version}"
bsp_dir="${source_dir}/bsp/at91/at91sam9g45"

profile_fragment="${script_dir}/profiles/${profile}.conf"
if [[ ! -f "${profile_fragment}" ]]; then
    echo "unknown RT-Thread profile: ${profile}" >&2
    exit 2
fi
artifacts_dir="${build_root}/artifacts/${profile}"
profile_stamp="${source_root}/.g45-profile"

if [[ ! -f "${archive}" ]]; then
    echo "missing RT-Thread archive: ${archive}" >&2
    exit 1
fi

actual_sha256="$(sha256sum "${archive}" | awk '{print $1}')"
if [[ ${actual_sha256} != "${archive_sha256}" ]]; then
    echo "RT-Thread archive SHA-256 mismatch" >&2
    echo "expected: ${archive_sha256}" >&2
    echo "actual:   ${actual_sha256}" >&2
    exit 1
fi

case "${build_root}" in
    "${repo_root}"/build/*) ;;
    *)
        echo "refusing build root outside ${repo_root}/build: ${build_root}" >&2
        exit 1
        ;;
esac

if [[ -f "${profile_stamp}" && $(<"${profile_stamp}") != "${profile}" ]]; then
    clean=1
fi
if [[ ${clean} -eq 1 && -d "${source_dir}" ]]; then
    rm -rf -- "${source_dir}"
fi

mkdir -p -- "${source_root}" "${artifacts_dir}"
if [[ ! -d "${source_dir}" ]]; then
    tar -xzf "${archive}" -C "${source_root}"
fi
if [[ ! -f "${bsp_dir}/SConstruct" ]]; then
    echo "archive does not contain the expected SAM9G45 BSP" >&2
    exit 1
fi

for overlay_file in "${script_dir}"/overlay/*.[ch]; do
    install -m 0644 "${overlay_file}" \
        "${bsp_dir}/applications/$(basename -- "${overlay_file}")"
done

serial_patch="${script_dir}/patches/0001-at91sam9g45-expose-serial-ports.patch"
if [[ ${version} == "5.2.2" ]]; then
    serial_patch="${script_dir}/patches/0001-at91sam9g45-expose-serial-ports-v5.2.2.patch"
fi
if ! grep -q '^config RT_USING_DBGU$' "${bsp_dir}/Kconfig"; then
    patch -d "${source_dir}" -p1 < "${serial_patch}"
fi
peripheral_patch="${script_dir}/patches/0002-at91sam9g45-fix-i2c-macb-port.patch"
if grep -qE 'AT91SAM9260_ID_EMAC|PIO_PUER' \
        "${bsp_dir}/drivers/macb.c" "${bsp_dir}/drivers/at91_i2c_gpio.c"; then
    patch -d "${source_dir}" -p1 < "${peripheral_patch}"
fi
if [[ ${version} == "5.2.2" ]] && \
        ! grep -q 'sync-cp15dmb\.specs' "${bsp_dir}/rtconfig.py"; then
    patch -d "${source_dir}" -p1 < \
        "${script_dir}/patches/0003-at91sam9g45-use-arm9-barrier-v5.2.2.patch"
fi
if [[ ${version} == "5.2.2" ]] && \
        ! grep -q 'LWIP_NETIF_LOOPBACK' \
        "${source_dir}/components/net/lwip/port/lwipopts.h"; then
    patch -d "${source_dir}" -p1 < \
        "${script_dir}/patches/0004-lwip-enable-loopback-v5.2.2.patch"
fi

"${script_dir}/configure-profile.py" \
    --source "${source_dir}" \
    --fragment "${profile_fragment}" \
    --packages "${script_dir}/profiles/empty-packages"
printf '%s\n' "${profile}" > "${profile_stamp}"

declare -a scons_command
if [[ -n ${SCONS:-} ]]; then
    scons_command=("${SCONS}")
elif command -v scons >/dev/null 2>&1; then
    scons_command=("$(command -v scons)")
else
    bundled_scons="${repo_root}/images/other-os/rt-thread/toolchain/root/usr/bin/scons"
    bundled_python="${repo_root}/images/other-os/rt-thread/toolchain/root/usr/lib/python3/dist-packages"
    if [[ ! -f "${bundled_scons}" || ! -d "${bundled_python}" ]]; then
        echo "SCons not found; install it or set SCONS=/path/to/scons" >&2
        exit 1
    fi
    export PYTHONPATH="${bundled_python}${PYTHONPATH:+:${PYTHONPATH}}"
    scons_command=(python3 "${bundled_scons}")
fi

for tool in arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "missing cross-toolchain program: ${tool}" >&2
        exit 1
    fi
done

build_log="${artifacts_dir}/build.log"
(
    cd -- "${bsp_dir}"
    "${scons_command[@]}" --useconfig=.config
)
(
    cd -- "${bsp_dir}"
    "${scons_command[@]}" -j "${jobs}"
) 2>&1 | tee "${build_log}"

install -m 0644 "${bsp_dir}/rtthread-at91sam9g45.elf" \
    "${artifacts_dir}/rtthread-at91sam9g45.elf"
install -m 0644 "${bsp_dir}/rtthread.bin" \
    "${artifacts_dir}/rtthread.bin"
install -m 0644 "${bsp_dir}/rtthread_at91sam9g45.map" \
    "${artifacts_dir}/rtthread_at91sam9g45.map"

(
    cd -- "${artifacts_dir}"
    sha256sum rtthread-at91sam9g45.elf rtthread.bin \
        rtthread_at91sam9g45.map > SHA256SUMS
)

{
    echo "rtthread_version=${version}"
    echo "profile=${profile}"
    echo "rtthread_archive=${archive}"
    echo "rtthread_archive_sha256=${archive_sha256}"
    echo "overlay_sha256=$(sha256sum "${script_dir}"/overlay/*.[ch] | sha256sum | awk '{print $1}')"
    echo "profile_sha256=$(sha256sum "${profile_fragment}" | awk '{print $1}')"
    echo "compiler=$(arm-none-eabi-gcc --version | head -1)"
    echo "scons=$(${scons_command[@]} --version | head -1)"
    echo "kconfiglib=$(python3 -c 'import kconfiglib; print(".".join(map(str, kconfiglib.VERSION)))')"
    echo "built_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "${artifacts_dir}/BUILD-METADATA"

echo "Built ${artifacts_dir}/rtthread-at91sam9g45.elf"
arm-none-eabi-size "${artifacts_dir}/rtthread-at91sam9g45.elf"
cat "${artifacts_dir}/SHA256SUMS"
