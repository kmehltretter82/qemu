#!/usr/bin/env python3
"""Resolve an RT-Thread SAM9G45 test profile through its bundled Kconfiglib.

SPDX-License-Identifier: GPL-2.0-or-later
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import sys


ASSIGN_RE = re.compile(r"^CONFIG_([A-Za-z0-9_]+)=(.*)$")
UNSET_RE = re.compile(r"^# CONFIG_([A-Za-z0-9_]+) is not set$")


def parse_fragment(path: Path) -> dict[str, str]:
    requested: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        line = raw_line.strip()
        if not line or line.startswith("##"):
            continue
        unset = UNSET_RE.match(line)
        if unset:
            requested[unset.group(1)] = "n"
            continue
        assignment = ASSIGN_RE.match(line)
        if assignment:
            value = assignment.group(2)
            if len(value) >= 2 and value[0] == value[-1] == '"':
                value = value[1:-1]
            requested[assignment.group(1)] = value
            continue
        raise ValueError(f"{path}:{line_number}: invalid profile line: {raw_line}")
    return requested


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--fragment", type=Path, required=True)
    parser.add_argument("--packages", type=Path, required=True)
    args = parser.parse_args()

    source = args.source.resolve()
    fragment = args.fragment.resolve()
    bsp = source / "bsp/at91/at91sam9g45"
    tools = source / "tools"
    base_config = bsp / ".config"
    if not (bsp / "Kconfig").is_file() or not base_config.is_file():
        parser.error(f"not an RT-Thread source tree with a SAM9G45 BSP: {source}")
    if not (args.packages / "Kconfig").is_file():
        parser.error(f"missing empty package catalog: {args.packages}/Kconfig")

    sys.path.insert(0, str(tools))
    try:
        import kconfiglib  # type: ignore[import-not-found]
    except ModuleNotFoundError as error:
        raise ValueError(
            "Kconfiglib is required; install python3-kconfiglib "
            "(v4.1.1 also bundles a compatible copy)"
        ) from error

    os.environ["BSP_ROOT"] = str(bsp)
    os.environ["RTT_ROOT"] = str(source)
    os.environ["PKGS_ROOT"] = str(args.packages.resolve())

    previous_cwd = Path.cwd()
    try:
        # Current RT-Thread BSP Kconfig files use paths relative to the BSP,
        # while older releases took the same paths from environment symbols.
        # Running Kconfiglib in the BSP directory supports both forms.
        os.chdir(bsp)
        kconf = kconfiglib.Kconfig("Kconfig", warn=False)
        kconf.warn_assign_undef = False
        kconf.load_config(str(base_config), replace=True)
        requested = parse_fragment(fragment)
        for name, value in requested.items():
            symbol = kconf.syms.get(name)
            if symbol is None:
                raise ValueError(f"profile requests unknown Kconfig symbol {name}")
            if not symbol.set_value(value):
                raise ValueError(f"Kconfig rejected {name}={value}")

        mismatches = []
        for name, requested_value in requested.items():
            actual = kconf.syms[name].str_value
            if actual != requested_value:
                mismatches.append(
                    f"{name}: requested {requested_value}, resolved {actual}"
                )
        if mismatches:
            raise ValueError(
                "unresolved profile settings:\n  " + "\n  ".join(mismatches)
            )

        message = kconf.write_config(str(base_config))
    finally:
        os.chdir(previous_cwd)
    print(message)
    print(f"Resolved {len(requested)} settings from {fragment.name}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"configure-profile: {error}", file=sys.stderr)
        raise SystemExit(1)
