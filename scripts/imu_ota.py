#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 RoboParty

# Installed command examples:
#   /opt/roboparty/bin/imu_ota.py hipnuc info --interface can0 --node 8
#   /opt/roboparty/bin/imu_ota.py hipnuc update firmware.hex --interface can0 --node 8
#   /opt/roboparty/bin/imu_ota.py mct --firmware /path/to/firmware.bin --interface can0
# The first argument selects the IMU family; the remaining arguments are the
# selected module's own options. Each module also runs standalone.

"""IMU firmware update entry point.

Dispatches to the per-family update modules:

* ``hipnuc`` — HiPNUC IMU over classic CAN (hipnuc_ota.py)
* ``mct``    — MCT7123/MD7123 IMU over CAN-FD (mct_ota.py)

Usage: imu_ota.py {hipnuc|mct} [module options...]
"""

from __future__ import annotations

import sys
from typing import Optional

_MODULES = {
    "hipnuc": ("hipnuc_ota", "HiPNUC IMU: classic CAN, HIREG registers + SDO bootloader"),
    "mct": ("mct_ota", "MCT7123/MD7123 IMU: CAN-FD, UCB bootloader"),
}


def _usage() -> None:
    print("usage: imu_ota.py {hipnuc|mct} [module options...]")
    print()
    for name, (module, description) in _MODULES.items():
        print(f"  {name:6} {description} ({module}.py)")
    print()
    print("Run imu_ota.py <type> --help for the selected module's options.")


def main(argv: Optional[list[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in ("-h", "--help"):
        _usage()
        return 0
    choice = args[0].lower()
    if choice not in _MODULES:
        print(
            f"error: unknown IMU type {args[0]!r}; expected one of: {', '.join(_MODULES)}",
            file=sys.stderr,
        )
        return 2
    import importlib

    module = importlib.import_module(_MODULES[choice][0])
    return module.main(args[1:])


if __name__ == "__main__":
    sys.exit(main())
