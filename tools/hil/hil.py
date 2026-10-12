#!/usr/bin/env python3
"""Hardware-in-the-loop test bench for the STEP800 with a Digilent Analog Discovery 2.

Usage: hil.py [--config bench.toml] <selftest|timing|brake|powerup|reboot|compare> ...
See README.md.
"""

from __future__ import annotations

import argparse
import sys

from hil import config
from hil.scenarios import SCENARIOS


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", help="config file (default: tools/hil/bench.toml)")
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("selftest", help="check wiring, probes, brake outputs, home switch and excitation")
    s.add_argument("--only", nargs="+", choices=["probes", "brake_map", "home_sw", "excitation", "sync"],
                   help="run only these checks")
    s.add_argument("--sync", action="store_true",
                   help="also measure the Scope / Digital In start offset (Scope 2 must be moved to P0)")

    s = sub.add_parser("timing", help="measure loop and handler timing from the probes")
    s.add_argument("--duration", type=float, help="seconds (default: [timing] duration)")
    s.add_argument("--rate", type=float, help="digital sample rate (default: [timing] digital_rate)")
    s.add_argument("--label", required=True, help="name of the measurement, e.g. baseline / head")
    s.add_argument("--no-reboot", action="store_true", help="do not reboot the board first")

    s = sub.add_parser("brake", help="run the brake scenarios")
    s.add_argument("--scenario", choices=[sc.id for sc in SCENARIOS], help="one scenario (default: all)")

    s = sub.add_parser("powerup", help="record the brake outputs at power-up (needs a human)")
    s.add_argument("--repeat", type=int, default=5)
    s.add_argument("--duration", type=float, default=10.0, help="seconds after the 3.3 V rise")

    s = sub.add_parser("reboot", help="brake outputs and driver state across a restart")
    s.add_argument("--how", choices=["R", "resetDevice", "standby"], required=True,
                   help="R: serial reboot, resetDevice: OSC, standby: 'Y' (driver reset line floating)")

    s = sub.add_parser("compare", help="compare two timing results")
    s.add_argument("result_dirs", nargs=2)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = config.load(args.config)
    from hil import commands, restart

    handler = {
        "selftest": commands.cmd_selftest,
        "timing": commands.cmd_timing,
        "brake": commands.cmd_brake,
        "powerup": commands.cmd_powerup,
        "compare": commands.cmd_compare,
        "reboot": restart.cmd_reboot,
    }[args.command]
    return handler(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
