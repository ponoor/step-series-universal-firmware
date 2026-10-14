#!/usr/bin/env python3
"""Hardware-in-the-loop test bench for the STEP800 with a Digilent Analog Discovery 2.

Usage: hil.py [--config bench.toml] <selftest|timing|brake|powerup|reboot|compare|check|arp> [--id N] ...
See README.md.
"""

from __future__ import annotations

import argparse
import sys

from hil import config
from hil.arp import STEPS as ARP_STEPS
from hil.checks import TESTS as CHECKS
from hil.scenarios import SCENARIOS


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", help="config file (default: tools/hil/bench.toml)")
    # Also accepted after the subcommand (parents=[common]).
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--id", type=int, default=argparse.SUPPRESS,
                        help="board ID (DIP switch) instead of board.id of bench.toml: IP 10.0.0.100 + id, "
                             "send port 50100 + id")
    p.add_argument("--id", type=int, default=None, help=argparse.SUPPRESS)
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("selftest", parents=[common], help="check wiring, probes, brake outputs, home switch and excitation")
    s.add_argument("--only", nargs="+", choices=["probes", "brake_map", "home_sw", "excitation", "sync"],
                   help="run only these checks")
    s.add_argument("--sync", action="store_true",
                   help="also measure the Scope / Digital In start offset (Scope 2 must be moved to P0)")

    s = sub.add_parser("timing", parents=[common], help="measure loop and handler timing from the probes")
    s.add_argument("--duration", type=float, help="seconds (default: [timing] duration)")
    s.add_argument("--rate", type=float, help="digital sample rate (default: [timing] digital_rate)")
    s.add_argument("--label", required=True, help="name of the measurement, e.g. baseline / head")
    s.add_argument("--no-reboot", action="store_true", help="do not reboot the board first")
    s.add_argument("--position-report-ms", type=int,
                   help="position report interval, 0: no reports (default: [timing] position_report_ms)")
    s.add_argument("--position-list-report-ms", type=int,
                   help="report the positions of all motors in one message (/setPositionListReportInterval) "
                        "instead of /position per motor; not with --position-report-ms")
    s.add_argument("--burst-hz", type=float, default=0.0,
                   help="also send /getPosition 255 (8 replies) at this rate, 0: none (default)")
    s.add_argument("--target-rate-hz", type=float,
                   help="/setTargetPositionList rate (default: [timing] target_rate_hz)")

    s = sub.add_parser("brake", parents=[common], help="run the brake scenarios")
    s.add_argument("--scenario", choices=[sc.id for sc in SCENARIOS], help="one scenario (default: all)")

    s = sub.add_parser("powerup", parents=[common], help="record the brake outputs at power-up (needs a human)")
    s.add_argument("--repeat", type=int, default=5)
    s.add_argument("--duration", type=float, default=10.0, help="seconds after the 3.3 V rise")

    s = sub.add_parser("reboot", parents=[common], help="brake outputs and driver state across a restart")
    s.add_argument("--how", choices=["R", "resetDevice", "standby"], required=True,
                   help="R: serial reboot, resetDevice: OSC, standby: 'Y' (driver reset line floating)")

    s = sub.add_parser("compare", parents=[common], help="compare two timing results")
    s.add_argument("result_dirs", nargs=2)

    s = sub.add_parser("check", parents=[common], help="remaining checks C1-C12, C6e (see README)")
    s.add_argument("--test", nargs="+", choices=list(CHECKS), help="checks to run")
    s.add_argument("--all-auto", action="store_true", help="C1-C7 and C12 (no human operation needed)")
    s.add_argument("--cases", help="C6: loads, subset of abcdw (default abcd); C6e: of bc; C7: of abcde; "
                   "C8: of abc; C9: of abcd")
    s.add_argument("--repeat", type=int, help="C8 / C9 / C11: runs per case (default 10 / 3 / 5)")
    s.add_argument("--dead-ip", default="10.0.0.250", help="C6: unused address in the board's subnet")
    s.add_argument("--duration", type=float, help="C6: seconds per load (default 10); C6e: at least this long "
                   "after the removal (default 30)")
    s.add_argument("--max-duration", type=float, help="C6e: at most this long after the removal if nothing "
                   "changes (default 300)")
    s.add_argument("--alias-ip", default="10.0.0.251", help="C6e: unused address in the board's subnet, added "
                   "to the Mac's interface by a human")
    s.add_argument("--label", help="suffix of the result directory, e.g. prefix for the firmware before a fix")
    s.add_argument("--dip-values", help="C10: comma-separated DIP switch values (default 1,2,4,...,128)")
    s.add_argument("--dip", type=int, default=85, help="C10boot: DIP switch value set before the restart")

    s = sub.add_parser("arp", parents=[common], help="client replaced by another MAC with the same IP (needs a human)")
    s.add_argument("--topology", choices=["T1", "T2"], required=True,
                   help="T1: board and both NICs at a switch, T2: board cabled directly to one NIC")
    s.add_argument("--step", choices=list(ARP_STEPS), required=True,
                   help="check: NIC A only; e1: E1 1-6; e2a / e2b: E2; c1 / c2: candidate fixes (HIL_PROBE); "
                        "n1 / n2: the fix commands while the client is reachable; gap / fix: the ARP refresh")
    s.add_argument("--wait", type=float, help="E1 3: seconds on NIC B before /getVersion (e1: 300, others: 30)")
    s.add_argument("--observe", type=float, default=30.0, help="seconds after /setDestIp from NIC B (E1 5)")
    s.add_argument("--heartbeat", action="store_true", help="fix: /setDestIp every 1.0 s (F1); without: F2")
    s.add_argument("--label", help="suffix of the result directory")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = config.load(args.config)
    if getattr(args, "id", None) is not None:
        cfg.override_board_id(args.id)
    from hil import arp, checks, commands, restart

    handler = {
        "selftest": commands.cmd_selftest,
        "timing": commands.cmd_timing,
        "brake": commands.cmd_brake,
        "powerup": commands.cmd_powerup,
        "compare": commands.cmd_compare,
        "reboot": restart.cmd_reboot,
        "check": checks.cmd_check,
        "arp": arp.cmd_arp,
    }[args.command]
    return handler(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
