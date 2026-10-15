#!/usr/bin/env python3
"""Hardware-in-the-loop test bench for the STEP800 / STEP400 with a Digilent Analog Discovery 2.

Usage: hil.py [--config bench.toml] <selftest|timing|brake|powerup|reboot|compare|check|arp|motor> [--id N] ...
See README.md.
"""

from __future__ import annotations

import argparse
import sys

from hil import config
from hil.arp import STEPS as ARP_STEPS
from hil.checks import TESTS as CHECKS
from hil.motor import STEPS as MOTOR_STEPS
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
    s.add_argument("--only", nargs="+", choices=["v33", "probes", "brake_map", "home_sw", "limit_sw", "excitation", "sync"],
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
    s.add_argument("--how", choices=["R", "resetDevice", "W", "button", "standby"], required=True,
                   help="R: serial reboot, resetDevice: OSC, W: watchdog (serial 'W', hang), button: the "
                        "reset button (human, waits for /booted), standby: 'Y' (driver reset line floating)")
    s.add_argument("--analog-rate", type=float,
                   help="Scope rate (default 10 kS/s across a restart, [brake] analog_rate for standby); "
                        "the PWM pulses of OUT1A (about 2 us) need 500 kS/s or more")
    s.add_argument("--move", type=int, default=0,
                   help="/move the test axis this many microsteps after the release (e.g. 64), so OUT1A shows "
                        "PWM while energized")
    s.add_argument("--all-motors", action="store_true",
                   help="release the brakes of all motors (/activate 255 1) instead of the test axis only")

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

    s = sub.add_parser("motor", parents=[common], help="tests with a real motor and brake, record only (STEP400)")
    s.add_argument("--step", choices=list(MOTOR_STEPS), help="step to run (see hil/motor.py)")
    s.add_argument("--note", nargs="+", metavar=("RESULT_DIR", "KEY=VALUE"),
                   help="store operator observations in summary.json of a result, no board access")
    s.add_argument("--label", help="suffix of the result directory")
    s.add_argument("--rate", type=float, help="digital sample rate (default 100 kS/s, 10 kS/s with a restart)")
    s.add_argument("--reboot", action="store_true", help="reboot the board (R) before the setup")
    return p


# Subcommands that judge the excitation on OUT1A: not with a motor connected.
OUT1A_COMMANDS = ("brake", "reboot", "powerup")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = config.load(args.config)
    if getattr(args, "id", None) is not None:
        cfg.override_board_id(args.id)
    from hil import arp, checks, commands, motor, restart

    if args.command == "arp" and cfg.model != "step800":
        print(f"error: arp needs P5 (UDP send), which {cfg.model} has no pin for")
        return 2
    motor_connected = bool(cfg.raw["motor"].get("motor_connected", False))
    if motor_connected and args.command in OUT1A_COMMANDS:
        print(f"error: motor.motor_connected = true: {args.command} judges the excitation on OUT1A, which "
              "is set up without a motor. Use hil.py motor (record only).")
        return 2
    if args.command == "check":
        skip = checks.UNSUPPORTED.get(cfg.model, {})
        if args.all_auto and skip:
            # --all-auto: the checks this board supports.
            args.test, args.all_auto = [t for t in checks.AUTO_TESTS if t not in skip], False
            print(f"--all-auto on {cfg.model}: {' '.join(args.test)} (not: "
                  f"{', '.join(t for t in checks.AUTO_TESTS if t in skip)})")
        tests = list(checks.AUTO_TESTS) if args.all_auto else (args.test or [])
        unsupported = [t for t in tests if t in skip]
        if unsupported:
            print("error: not on " + cfg.model + ": " +
                  "; ".join(f"{t} ({checks.UNSUPPORTED[cfg.model][t]})" for t in unsupported))
            return 2
    # The board must be the one of board.model, with its HIL_PROBE build. Not for compare
    # (no board), nor for the STEP800 power-up (its USB cable is unplugged then).
    if args.command == "motor" and args.note:
        return motor.cmd_motor(cfg, args)
    if args.command != "compare" and not (args.command == "powerup" and cfg.model == "step800"):
        try:
            board = commands.preflight(cfg)
        except (commands.BoardMismatch, commands.DiagError, OSError) as ex:
            print(f"error: {ex}")
            return 2
        print(f"board: {board['firmware_name']}, HIL probes " +
              " ".join(f"P{ch}={v}" for ch, v in board["probe_map"].items()))

    handler = {
        "selftest": commands.cmd_selftest,
        "timing": commands.cmd_timing,
        "brake": commands.cmd_brake,
        "powerup": commands.cmd_powerup,
        "compare": commands.cmd_compare,
        "reboot": restart.cmd_reboot,
        "check": checks.cmd_check,
        "arp": arp.cmd_arp,
        "motor": motor.cmd_motor,
    }[args.command]
    return handler(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
