#!/usr/bin/env python3
"""Hardware-in-the-loop test bench for the STEP800 / STEP400 with a Digilent Analog Discovery 2.

Usage: hil.py [--config bench.toml] <selftest|timing|brake|powerup|reboot|compare|check|arp|motor|usb|swd> [--id N] ...
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
from hil.usb import STEPS as USB_STEPS


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
    s.add_argument("--how", choices=["R", "resetDevice", "W", "X", "button", "standby"], required=True,
                   help="R: serial reboot, resetDevice: OSC, W: watchdog (serial 'W', hang), X: HardFault "
                        "(serial 'X'), button: the reset button (human, waits for /booted), standby: 'Y' "
                        "(driver reset line floating)")
    s.add_argument("--transition-ms", type=int,
                   help="brakeTransitionDuration of every motor (default: [brake] transition_ms)")
    s.add_argument("--brake-disabled", action="store_true",
                   help="brake disabled on every motor (W / X: the handler does not wait)")
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

    s = sub.add_parser("usb", parents=[common], help="USB cable unplugged / plugged in while running, record only "
                       "(needs a human)")
    s.add_argument("--step", choices=list(USB_STEPS), help="condition U1-U6 (see hil/usb.py)")
    s.add_argument("--note", nargs="+", metavar=("RESULT_DIR", "KEY=VALUE"),
                   help="store operator observations in summary.json of a result, no board access")
    s.add_argument("--label", help="suffix of the result directory")
    s.add_argument("--rate", type=float, help="digital sample rate (default 10 kS/s: across a restart)")
    s.add_argument("--no-ad2", action="store_true",
                   help="no AD2 (nor SWD): judged from the heartbeat gap and /booted resetCause only")
    s.add_argument("--conditions", nargs="+", metavar="KEY=VALUE",
                   help="--no-ad2: what the run's wiring / power / firmware are, stored in summary.json")
    s.add_argument("--read-record", metavar="RESULT_DIR",
                   help="after the plug-in: read the HIL fault record ('H') into RESULT_DIR/summary.json")
    s.add_argument("--wait", type=float, default=60.0, help="--read-record: seconds to wait for the USB serial")
    s.add_argument("--no-probe", action="store_true",
                   help="firmware without the HIL probes (e.g. main): no board check, judged from the heartbeat "
                        "gaps, J7 and /booted only")

    s = sub.add_parser("wdt", parents=[common], help="longest interval between watchdog resets under the loads "
                       "W1-W10 (wdtGapMax of 'p', HIL_PROBE builds); no AD2")
    s.add_argument("--cases", nargs="+", choices=["w1", "w2", "w3", "w4", "w5", "w6", "w7", "w8", "w9", "w10"],
                   help="cases to run (default: those without a human: w1 w2 w8 w9 w10)")
    s.add_argument("--repeat", type=int, default=3, help="runs per case (default 3)")
    s.add_argument("--duration", type=float, default=10.0, help="W1 / W4: seconds of load (default 10)")
    s.add_argument("--dead-ip", default="10.0.0.250", help="W3 / W4: unused address in the board's subnet")
    s.add_argument("--elf", help="firmware.elf on the board: the reset() call sites as function and line")
    s.add_argument("--no-motion", action="store_true",
                   help="motor on the test axis: servo target kept at the position, /homing on the next axis")
    s.add_argument("--label", help="suffix of the result directory")
    s = sub.add_parser("swd", parents=[common], help="CPU state through SWD (Atmel-ICE, OpenOCD); no reset")
    g = s.add_mutually_exclusive_group(required=True)
    g.add_argument("--info", action="store_true", help="no halt: chip ID, reset cause, debug state, flash check")
    g.add_argument("--halt", action="store_true", help="halt now, read the state, resume")
    g.add_argument("--on-hang", action="store_true",
                   help="/getVersion every 100 ms, halt when no reply for 0.3 s (operator unplugs the USB)")
    s.add_argument("--elf", help="firmware.elf of the firmware on the board (symbols, HIL record)")
    s.add_argument("--bin", help="firmware.bin to compare with the flash (default: next to --elf)")
    s.add_argument("--gdb", action="store_true", help="also a gdb backtrace after the dump")
    s.add_argument("--stay-halted", action="store_true", help="leave the CPU halted after the dump")
    s.add_argument("--timeout", type=float, default=120.0, help="--on-hang: seconds to wait for the stop")
    s.add_argument("--setup", choices=list(USB_STEPS), help="--on-hang: apply the setup of this hil.py usb step "
                   "and its messages before the unplug (e.g. u1: brake enabled and released)")
    s.add_argument("--label", help="suffix of the result directory")
    return p


# Subcommands that judge the excitation on OUT1A: not with a motor connected.
OUT1A_COMMANDS = ("brake", "reboot", "powerup")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = config.load(args.config)
    if getattr(args, "id", None) is not None:
        cfg.override_board_id(args.id)
    from hil import arp, checks, commands, motor, restart, swd, usb, wdt

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
    if args.command in ("motor", "usb") and args.note:
        return motor.add_note(args.note[0], args.note[1:])
    if args.command == "usb" and not args.step and not args.read_record:
        print("error: --step or --note is needed")
        return 2
    no_board_check = (args.command == "powerup" and cfg.model == "step800") or \
        (args.command == "usb" and (args.no_probe or args.read_record)) or args.command == "swd"
    if args.command != "compare" and not no_board_check:
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
        "usb": usb.cmd_usb,
        "swd": swd.cmd_swd,
        "wdt": wdt.cmd_wdt,
    }[args.command]
    return handler(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
