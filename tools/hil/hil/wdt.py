"""hil.py wdt: the longest interval between two watchdog resets under the loads W1-W10
(TASK_fault_reset Part 1).

Reads wdtGapMax of 'p' (HIL_PROBE builds: the longest interval between two
Watchdog.reset() calls while the watchdog is enabled, with the return addresses of the two
calls). Each run: 'p' (clears it), the load, 'p'. No AD2: the board needs only the USB
serial and the network, so the same steps run on STEP400_R1 and STEP800_R1. A /booted
during a run (a restart) is recorded. With --elf the addresses are turned into functions
and source lines with arm-none-eabi-addr2line.

Steps that need a human print OPERATOR lines and wait for Enter.
"""

from __future__ import annotations

import glob
import math
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .checks import SerialLog, address_unused
from .config import Config
from .diag import DiagError, parse_perf


@dataclass(frozen=True)
class Case:
    id: str
    title: str
    human: str = ""  # what the operator does, empty: nothing


CASES = {c.id: c for c in (
    Case("w1", "servo mode on all motors, /setTargetPositionList at [timing] target_rate_hz, "
               "/setPositionReportInterval 255 20 (hil.py timing r20)"),
    Case("w2", "/get* 255 once, reachable destination"),
    Case("w3", "/get* 255 once, unreachable destination (--dead-ip through '{', no SD card)",
         "the SD card must be out"),
    Case("w4", "/setPositionReportInterval 255 20 to the unreachable destination (--duration)",
         "the SD card must be out"),
    Case("w5", "Ethernet cable unplugged and plugged in, position reports 255 20 on",
         "unplug the Ethernet cable, wait a few seconds, plug it in"),
    Case("w6", "DIP switch ID changed while running (initEthernet()), then back",
         "change the DIP switch ID, then set it back"),
    Case("w7", "config tool commands: G, T, s, c, '{' (the config of G back: saved to the SD card if "
               "there is one), /saveConfig",
         "confirm the SD card state; '{' and /saveConfig rewrite the config file on it"),
    Case("w8", "USB serial not read: (a) port closed while /resetMotorDriver 255 prints, (b) port open, "
               "'s' x 20 without reading"),
    Case("w9", "/homing on the test axis, /resetMotorDriver 255, servo mode on all motors"),
    Case("w10", "restart ('R') and the first seconds (setup() to /booted), 'p' 5 s after /booted"),
)}


# --------------------------------------------------------------------------------------
# addresses


def addr2line_tool() -> str | None:
    found = sorted(glob.glob(os.path.expanduser(
        "~/.platformio/packages/toolchain-gccarmnoneeabi*/bin/arm-none-eabi-addr2line")))
    return found[0] if found else None


def resolve(elf: str | None, addrs: list[str]) -> dict[str, str]:
    """{'0x0000A1B2': 'loop at main.cpp:594'}; empty without an ELF or the tool."""
    tool = addr2line_tool()
    addrs = [a for a in addrs if a and int(a, 16)]
    if not elf or not tool or not addrs:
        return {}
    # A return address is after the call: the call is the instruction before it.
    query = [f"0x{int(a, 16) - 2:08X}" for a in addrs]
    out = subprocess.run([tool, "-f", "-C", "-e", elf, *query], capture_output=True, text=True).stdout.splitlines()
    res = {}
    for i, a in enumerate(addrs):
        fn, loc = (out[2 * i], out[2 * i + 1]) if len(out) >= 2 * i + 2 else ("?", "?")
        res[a] = f"{fn} at {Path(loc.split(' ')[0]).name}"
    return res


# --------------------------------------------------------------------------------------
# context


class WdtBench:
    def __init__(self, cfg: Config, args, out: Path):
        from . import commands

        self.cfg = cfg
        self.args = args
        self.out = out
        self.commands = commands
        self.ser = SerialLog(cfg.serial_port)
        self.link = commands.open_link(cfg)
        self.t0_ns = time.monotonic_ns()

    def __enter__(self) -> "WdtBench":
        self.ser.open()
        self.link.open()
        return self

    def __exit__(self, *exc) -> None:
        self.link.close()
        self.ser.close()

    def connect(self, timeout: float = 15.0) -> str:
        return self.commands.connect_board(self.link, timeout)

    def send(self, address: str, *args):
        return self.link.send(address, *args)

    def perf(self) -> dict:
        if self.args.product:
            # Product build: no wdtGapMax; the last line of 'p' is arpDropped
            # (arpRefreshFailed before the resolution of the MAC address of destIp).
            p = parse_perf(self.ser.command("p", until=r"(arpDropped|arpRefreshFailed)\s+\d+"))
            p["wdtGapMax"] = {"max_us": None, "from": None, "to": None}
            return p
        text = self.ser.command("p", until=r"wdtGapMax\s+\d+ us\s+from 0x[0-9A-Fa-f]+ to 0x[0-9A-Fa-f]+")
        p = parse_perf(text)
        if "wdtGapMax" not in p:
            raise DiagError("no wdtGapMax in 'p': not a HIL_PROBE build of TASK_fault_reset Part 1")
        return p

    def booted_since(self, t_ns: int) -> list:
        return [list(e.args) for e in self.link.entries("rx", t_ns) if e.address == "/booted"]

    def human(self, message: str) -> None:
        """Enter with a terminal; without one (e.g. run by an agent), waits up to 10 minutes
        for the file OPERATOR_DONE in the results directory, then removes it."""
        if sys.stdin.isatty():
            self.commands.prompt(f"OPERATOR: {message} -- Enter when done ")
            return
        done = self.commands.RESULTS_DIR / "OPERATOR_DONE"
        print(f"OPERATOR: {message} -- waiting for {done}", flush=True)
        deadline = time.monotonic() + 600.0
        while not done.exists():
            if time.monotonic() > deadline:
                raise DiagError("no OPERATOR_DONE within 10 minutes")
            time.sleep(0.2)
        done.unlink()


def run_once(b: WdtBench, load: Callable[[WdtBench], dict | None], settle: float = 1.0) -> dict:
    """'p' (clears), load, settle, 'p'."""
    b.perf()
    t = time.monotonic_ns()
    extra = load(b) or {}
    time.sleep(settle)
    p = b.perf()
    gap = p["wdtGapMax"]
    return {"wdt_gap_ms": gap["max_us"] / 1e3 if gap["max_us"] is not None else None,
            "from": gap["from"], "to": gap["to"],
            "loop_max_ms": p.get("loop", {}).get("max_us", 0) / 1e3, "slotLate": p.get("slotLate"),
            "udpSendFailed": p.get("udpSendFailed"), "booted": b.booted_since(t), **extra}


# --------------------------------------------------------------------------------------
# loads


def servo_load(b: WdtBench, duration: float) -> dict:
    t = b.cfg.section("timing")
    rate, amp, period = float(t["target_rate_hz"]), float(t["sine_amplitude"]), float(t["sine_period"])
    if b.args.no_motion:
        amp = 0.0  # the same servo updates and messages, the target stays at the position
    axis, n = b.cfg.test_axis, b.cfg.num_motors
    pos0 = int(b.link.request("/getPosition", axis, reply="/position",
                              predicate=lambda a: int(a[0]) == axis).args[1])
    t_ns = b.send("/enableServoMode", 255, 1).t_ns
    b.send("/setPositionReportInterval", 255, 20)
    start = time.monotonic()
    next_t = start
    while time.monotonic() - start < duration:
        targets = [0] * n
        targets[axis - 1] = pos0 + int(round(amp * math.sin(2 * math.pi * (next_t - start) / period)))
        b.send("/setTargetPositionList", *targets)
        next_t += 1.0 / rate
        time.sleep(max(0.0, next_t - time.monotonic()))
    b.send("/setPositionReportInterval", 255, 0)
    b.send("/enableServoMode", 255, 0)
    b.send("/softHiZ", 255)
    errors = [list(e.args) for e in b.link.entries("rx", t_ns) if e.address == "/error/command"]
    return {"errors": errors[:8]}


def wildcard_load(b: WdtBench) -> dict:
    t = b.send("/get*", 255).t_ns
    time.sleep(3.0)
    return {"replies": len(b.link.entries("rx", t))}


def set_dest(b: WdtBench, ip: str) -> str | None:
    t = b.ser.write('{"network":{"destIp":[' + ip.replace(".", ",") + "]}}")
    m = b.ser.wait_for(r"\{\"result\".*\}\r?\n", t, 5.0)
    return m.group(0).strip() if m else None


def no_sd(b: WdtBench) -> bool:
    cfgname = b.link.request("/getConfigName", reply="/configName").args
    return int(cfgname[1]) == 0


# --------------------------------------------------------------------------------------
# cases


def case_w1(b: WdtBench) -> list[dict]:
    return [run_once(b, lambda b: servo_load(b, b.args.duration)) for _ in range(b.args.repeat)]


def case_w2(b: WdtBench) -> list[dict]:
    return [run_once(b, wildcard_load) for _ in range(b.args.repeat)]


def _dead(b: WdtBench, load) -> list[dict]:
    if not no_sd(b):
        raise DiagError("sdInitializeSucceeded is 1: remove the SD card first ('{' would save the dead "
                        "destination to it), then restart the board")
    unused, why = address_unused(b.args.dead_ip)
    if not unused:
        raise DiagError(f"{b.args.dead_ip} is not unused ({why}); choose another --dead-ip")
    reply = set_dest(b, b.args.dead_ip)
    try:
        runs = [dict(run_once(b, load), dest_reply=reply) for _ in range(b.args.repeat)]
    finally:
        b.connect(5.0)  # /setDestIp: back to this Mac
    return runs


def case_w3(b: WdtBench) -> list[dict]:
    return _dead(b, wildcard_load)


def case_w4(b: WdtBench) -> list[dict]:
    def load(b):
        b.send("/setPositionReportInterval", 255, 20)
        time.sleep(b.args.duration)
        b.send("/setPositionReportInterval", 255, 0)
    return _dead(b, load)


def case_w5(b: WdtBench) -> list[dict]:
    def load(b):
        b.send("/setPositionReportInterval", 255, 20)
        time.sleep(0.5)
        b.human("unplug the Ethernet cable of the board, wait about 5 s, plug it in again")
        time.sleep(5.0)  # link up, ARP
        try:
            b.connect(15.0)
        finally:
            b.send("/setPositionReportInterval", 255, 0)
    return [run_once(b, load) for _ in range(b.args.repeat)]


def case_w6(b: WdtBench) -> list[dict]:
    # The IP address follows the ID (isMyIpAddId): only the USB serial is used until the
    # switch is back; 'p' covers both changes.
    def load(b):
        b.human("change the DIP switch ID (any other value), wait about 2 s, set it back")
        time.sleep(2.0)
        b.connect(15.0)
        return {"new_ip_lines": re.findall(r"New IP: [\d.]+", b.ser.text(b.t0_ns))[-2:]}
    return [run_once(b, load) for _ in range(b.args.repeat)]


def case_w7(b: WdtBench) -> list[dict]:
    runs = []
    for _ in range(b.args.repeat):
        for name, until in (("G", r"\{\"board\".*\}\r?\n"), ("T", r"\{\"board\".*\]\}\r?\n"),
                            ("s", None), ("c", None)):
            def load(b, name=name, until=until):
                t = b.ser.write(name)
                if until:
                    b.ser.wait_for(until, t, 10.0)
                else:
                    time.sleep(2.0)
            runs.append(dict(run_once(b, load), command=name))
        g = b.ser.command("G", until=r"\{\"board\".*\}\r?\n", timeout=10.0)
        text = next(line.strip() for line in g.splitlines() if line.startswith('{"board"'))
        # '{' consumes the first '{' as the command and parses the rest as the config.

        def brace(b, text=text):
            t = b.ser.write(text)
            m = b.ser.wait_for(r"\{\"result\".*\}\r?\n", t, 15.0)
            return {"reply": m.group(0).strip() if m else None}
        runs.append(dict(run_once(b, brace), command="{"))

        def save(b):
            t = b.send("/saveConfig").t_ns
            r = b.link.wait_for("/saveConfig", 5.0, t) or b.link.wait_for("/error/saveConfig", 0.5, t)
            return {"reply": [r.address, *r.args] if r else None}
        runs.append(dict(run_once(b, save), command="/saveConfig"))
    return runs


def case_w8(b: WdtBench) -> list[dict]:
    runs = []
    for _ in range(b.args.repeat):
        def closed(b):
            b.ser.close()
            for _ in range(3):
                b.send("/resetMotorDriver", 255)  # prints a line per motor
                time.sleep(0.5)
            b.ser.reconnect(5.0)
        runs.append(dict(run_once(b, closed), case="port closed, /resetMotorDriver 255 x 3"))

        def not_read(b):
            import serial

            b.ser.close()
            s = serial.Serial(b.ser.port, 115200, timeout=0)
            try:
                for _ in range(20):
                    s.write(b"s")
                    time.sleep(0.2)
                time.sleep(1.0)
                s.reset_input_buffer()
            finally:
                s.close()
            b.ser.reconnect(5.0)
        runs.append(dict(run_once(b, not_read), case="port open, 's' x 20 not read"))
    return runs


def case_w9(b: WdtBench) -> list[dict]:
    runs = []
    for _ in range(b.args.repeat):
        # --no-motion: an axis other than the test axis (no motor there on the bench)
        axis = b.cfg.test_axis % b.cfg.num_motors + 1 if b.args.no_motion else b.cfg.test_axis

        def homing(b, axis=axis):
            b.send("/homing", axis)
            time.sleep(5.0)
            b.send("/hardHiZ", axis)
        runs.append(dict(run_once(b, homing), case="/homing"))

        def reset_drivers(b):
            b.send("/resetMotorDriver", 255)
            time.sleep(1.0)
        runs.append(dict(run_once(b, reset_drivers), case="/resetMotorDriver 255"))
        runs.append(dict(run_once(b, lambda b: servo_load(b, 5.0)), case="servo all motors"))
    return runs


def case_w10(b: WdtBench) -> list[dict]:
    runs = []
    for _ in range(b.args.repeat):
        t = b.ser.write("R")
        b.ser.wait_gone(3.0)
        b.ser.reconnect(15.0)
        booted = b.link.wait_for("/booted", 15.0, t)
        time.sleep(5.0)
        p = b.perf()
        gap = p["wdtGapMax"]
        b.connect(5.0)
        runs.append({"wdt_gap_ms": gap["max_us"] / 1e3 if gap["max_us"] is not None else None,
                     "from": gap["from"], "to": gap["to"],
                     "loop_max_ms": p.get("loop", {}).get("max_us", 0) / 1e3,
                     "booted": [list(booted.args)] if booted else []})
    return runs


def _status(b: WdtBench) -> dict:
    t = b.ser.command("T", until=r"\{\"board\".*\]\}\r?\n", timeout=5.0)
    m = re.search(r'"resetCause":(\d+),"resetCauseRaw":(\d+)', t)
    return {"resetCause": int(m.group(1)), "resetCauseRaw": int(m.group(2))} if m else {}


RUNNERS = {"w1": case_w1, "w2": case_w2, "w3": case_w3, "w4": case_w4, "w5": case_w5, "w6": case_w6,
           "w7": case_w7, "w8": case_w8, "w9": case_w9, "w10": case_w10}


def cmd_wdt(cfg: Config, args) -> int:
    from . import commands

    cases = args.cases or [c for c in CASES if not CASES[c].human]
    out = commands.new_result_dir("wdt_" + "_".join(cases) + (f"_{args.label}" if args.label else ""))
    summary: dict = {"model": cfg.model, "elf": args.elf, "label": args.label, "cases": {}}
    rows = []
    with WdtBench(cfg, args, out) as b:
        try:
            summary["local_ip"] = b.connect()
            summary["firmware"] = commands.firmware_version(b.link)
            summary["product"] = bool(args.product)
            summary["status_before"] = _status(b)
            print(f"firmware: {summary['firmware']}")
            for cid in cases:
                case = CASES[cid]
                print(f"{cid.upper()}: {case.title}", flush=True)
                if case.human:
                    b.human(f"{cid.upper()} needs: {case.human}")
                try:
                    runs = RUNNERS[cid](b)
                except Exception as ex:  # keep the other cases
                    runs = [{"error": f"{type(ex).__name__}: {ex}"}]
                names = resolve(args.elf, [a for r in runs for a in (r.get("from"), r.get("to")) if a])
                for r in runs:
                    r["from_at"] = names.get(r.get("from"))
                    r["to_at"] = names.get(r.get("to"))
                    rows.append([cid, r.get("command") or r.get("case") or "", r.get("wdt_gap_ms"),
                                 r.get("loop_max_ms"), r.get("from_at") or r.get("from"),
                                 r.get("to_at") or r.get("to"), r.get("booted") or r.get("error") or ""])
                summary["cases"][cid] = {"title": case.title, "runs": runs}
        finally:
            b.ser.save(out / "serial_log.txt", b.t0_ns)
            b.link.save_log(out / "osc_log.jsonl")
    # Product build: no early warning or restart may happen (W10 restarts on purpose).
    summary["status_after"] = None
    try:
        with WdtBench(cfg, args, out) as b2:
            summary["status_after"] = _status(b2)
    except Exception as ex:  # noqa: BLE001
        summary["status_after"] = f"error: {ex}"
    commands.write_json(out / "summary.json", summary)
    print(f"resetCause before {summary['status_before']}, after {summary['status_after']}")
    commands.print_table(["case", "what", "wdtGap ms", "loop max ms", "from", "to", "booted / error"],
                         [[r[0], r[1], commands.fmt(r[2], 2), commands.fmt(r[3], 2), r[4], r[5], r[6]]
                          for r in rows])
    print(f"results: {out}")
    return 0
