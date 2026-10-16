"""hil.py usb: the firmware stops when the board's USB cable is unplugged (TASK_usb_unplug).

Record only, also with motor.motor_connected = true (nothing is judged from the drivers or
OUT1A). While the AD2 Digital In records the brake outputs (J7), P0, P1, P2, the 3.3 V and,
if wired, the USB VBUS (usb.vbus_dio, through a divider), the host

- sends /getVersion every HEARTBEAT_S (alive check on the host clock, an aid to the AD2
  times: each request is also a P1 pulse, used for the host -> AD2 clock fit);
- asks the operator to unplug (or plug) the USB cable;
- waits for the replies to stop and for /booted (or a timeout).

The cable is plugged back in after the recording (operator request), then the USB serial is
reopened and, with a firmware that has it, the fault record ('H', HIL_PROBE builds) is read.

summary.json, per MCU stop (a P0 gap of STOP_GAP_S or more):
- stop: the last P0 edge before the gap (the MCU stopped there);
- early warning: a long P2 pulse (EW_MIN_S or more) in the gap. P2 also pulses during
  every servo update (microseconds), the early warning holds it high until the reset;
  the HIL HardFault handler toggles it every 0.5 ms instead (p2_edges_in_stop);
- brake outputs: the J7 changes in the gap (engaged by the early warning, or when the pins
  float at the MCU reset);
- loop resumes: the first P0 edge that starts the loop toggling after the gap (restart.py);
- the /booted resetCause and the heartbeat gap (host);
- with VBUS: the unplug (VBUS fall) to the stop.
"""

from __future__ import annotations

import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import analyze, motor, restart
from .config import Config
from .diag import DiagError, SerialDiag
from .motor import AXIS, Act, close_serial, osc, wait
from .osc import OscLink

HEARTBEAT_S = 0.1
# A P0 gap this long is an MCU stop (the loop toggles P0 about every 50 us; the longest
# normal pass, the configTool JSON or an SD write, is well below this).
STOP_GAP_S = 0.2
# The early warning holds P2 high for the brake wait (100 ms on the bench); the servo
# update pulses are microseconds.
EW_MIN_S = 0.005
# A heartbeat gap this long counts as "no reply" (the board answers within a few ms).
HEARTBEAT_GAP_S = 0.3
# The HIL HardFault handler toggles P2 every 0.5 ms (src/hilFault.h): this many P2 edges
# in a stop, without a long P2 pulse, mark a HardFault.
HARDFAULT_MIN_P2_EDGES = 20
# Waits after the operator request: for the replies to stop, then for /booted.
UNPLUG_WAIT_S = 120.0
BOOTED_WAIT_S = 10.0


@dataclass(frozen=True)
class UsbStep:
    id: str
    title: str
    observe: str
    actions: tuple
    expect: str = ""
    brake: bool = True
    position_report_ms: int = 0
    port_open: bool = False  # keep the host's USB serial port open through the unplug
    plug: bool = False  # U6: the plug-in is recorded (the unplug comes first, also recorded)


def unplug(label: str = "unplug") -> Act:
    """Operator request, then waits for the heartbeat to stop (UNPLUG_WAIT_S) and for
    /booted (BOOTED_WAIT_S after the stop). Neither is an error: no stop is a result."""
    return Act("unplug", "unplug the USB cable of the board now (the AD2 USB stays connected)", label=label)


def plug(label: str = "plug") -> Act:
    return Act("plug", "plug the USB cable of the board back in now", label=label)


_RELEASED = (osc("/activate", AXIS, 1, label="activate1"), wait(1.0))

STEPS: dict[str, UsbStep] = {s.id: s for s in [
    UsbStep("u1", "U1: brake enabled, RELEASED (/activate 1), stopped, host port closed",
            "rod: any drop at the unplug and at the restart",
            _RELEASED + (close_serial(label="close"), wait(1.0), unplug(), wait(2.0)),
            "hypothesis: P0 stops at the unplug, no early warning, J7 released until the watchdog reset "
            "about 4 s later (resetCause 4)"),
    UsbStep("u2", "U2: as U1 with the host port open",
            "rod: any drop at the unplug and at the restart",
            _RELEASED + (wait(1.0), unplug(), wait(2.0)), "as U1", port_open=True),
    UsbStep("u3", "U3: brake enabled, ENGAGED (/activate 0), stopped, host port closed",
            "brake: stays engaged",
            (osc("/activate", AXIS, 1), wait(1.0), osc("/activate", AXIS, 0, label="activate0"), wait(1.0),
             close_serial(label="close"), wait(1.0), unplug(), wait(2.0)),
            "a stop as U1 (the brake is engaged already); does the early warning run"),
    UsbStep("u4", "U4: test axis turning at 5 step/s (/run, brake released), host port closed; weight off",
            "rod: does it keep turning while the board is stopped (until the restart)",
            _RELEASED + (osc("/run", AXIS, 5.0, label="run"), wait(2.0), close_serial(label="close"), wait(1.0),
                         unplug(), wait(2.0)),
            "the driver keeps turning until the MCU reset puts it in standby"),
    UsbStep("u5", "U5: as U1 with the position report every 20 ms",
            "rod: any drop at the unplug and at the restart",
            _RELEASED + (close_serial(label="close"), wait(1.0), unplug(), wait(2.0)),
            "as U1; the stop time may depend on the OSC load", position_report_ms=20),
    UsbStep("u6", "U6: plug the cable in (after the unplug and its /booted), RELEASED, port closed",
            "rod: any drop at the plug-in",
            _RELEASED + (close_serial(label="close"), wait(1.0), unplug(), wait(2.0),
                         osc("/activate", AXIS, 1, label="activate1_again"), wait(1.0), plug(), wait(10.0)),
            "does the board stop at the plug-in too (enumeration)", plug=True),
]}


class Heartbeat:
    """Sends /getVersion every period seconds from a thread until stop()."""

    def __init__(self, link: OscLink, period: float = HEARTBEAT_S):
        self.link, self.period = link, period
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="heartbeat", daemon=True)

    def start(self) -> "Heartbeat":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(2.0)

    def _run(self) -> None:
        nxt = time.monotonic()
        while not self._stop.is_set():
            try:
                self.link.send("/getVersion")
            except OSError:  # no route while the board restarts
                pass
            nxt += self.period
            self._stop.wait(max(0.0, nxt - time.monotonic()))


class UsbRunner(motor.Runner):
    """motor.Runner with the unplug / plug actions and the setup of the USB step."""

    def __init__(self, cfg: Config, ad2, diag: SerialDiag, link: OscLink, step: UsbStep):
        super().__init__(cfg, ad2, diag, link, step.brake)
        self.step = step

    def setup(self) -> None:
        for msg in setup_actions(self.cfg, self.step):
            self.link.send(*msg)
            time.sleep(0.02)

    def run(self, actions) -> None:
        rest = []
        for act in actions:
            if act.kind in ("unplug", "plug"):
                super().run(rest)
                rest = []
                self._operator_wait(act)
            else:
                rest.append(act)
        super().run(rest)

    def _operator_wait(self, act: Act) -> None:
        t_ask = time.monotonic_ns()
        self._event("operator", act.address, act.label)
        print(f"OPERATOR: {act.address}" + (", then press Enter" if sys.stdin.isatty() else ""), flush=True)
        if sys.stdin.isatty():
            input()
            self._event("operator_done", "Enter pressed", act.label)
        if act.kind == "plug":
            return
        t_lost = wait_heartbeat_lost(self.link, t_ask, UNPLUG_WAIT_S)
        if t_lost is None:
            self._event("heartbeat", f"replies did not stop within {UNPLUG_WAIT_S:.0f} s", act.label)
            return
        self._event("heartbeat", "replies stopped (last reply)", act.label, t_ns=t_lost)
        e = self.link.wait_for("/booted", BOOTED_WAIT_S, t_lost)
        self._event("wait_osc", f"/booted {'received' if e else 'NOT received'}", act.label)
        if e is not None:
            time.sleep(1.0)
            try:
                self.link.set_dest_ip()
                self.setup()
                self._event("setup", "setup applied again")
            except OSError as ex:
                self.errors.append(f"setup after /booted: {ex}")
            except Exception as ex:  # OscError
                self.errors.append(f"setup after /booted: {ex}")


def wait_heartbeat_lost(link: OscLink, since_ns: int, timeout: float, gap: float = HEARTBEAT_GAP_S) -> int | None:
    """Waits until no /version has arrived for gap seconds (after one arrived since since_ns).
    Returns the host time of the last /version, None on timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rx = [e.t_ns for e in link.entries("rx", since_ns) if e.address == "/version"]
        if rx and time.monotonic_ns() - rx[-1] >= gap * 1e9:
            return rx[-1]
        time.sleep(0.05)
    return None


# --------------------------------------------------------------------------------------
# Analysis (pure functions of the edges, tested in tests/test_usb.py)
# --------------------------------------------------------------------------------------


def find_stops(p0: np.ndarray, t_end: float, gap: float = STOP_GAP_S) -> list[tuple[float, float | None]]:
    """(last P0 edge, loop resume) of each P0 gap of at least gap seconds, also one up to
    the end of the capture (t_end). The early warning's P0 marker edge (WDT_Handler) and
    the edges of the boot inside a gap are not a resume: the gap ends where the loop
    toggling starts again (restart.loop_resume), None if it does not within the capture."""
    t = np.asarray(p0, float)
    out: list[tuple[float, float | None]] = []
    i = 0
    while i < t.size:
        nxt = t[i + 1] if i + 1 < t.size else t_end
        if nxt - t[i] >= gap:
            resume = restart.loop_resume(t, float(t[i]))
            out.append((float(t[i]), resume))
            if resume is None:
                break
            i = int(np.searchsorted(t, resume))
        else:
            i += 1
    return out


def early_warning(p2: analyze.Edges, t_from: float, t_to: float, min_width: float = EW_MIN_S) -> tuple[float, float | None] | None:
    """First P2 rise in (t_from, t_to) that stays high min_width or more: (rise, fall), fall
    None when P2 is still high at t_to (the reset)."""
    for r in p2.rise[(p2.rise > t_from) & (p2.rise < t_to)]:
        f = p2.fall[p2.fall > r]
        if f.size == 0 or f[0] - r >= min_width:
            return float(r), (float(f[0]) if f.size and f[0] <= t_to else None)
    return None


def analyze_stop(t_stop: float, t_resume: float | None, p2: analyze.Edges, brakes: dict[int, analyze.Edges],
                 vbus: analyze.Edges | None, t_end: float) -> dict:
    """One stop: times in ms from the stop (the last P0 edge)."""
    t_to = t_resume if t_resume is not None else t_end
    ms = lambda t: None if t is None else round((t - t_stop) * 1e3, 2)  # noqa: E731
    ew = early_warning(p2, t_stop, t_to)
    res: dict = {"stop_s": round(t_stop, 6), "loop_resumes_ms": ms(t_resume),
                 "early_warning_ms": ms(ew[0]) if ew else None,
                 "early_warning_p2_high_ms": round((ew[1] - ew[0]) * 1e3, 2) if ew and ew[1] is not None else None,
                 "p2_edges_in_stop": int(np.sum((p2.all > t_stop) & (p2.all < t_to))),
                 "brake_changes_ms": {}}
    for m, e in brakes.items():
        ch = [[ms(t), "engaged"] for t in e.fall if t_stop < t < t_to] + \
             [[ms(t), "released"] for t in e.rise if t_stop < t < t_to]
        lvl = int(level_at(e, t_stop))
        if ch or lvl:
            res["brake_changes_ms"][m] = {"at_stop": "released" if lvl else "engaged", "changes": sorted(ch)}
    if vbus is not None:
        falls = vbus.fall[vbus.fall < t_stop + 0.01]
        res["vbus_fall_before_stop_ms"] = ms(float(falls[-1])) if falls.size else None
        rises = vbus.rise[vbus.rise < t_stop + 0.01]
        res["vbus_rise_before_stop_ms"] = ms(float(rises[-1])) if rises.size else None
    return res


def level_at(e: analyze.Edges, t: float) -> bool:
    return bool(analyze.Step.from_edges(e).at(t))


def heartbeat_gaps(rx_s: np.ndarray, gap: float = HEARTBEAT_GAP_S) -> list[tuple[float, float]]:
    """(last reply, next reply) of each gap of at least gap seconds between /version replies."""
    t = np.sort(np.asarray(rx_s, float))
    if t.size < 2:
        return []
    d = np.diff(t)
    return [(float(t[i]), float(t[i + 1])) for i in np.flatnonzero(d >= gap)]


def booted_causes(rx: list) -> list[dict]:
    """/booted id resetCause (src/utils.cpp sendBootMsg)."""
    return [{"t_ns": e.t_ns, "id": e.args[0] if e.args else None,
             "resetCause": e.args[1] if len(e.args) > 1 else None} for e in rx if e.address == "/booted"]


def heartbeat_stops(gaps: list[tuple[float, float]], brakes: dict[int, analyze.Edges], boots: list[dict],
                    to_s) -> list[dict]:
    """--no-probe (a firmware without the HIL probes, e.g. main): each heartbeat gap with the
    J7 changes in it, times in ms from the last reply (host times mapped to the AD2 time;
    network and host latency included, so judged with a wide window)."""
    out = []
    for a, b in gaps:
        ms = lambda t: round((t - a) * 1e3, 1)  # noqa: E731
        res: dict = {"last_reply_s": round(a, 3), "next_reply_ms": ms(b), "brake_changes_ms": {}}
        for m, e in brakes.items():
            ch = [[ms(t), "engaged"] for t in e.fall if a < t < b] + [[ms(t), "released"] for t in e.rise if a < t < b]
            lvl = int(level_at(e, a))
            if ch or lvl:
                res["brake_changes_ms"][m] = {"at_last_reply": "released" if lvl else "engaged", "changes": sorted(ch)}
        bt = [x for x in boots if to_s(x["t_ns"]) > a]
        if bt:
            res["booted_ms"] = ms(to_s(bt[0]["t_ns"]))
            res["resetCause"] = bt[0]["resetCause"]
        out.append(res)
    return out


def analyze_usb(cfg: Config, cap, rx: list, fit, vbus_dio: int | None, probes: bool = True) -> dict:
    seg = cap.digital
    probe = cfg.probe_dio
    p0 = analyze.bit_edges(seg, probe[0]).all if probes else np.zeros(0)
    p2 = analyze.bit_edges(seg, probe[2])
    brakes = {m: analyze.bit_edges(seg, int(d)) for m, d in enumerate(cfg.ad2["dio"]["brake"], start=1)}
    vbus = analyze.bit_edges(seg, vbus_dio) if vbus_dio is not None else None
    host = lambda t_ns: (t_ns - cap.host_trigger_ns) / 1e9  # noqa: E731
    to_ad2 = (lambda t: float(fit.to_ad2(t))) if fit else (lambda t: t)
    stops = [analyze_stop(a, b, p2, brakes, vbus, seg.duration) for a, b in find_stops(p0, seg.duration)]
    hb = [to_ad2(host(e.t_ns)) for e in rx if e.address == "/version"]
    gaps = heartbeat_gaps(np.array(hb))
    boots = booted_causes(rx)
    for s in stops:
        t0 = s["stop_s"]
        g = [x for x in gaps if x[0] <= t0 + HEARTBEAT_GAP_S and x[1] > t0]
        if g:
            s["heartbeat_last_ms"] = round((g[0][0] - t0) * 1e3, 1)
            s["heartbeat_next_ms"] = round((g[0][1] - t0) * 1e3, 1)
        b = [x for x in boots if to_ad2(host(x["t_ns"])) > t0]
        if b:
            s["booted_ms"] = round((to_ad2(host(b[0]["t_ns"])) - t0) * 1e3, 1)
            s["resetCause"] = b[0]["resetCause"]
        s["kind"] = classify(s)
    return {"stops": stops,
            "heartbeat_stops": None if probes else heartbeat_stops(gaps, brakes, boots, lambda t_ns: to_ad2(host(t_ns))),
            "heartbeat_gaps_ms": [[round(a * 1e3, 1), round((b - a) * 1e3, 1)] for a, b in gaps],
            "booted": [{"host_s": round(host(x["t_ns"]), 3), "resetCause": x["resetCause"]} for x in boots],
            "vbus": None if vbus is None else {"fall_s": vbus.fall.round(6).tolist(),
                                                "rise_s": vbus.rise.round(6).tolist()}}


def classify(stop: dict) -> str:
    if stop.get("loop_resumes_ms") is None:
        return "no restart in the capture"
    if stop.get("early_warning_ms") is not None:
        return "stop, early warning, reset"
    if stop.get("p2_edges_in_stop", 0) >= HARDFAULT_MIN_P2_EDGES:
        return "HardFault (HIL marker on P2), reset (watchdog period)"
    return "stop, no early warning, reset (watchdog period)"


# --------------------------------------------------------------------------------------
# Command
# --------------------------------------------------------------------------------------


def setup_actions(cfg: Config, step: UsbStep) -> list[tuple]:
    out = [m for m in motor.setup_actions(cfg, step.brake) if m[0] != "/setPositionReportInterval"]
    out.append(("/setPositionReportInterval", cfg.test_axis, step.position_report_ms))
    return out


def _safe_close(diag: SerialDiag) -> None:
    try:
        diag.close()
    except Exception:  # the port is gone after the unplug
        diag._ser = None


def read_fault_record(diag: SerialDiag) -> str | None:
    """'H' (HIL_PROBE builds with the fault record): prints and clears the record. None
    when the firmware does not answer it."""
    try:
        return diag.command("H", until=r"HIL record done", timeout=3.0)
    except DiagError:
        return None


# A heartbeat gap at least this long with /booted resetCause 4 after it: a stop until the
# watchdog period (HardFault or a hang); the HardFault runs measured 4.0-4.4 s.
WATCHDOG_STOP_MIN_S = 3.0
RESET_CAUSES = {0: "unknown", 1: "power-on", 2: "brown-out", 3: "external", 4: "watchdog", 5: "software"}


def classify_gap(gap_s: float | None, booted: bool, reset_cause) -> str:
    """No-AD2 judgement of one unplug from the host only (TASK_usb_unplug_followup)."""
    if gap_s is None:
        return "no stop (no gap in the replies)"
    if not booted:
        return "replies stopped, no /booted"
    if reset_cause is None:
        return "restart, /booted without resetCause (main)"
    if reset_cause == 4:
        return ("stop, watchdog reset (HardFault or hang)" if gap_s >= WATCHDOG_STOP_MIN_S
                else "watchdog reset (short stop)")
    return f"{RESET_CAUSES.get(reset_cause, reset_cause)} reset"


def host_gaps(rx: list, t0_ns: int, t_end_s: float | None = None, gap: float = HEARTBEAT_GAP_S,
              booted_within: float = 3.0) -> list[dict]:
    """No AD2: each gap of the /version replies (host times, s from t0_ns) with the first
    /booted from the gap's start to booted_within s after its end. With t_end_s (the end of
    the heartbeat), also a gap from the last reply to t_end_s (no reply came back: gap_s is
    then at least that, "open")."""
    s = lambda t_ns: (t_ns - t0_ns) / 1e9  # noqa: E731
    hb = np.sort(np.array([s(e.t_ns) for e in rx if e.address == "/version"]))
    boots = booted_causes(rx)
    spans = [(a, b, False) for a, b in heartbeat_gaps(hb, gap)]
    if t_end_s is not None and hb.size and t_end_s - hb[-1] >= gap:
        spans.append((float(hb[-1]), float(t_end_s), True))
    out = []
    for a, b, open_end in spans:
        bt = [x for x in boots if a < s(x["t_ns"]) <= b + booted_within]
        cause = bt[0]["resetCause"] if bt else None
        out.append({"last_reply_s": round(a, 3), "gap_s": round(b - a, 3), "open": open_end,
                    "booted_after_last_reply_s": round(s(bt[0]["t_ns"]) - a, 3) if bt else None,
                    "resetCause": cause, "kind": classify_gap(b - a, bool(bt), cause)
                    + (" (replies did not come back)" if open_end else "")})
    return out


def parse_conditions(items: list[str] | None) -> dict:
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        out[k.strip()] = v.strip()
    return out


def cmd_usb_no_ad2(cfg: Config, args) -> int:
    """--no-ad2: no AD2 (and no SWD); judged from the heartbeat gap and /booted only. The
    fault record ('H') is read after the plug-in with --read-record."""
    from . import commands

    step = STEPS[args.step]
    out = commands.new_result_dir(f"usb_{step.id}_noad2" + (f"_{args.label}" if args.label else ""))
    print(f"{step.id} (no AD2): {step.title}\nOBSERVE: {step.observe}", flush=True)
    diag = SerialDiag(cfg.serial_port).open()
    with commands.open_link(cfg) as link:
        try:
            commands.connect_board(link)
            version = commands.firmware_version(link)
            print(f"firmware: {version}")
            runner = UsbRunner(cfg, None, diag, link, step)
            runner.setup()
            time.sleep(0.3)
            before = motor._get(link, "/getBrakeStatus", "/brakeStatus", cfg.test_axis)
            t0 = time.monotonic_ns()
            hb = Heartbeat(link).start()
            time.sleep(0.3)
            try:
                runner.run(step.actions)
                time.sleep(3.0)  # replies after the restart, for the end of the gap
            finally:
                hb.stop()
                t_end_s = (time.monotonic_ns() - t0) / 1e9
            after = motor._get(link, "/getBrakeStatus", "/brakeStatus", cfg.test_axis)
        finally:
            _safe_close(diag)
    link.save_log(out / "osc_log.jsonl")
    rx = link.entries("rx", t0)
    gaps = host_gaps(rx, t0, t_end_s)
    summary = {"step": step.id, "title": step.title, "mode": "no AD2", "firmware": version,
               "conditions": parse_conditions(args.conditions), "axis": cfg.test_axis,
               "setup": [list(m) for m in setup_actions(cfg, step)], "before": before, "after": after,
               "gaps": gaps, "booted": [{"host_s": round((x["t_ns"] - t0) / 1e9, 3), "resetCause": x["resetCause"]}
                                        for x in booted_causes(rx)],
               "events": [{**e, "host_s": round((e["t_ns"] - t0) / 1e9, 3)} for e in runner.events],
               "fault_record": None, "errors": runner.errors, "observations": {}}
    commands.write_json(out / "summary.json", summary)
    for g in gaps:
        print(f"gap {g['gap_s']} s from {g['last_reply_s']} s: {g['kind']} (resetCause {g['resetCause']})")
    if not gaps:
        print("no gap in the replies")
    if runner.errors:
        print(f"errors: {runner.errors}")
    print(f"results: {out}")
    return 0


def cmd_read_record(cfg: Config, args) -> int:
    """--read-record DIR: after the plug-in, reopens the USB serial, reads 'H' (HIL builds
    with the fault record) and stores it in DIR/summary.json."""
    import json
    from . import commands

    path = Path(args.read_record) / "summary.json"
    data = json.loads(path.read_text())
    diag = SerialDiag(cfg.serial_port)
    try:
        diag.reconnect(args.wait)
        record = read_fault_record(diag)
    except (DiagError, OSError) as ex:
        record = f"error: serial reconnect: {ex}"
    finally:
        _safe_close(diag)
    data["fault_record"] = record
    data["fault"] = parse_fault_record(record)
    commands.write_json(path, data)
    print(record if record else "no fault record (firmware without 'H')")
    print(f"fault: {data['fault']}")
    return 0


FAULT_LINE = re.compile(r"HIL (hardfault|early warning): pc 0x([0-9A-F]+) lr 0x([0-9A-F]+) xpsr 0x([0-9A-F]+)")
RECORD_LINE = re.compile(r"HIL record: boot (\d+) flags 0x([0-9A-F]+)")


def parse_fault_record(text: str | None) -> dict | None:
    """'H' output -> {flags, hardfault: {pc, lr, xpsr}, early_warning: {...}}."""
    if not text or text.startswith("error"):
        return None
    m = RECORD_LINE.search(text)
    out: dict = {"boot": int(m.group(1)), "flags": int(m.group(2), 16)} if m else {}
    for kind, pc, lr, xpsr in FAULT_LINE.findall(text):
        out[kind.replace(" ", "_")] = {"pc": f"0x{pc}", "lr": f"0x{lr}", "xpsr": f"0x{xpsr}"}
    return out


def cmd_usb(cfg: Config, args) -> int:
    from . import commands

    if getattr(args, "read_record", None):
        return cmd_read_record(cfg, args)
    if getattr(args, "no_ad2", False):
        return cmd_usb_no_ad2(cfg, args)
    step = STEPS[args.step]
    vbus_dio = cfg.raw.get("usb", {}).get("vbus_dio")
    vbus_dio = int(vbus_dio) if vbus_dio is not None and int(vbus_dio) >= 0 else None
    out = commands.new_result_dir(f"usb_{step.id}" + (f"_{args.label}" if args.label else ""))
    print(f"{step.id}: {step.title}\nOBSERVE: {step.observe}\nexpect: {step.expect}", flush=True)
    if vbus_dio is None:
        print("note: no usb.vbus_dio: the unplug time is not recorded (the stop is the last P0 edge)")
    probe = cfg.probe_dio
    probes = not args.no_probe
    dios = ([int(d) for d in cfg.ad2["dio"]["brake"]] + (probe[:3] if probes else [])
            + ([cfg.v33_dio] if cfg.v33_dio is not None else []))
    if vbus_dio is not None:
        dios.append(vbus_dio)
    mask = commands.mask_of(dios)
    rate = args.rate or 1e4
    diag = SerialDiag(cfg.serial_port).open()
    with commands.open_ad2(cfg) as ad2, commands.open_link(cfg) as link:
        version = None
        try:
            commands.connect_board(link)
            version = commands.firmware_version(link)
            print(f"firmware: {version}")
            runner = UsbRunner(cfg, ad2, diag, link, step)
            runner.setup()
            time.sleep(0.3)
            before = motor._get(link, "/getBrakeStatus", "/brakeStatus", cfg.test_axis)
            rec = ad2.start_recording(None, digital_rate=rate, sensible_mask=mask)
            hb = Heartbeat(link).start()
            time.sleep(0.3)
            try:
                runner.run(step.actions)
                time.sleep(0.5)
            finally:
                hb.stop()
                cap = rec.stop()
            cap.save(out / "capture.npz")
            after = motor._get(link, "/getBrakeStatus", "/brakeStatus", cfg.test_axis)
        finally:
            _safe_close(diag)
    link.save_log(out / "osc_log.jsonl")
    tx = link.entries("tx", cap.host_trigger_ns, cap.host_stop_ns)
    rx = link.entries("rx", cap.host_trigger_ns, cap.host_stop_ns)
    # Every heartbeat request is a P1 pulse too: all sent messages go into the clock fit;
    # the heartbeat rows are left out of the timeline.
    rows, info = motor.build_timeline(cfg, cap, tx, rx, runner.events)
    rows = [r for r in rows if not r["detail"].startswith(("/getVersion", "/version"))]
    motor._write_rows(out / "timeline.csv", rows)
    fit, _, _ = motor.clock_fit(np.array([commands.host_s(cap, e.t_ns) for e in tx]),
                                analyze.bit_edges(cap.digital, probe[1]).rise)
    res = analyze_usb(cfg, cap, rx, fit if probes else None, vbus_dio, probes)
    commands.prompt("OPERATOR: plug the USB cable of the board back in (if it is out), then press Enter ")
    try:
        diag.reconnect(30.0)
        record = read_fault_record(diag) if probes else None
    except (DiagError, OSError) as ex:
        record = f"error: serial reconnect: {ex}"
    finally:
        _safe_close(diag)
    summary = {"step": step.id, "title": step.title, "observe": step.observe, "expect": step.expect,
               "firmware": version, "probes": probes, "axis": cfg.test_axis, "digital_rate": rate, "vbus_dio": vbus_dio,
               "capture_ok": cap.ok, "problems": cap.problems(), "setup": [list(m) for m in setup_actions(cfg, step)],
               "before": before, "after": after, "clock": info, **res, "fault_record": record,
               "queries": runner.queries, "errors": runner.errors, "observations": {}}
    commands.write_json(out / "summary.json", summary)
    for r in rows:
        if r["kind"] != "osc_tx" or not r["detail"].startswith(("/getBrakeStatus",)):
            t = "-" if r["t_ms"] is None else f"{r['t_ms']:10.1f}"
            print(f"{t:>10}  {r['source']:4}  {r['kind']:8}  {r['detail']}")
    for s in res["stops"]:
        print(f"stop at {s['stop_s']:.3f} s: {s['kind']}; early warning {s['early_warning_ms']} ms, loop resumes "
              f"{s['loop_resumes_ms']} ms, brakes {s['brake_changes_ms']}, resetCause {s.get('resetCause')}")
    for s in res["heartbeat_stops"] or []:
        print(f"no reply from {s['last_reply_s']:.3f} s for {s['next_reply_ms']} ms: brakes {s['brake_changes_ms']}, "
              f"/booted {s.get('booted_ms')} ms resetCause {s.get('resetCause')}")
    if probes and not res["stops"]:
        print("no MCU stop (no P0 gap of 0.2 s or more)")
    if record:
        print(record)
    if runner.errors:
        print(f"errors: {runner.errors}")
    print(f"results: {out}")
    print(f"enter the observations with: hil.py --config <toml> usb --note {out} key=value ...")
    return 0 if cap.ok else 1
