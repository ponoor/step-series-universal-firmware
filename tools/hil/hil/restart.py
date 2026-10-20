"""Brake outputs and driver state across a restart of the board (hil.py reboot).

The driver reset line (L6470 STBY/RST, A2) is not reachable on the board, so the driver
state is judged from:
- OUT1A of the test axis (Scope 1): energized, HiZ (about 1.8 V) or another level, shown
  as level segments of the 1 ms window medians.
- the driver registers read first in setup(), before the drivers are reset (HIL_PROBE
  builds, serial 'D'): CONFIG at its power-on default (L6470 0x2E88, PowerSTEP01 0x2C88)
  means the drivers were reset (standby) while the MCU restarted.
- 'Y' (--how standby): the MCU floats the reset line for 300 ms (P2 high) without
  restarting, so OUT1A can be recorded at the full rate during a known standby.

The MCU reset is the last P0 (loop toggle) edge before the longest gap: P0 stops when
the MCU resets and resumes in loop() after setup() (a blocking command such as the
brake wait of /resetDevice makes a shorter gap before it). safeSystemReset() sets P0
high just before the reset in HIL_PROBE builds. On the STEP800, P0 (D1, PA10) is also the
UART TX pin, which the bootloader drives about 30 ms after the reset, so this time is
approximate (on the STEP400 P0 is D38, and D1 is the brake output EM_4).

Brake outputs during the boot: from the MCU reset to the first P0 edge of loop() the
firmware does not release a brake, so every brake output must stay low (engaged); a high
span there comes from the bootloader or the floating pins (`brakes_high_during_boot`).

The driver standby entry is judged on OUT1A: an energized driver going to standby
makes OUT1A jump to about 6 V and decay to about 0.13 V within 80 ms (bench, 12 V
motor supply, no motor). The entry is the first 1 ms window whose median exceeds 3 V
(energized: about 0 V, HiZ: about 1.8 V).
"""

from __future__ import annotations

import re
import time

import numpy as np

from . import analyze, commands
from . import scenarios as sc
from .config import Config
from .diag import SerialDiag

CONFIG_DEFAULT = 0x2E88  # L6470 CONFIG after power-on / standby (Config.board.config_default)
BOOT_LINE = re.compile(r"HIL boot driver (\d+): CONFIG 0x([0-9A-F]{4}) STATUS 0x([0-9A-F]{4}) now CONFIG 0x([0-9A-F]{4})")


def parse_boot_drivers(text: str) -> dict[int, dict]:
    """'D' output -> {motor: {config, status, config_now}}."""
    return {int(m.group(1)): {"config": int(m.group(2), 16), "status": int(m.group(3), 16),
                              "config_now": int(m.group(4), 16)} for m in BOOT_LINE.finditer(text)}


def mcu_reset_time(p0_edges: np.ndarray, gap: float = 0.2) -> float | None:
    """Last P0 edge before the longest gap, if it is at least `gap` seconds (else None)."""
    t = np.asarray(p0_edges, float)
    if t.size < 2:
        return None
    d = np.diff(t)
    k = int(np.argmax(d))
    return float(t[k]) if d[k] >= gap else None


STANDBY_JUMP_V = 3.0


def standby_entry(volts: np.ndarray, rate: float, after: float = 0.0, window: float = 1e-3,
                  level: float = STANDBY_JUMP_V) -> float | None:
    """Start (scope time) of the first window after `after` whose median exceeds level."""
    med = analyze.window_medians(np.asarray(volts, float), rate, window)
    k = np.flatnonzero((med > level) & (np.arange(med.size) * window >= after))
    return float(k[0] * window) if k.size else None


PWM_STOP_MAX_V = 0.15  # window max of OUT1A in standby (about 0 V)
PWM_ON_MIN_V = 0.3  # window max while energized off EL_POS 0 (PWM; 0.5 V mean at 10 kS/s)


def pwm_stop(volts: np.ndarray, rate: float, after: float = 0.0, window: float = 1e-3) -> float | None:
    """Start (scope time) of the first window after `after` whose max is below PWM_STOP_MAX_V
    after a window with a max of PWM_ON_MIN_V or more: OUT1A switching (energized off EL_POS 0)
    stops. The PowerSTEP01 output sits at about 0 V in standby and floats at the motor supply
    when HiZ, so its standby has no jump (STEP400). Needs the test axis off EL_POS 0 (--move),
    where OUT1A is held at 0 V also while energized."""
    v = np.asarray(volts, float)
    n = int(round(window * rate))
    k = v.size // n
    if k < 2:
        return None
    mx = v[:k * n].reshape(k, n).max(axis=1)
    for i in range(max(1, int(np.ceil(after / window))), k):
        if mx[i] < PWM_STOP_MAX_V and mx[i - 1] >= PWM_ON_MIN_V:
            return float(i * window)
    return None


def driver_standby(cfg: Config, volts: np.ndarray, rate: float, after: float = 0.0) -> float | None:
    """Driver standby entry on OUT1A: the jump of the L6470 (STEP800), the PWM stop of the
    PowerSTEP01 (STEP400)."""
    if cfg.model == "step400":
        return pwm_stop(volts, rate, after)
    return standby_entry(volts, rate, after)


def long_pulse(edges, min_width: float) -> tuple[float, float] | None:
    """First high pulse at least min_width seconds long (rise, fall)."""
    for r in edges.rise:
        f = edges.fall[edges.fall > r]
        if f.size and f[0] - r >= min_width:
            return float(r), float(f[0])
    return None


def level_segments(volts: np.ndarray, rate: float, window: float = 1e-3, tol: float = 0.15,
                   t0: float = 0.0) -> list[dict]:
    """Groups consecutive window medians that stay within tol volts of the segment's first
    median. Returns [{from_ms, to_ms, median_v}] relative to t0."""
    med = analyze.window_medians(np.asarray(volts, float), rate, window)
    out: list[dict] = []
    start = 0
    for i in range(1, med.size + 1):
        if i == med.size or abs(med[i] - med[start]) > tol:
            seg = med[start:i]
            out.append({"from_ms": round((start * window - t0) * 1e3, 1), "to_ms": round((i * window - t0) * 1e3, 1),
                        "median_v": round(float(np.median(seg)), 3)})
            start = i
    return out


def high_spans(step, t_from: float, t_to: float, ref: float) -> list[list[float]]:
    """High (released) spans of a brake output within [t_from, t_to], as [start ms from
    ref, length ms]."""
    return [[round((a - ref) * 1e3, 2), round((b - a) * 1e3, 2)]
            for a, b in analyze.intersect([(t_from, t_to)], step.intervals(True))]


def engage_lead(step, t_standby: float, ref: float) -> dict:
    """Last engage (fall) of a brake output before the drivers' standby entry: ms before the
    standby, and whether it stays engaged from there to the end."""
    t = np.asarray(step.times, float)
    v = np.asarray(step.values, bool)
    falls = t[1:][(~v[1:]) & (t[1:] <= t_standby)]
    engaged = not bool(step.at(t_standby))
    stays = not analyze.intersect([(t_standby, step.t_end)], step.intervals(True))
    return {"engaged_at_standby": engaged, "stays_engaged": stays,
            "engage_before_standby_ms": round((t_standby - falls[-1]) * 1e3, 2) if falls.size else None,
            "engage_ms_from_ref": round((falls[-1] - ref) * 1e3, 2) if falls.size else None}


def _last_fall_before(step, t: float) -> float | None:
    """Last engage (fall) of a brake output at or before t, None if it did not fall."""
    times = np.asarray(step.times, float)
    v = np.asarray(step.values, bool)
    falls = times[1:][(~v[1:]) & (times[1:] <= t)]
    return float(falls[-1]) if falls.size else None


def reset_before_loop(p0_edges: np.ndarray, gap: float = 0.2) -> float | None:
    """Last P0 edge before the last gap of at least `gap` seconds that loop() resumes after."""
    t = np.asarray(p0_edges, float)
    if t.size < 2:
        return None
    k = np.flatnonzero(np.diff(t) >= gap)
    for i in k[::-1]:
        if loop_resume(t, float(t[i])) is not None:
            return float(t[i])
    return None


def loop_resume(p0_edges: np.ndarray, t_reset: float, min_edges: int = 10, within: float = 5e-3) -> float | None:
    """First P0 edge after the reset that starts the loop toggling: followed by at least
    min_edges edges within `within` seconds (a loop pass takes about 50 us). Single edges
    of the bootloader (the STEP800 P0 is the UART TX pin) or of the floating pin are not."""
    t = np.asarray(p0_edges, float)
    t = t[t > t_reset]
    for i in range(t.size):
        if np.sum(t[i:] <= t[i] + within) >= min_edges:
            return float(t[i])
    return None


def _first_after(t: np.ndarray, after: float) -> float | None:
    t = np.asarray(t, float)
    sel = t > after
    return float(t[sel][0]) if np.any(sel) else None


def analyze_restart(cfg: Config, cap, how: str, t_cmd: float | None) -> dict:
    seg = cap.digital
    probe = cfg.probe_dio
    res: dict = {"how": how}
    p0 = analyze.bit_edges(seg, probe[0]).all
    t_reset = mcu_reset_time(p0) if how != "standby" else None
    if how != "standby":
        # The hang before the early warning (W) and the brake waits (X, and R / resetDevice
        # with a long transition time: safeSystemReset() does not toggle P0) are long P0
        # gaps too, merged with the boot if P0 is already high at the marker: the reset is
        # the last P0 edge (the HIL reset marker) before the gap that loop() resumes after.
        t_reset = reset_before_loop(p0) or t_reset
    ref = t_reset if t_reset is not None else (t_cmd or 0.0)
    ch = int(cfg.ad2["scope"]["out1a"])
    shift = res.get("scope_minus_digital_s", 0.0)
    t_sb = driver_standby(cfg, cap.analog[ch], cap.analog_rate)
    t_sb = t_sb + shift if t_sb is not None else None
    if how != "standby" and t_sb is not None:
        ref = t_sb
    res["t_ref"] = ("driver standby entry (" + ("OUT1A PWM stop" if cfg.model == "step400" else "OUT1A jump") + ")"
                    if how != "standby" and t_sb is not None
                    else "MCU reset (last P0 edge)" if t_reset is not None else "command")
    res["standby_entry_s"] = t_sb
    res["mcu_reset_s"] = t_reset
    if t_reset is not None and t_sb is not None:
        res["p0_reset_marker_to_standby_ms"] = (t_sb - t_reset) * 1e3
    res["command_s"] = t_cmd
    if t_cmd is not None and t_sb is not None:
        res["command_to_standby_ms"] = (t_sb - t_cmd) * 1e3
    released = analyze.Step.from_edges(analyze.bit_edges(seg, cfg.brake_dio))
    res["brake_edges_ms"] = [[round((t - ref) * 1e3, 2), bool(v)] for t, v in zip(released.times[1:], released.values[1:])]
    res["brake_released_at_start"] = bool(released.values[0])
    res["other_brakes_released_after_ref_ms"] = {}
    steps = {}
    for motor, d in enumerate(cfg.ad2["dio"]["brake"], start=1):
        steps[motor] = analyze.Step.from_edges(analyze.bit_edges(seg, int(d)))
        high = steps[motor].intervals(True)
        after = [(round((a - ref) * 1e3, 1), round((b - ref) * 1e3, 1)) for a, b in high if b > ref]
        if after:
            res["other_brakes_released_after_ref_ms"][motor] = after
    if how != "standby" and t_sb is not None:
        res["brake_engage_vs_standby"] = {m: engage_lead(st, t_sb, ref) for m, st in steps.items()}
    if how in ("W", "X") and t_reset is not None:
        # P2 goes high in the early warning / HardFault handler after the brakes are engaged
        # and stays high to the reset (the servo update pulses are microseconds): the
        # handler's brake wait is from that rise to the reset marker on P0.
        p2r = analyze.bit_edges(seg, probe[2]).rise
        before = p2r[p2r <= t_reset]
        if before.size:
            t_p2 = float(before[-1])
            res["handler_wait_ms"] = round((t_reset - t_p2) * 1e3, 2)
            res["engage_to_handler_ms"] = {m: round((t_p2 - fall) * 1e3, 3)
                                           for m, st in steps.items()
                                           for fall in [_last_fall_before(st, t_p2)] if fall is not None}
            if how == "W":
                # The hang: loop() stops toggling P0. The early warning comes the watchdog
                # early warning time after the last Watchdog.reset() (every 10 ms in loop()).
                last = p0[p0 < t_p2 - 1e-4]
                res["hang_to_early_warning_ms"] = round((t_p2 - float(last[-1])) * 1e3, 2) if last.size else None
    if t_reset is not None:
        nxt = _first_after(p0, t_reset + 0.02)
        res["p0_resumes_ms"] = (nxt - ref) * 1e3 if nxt is not None else None
        if cfg.probe(4) is not None:
            p4 = analyze.bit_edges(seg, probe[4]).rise
            nxt = _first_after(p4, t_reset)
            res["first_p4_after_reset_ms"] = (nxt - ref) * 1e3 if nxt is not None else None
        # From the reset to loop() (loop_resume(); the end of the capture if loop() does not
        # resume): no brake output may be high.
        t_loop = loop_resume(p0, t_reset)
        t_loop = t_loop if t_loop is not None else seg.duration
        res["loop_resumes_ms"] = round((t_loop - ref) * 1e3, 2)
        res["boot_window_ms"] = [round((t_reset - ref) * 1e3, 2), round((t_loop - ref) * 1e3, 2)]
        # A brake released up to the reset falls when its pin floats (the Brake Board
        # pull-down): that first span, under 1 ms from the reset, is the engage, not a release.
        high = {}
        for m, st in steps.items():
            sp = high_spans(st, t_reset, t_loop, ref)
            if sp and abs(sp[0][0] - round((t_reset - ref) * 1e3, 2)) <= 0.01 and sp[0][1] < 1.0:
                res.setdefault("engage_at_reset_ms", {})[m] = sp[0][1]
                sp = sp[1:]
            if sp:
                high[m] = sp
        res["brakes_high_during_boot"] = high
        res["p0_edges_during_boot_ms"] = [round((t - ref) * 1e3, 2) for t in p0 if t_reset < t < t_loop]
    if how == "standby":
        # P2 also pulses every servo period; the standby window is the long pulse.
        win = long_pulse(analyze.bit_edges(seg, probe[2]), 0.05)
        res["standby_window_ms"] = [0.0, (win[1] - win[0]) * 1e3] if win else None
        ref = win[0] if win else ref
        res["brake_edges_ms"] = [[round((t - ref) * 1e3, 2), bool(v)] for t, v in zip(released.times[1:], released.values[1:])]
    res["out1a_segments"] = level_segments(cap.analog[ch], cap.analog_rate, t0=ref - shift)
    excited, _, _ = commands.excitation_step(cfg, cap, shift)
    res["excited_edges_ms"] = [[round((t - ref) * 1e3, 1), bool(v)] for t, v in zip(excited.times[1:], excited.values[1:])]
    # OUT1A comparator edges per 10 ms from the reference: PWM (energized off EL_POS 0) or none.
    v = np.asarray(cap.analog[ch], float)
    e = analyze.comparator_edges(v, cap.analog_rate, float(cfg.section("excitation")["level_v"]),
                                 float(cfg.section("excitation")["hysteresis_v"])) + shift
    if t_reset is not None:
        bins = np.arange(t_reset - 0.2, min(seg.duration, t_reset + 1.5), 0.01)
        h = np.histogram(e, bins)[0]
        res["out1a_edges_per_10ms_from_reset"] = [[round((b - t_reset) * 1e3), int(n)] for b, n in zip(bins[:-1], h)]
        pwm = [b for b, n in zip(bins[:-1], h) if n >= 3 and b > t_reset]
        res["out1a_pwm_after_reset_until_ms"] = round((max(pwm) + 0.01 - t_reset) * 1e3) if pwm else None
    return res


# resetCause of /booted (src/resetCause.h) expected after each restart.
EXPECTED_RESET_CAUSE = {"R": 5, "resetDevice": 5, "W": 4, "button": 3, "X": 6}


def judge_restart(cfg: Config, how: str, res: dict, transition_ms: float, tolerance_ms: tuple = (-1.0, 6.0),
                  brake_enabled: bool = True) -> dict:
    """PASS / FAIL items of a restart (STEP400 S4-2; recorded the same way on the STEP800).
    engage: every brake output engaged before the drivers' standby and engaged from there
    on; for R / resetDevice the engage comes transition_ms (+tolerance) before the standby
    (safeSystemReset() waits the longest transition time). boot: no brake output high from
    the reset to loop(). drivers: CONFIG at the power-on default at boot. cause: resetCause."""
    items = {}
    lead = res.get("brake_engage_vs_standby")
    if how in ("W", "X"):
        # The handler waits transition_ms (brake enabled and released), or not at all.
        wait = res.get("handler_wait_ms")
        lo, hi = (transition_ms * 0.97 - 1.0, transition_ms * 1.03 + 2.0) if brake_enabled else (-1.0, 1.0)
        items["wait"] = {"ok": wait is not None and lo <= wait <= hi,
                         "detail": f"handler brake wait {wait} ms (P2 rise to the reset marker), expected "
                                   f"{lo:.1f}-{hi:.1f} ms"}
    if how != "standby" and brake_enabled:
        if lead is None:
            items["engage"] = {"ok": False, "detail": "no driver standby entry found"}
        else:
            bad = []
            for m, v in lead.items():
                if not (v["engaged_at_standby"] and v["stays_engaged"]):
                    bad.append(f"M{m} not engaged at the standby or released again")
                elif how in ("R", "resetDevice"):
                    ms = v["engage_before_standby_ms"]
                    if ms is None or not transition_ms + tolerance_ms[0] <= ms <= transition_ms + tolerance_ms[1]:
                        bad.append(f"M{m} engaged {ms} ms before the standby")
            items["engage"] = {"ok": not bad, "detail": "; ".join(bad) or "all engaged before the standby: " +
                               ", ".join(f"M{m} {v['engage_before_standby_ms']} ms" for m, v in lead.items())}
    if "brakes_high_during_boot" in res:
        high = res["brakes_high_during_boot"]
        items["boot"] = {"ok": not high, "detail": f"high from the reset to loop(): {high}" if high
                         else f"all low from the reset to loop() ({res['boot_window_ms']} ms)"}
    drivers = res.get("boot_drivers_raw", {})
    if how != "standby":
        default = cfg.board.config_default
        n = cfg.num_motors
        ok = len(drivers) == n and all(d["config"] == default for d in drivers.values())
        items["drivers"] = {"ok": ok, "detail": f"CONFIG at boot {[f'0x{d['config']:04X}' for d in drivers.values()]}, "
                                                f"power-on default 0x{default:04X}"}
    if how in EXPECTED_RESET_CAUSE:
        causes = res.get("booted_reset_cause", [])
        items["cause"] = {"ok": causes == [EXPECTED_RESET_CAUSE[how]],
                          "detail": f"/booted resetCause {causes}, expected {EXPECTED_RESET_CAUSE[how]}"}
    return items


def cmd_reboot(cfg: Config, args) -> int:
    how = args.how
    all_motors = getattr(args, "all_motors", False)
    brake_disabled = getattr(args, "brake_disabled", False)
    out = commands.new_result_dir(f"reboot_{how}" + ("_all" if all_motors else "") + ("_nobrake" if brake_disabled else "")
                                  + (f"_{args.transition_ms}ms" if getattr(args, "transition_ms", None) else ""))
    b = cfg.section("brake")
    values = commands.scenario_values(cfg)
    if getattr(args, "transition_ms", None):
        values["transition_ms"] = int(args.transition_ms)
    transition_ms = float(values["transition_ms"])
    # The test axis, or every motor (brake enabled on all by COMMON_SETUP), released: the
    # brake releases transition_ms after /activate 1.
    activate = (sc.osc("/activate", sc.ALL if all_motors else sc.AXIS, 1), sc.wait(0.5 + transition_ms / 1000.0))
    standby = how == "standby"
    move = getattr(args, "move", 0) or 0
    if move:
        # Off EL_POS 0, where OUT1A is held at 0 V also while energized: with phase A
        # switching, energized (PWM) and standby (no PWM) can be told apart.
        activate += (sc.osc("/move", sc.AXIS, move), sc.wait(0.5))
    with commands.open_ad2(cfg) as ad2, SerialDiag(cfg.serial_port) as diag, commands.open_link(cfg) as link:
        diag.reboot()
        commands.connect_board(link)
        setup = sc.COMMON_SETUP
        if brake_disabled:
            # Brake disabled on every motor (the outputs stay engaged): /activate 1 only energizes.
            setup = (sc.osc("/activate", sc.ALL, 0), sc.wait(0.3), sc.osc("/enableElectromagnetBrake", sc.ALL, 0),
                     sc.osc("/setKval", sc.AXIS, sc.KVAL, sc.KVAL, sc.KVAL, sc.KVAL), sc.wait(0.3))
        commands._run_steps(setup + activate, values, link, ad2, None)
        probe = cfg.probe_dio
        brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
        out1a = int(cfg.ad2["scope"]["out1a"])
        ref = int(cfg.ad2["scope"].get("p0_ref", 0)) if standby else 0
        # A restart re-enumerates the board's USB, which stalls the AD2 stream: 10 kS/s.
        rate = float(b["analog_rate"]) if standby else 1e4
        if getattr(args, "analog_rate", None):
            rate = float(args.analog_rate)
        rec = ad2.start_recording(None, digital_rate=float(b["digital_rate"]),
                                  sensible_mask=commands.mask_of(brake_dios + probe),
                                  analog_channels=(out1a, ref) if ref else (out1a,), analog_rate=rate)
        time.sleep(0.3)
        t_host = time.monotonic_ns()
        if how == "R":
            diag.send("R")
        elif how == "resetDevice":
            link.send("/resetDevice")
        elif how == "W":
            diag.send("W")  # hang: early warning after WATCHDOG_TIMEOUT_MS, then the brake wait
        elif how == "X":
            diag.send("X")  # HardFault: brakes, brake wait, reset
        elif how == "button":
            print("OPERATOR: press the reset button of the board once (waiting up to 10 minutes)", flush=True)
            if link.wait_for("/booted", 600.0, t_host) is None:
                print("no /booted within 10 minutes")
        else:
            diag.send("Y")
        time.sleep(1.0 if standby else 8.0 if how == "W" else 1.5 if how == "button" else 4.0)
        cap = rec.stop()
        cap.save(out / f"reboot_{how}.npz")
        if not standby:
            diag.reconnect()
        time.sleep(0.5)
        boot_text = diag.command("D", until=r"HIL boot drivers (done|: not captured)")
        if not standby:
            link.wait_for("/booted", 5.0, t_host)  # sent once the link is up (BOOT_MSG_WAIT_TIME)
    t_cmd = None
    if how == "resetDevice":
        p1 = analyze.bit_edges(cap.digital, probe[1]).rise
        t_cmd = float(p1[0]) if p1.size else None
    res = {"capture_ok": cap.ok, "problems": cap.problems()}
    if standby and ref:
        shift, align = commands.scope_offset(cfg, cap)
        res["scope_alignment"] = align
        res["scope_minus_digital_s"] = shift
    res.update(analyze_restart(cfg, cap, how, t_cmd) if cap.ok else {})
    res["host_command_ns"] = t_host
    res["all_motors"] = all_motors
    drivers = parse_boot_drivers(boot_text)
    res["boot_drivers"] = {m: {k: f"0x{v:04X}" for k, v in d.items()} for m, d in drivers.items()}
    res["drivers_reset_at_boot"] = {m: d["config"] == cfg.board.config_default for m, d in drivers.items()}
    # /booted <myId> <resetCause> (src/utils.cpp sendBootMsg())
    res["booted_reset_cause"] = [int(e.args[1]) for e in link.entries("rx", t_host)
                                 if e.address == "/booted" and len(e.args) >= 2]
    res["boot_drivers_raw"] = drivers
    res["transition_ms"] = transition_ms
    res["brake_disabled"] = brake_disabled
    res["judged"] = judge_restart(cfg, how, res, transition_ms, brake_enabled=not brake_disabled)
    del res["boot_drivers_raw"]
    commands.write_json(out / "summary.json", res)
    print(f"how: {how}, capture ok: {cap.ok} {'; '.join(cap.problems())}")
    for k in ("handler_wait_ms", "hang_to_early_warning_ms", "engage_to_handler_ms", "t_ref", "command_to_standby_ms", "p0_reset_marker_to_standby_ms", "p0_resumes_ms", "first_p4_after_reset_ms", "standby_window_ms",
              "brake_released_at_start", "brake_edges_ms"):
        if k in res:
            print(f"{k}: {res[k]}")
    print("OUT1A segments (ms from the reference, median V):")
    for s in res.get("out1a_segments", []):
        if s["to_ms"] - s["from_ms"] >= 2 or abs(s["from_ms"]) < 50:
            print(f"  {s['from_ms']:9.1f} .. {s['to_ms']:9.1f}  {s['median_v']:6.3f}")
    print(f"excited edges (ms): {res.get('excited_edges_ms')}")
    print(f"boot driver registers: {res['boot_drivers']}")
    for k, v in res["judged"].items():
        print(f"{k:8s} {'PASS' if v['ok'] else 'FAIL'}  {v['detail']}")
    print(f"results: {out}")
    return 0
