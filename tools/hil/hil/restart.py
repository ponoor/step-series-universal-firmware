"""Brake outputs and driver state across a restart of the board (hil.py reboot).

The driver reset line (L6470 STBY/RST, A2) is not reachable on the board, so the driver
state is judged from:
- OUT1A of the test axis (Scope 1): energized, HiZ (about 1.8 V) or another level, shown
  as level segments of the 1 ms window medians.
- the driver registers read first in setup(), before the drivers are reset (HIL_PROBE
  builds, serial 'D'): CONFIG at its power-on default (0x2E88) means the drivers were
  reset (standby) while the MCU restarted.
- 'Y' (--how standby): the MCU floats the reset line for 300 ms (P2 high) without
  restarting, so OUT1A can be recorded at the full rate during a known standby.

The MCU reset is the last P0 (loop toggle) edge before the longest gap: P0 stops when
the MCU resets and resumes in loop() after setup() (a blocking command such as the
brake wait of /resetDevice makes a shorter gap before it).
"""

from __future__ import annotations

import re
import time

import numpy as np

from . import analyze, commands
from . import scenarios as sc
from .config import Config
from .diag import SerialDiag

CONFIG_DEFAULT = 0x2E88  # L6470 CONFIG after power-on / standby
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
    ref = t_reset if t_reset is not None else (t_cmd or 0.0)
    res["t_ref"] = "MCU reset (last P0 edge)" if t_reset is not None else "command"
    res["mcu_reset_s"] = t_reset
    res["command_s"] = t_cmd
    if t_cmd is not None and t_reset is not None:
        res["command_to_mcu_reset_ms"] = (t_reset - t_cmd) * 1e3
    released = analyze.Step.from_edges(analyze.bit_edges(seg, cfg.brake_dio))
    res["brake_edges_ms"] = [[round((t - ref) * 1e3, 2), bool(v)] for t, v in zip(released.times[1:], released.values[1:])]
    res["brake_released_at_start"] = bool(released.values[0])
    res["other_brakes_released_after_ref_ms"] = {}
    for motor, d in enumerate(cfg.ad2["dio"]["brake"], start=1):
        high = analyze.Step.from_edges(analyze.bit_edges(seg, int(d))).intervals(True)
        after = [(round((a - ref) * 1e3, 1), round((b - ref) * 1e3, 1)) for a, b in high if b > ref]
        if after:
            res["other_brakes_released_after_ref_ms"][motor] = after
    if t_reset is not None:
        nxt = _first_after(p0, t_reset + 0.02)
        res["p0_resumes_ms"] = (nxt - ref) * 1e3 if nxt is not None else None
        p4 = analyze.bit_edges(seg, probe[4]).rise
        nxt = _first_after(p4, t_reset)
        res["first_p4_after_reset_ms"] = (nxt - ref) * 1e3 if nxt is not None else None
    if how == "standby":
        # P2 also pulses every servo period; the standby window is the long pulse.
        win = long_pulse(analyze.bit_edges(seg, probe[2]), 0.05)
        res["standby_window_ms"] = [0.0, (win[1] - win[0]) * 1e3] if win else None
        ref = win[0] if win else ref
        res["brake_edges_ms"] = [[round((t - ref) * 1e3, 2), bool(v)] for t, v in zip(released.times[1:], released.values[1:])]
    ch = int(cfg.ad2["scope"]["out1a"])
    shift = res.get("scope_minus_digital_s", 0.0)
    res["out1a_segments"] = level_segments(cap.analog[ch], cap.analog_rate, t0=ref - shift)
    excited, _, _ = commands.excitation_step(cfg, cap, shift)
    res["excited_edges_ms"] = [[round((t - ref) * 1e3, 1), bool(v)] for t, v in zip(excited.times[1:], excited.values[1:])]
    return res


def cmd_reboot(cfg: Config, args) -> int:
    how = args.how
    out = commands.new_result_dir(f"reboot_{how}")
    b = cfg.section("brake")
    values = commands.scenario_values(cfg)
    standby = how == "standby"
    with commands.open_ad2(cfg) as ad2, SerialDiag(cfg.serial_port) as diag, commands.open_link(cfg) as link:
        diag.reboot()
        commands.connect_board(link)
        commands._run_steps(sc.COMMON_SETUP + (sc.osc("/activate", sc.AXIS, 1), sc.wait(0.5)), values, link, ad2, None)
        probe = cfg.probe_dio
        brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
        out1a = int(cfg.ad2["scope"]["out1a"])
        ref = int(cfg.ad2["scope"].get("p0_ref", 0)) if standby else 0
        # A restart re-enumerates the board's USB, which stalls the AD2 stream: 10 kS/s.
        rate = float(b["analog_rate"]) if standby else 1e4
        rec = ad2.start_recording(None, digital_rate=float(b["digital_rate"]),
                                  sensible_mask=commands.mask_of(brake_dios + probe),
                                  analog_channels=(out1a, ref) if ref else (out1a,), analog_rate=rate)
        time.sleep(0.3)
        t_host = time.monotonic_ns()
        if how == "R":
            diag.send("R")
        elif how == "resetDevice":
            link.send("/resetDevice")
        else:
            diag.send("Y")
        time.sleep(1.0 if standby else 4.0)
        cap = rec.stop()
        cap.save(out / f"reboot_{how}.npz")
        if not standby:
            diag.reconnect()
        time.sleep(0.5)
        boot_text = diag.command("D", until=r"HIL boot drivers (done|: not captured)")
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
    drivers = parse_boot_drivers(boot_text)
    res["boot_drivers"] = {m: {k: f"0x{v:04X}" for k, v in d.items()} for m, d in drivers.items()}
    res["drivers_reset_at_boot"] = {m: d["config"] == CONFIG_DEFAULT for m, d in drivers.items()}
    commands.write_json(out / "summary.json", res)
    print(f"how: {how}, capture ok: {cap.ok} {'; '.join(cap.problems())}")
    for k in ("command_to_mcu_reset_ms", "p0_resumes_ms", "first_p4_after_reset_ms", "standby_window_ms",
              "brake_released_at_start", "brake_edges_ms"):
        if k in res:
            print(f"{k}: {res[k]}")
    print("OUT1A segments (ms from the reference, median V):")
    for s in res.get("out1a_segments", []):
        if s["to_ms"] - s["from_ms"] >= 2 or abs(s["from_ms"]) < 50:
            print(f"  {s['from_ms']:9.1f} .. {s['to_ms']:9.1f}  {s['median_v']:6.3f}")
    print(f"excited edges (ms): {res.get('excited_edges_ms')}")
    print(f"boot driver registers: {res['boot_drivers']}")
    print(f"results: {out}")
    return 0
