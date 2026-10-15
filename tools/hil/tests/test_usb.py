"""Tests of hil.py usb (USB cable unplugged while running, record only), with synthetic data
(no hardware)."""

import numpy as np

from hil import analyze, config, usb
from hil.ad2 import Capture
from hil.osc import LogEntry

RATE = 1e4
# bench.step400.example.toml: J7 DIO 0-3, P0 DIO 8, P1 DIO 9, P2 DIO 10, 3.3 V DIO 4.
J7_1, V33, P0, P2, VBUS = 0, 4, 8, 10, 6


def usb_cfg(tmp_path, extra: str = ""):
    path = tmp_path / "bench.usb.toml"
    path.write_text('[board]\nmodel = "step400"\nid = 0\n\n[motor]\ntest_axis = 1\nmotor_connected = true\n'
                    + extra)
    return config.load(path)


def capture(duration: float, stop: float, resume: float | None, ew: tuple | None = None,
            j7_fall: float | None = None, vbus_fall: float | None = None) -> Capture:
    """P0 toggles every 0.2 ms (sampled at 10 kS/s: every other sample) until stop and again
    from resume; J7-1 released (high) until j7_fall; P2 high from ew[0] to ew[1]."""
    n = int(duration * RATE)
    t = np.arange(n) / RATE
    w = np.zeros(n, np.uint32)
    running = (t < stop) | ((t >= resume) if resume is not None else False)
    toggle = (np.arange(n) // 2) % 2 == 1
    w |= np.where(running & toggle, 1 << P0, 0).astype(np.uint32)
    w |= np.uint32(1 << V33)
    if j7_fall is not None:
        w |= np.where(t < j7_fall, 1 << J7_1, 0).astype(np.uint32)
    if ew is not None:
        w |= np.where((t >= ew[0]) & (t < ew[1]), 1 << P2, 0).astype(np.uint32)
    if vbus_fall is not None:
        w |= np.where(t < vbus_fall, 1 << VBUS, 0).astype(np.uint32)
    return Capture(digital=analyze.segments_from_samples(w, RATE), host_trigger_ns=0, host_stop_ns=int(duration * 1e9))


def test_steps_defined():
    assert list(usb.STEPS) == ["u1", "u2", "u3", "u4", "u5", "u6"]
    assert usb.STEPS["u2"].port_open and not usb.STEPS["u1"].port_open
    assert usb.STEPS["u5"].position_report_ms == 20 and usb.STEPS["u1"].position_report_ms == 0
    assert any(a.kind == "plug" for a in usb.STEPS["u6"].actions)
    for s in usb.STEPS.values():
        kinds = [a.kind for a in s.actions]
        assert kinds.count("unplug") == 1
        assert ("close_serial" in kinds) != s.port_open


def test_setup_position_report(tmp_path):
    cfg = usb_cfg(tmp_path, "position_report_ms = 20\n")
    msgs = usb.setup_actions(cfg, usb.STEPS["u1"])
    assert [m for m in msgs if m[0] == "/setPositionReportInterval"] == [("/setPositionReportInterval", 1, 0)]
    msgs = usb.setup_actions(cfg, usb.STEPS["u5"])
    assert [m for m in msgs if m[0] == "/setPositionReportInterval"] == [("/setPositionReportInterval", 1, 20)]
    assert ("/enableElectromagnetBrake", 1, 1) in msgs


def test_find_stops_no_ew():
    # Stop at 2.0 s, watchdog reset 4 s later, loop again at 6.5 s (boot).
    cap = capture(8.0, 2.0, 6.5)
    p0 = analyze.bit_edges(cap.digital, P0).all
    stops = usb.find_stops(p0, 8.0)
    assert len(stops) == 1
    a, b = stops[0]
    assert abs(a - 2.0) < 1e-3 and abs(b - 6.5) < 1e-3


def test_find_stops_ignores_single_edges():
    # The WDT_Handler P0 marker (one edge) inside the gap is not a resume.
    cap = capture(8.0, 2.0, 5.0)
    seg = cap.digital
    p0 = np.sort(np.append(analyze.bit_edges(seg, P0).all, 4.1))
    stops = usb.find_stops(p0, 8.0)
    assert len(stops) == 1 and abs(stops[0][1] - 5.0) < 1e-3


def test_find_stops_none_and_unfinished():
    assert usb.find_stops(analyze.bit_edges(capture(3.0, 10.0, None).digital, P0).all, 3.0) == []
    stops = usb.find_stops(analyze.bit_edges(capture(3.0, 1.0, None).digital, P0).all, 3.0)
    assert len(stops) == 1 and stops[0][1] is None


def test_analyze_stop_without_early_warning(tmp_path):
    cfg = usb_cfg(tmp_path)
    # Released until the pins float at the watchdog reset (about 4 s after the stop).
    cap = capture(8.0, 2.0, 6.5, j7_fall=5.95)
    rx = [LogEntry(int(1e9 * t), "rx", "/version", ("x",)) for t in np.arange(0.1, 2.0, 0.1)]
    rx += [LogEntry(int(1e9 * t), "rx", "/version", ("x",)) for t in np.arange(7.0, 8.0, 0.1)]
    rx.append(LogEntry(int(6.9e9), "rx", "/booted", (0, 4)))
    res = usb.analyze_usb(cfg, cap, rx, None, None)
    (s,) = res["stops"]
    assert s["early_warning_ms"] is None
    assert s["kind"] == "stop, no early warning, reset (watchdog period)"
    b = s["brake_changes_ms"][1]
    assert b["at_stop"] == "released" and b["changes"] == [[3950.0, "engaged"]]
    assert s["resetCause"] == 4 and abs(s["booted_ms"] - 4900) < 1
    assert abs(s["heartbeat_last_ms"] + 100) < 1
    assert res["heartbeat_gaps_ms"][0][0] == 1900.0


def test_analyze_stop_with_early_warning(tmp_path):
    cfg = usb_cfg(tmp_path, "\n[usb]\nvbus_dio = 6\n")
    # Early warning 2 s after the stop: P2 high and J7 engaged, reset 103 ms later.
    cap = capture(8.0, 2.0, 4.6, ew=(4.0, 4.103), j7_fall=4.0002, vbus_fall=1.95)
    res = usb.analyze_usb(cfg, cap, [], None, VBUS)
    (s,) = res["stops"]
    assert s["early_warning_ms"] == 2000.0
    assert s["early_warning_p2_high_ms"] == 103.0
    assert s["brake_changes_ms"][1]["changes"] == [[2000.2, "engaged"]]
    assert s["vbus_fall_before_stop_ms"] == -50.0
    assert s["kind"] == "stop, early warning, reset"


def test_early_warning_ignores_servo_pulses():
    p2 = analyze.Edges(np.array([1.0, 1.005, 2.0]), np.array([1.00002, 1.00502, 2.1]), 0, 0, 3.0)
    assert usb.early_warning(p2, 0.5, 3.0) == (2.0, 2.1)
    # Still high at the end (the reset).
    p2 = analyze.Edges(np.array([2.0]), np.array([]), 0, 1, 3.0)
    assert usb.early_warning(p2, 0.5, 3.0) == (2.0, None)


def test_heartbeat_gaps():
    t = np.array([0.0, 0.1, 0.2, 0.3, 4.5, 4.6])
    assert usb.heartbeat_gaps(t) == [(0.3, 4.5)]
    assert usb.heartbeat_gaps(np.array([0.0])) == []


def test_cli_usb_needs_step(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("hil_cli", config.HIL_DIR / "hil.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    args = mod.build_parser().parse_args(["usb", "--step", "u1"])
    assert args.command == "usb" and args.step == "u1"
    cfg_path = tmp_path / "b.toml"
    cfg_path.write_text('[board]\nmodel = "step400"\nid = 0\n\n[motor]\ntest_axis = 1\nmotor_connected = true\n')
    assert mod.main(["--config", str(cfg_path), "usb"]) == 2


def test_hardfault_marker(tmp_path):
    cfg = usb_cfg(tmp_path)
    cap = capture(8.0, 2.0, 6.5)
    n = int(8.0 * RATE)
    t = np.arange(n) / RATE
    words = np.zeros(n, np.uint32)
    seg = cap.digital
    # Rebuild the samples and add P2 toggling every 0.5 ms from the stop to the reset.
    for k in range(seg.start.size):
        end = seg.start[k + 1] if k + 1 < seg.start.size else n
        words[seg.start[k]:end] = seg.value[k]
    toggling = (t > 2.0) & (t < 6.0) & (((t * 2000).astype(int) % 2) == 1)
    words |= np.where(toggling, 1 << P2, 0).astype(np.uint32)
    cap.digital = analyze.segments_from_samples(words, RATE)
    (s,) = usb.analyze_usb(cfg, cap, [], None, None)["stops"]
    assert s["early_warning_ms"] is None and s["p2_edges_in_stop"] > 1000
    assert s["kind"].startswith("HardFault")


def test_no_probe_heartbeat_stops(tmp_path):
    cfg = usb_cfg(tmp_path)
    # main: a 62 ms watchdog, so a hang is a short reset; J7 engaged when the pins float.
    cap = capture(4.0, 10.0, None, j7_fall=2.05)
    rx = [LogEntry(int(1e9 * t), "rx", "/version", ("x",)) for t in np.arange(0.1, 2.01, 0.1)]
    rx += [LogEntry(int(1e9 * t), "rx", "/version", ("x",)) for t in np.arange(3.0, 4.0, 0.1)]
    rx.append(LogEntry(int(2.9e9), "rx", "/booted", (0,)))
    res = usb.analyze_usb(cfg, cap, rx, None, None, probes=False)
    assert res["stops"] == []
    (h,) = res["heartbeat_stops"]
    assert abs(h["last_reply_s"] - 2.0) < 1e-3 and abs(h["next_reply_ms"] - 1000) < 1
    assert h["brake_changes_ms"][1]["changes"] == [[50.0, "engaged"]]
    assert abs(h["booted_ms"] - 900) < 1 and h["resetCause"] is None


def test_cli_no_probe_flag():
    import importlib.util
    spec = importlib.util.spec_from_file_location("hil_cli", config.HIL_DIR / "hil.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.build_parser().parse_args(["usb", "--step", "u1", "--no-probe"]).no_probe
