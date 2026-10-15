"""Tests of hil.py motor (tests with a real motor, record only), with synthetic data (no
hardware)."""

import importlib.util
import json
from argparse import Namespace

import numpy as np

from hil import analyze, config, motor
from hil.ad2 import Capture
from hil.osc import LogEntry

STEP400_EXAMPLE = config.HIL_DIR / "bench.step400.example.toml"
RATE = 1e5


def load_cli():
    spec = importlib.util.spec_from_file_location("hil_cli", config.HIL_DIR / "hil.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def motor_cfg(tmp_path, extra: str = ""):
    path = tmp_path / "bench.motor.toml"
    path.write_text('[board]\nmodel = "step400"\nid = 0\n\n[motor]\ntest_axis = 1\nmotor_connected = true\n'
                    + extra)
    return config.load(path)


# --- setup ------------------------------------------------------------------------------


def test_setup_actions(tmp_path):
    cfg = motor_cfg(tmp_path, "bemf_param = [1032, 9, 9, 9]\nkval = [16, 16, 16, 16]\n"
                              "speed_profile = [100, 100, 30]\nbrake_transition_ms = 100\n")
    msgs = motor.setup_actions(cfg, brake=True)
    assert ("/setBemfParam", 1, 1032, 9, 9, 9) in msgs
    assert ("/setKval", 1, 16, 16, 16, 16) in msgs
    assert ("/setSpeedProfile", 1, 100.0, 100.0, 30.0) in msgs
    assert ("/enableElectromagnetBrake", 1, 1) in msgs and ("/setBrakeTransitionDuration", 1, 100) in msgs
    assert ("/setPositionReportInterval", 1, 20) in msgs
    without = motor.setup_actions(cfg, brake=False)
    assert not any(m[0] == "/enableElectromagnetBrake" for m in without)


def test_setup_actions_defaults(tmp_path):
    msgs = motor.setup_actions(motor_cfg(tmp_path), brake=False)
    assert not any(m[0] in ("/setBemfParam", "/setKval", "/setSpeedProfile") for m in msgs)


# --- host time <-> AD2 time ---------------------------------------------------------------


def test_clock_fit_count():
    tx = np.array([0.1, 0.5, 1.2, 3.0])
    p1 = tx * 1.00001 + 0.002
    fit, t_ad2, method = motor.clock_fit(tx, p1)
    assert method == "count"
    assert np.allclose(t_ad2, p1)
    assert abs(fit.to_ad2(2.0) - (2.0 * 1.00001 + 0.002)) < 1e-6


def test_clock_fit_missing_pulses():
    # The board was restarting: two messages caused no P1 pulse; one extra pulse.
    tx = np.array([0.1, 0.5, 1.2, 1.3, 3.0, 4.0])
    p1 = np.array([0.1013, 0.5012, 2.2, 3.0011, 4.0012])
    fit, t_ad2, method = motor.clock_fit(tx, p1)
    assert method == "nearest"
    assert np.isnan(t_ad2[2]) and np.isnan(t_ad2[3])
    assert np.allclose(t_ad2[[0, 1, 4, 5]], [0.1013, 0.5012, 3.0011, 4.0012])
    assert abs(fit.to_ad2(2.0) - 2.0012) < 1e-3


def test_p0_gaps():
    t = np.concatenate((np.arange(0, 0.1, 50e-6), np.arange(0.3, 0.4, 50e-6)))
    edges = analyze.Edges(t[::2], t[1::2], 0, 0, 0.5)
    gaps = motor.p0_gaps(edges)
    assert len(gaps) == 1
    a, b = gaps[0]
    assert abs(a - 0.1) < 1e-3 and abs(b - 0.3) < 1e-9


# --- timeline -----------------------------------------------------------------------------


def synthetic_capture(cfg, p1_times, brake_high):
    """Digital capture: P1 pulses (20 us) at p1_times, brake of motor 1 high over
    brake_high (t0, t1), P0 toggling every 50 us except 1.5-1.8 s, 3.3 V high."""
    n = int(2.0 * RATE)
    t = np.arange(n) / RATE
    v = np.zeros(n, np.uint32)
    p0 = ((np.arange(n) // 5) % 2).astype(bool) & ~((t >= 1.5) & (t < 1.8))
    v |= p0.astype(np.uint32) << cfg.probe_dio[0]
    for tp in p1_times:
        v[(t >= tp) & (t < tp + 20e-6)] |= np.uint32(1 << cfg.probe_dio[1])
    a, b = brake_high
    v[(t >= a) & (t < b)] |= np.uint32(1 << int(cfg.ad2["dio"]["brake"][0]))
    v |= np.uint32(1 << cfg.v33_dio)
    v |= np.uint32((1 << 14) | (1 << 15))  # switches released (pulled up)
    return Capture(digital=analyze.segments_from_samples(v, RATE), host_trigger_ns=0, host_stop_ns=int(2e9))


def test_build_timeline(tmp_path):
    cfg = motor_cfg(tmp_path)
    tx = [LogEntry(int(0.2e9), "tx", "/activate", (1, 1)), LogEntry(int(1.0e9), "tx", "/activate", (1, 0))]
    rx = [LogEntry(int(0.205e9), "rx", "/HiZ", (1, 0)), LogEntry(int(0.5e9), "rx", "/position", (1, 0)),
          LogEntry(int(0.6e9), "rx", "/uvlo", (1, 0)), LogEntry(int(0.9e9), "rx", "/uvlo", (1, 0)),
          LogEntry(int(1.12e9), "rx", "/HiZ", (1, 1))]
    events = [{"t_ns": int(1.4e9), "kind": "serial", "detail": "R", "label": "R_1"}]
    cap = synthetic_capture(cfg, [0.2007, 1.0007], (0.3007, 1.0030))
    rows, info = motor.build_timeline(cfg, cap, tx, rx, events)
    assert info["method"] == "count" and info["matched"] == 2
    times = [r["t_ms"] for r in rows]
    assert times == sorted(times)
    kinds = [(r["kind"], r["detail"]) for r in rows]
    assert ("osc_tx", "/activate 1 1") in kinds and ("osc_rx", "/HiZ 1 1") in kinds
    assert not any(d.startswith("/position") for _, d in kinds)
    assert sum(d == "/uvlo 1 0" for _, d in kinds) == 1  # polled replies: changes only
    assert ("brake", "motor 1 released (J7 high)") in kinds and ("brake", "motor 1 engaged (J7 low)") in kinds
    assert ("serial", "R [R_1]") in kinds
    mcu = [r for r in rows if r["kind"] == "mcu"]
    assert len(mcu) == 2 and abs(mcu[0]["t_ms"] - 1500) < 1 and abs(mcu[1]["t_ms"] - 1800) < 1
    delays = motor.brake_delays(rows, 1)
    assert [d["change"] for d in delays] == ["released", "engaged"]
    assert abs(delays[0]["delay_ms"] - 100.0) < 0.1 and delays[0]["after"] == "/activate 1 1"
    assert abs(delays[1]["delay_ms"] - 2.3) < 0.1 and delays[1]["after"] == "/activate 1 0"


# --- observations, command checks -----------------------------------------------------------


def test_add_note(tmp_path):
    (tmp_path / "summary.json").write_text(json.dumps({"step": "m1-12", "observations": {}}))
    assert motor.add_note(str(tmp_path), ["drop_mm=0", "clicks = two per cycle", "note=a=b"]) == 0
    obs = json.loads((tmp_path / "summary.json").read_text())["observations"]
    assert obs["drop_mm"] == "0" and obs["clicks"] == "two per cycle" and obs["note"] == "a=b"
    assert "_entered" in obs


def test_motor_needs_motor_connected(tmp_path, capsys):
    cfg = config.load(STEP400_EXAMPLE)  # motor_connected = false
    args = Namespace(note=None, step="m0", label=None, rate=None, reboot=False)
    assert motor.cmd_motor(cfg, args) == 2
    assert "motor_connected is false" in capsys.readouterr().out


def test_out1a_commands_refused_with_motor(tmp_path, capsys):
    cli = load_cli()
    path = tmp_path / "bench.motor.toml"
    path.write_text('[board]\nmodel = "step400"\nid = 0\n\n[motor]\nmotor_connected = true\n')
    for command in (["brake"], ["reboot", "--how", "R"], ["powerup"]):
        assert cli.main(["--config", str(path)] + command) == 2
        assert "motor_connected = true" in capsys.readouterr().out


def test_steps_well_formed():
    known = {"osc", "serial", "wait", "idle", "wait_osc", "press", "release", "operator", "reconnect", "query",
             "sine", "ocd_sweep", "wait_reply", "close_serial"}
    for step in motor.STEPS.values():
        kinds = [a.kind for a in step.actions]
        assert set(kinds) <= known, step.id
        for a in step.actions:
            if a.kind in ("press", "release"):
                assert a.address in ("home", "limit"), step.id
        # A restart (serial R, /resetDevice, W, operator restart) is followed by a wait for /booted,
        # and those steps record at 10 kS/s.
        restarts = [i for i, a in enumerate(step.actions)
                    if (a.kind == "serial" and a.address in ("R", "W")) or a.address == "/resetDevice"]
        for i in restarts:
            assert any(a.kind == "wait_osc" and a.address == "/booted" for a in step.actions[i:]), step.id
        if restarts:
            assert step.restart, step.id
        # Every operator request is announced and followed by something that waits for its effect.
        for i, a in enumerate(step.actions):
            if a.kind == "operator":
                assert i + 1 < len(step.actions) and step.actions[i + 1].kind in ("wait", "wait_osc", "wait_reply")
