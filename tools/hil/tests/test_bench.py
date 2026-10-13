"""Tests of the scenario analysis, timing series, scenarios, config and serial parsing
with synthetic captures (no hardware)."""

from pathlib import Path

import numpy as np
import pytest

from hil import analyze, commands, config, diag
from hil import scenarios as sc
from hil.ad2 import Capture
from hil.osc import LogEntry

DIG_RATE = 1e5
ANA_RATE = 2e5


def default_cfg():
    # The example config, independent of a local bench.toml.
    return config.load(config.EXAMPLE_CONFIG)


class FakeLink:
    def __init__(self, log):
        self.log = log

    def entries(self, direction=None, t_from_ns=0, t_to_ns=None):
        return [e for e in self.log if (direction is None or e.direction == direction)
                and e.t_ns >= t_from_ns and (t_to_ns is None or e.t_ns <= t_to_ns)]


def make_capture(cfg, duration, p1_times, brake_high, excited, host_trigger_ns=10**12, p5_times=()):
    """Digital: P1 pulses (200 us) and the brake output of the test axis.
    Analog: 15.6 kHz PWM on OUT1A while excited, 5 V otherwise."""
    n = int(duration * DIG_RATE)
    s = np.zeros(n, np.uint32)
    p1_bit = cfg.probe_dio[1]
    for t in p1_times:
        i = int(round(t * DIG_RATE))
        s[i:i + 20] |= np.uint32(1 << p1_bit)
    for t in p5_times:
        i = int(round(t * DIG_RATE))
        s[i:i + 25] |= np.uint32(1 << cfg.probe_dio[5])
    for lo, hi in brake_high:
        s[int(lo * DIG_RATE):int(hi * DIG_RATE)] |= np.uint32(1 << cfg.brake_dio)
    t = np.arange(int(duration * ANA_RATE)) / ANA_RATE
    v = np.full(t.size, 5.0)
    pwm = np.where((t * 15.6e3) % 1.0 < 0.3, 12.0, 0.0)
    for lo, hi in excited:
        sel = (t >= lo) & (t < hi)
        v[sel] = pwm[sel]
    cap = Capture(digital=analyze.segments_from_samples(s, DIG_RATE), analog_rate=ANA_RATE,
                  analog={int(cfg.ad2["scope"]["out1a"]): v}, host_trigger_ns=host_trigger_ns,
                  host_stop_ns=host_trigger_ns + int(duration * 1e9))
    return cap


def run_b8b(tmp_path, excited_until, drop_reply_at=None, board_skip=None):
    """B8b: /activate 1 at 0.3 s, /hardHiZ at 0.8 s (AD2 time = host time + 2 ms)."""
    cfg = default_cfg()
    scenario = sc.get("B8b")
    t0 = 10**12
    delay = 2e-3
    steps = [s for s in scenario.steps if s.kind == "osc"]
    host = [0.3, 0.8]
    log, tx = [], []
    for st, h in zip(steps, host):
        e = LogEntry(t0 + int(h * 1e9), "tx", st.address, (1, 1))
        tx.append(e)
        log.append((st, e.t_ns, e))
    # Position polling every 20 ms with constant position.
    entries = list(tx)
    for k in range(70):
        h = 0.05 + 0.02 * k
        entries.append(LogEntry(t0 + int(h * 1e9), "tx", "/getPosition", (1,)))
        if k != drop_reply_at:
            entries.append(LogEntry(t0 + int((h + 0.001) * 1e9), "rx", "/position", (1, 1234)))
    entries.sort(key=lambda e: e.t_ns)
    p1 = [(e.t_ns - t0) / 1e9 + delay for e in entries if e.direction == "tx"]
    a_t, h_t = 0.3 + delay, 0.8 + delay
    # The board answers each /getPosition (P5 1 ms after its P1), except a dropped one.
    polls = [(e.t_ns - t0) / 1e9 + delay for e in entries if e.address == "/getPosition"]
    p5 = [t + 0.001 for k, t in enumerate(polls) if k != board_skip]
    cap = make_capture(cfg, 1.5, p1, brake_high=[(a_t + 0.11, h_t + 0.012)], excited=[(a_t, excited_until)],
                       p5_times=p5)
    return commands.analyze_scenario(cfg, scenario, cap, FakeLink(entries), log, 0.1, 0.03, tmp_path)


def test_scenario_analysis_pass(tmp_path):
    res = run_b8b(tmp_path, excited_until=0.8 + 0.002 + 0.1)
    assert res["outcome"] == "PASS", res
    assert res["as_expected"] is True
    assert res["clock_fit"]["residual_max_ms"] < 0.1
    act = next(d for d in res["delays"] if d["step"] == "activate")
    assert act["brake_change_ms"] == pytest.approx(110, abs=0.1)
    assert (tmp_path / "B8b_timing.png").exists()
    assert (tmp_path / "B8b_events.csv").exists()


def test_scenario_analysis_detects_late_hiz(tmp_path):
    # Motor stays excited 400 ms after the brake engaged: INV-2 and the HiZ check fail.
    res = run_b8b(tmp_path, excited_until=0.8 + 0.002 + 0.4)
    assert res["outcome"] == "FAIL"
    assert res["failed"] == {"INV-2", "HIZ-AFTER-TRANSITION"}
    assert res["as_expected"] is False


def test_scenario_analysis_count_mismatch(tmp_path):
    cfg = default_cfg()
    scenario = sc.get("B8b")
    e = LogEntry(10**12 + int(0.3e9), "tx", "/activate", (1, 1))
    cap = make_capture(cfg, 1.0, [0.302, 0.5], [], [])
    res = commands.analyze_scenario(cfg, scenario, cap, FakeLink([e]), [(scenario.steps[0], e.t_ns, e)],
                                    0.1, 0.03, tmp_path)
    assert res["outcome"] == "ERROR" and "P1" in res["error"]


def test_timing_series():
    cfg = default_cfg()
    n = 100000
    s = np.zeros(n, np.uint32)
    d = cfg.probe_dio
    # P0 toggles every 40 samples (40 us at 1 MHz); P2 pulses of 300 us every 10 ms.
    level = (np.arange(n) // 40) % 2
    s |= (level.astype(np.uint32) << d[0])
    for k in range(9):
        s[1000 + 10000 * k:1300 + 10000 * k] |= np.uint32(1 << d[2])
    series = commands.timing_series(cfg, analyze.segments_from_samples(s, 1e6))
    assert np.allclose(series["loop"], 40.0)
    assert np.allclose(series["updateServo"], 300.0)
    assert np.allclose(series["servoInterval"], 10000.0)
    assert series["oscReceive"].size == 0


def test_servo_delays():
    # P3 every 10 ms; P2 alternately 1 ms and 3.5 ms after it.
    p3 = np.arange(10) * 0.010
    p2 = p3 + np.where(np.arange(10) % 2, 0.0035, 0.001)
    assert np.allclose(commands.servo_delays(p3, p2), np.where(np.arange(10) % 2, 0.0035, 0.001))
    # The last P3 at or before each P2 is used, also when P3 has extra rises.
    p3x = np.sort(np.concatenate((p3, p3 + 0.0005)))
    assert np.allclose(commands.servo_delays(p3x, p2), np.where(np.arange(10) % 2, 0.003, 0.0005))
    # P2 at the same time as P3 counts as 0.
    assert np.allclose(commands.servo_delays(p3, p3), 0.0)


def test_servo_delays_excludes():
    p3 = np.array([0.010, 0.020, 0.100])
    # before the first P3; 3.9 ms after (kept); 4.1 ms and 5 ms after (left out: a 5 ms
    # slot whose P3 is missing must not be paired with the P3 of the slot before); 2 ms after
    p2 = np.array([0.005, 0.0239, 0.0241, 0.025, 0.102])
    assert np.allclose(commands.servo_delays(p3, p2), [0.0039, 0.002])
    assert commands.servo_delays(np.array([]), p2).size == 0
    assert commands.servo_delays(p3, np.array([])).size == 0


def test_timing_series_servo_delay():
    cfg = default_cfg()
    n = 100000
    s = np.zeros(n, np.uint32)
    d = cfg.probe_dio
    # P3 (230 us) every 10 ms; P2 (300 us) starts 1000 / 3500 us after P3 alternately.
    for k in range(9):
        t3 = 1000 + 10000 * k
        t2 = t3 + (3500 if k % 2 else 1000)
        s[t3:t3 + 230] |= np.uint32(1 << d[3])
        s[t2:t2 + 300] |= np.uint32(1 << d[2])
    series = commands.timing_series(cfg, analyze.segments_from_samples(s, 1e6))
    assert np.allclose(series["servoDelay"], [1000.0 if k % 2 == 0 else 3500.0 for k in range(9)])
    assert np.allclose(series["servoInterval"], [12500.0 if k % 2 == 0 else 7500.0 for k in range(8)])
    assert np.allclose(series["checkStatus"], 230.0)


def test_timing_series_slot_interval():
    cfg = default_cfg()
    n = 100000
    s = np.zeros(n, np.uint32)
    d = cfg.probe_dio
    # 5 ms slots: P3 and P2 in every slot, P4 (getMyId) in every other slot.
    for k in range(19):
        t3 = 1000 + 5000 * k
        s[t3:t3 + 230] |= np.uint32(1 << d[3])
        s[t3 + 240:t3 + 860] |= np.uint32(1 << d[2])
        if k % 2 == 0:
            s[t3 + 2000:t3 + 2013] |= np.uint32(1 << d[4])
    series = commands.timing_series(cfg, analyze.segments_from_samples(s, 1e6))
    assert np.allclose(series["slotInterval"], 5000.0)
    assert np.allclose(series["servoInterval"], 5000.0)
    assert np.allclose(series["pollInterval"], 10000.0)
    assert np.allclose(series["servoDelay"], 240.0) and series["servoDelay"].size == 19


def report_capture(cfg, report_slots, reply_slots=(), n_slots=40, sends=8):
    """5 ms slots: P3, then P2 (servo), then in every other slot (housekeeping) the report
    sends (P5) if the slot is in report_slots, then P4. A P1 pulse with 8 P5 replies (an
    OSC packet such as /getPosition 255) follows the slots in reply_slots."""
    d = cfg.probe_dio
    s = np.zeros(n_slots * 5000 + 5000, np.uint32)
    for k in range(n_slots):
        t = 1000 + 5000 * k
        s[t:t + 230] |= np.uint32(1 << d[3])
        s[t + 240:t + 860] |= np.uint32(1 << d[2])
        u = t + 900
        if k % 2 == 0:
            if k in report_slots:
                for j in range(sends):
                    s[u:u + 245] |= np.uint32(1 << d[5])
                    u += 250
            s[u + 10:u + 23] |= np.uint32(1 << d[4])
            u += 30
        if k in reply_slots:
            s[u + 100:u + 2800] |= np.uint32(1 << d[1])
            for j in range(8):
                s[u + 300 + 300 * j:u + 545 + 300 * j] |= np.uint32(1 << d[5])
    return analyze.segments_from_samples(s, 1e6)


def test_report_bursts():
    cfg = default_cfg()
    # Reports every 20 ms (every other housekeeping slot), one late by a slot pair (30 ms).
    slots = [0, 4, 8, 12, 18, 22, 26]
    seg = report_capture(cfg, slots, reply_slots=(2, 3, 9))
    e = [analyze.bit_edges(seg, cfg.probe_dio[c]) for c in range(7)]
    starts, sizes = commands.report_bursts(e[2], e[4].rise, e[5].rise)
    assert sizes.tolist() == [8] * len(slots)  # the replies inside P1 are not counted
    assert np.allclose(starts * 1e6, [1000 + 5000 * k + 900 for k in slots])
    series = commands.timing_series(cfg, seg)
    assert np.allclose(series["reportInterval"], [20000, 20000, 20000, 30000, 20000, 20000])
    chk = commands.report_interval_check(series["reportInterval"], {"position_report_ms": 20})
    assert chk["off"] == 1 and chk["off_values_ms"] == [30.0] and chk["expected_ms"] == 20
    # /positionList: one send per burst
    seg = report_capture(cfg, [0, 4, 8], sends=1)
    e = [analyze.bit_edges(seg, cfg.probe_dio[c]) for c in range(7)]
    assert commands.report_bursts(e[2], e[4].rise, e[5].rise)[1].tolist() == [1, 1, 1]
    chk = commands.report_interval_check(np.array([20000.0, 21900.0, 22000.0, 18000.0]),
                                         {"position_report_ms": 0, "position_list_report_ms": 20})
    assert chk["off"] == 2  # 2 ms off counts
    assert commands.report_interval_check(np.array([]), {"position_report_ms": 0})["off"] is None
    assert commands.report_interval_check(np.array([10000.0]), None)["off"] is None


def test_cpu_usage():
    # 90 idle loops of 50 us and 10 loops of 550 us: 10 x 500 us busy of 9500 us.
    loop = np.array([50.0] * 90 + [550.0] * 10)
    cpu = commands.cpu_usage(loop)
    assert cpu["base_loop_us"] == pytest.approx(50.0)
    assert cpu["busy_fraction"] == pytest.approx(5000.0 / 10000.0)
    # Stage 1 figure: (mean - min) / mean of the loop periods.
    loop = np.array([52.0] * 99 + [2100.0])
    assert commands.cpu_usage(loop)["busy_fraction"] == pytest.approx((loop.mean() - 52.0) / loop.mean())
    assert commands.cpu_usage(np.array([]))["busy_fraction"] is None


class TimingArgs:
    def __init__(self, **kw):
        self.position_report_ms = kw.get("position_report_ms")
        self.position_list_report_ms = kw.get("position_list_report_ms")
        self.burst_hz = kw.get("burst_hz", 0.0)
        self.target_rate_hz = kw.get("target_rate_hz")


def test_timing_load():
    t = default_cfg().section("timing")
    load = commands.timing_load(t, TimingArgs())
    assert load["position_report_ms"] == int(t["position_report_ms"]) and load["position_list_report_ms"] == 0
    assert load["target_rate_hz"] == float(t["target_rate_hz"]) and load["burst_hz"] == 0.0
    assert load["burst"] is None
    load = commands.timing_load(t, TimingArgs(position_list_report_ms=20, burst_hz=10, target_rate_hz=200))
    assert load["position_report_ms"] == 0 and load["position_list_report_ms"] == 20
    assert load["burst_hz"] == 10.0 and load["burst"] == "/getPosition 255" and load["target_rate_hz"] == 200.0
    assert commands.timing_load(t, TimingArgs(position_report_ms=0))["position_report_ms"] == 0
    with pytest.raises(commands.TimingArgsError):
        commands.timing_load(t, TimingArgs(position_report_ms=20, position_list_report_ms=20))
    with pytest.raises(commands.TimingArgsError):
        commands.timing_load(t, TimingArgs(target_rate_hz=0))


def test_timing_cli_options():
    import importlib.util

    spec = importlib.util.spec_from_file_location("hil_cli", Path(__file__).parent.parent / "hil.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    a = mod.build_parser().parse_args(["timing", "--label", "x", "--position-list-report-ms", "20",
                                       "--burst-hz", "10", "--target-rate-hz", "200"])
    assert a.position_list_report_ms == 20 and a.burst_hz == 10.0 and a.target_rate_hz == 200.0
    assert a.position_report_ms is None


# --- scenarios --------------------------------------------------------------------------


@pytest.mark.parametrize("scenario", sc.SCENARIOS, ids=lambda s: s.id)
def test_scenario_definitions(scenario):
    labels = scenario.labels()
    assert len(labels) == len(set(labels))
    assert scenario.expect in ("PASS", "FAIL", "RECORD")
    for c in scenario.checks:
        assert c.from_label in labels and (c.to_label is None or c.to_label in labels)
        assert c.signal in ("brake", "excited")
    for ex in scenario.exempt:
        assert ex.from_label in labels and (ex.to_label is None or ex.to_label in labels)
        assert ex.span is None or ex.to_label is None
    for r in scenario.replies:
        assert r.from_label in labels and r.address.startswith("/")
    for m in scenario.motion:
        assert m.from_label in labels
        assert getattr(m, "to_label", None) is None or m.to_label in labels
    if any(isinstance(m, (sc.Motion, sc.EngageAfterStop)) for m in scenario.motion):
        assert scenario.poll_position
    ids = scenario.check_ids()
    assert len(ids) == len(scenario.invariants) + len(scenario.checks) + len(scenario.replies) + len(scenario.motion)
    if scenario.expect == "FAIL":
        assert scenario.expected_failures and scenario.expected_failures <= ids
    values = {"axis": 1, "kval": 16, "max_speed": 200.0, "transition_ms": 100}
    for st in sc.COMMON_SETUP + scenario.extra_setup + scenario.steps:
        for arg in sc.resolve_args(st.args, values):
            assert isinstance(arg, (int, float)) and not isinstance(arg, bool)
        if st.kind == "serial":
            assert st.label and st.edge in (sc.EXCITATION_OFF, sc.BRAKE_ENGAGE)
    if scenario.reboots:
        # Restarting scenarios run last.
        later = sc.SCENARIOS[sc.SCENARIOS.index(scenario):]
        assert all(s.reboots for s in later) and not scenario.poll_position


def test_verdict():
    s_fail = sc.Scenario("X", "known defect", "FAIL", steps=(), expected_failures=frozenset({"INV-2", "INV-3"}))
    assert sc.verdict(s_fail, {"INV-2", "INV-3"}) == ("FAIL", True)
    assert sc.verdict(s_fail, {"INV-2"}) == ("FAIL", False)
    assert sc.verdict(sc.get("B1"), set()) == ("PASS", True)
    assert sc.verdict(sc.get("B2"), {"INV-2"}) == ("FAIL", False)
    assert sc.verdict(sc.get("B5"), {"INV-1"}) == ("FAIL", None)


def test_resolve_delay():
    assert sc.resolve_delay(sc.LATENCY, 0.1, 0.03) == pytest.approx(0.03)
    assert sc.resolve_delay(sc.TRANSITION, 0.1, 0.03) == pytest.approx(0.13)
    assert sc.resolve_delay(sc.POLL, 0.1, 0.03) == pytest.approx(0.04)
    assert sc.resolve_delay(0.5, 0.1, 0.03) == 0.5


def test_check_reply():
    t0 = 10**12
    entries = [LogEntry(t0 + 5 * 10**6, "rx", "/error/command", ("BrakeEngaged", 1)),
               LogEntry(t0 + 400 * 10**6, "rx", "/error/command", ("CommandIgnored", 1))]
    link = FakeLink(entries)
    present = sc.Reply("R", "x", "/error/command", ("BrakeEngaged", sc.AXIS))
    args = sc.resolve_args(present.args, {"axis": 1})
    assert commands.check_reply(link, present, t0, args).passed
    assert not commands.check_reply(link, present, t0, ("BrakeEngaged", 2)).passed
    # Outside the 300 ms window
    late = sc.Reply("R", "x", "/error/command", ("CommandIgnored", 1))
    assert not commands.check_reply(link, late, t0, late.args).passed
    absent = sc.Reply("R", "x", "/error/command", ("CommandIgnored", 1), present=False)
    assert commands.check_reply(link, absent, t0, absent.args).passed
    assert not commands.check_reply(link, sc.Reply("R", "x", "/error/command", ("BrakeEngaged", 1), False),
                                    t0, ("BrakeEngaged", 1)).passed
    # Numeric arguments compare by value; a string never matches a number.
    assert commands.check_reply(FakeLink([LogEntry(t0 + 1, "rx", "/homingStatus", (1, 1))]),
                                sc.Reply("R", "x", "/homingStatus", (1, 1)), t0, (1, 1.0)).passed
    assert not commands.check_reply(FakeLink([LogEntry(t0 + 1, "rx", "/a", ("1",))]),
                                    sc.Reply("R", "x", "/a", (1,)), t0, (1,)).passed


def run_b15(tmp_path, engage_after):
    """B15: /activate 1 at 0.3 s (AD2 = host + 2 ms), brake released at 0.41 s, serial Z at
    host 0.9 s; the motor goes HiZ at 0.93 s (the serial latency is unknown to the host) and
    the brake engages engage_after seconds later."""
    cfg = default_cfg()
    scenario = sc.get("B15")
    t0 = 10**12
    act, z = scenario.steps[0], scenario.steps[2]
    e = LogEntry(t0 + int(0.3e9), "tx", "/activate", (1, 1))
    log = [(act, e.t_ns, e), (z, t0 + int(0.9e9), None)]
    entries = [e]
    pos_tx = []
    for k in range(60):
        h = 0.05 + 0.02 * k
        q = LogEntry(t0 + int(h * 1e9), "tx", "/getPosition", (1,))
        entries.append(q)
        pos_tx.append(q)
        entries.append(LogEntry(q.t_ns + 10**6, "rx", "/position", (1, 0)))
    entries.append(LogEntry(t0 + int(0.95e9), "rx", "/error/command", ("BrakeEngagedOnHiZ", 1)))
    entries.sort(key=lambda x: x.t_ns)
    p1 = [(x.t_ns - t0) / 1e9 + 0.002 for x in entries if x.direction == "tx"]
    t_hiz = 0.93
    cap = make_capture(cfg, 1.4, p1, brake_high=[(0.41, t_hiz + engage_after)], excited=[(0.302, t_hiz)])
    return commands.analyze_scenario(cfg, scenario, cap, FakeLink(entries), log, 0.1, 0.03, tmp_path)


def test_serial_step_timed_by_excitation(tmp_path):
    res = run_b15(tmp_path, engage_after=0.012)
    assert res["outcome"] == "PASS", res
    z = next(d for d in res["delays"] if d["step"] == "Z")
    assert z["t_s"] == pytest.approx(0.93, abs=0.002)
    assert "step_timing_notes" not in res


def test_serial_step_late_engage_fails(tmp_path):
    res = run_b15(tmp_path, engage_after=0.08)
    assert res["outcome"] == "FAIL"
    assert res["failed"] == {"INV-1", "ENGAGED-AFTER-Z"}


def run_s1(tmp_path, fixed: bool):
    """S1 (brake disabled), host times between the 20 ms polls: /enableServoMode 1 at 0.305 s,
    target at 0.605 s, serial Z at 0.8 s (the driver goes HiZ at AD2 0.83 s), /enableServoMode 1
    at 1.835 s, target 0 at 1.935 s, /enableServoMode 0 at 2.135 s; AD2 = host + 2 ms.
    fixed: the firmware cancels the servo mode on HiZ. Otherwise the servo update energizes
    the motor again 10 ms after Z, the motor keeps moving and no error is sent."""
    cfg = default_cfg()
    scenario = sc.get("S1")
    t0 = 10**12
    d = 0.002
    t_hiz = 0.83
    osc_steps = [st for st in scenario.steps if st.kind == "osc"]
    host = dict(zip([st.label for st in osc_steps], [0.305, 0.605, 1.835, 1.935, 2.135]))
    entries, log = [], []
    for st in scenario.steps:
        if st.kind == "osc":
            e = LogEntry(t0 + int(host[st.label] * 1e9), "tx", st.address, (1, 1))
            entries.append(e)
            log.append((st, e.t_ns, e))
        elif st.kind == "serial":
            log.append((st, t0 + int(0.8e9), None))
    t_target2 = host["target2"] + d

    def position(t):
        if t < 0.6 + d:
            return 0
        if t < t_hiz or (not fixed and t < t_target2):
            return int((t - 0.6) * 10000)
        if t < t_target2:
            return int((t_hiz - 0.6) * 10000)
        return 5000 - int((t - t_target2) * 10000)

    for k in range(122):
        h = 0.05 + 0.02 * k
        q = LogEntry(t0 + int(h * 1e9), "tx", "/getPosition", (1,))
        entries.append(q)
        entries.append(LogEntry(q.t_ns + 10**6, "rx", "/position", (1, position(h + d))))
    if fixed:
        entries.append(LogEntry(t0 + int(0.85e9), "rx", "/error/command", (sc.SERVO_CANCELLED, 1)))
    entries.sort(key=lambda x: x.t_ns)
    p1 = [(x.t_ns - t0) / 1e9 + d for x in entries if x.direction == "tx"]
    excited = [(0.302, t_hiz), (host["servo2"] + d, 2.6)] if fixed else [(0.302, t_hiz), (t_hiz + 0.01, 2.6)]
    cap = make_capture(cfg, 2.6, p1, brake_high=[], excited=excited)
    return commands.analyze_scenario(cfg, scenario, cap, FakeLink(entries), log, 0.1, 0.02, tmp_path)


def test_s1_fixed_passes(tmp_path):
    res = run_s1(tmp_path, fixed=True)
    assert res["outcome"] == "PASS", res
    z = next(d for d in res["delays"] if d["step"] == "Z")
    assert z["t_s"] == pytest.approx(0.83, abs=0.002)


def test_s1_before_fix_fails(tmp_path):
    res = run_s1(tmp_path, fixed=False)
    assert res["outcome"] == "FAIL"
    assert res["failed"] == {"NOT-EXCITED-AFTER-Z", "POSITION-UNCHANGED-AFTER-Z", "CANCELLED-REPLY"}


def test_s3_absent_reply_windows():
    # The absent ServoModeCancelledOnHiZ replies cover the HiZ that follows each command.
    s3 = sc.get("S3")
    windows = {r.from_label: r.window_s for r in s3.replies if r.address == "/error/command"
               and r.args[0] == sc.SERVO_CANCELLED}
    assert set(windows) == {"hardHiZ", "softHiZ", "activate0", "free"}
    assert all(not r.present for r in s3.replies)
    assert windows["softHiZ"] >= 0.4 and windows["activate0"] >= 0.7


# --- config -----------------------------------------------------------------------------


def test_config_defaults():
    cfg = default_cfg()
    assert cfg.board_ip == "10.0.0.101"
    assert cfg.board_in_port == 50000 and cfg.board_out_port == 50101
    assert cfg.brake_dio == 0 and cfg.probe_dio == list(range(8, 15))


def test_config_override(tmp_path: Path):
    p = tmp_path / "bench.toml"
    p.write_text('[board]\nid = 3\n[motor]\ntest_axis = 4\n')
    cfg = config.load(p)
    assert cfg.board_ip == "10.0.0.103" and cfg.board_out_port == 50103
    assert cfg.brake_dio == 3
    assert cfg.section("brake")["transition_ms"] == 100  # default kept


def test_firmware_address_for_id():
    # src/idOffset.cpp: the ID is not added beyond .254
    assert config.fw_ip_for_id(0) == "10.0.0.100" and config.fw_ip_for_id(154) == "10.0.0.254"
    assert config.fw_ip_for_id(155) == "10.0.0.100" and config.fw_ip_for_id(255) == "10.0.0.100"
    assert config.fw_out_port_for_id(155) == 50255
    assert config.fw_out_port_for_id(1, base=65535) == 65535


def test_config_rejects_other_output_pin(tmp_path: Path):
    p = tmp_path / "bench.toml"
    p.write_text('[ad2.dio]\nhome_sw = 14\n')
    with pytest.raises(config.ConfigError):
        config.load(p)


def test_config_rejects_shared_dio(tmp_path: Path):
    p = tmp_path / "bench.toml"
    p.write_text('[ad2.dio]\nbrake = [0, 1, 2, 3, 4, 5, 6, 8]\n')
    with pytest.raises(config.ConfigError):
        config.load(p)


# --- serial output parsing --------------------------------------------------------------

PERF_TEXT = """loop           max   1234 us  avg     45 us  n 1300000
oscReceive     max    800 us  avg    120 us  n 6000
updateServo    max    900 us  avg    350 us  n 6000
checkStatus    max    700 us  avg    400 us  n 6000
servoInterval  max  10500 us  avg  10000 us  n 5999
oscSlowPath    0
oscNotMatched  2
"""


def test_parse_perf():
    p = diag.parse_perf(PERF_TEXT)
    assert p["loop"] == {"max_us": 1234, "avg_us": 45, "n": 1300000}
    assert p["servoInterval"]["avg_us"] == 10000
    assert p["oscSlowPath"] == 0 and p["oscNotMatched"] == 2
    assert diag.SLOT_LATE_COUNTER not in p
    # The 5 ms slot firmware adds a counter line; the other lines read as before.
    p = diag.parse_perf(PERF_TEXT + "slotLate       3\n")
    assert p[diag.SLOT_LATE_COUNTER] == 3 and p["loop"]["n"] == 1300000 and p["oscNotMatched"] == 2


def test_parse_probe_map():
    text = "myId:1\nHIL probe: P0=D1(PA10) P1=A1(PB08) P2=D8(PA06) P3=D9(PA07) P4=D38(PA13) P5=SCL(PA23) P6=SDA(PA22)\n"
    assert diag.parse_probe_map(text) == commands.EXPECTED_PROBE_MAP
    assert diag.parse_probe_map("nothing") is None


# --- capture files, compare, AD2 runtime loading ------------------------------------------


def test_capture_roundtrip(tmp_path: Path):
    cfg = default_cfg()
    cap = make_capture(cfg, 0.2, [0.05, 0.1], [(0.02, 0.08)], [(0.0, 0.1)])
    cap.digital_lost = 3
    cap.save(tmp_path / "c.npz")
    back = Capture.load(tmp_path / "c.npz")
    assert np.array_equal(back.digital.start, cap.digital.start)
    assert np.array_equal(back.digital.value, cap.digital.value)
    assert back.digital.rate == DIG_RATE and back.analog_rate == ANA_RATE
    assert np.allclose(back.analog[1], cap.analog[1], atol=1e-5)
    assert back.digital_lost == 3 and not back.ok


def test_compare(tmp_path: Path, capsys):
    cfg = default_cfg()
    dirs = []
    for label, period in (("baseline", 50), ("head", 40)):
        n = 200000
        level = ((np.arange(n) // period) % 2).astype(np.uint32) << cfg.probe_dio[0]
        for k in range(19):  # P3 every 10 ms, P2 starting 2 * period us later
            t3 = 1000 + 10000 * k
            level[t3:t3 + 100] |= np.uint32(1 << cfg.probe_dio[3])
            level[t3 + 2 * period:t3 + 2 * period + 100] |= np.uint32(1 << cfg.probe_dio[2])
        seg = analyze.segments_from_samples(level, 1e6)
        d = tmp_path / label
        d.mkdir()
        Capture(digital=seg).save(d / "capture.npz")
        stats = {k: analyze.stats(v) for k, v in commands.timing_series(cfg, seg).items()}
        commands.write_json(d / "summary.json", {"label": label, "stats_us": stats})
        dirs.append(str(d))

    class Args:
        result_dirs = dirs

    assert commands.cmd_compare(cfg, Args) == 0
    out = capsys.readouterr().out
    assert "loop" in out and "-10.0" in out  # p50 of the loop: 50 -> 40 us
    assert "servoDelay" in out and "-20.0" in out  # servoDelay: 100 -> 80 us
    assert "slotInterval" in out and "cpu" in out
    assert "reportInterval" in out
    assert (Path(dirs[1]) / "compare_baseline" / "compare.csv").exists()


def test_compare_report_interval(tmp_path: Path, capsys):
    cfg = default_cfg()
    dirs = []
    for label, slots in (("after", [0, 4, 8, 14, 18, 22]), ("fix", [0, 4, 8, 12, 16, 20])):
        seg = report_capture(cfg, slots)
        d = tmp_path / label
        d.mkdir()
        Capture(digital=seg).save(d / "capture.npz")
        commands.write_json(d / "summary.json", {"label": label, "load": {"position_report_ms": 20}})
        dirs.append(str(d))

    class Args:
        result_dirs = dirs

    assert commands.cmd_compare(cfg, Args) == 0
    rows = (Path(dirs[1]) / "compare_after" / "compare.csv").read_text().splitlines()
    off = next(r for r in rows if r.startswith("reportInterval,off"))
    assert off.split(",")[2:5] == ["1", "0", "-1"]


def test_load_dwfpy_redirect():
    from hil import ad2

    if not any(Path(p).exists() for p in (ad2.DWFPY_LIBRARY, ad2.APP_BUNDLE_LIBRARY)):
        pytest.skip("WaveForms is not installed")
    dwfpy = ad2.load_dwfpy()
    assert dwfpy.Application().get_version().count(".") >= 2


def test_scope_offset_from_p0():
    cfg = default_cfg()
    rng = np.random.default_rng(7)
    # P0 toggles with a jittered loop period; the Scope started 150 us before Digital In.
    t_edges = np.cumsum(rng.uniform(40e-6, 70e-6, 4000))
    n = int(0.2 * DIG_RATE)
    level = np.zeros(n, np.uint32)
    idx = (t_edges * DIG_RATE).astype(int)
    idx = idx[idx < n]
    toggles = np.zeros(n, np.int64)
    toggles[idx] = 1
    level |= ((np.cumsum(toggles) % 2).astype(np.uint32) << cfg.probe_dio[0])
    t_a = np.arange(int(0.2 * ANA_RATE)) / ANA_RATE
    p0_a = (np.searchsorted(t_edges, t_a - 150e-6) % 2) * 3.3
    cap = Capture(digital=analyze.segments_from_samples(level, DIG_RATE), analog_rate=ANA_RATE,
                  analog={1: np.zeros(t_a.size), 2: p0_a.astype(float)})
    shift, info = commands.scope_offset(cfg, cap)
    assert info["method"] == "p0_ref"
    assert shift == pytest.approx(-150e-6, abs=10e-6)


def test_reply_loss(tmp_path):
    res = run_b8b(tmp_path, excited_until=0.8 + 0.002 + 0.1)
    loss = res["reply_loss"]
    assert loss["requests"] == 70 and loss["missing"] == 0 and loss["board_no_reply"] == 0
    # Lost on the way: the board sent it (P5), the host did not receive it.
    loss = run_b8b(tmp_path, excited_until=0.902, drop_reply_at=10)["reply_loss"]
    assert loss["missing"] == 1 and loss["board_no_reply"] == 0 and loss["lost_in_transit"] == 1
    # Not answered by the board (no P5): located at the request's P1 time.
    loss = run_b8b(tmp_path, excited_until=0.902, drop_reply_at=10, board_skip=10)["reply_loss"]
    assert loss["missing"] == 1 and loss["board_no_reply"] == 1 and loss["lost_in_transit"] == 0
    assert loss["board_no_reply_events"][0]["t_s"] == pytest.approx(0.05 + 0.02 * 10 + 0.002, abs=1e-4)


class FakeAD2:
    def __init__(self, volts):
        self.volts = volts

    def record(self, duration, analog_channels=(), analog_rate=None, **kw):
        return Capture(analog_rate=analog_rate, analog={analog_channels[0]: np.full(100, self.volts)})


def test_hiz_precheck():
    cfg = default_cfg()
    assert commands.hiz_precheck(cfg, FakeAD2(1.82))["ok"]
    bad = commands.hiz_precheck(cfg, FakeAD2(-0.02))  # e.g. Scope 1- on OUT1B, or energized
    assert not bad["ok"] and "aborted" in bad["detail"]


def test_scope_offset_detects_silent_gap():
    cfg = default_cfg()
    rng = np.random.default_rng(8)
    t_edges = np.cumsum(rng.uniform(40e-6, 70e-6, 40000))
    dur = 2.0
    n = int(dur * DIG_RATE)
    toggles = np.zeros(n, np.int64)
    idx = (t_edges * DIG_RATE).astype(int)
    toggles[idx[idx < n]] = 1
    level = ((np.cumsum(toggles) % 2).astype(np.uint32) << cfg.probe_dio[0])
    t_a = np.arange(int(dur * ANA_RATE)) / ANA_RATE
    p0_a = (np.searchsorted(t_edges, t_a) % 2) * 3.3
    # The Scope drops 8192 samples at 1.8 s without reporting it (90 % of the edges still match).
    cut = int(1.8 * ANA_RATE)
    p0_a = np.concatenate((p0_a[:cut], p0_a[cut + 8192:]))
    cap = Capture(digital=analyze.segments_from_samples(level, DIG_RATE), analog_rate=ANA_RATE,
                  analog={1: np.zeros(p0_a.size), 2: p0_a.astype(float)})
    with pytest.raises(commands.ScopeGapError):
        commands.scope_offset(cfg, cap)
