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
    ids = set(scenario.invariants) | {c.id for c in scenario.checks}
    if scenario.expect == "FAIL":
        assert scenario.expected_failures and scenario.expected_failures <= ids
    values = {"axis": 1, "kval": 16, "max_speed": 200.0, "transition_ms": 100}
    for st in sc.COMMON_SETUP + scenario.extra_setup + scenario.steps:
        for arg in sc.resolve_args(st.args, values):
            assert isinstance(arg, (int, float)) and not isinstance(arg, bool)


def test_verdict():
    s_fail = sc.get("B2")
    assert sc.verdict(s_fail, {"INV-2", "INV-3"}) == ("FAIL", True)
    assert sc.verdict(s_fail, {"INV-2"}) == ("FAIL", False)
    assert sc.verdict(sc.get("B1"), set()) == ("PASS", True)
    assert sc.verdict(sc.get("B5"), {"INV-1"}) == ("FAIL", None)


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
    assert (Path(dirs[1]) / "compare_baseline" / "compare.csv").exists()


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
