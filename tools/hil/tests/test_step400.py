"""Tests of the STEP400 support of the bench (board model, switch outputs DIO 14 / 15,
board identity check, analyses without P3-P6), with synthetic data (no hardware). The
STEP800 defaults are checked to stay as they were."""

from pathlib import Path

import numpy as np
import pytest

from hil import ad2 as ad2mod
from hil import analyze, commands, config, restart
from hil import scenarios as sc
from hil.ad2 import AD2, Capture

STEP400_EXAMPLE = config.HIL_DIR / "bench.step400.example.toml"
RATE = 1e4


def step400_cfg():
    return config.load(STEP400_EXAMPLE)


def step800_cfg():
    return config.load(config.EXAMPLE_CONFIG)


# --- config -----------------------------------------------------------------------------


def test_step800_defaults_unchanged():
    cfg = step800_cfg()
    assert cfg.model == "step800" and cfg.num_motors == 8
    assert cfg.probe_dio == list(range(8, 15)) and cfg.probe_dios == list(range(8, 15))
    assert cfg.switch_dios == {"home": 15} and cfg.v33_dio is None
    assert cfg.board.config_default == 0x2E88
    assert commands.EXPECTED_PROBE_MAP == cfg.board.probe_map


def test_step400_example():
    cfg = step400_cfg()
    assert cfg.model == "step400" and cfg.num_motors == 4
    assert cfg.probe_dios == [8, 9, 10] and cfg.probe(3) is None and cfg.probe(2) == 10
    assert cfg.switch_dios == {"home": 15, "limit": 14} and cfg.v33_dio == 4
    assert cfg.brake_dio == 0 and cfg.board.config_default == 0x2C88
    assert cfg.board.probe_map == {0: "D38(PA13)", 1: "SCL(PA23)", 2: "SDA(PA22)"}


def test_step400_local_file_takes_step400_defaults(tmp_path: Path):
    p = tmp_path / "bench.step400.toml"
    p.write_text('[board]\nmodel = "step400"\nid = 2\n[motor]\ntest_axis = 3\n')
    cfg = config.load(p)
    assert cfg.num_motors == 4 and cfg.ad2["dio"]["brake"] == [0, 1, 2, 3] and cfg.brake_dio == 2
    assert cfg.board_ip == "10.0.0.102" and cfg.switch_dios["limit"] == 14
    assert cfg.section("brake")["transition_ms"] == 100  # from bench.example.toml


def test_config_path_relative_to_bench_dir(tmp_path: Path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert config.load("bench.step400.example.toml").model == "step400"


@pytest.mark.parametrize("text", [
    '[board]\nmodel = "step400"\n[motor]\ntest_axis = 5\n',  # 4 motors
    '[board]\nmodel = "step400"\n[ad2.dio]\nbrake = [0, 1, 2, 3, 5, 6, 7, 11]\n',
    '[board]\nmodel = "step400"\n[ad2.dio]\nlimit_sw = 13\n',  # only 14
    '[board]\nmodel = "step400"\n[ad2.dio]\nhome_sw = 14\nlimit_sw = -1\n',  # only 15
    '[board]\nmodel = "step400"\n[ad2.dio]\nprobes = [8, 9, 10, 11, -1, -1, -1]\n',  # no P3 pin
    '[board]\nmodel = "step400"\n[ad2.dio]\nprobes = [8, 9, -1, -1, -1, -1, -1]\n',  # P2 missing
    '[board]\nmodel = "step400"\n[ad2.dio]\nv33 = 3\n',  # shared with a brake output
    '[board]\nmodel = "step400"\n[ad2.dio]\nv33 = 14\n',  # the limit switch output
    '[board]\nmodel = "step400"\n[ad2.dio]\nbrake = [0, 1, 2, 15]\n',  # the home switch output
    '[board]\nmodel = "step400"\n[ad2.dio]\nv33 = -1\n',  # scope.v33 0 needs dio.v33
    '[board]\nmodel = "step500"\n',
    '[ad2.dio]\nlimit_sw = 14\n',  # STEP800: DIO14 is P6
])
def test_config_rejects(tmp_path: Path, text):
    p = tmp_path / "bench.toml"
    p.write_text(text)
    with pytest.raises(config.ConfigError):
        config.load(p)


# --- AD2 outputs: DIO 14 / 15 only, low or high impedance ---------------------------------


class FakeApi:
    """Records the digital IO calls; the output value must stay 0."""

    def __init__(self):
        self.calls = []
        self.enable = 0
        self.output = 0
        self.applied = []  # (enable, output) at each configure

    def dwf_digital_io_output_set(self, h, v):
        self.calls.append(("output", v))
        self.output = v

    def dwf_digital_io_output_enable_set(self, h, m):
        self.calls.append(("enable", m))
        self.enable = m

    def dwf_digital_io_configure(self, h):
        self.applied.append((self.enable, self.output))


class FakeDevice:
    handle = 1
    closed = False

    def close(self):
        self.closed = True


def fake_ad2(monkeypatch, switches):
    api = FakeApi()
    monkeypatch.setattr(ad2mod, "_api", lambda: api)
    a = AD2(switches=switches)
    a._device = FakeDevice()
    return a, api


def test_switch_outputs_step400(monkeypatch):
    a, api = fake_ad2(monkeypatch, step400_cfg().switch_dios)
    a.press("limit")
    assert api.applied[-1] == (1 << 14, 0)
    a.press("home")
    assert api.applied[-1] == ((1 << 14) | (1 << 15), 0)
    a.release("limit")
    assert api.applied[-1] == (1 << 15, 0)
    a.release()
    assert api.applied[-1] == (0, 0)
    # Never a high output value, and never an enabled pin other than 14 / 15.
    assert all(v == 0 for k, v in api.calls if k == "output")
    assert all(m & ~((1 << 14) | (1 << 15)) == 0 for k, m in api.calls if k == "enable")


def test_switch_outputs_step800_home_only(monkeypatch):
    a, api = fake_ad2(monkeypatch, step800_cfg().switch_dios)
    a.press()
    assert api.applied[-1] == (1 << 15, 0)
    with pytest.raises(ValueError):
        a.press("limit")  # DIO14 is the probe P6 on the STEP800
    a.release()
    assert api.applied[-1] == (0, 0)


@pytest.mark.parametrize("switches", [{"home": 14}, {"limit": 15}, {"home": 15, "limit": 13}, {"other": 3}])
def test_switch_outputs_refuse_other_pins(switches):
    with pytest.raises(ValueError):
        AD2(switches=switches)


def test_set_outputs_refuses_other_mask(monkeypatch):
    a, api = fake_ad2(monkeypatch, {"home": 15})
    with pytest.raises(ValueError):
        a._set_outputs(1 << 3)
    assert api.calls == []


def test_switches_released_on_exception(monkeypatch):
    a, api = fake_ad2(monkeypatch, step400_cfg().switch_dios)
    dev = a._device
    monkeypatch.setattr(AD2, "open", lambda self: self)
    with pytest.raises(RuntimeError):
        with a:
            a.press("limit")
            a.press("home")
            raise RuntimeError("test")
    assert api.applied[-1] == (0, 0) and dev.closed


def test_recording_refused_with_v33_low(monkeypatch):
    a, api = fake_ad2(monkeypatch, step400_cfg().switch_dios)
    a.v33_dio = 4
    monkeypatch.setattr(AD2, "read_dio", lambda self: 0xFFFF & ~(1 << 4))
    with pytest.raises(ad2mod.V33Error):
        a.start_recording(1.0, digital_rate=1e6)


def test_open_ad2_passes_board_switches():
    a = commands.open_ad2(step400_cfg())
    assert a.switches == {"home": 15, "limit": 14} and a.v33_dio == 4
    a = commands.open_ad2(step800_cfg())
    assert a.switches == {"home": 15} and a.v33_dio is None


def test_scenario_switch_steps():
    assert sc.press().switch == "home" and sc.release("r", switch="limit").switch == "limit"


# --- board identity ---------------------------------------------------------------------

STATUS_400 = "============== Current Status ==============\nFirmware name : STEP400_R1_UNIVERSAL\n"
STATUS_800 = "Firmware name : STEP800_R1_UNIVERSAL\r\n"


def test_board_identity():
    cfg4, cfg8 = step400_cfg(), step800_cfg()
    r = commands.check_board_identity(cfg4, STATUS_400, dict(cfg4.board.probe_map))
    assert r["firmware_name"] == "STEP400_R1_UNIVERSAL"
    assert commands.check_board_identity(cfg8, STATUS_800, dict(cfg8.board.probe_map))
    with pytest.raises(commands.BoardMismatch):  # STEP800 config on a STEP400
        commands.check_board_identity(cfg8, STATUS_400, dict(cfg4.board.probe_map))
    with pytest.raises(commands.BoardMismatch):  # STEP400 config on a STEP800
        commands.check_board_identity(cfg4, STATUS_800, dict(cfg8.board.probe_map))
    with pytest.raises(commands.BoardMismatch):  # not a HIL_PROBE build
        commands.check_board_identity(cfg4, STATUS_400, None)
    with pytest.raises(commands.BoardMismatch):  # STEP400 firmware with the STEP800 probe table
        commands.check_board_identity(cfg4, STATUS_400, dict(cfg8.board.probe_map))
    with pytest.raises(commands.BoardMismatch):
        commands.check_board_identity(cfg4, "no status", dict(cfg4.board.probe_map))


# --- analyses without P3-P6 ---------------------------------------------------------------


def test_find_selftest_three_probes():
    pulses = {d: ([], []) for d in (8, 9, 10)}
    t = 0.1
    for ch, d in enumerate((8, 9, 10)):
        for _ in range(ch + 1):
            pulses[d][0].append(t)
            pulses[d][1].append(10e-6)
            t += 20e-6
        t += 1e-3
    pulses = {d: (np.array(r), np.array(w)) for d, (r, w) in pulses.items()}
    assert analyze.find_selftest(pulses, n_probes=3) == {0: 8, 1: 9, 2: 10}


def test_timing_series_step400():
    cfg = step400_cfg()
    n = int(0.1 * 1e6)
    s = np.zeros(n, np.uint32)
    s[::100] |= np.uint32(1 << 8)  # P0 toggles
    s[1000:1200] |= np.uint32(1 << 10)
    s[6000:6200] |= np.uint32(1 << 10)
    series = commands.timing_series(cfg, analyze.segments_from_samples(s, 1e6))
    assert set(series) == {"loop", "oscReceive", "updateServo", "servoInterval"}
    assert series["servoInterval"] == pytest.approx([5000.0])


def test_mask_of_skips_missing_probes():
    assert commands.mask_of([8, 9, 10, -1, -1]) == 0b111 << 8


def digital(cfg, duration, highs: dict[int, list[tuple[float, float]]], toggles: dict[int, list[float]] = ()):
    n = int(duration * RATE)
    s = np.zeros(n, np.uint32)
    for dio, spans in highs.items():
        for lo, hi in spans:
            s[int(lo * RATE):int(hi * RATE)] |= np.uint32(1 << dio)
    for dio, times in dict(toggles).items():
        level = 0
        last = 0
        for t in list(times) + [duration]:
            i = int(t * RATE)
            if level:
                s[last:i] |= np.uint32(1 << dio)
            level ^= 1
            last = i
    return analyze.segments_from_samples(s, RATE)


def test_powerup_dio_pass_and_fail():
    cfg = step400_cfg()
    p0_toggles = list(0.3 + np.arange(200) * 1e-3)
    ok = Capture(digital=digital(cfg, 1.0, {4: [(0.01, 1.0)]}, {8: p0_toggles}))
    r = commands.analyze_powerup_dio(cfg, ok)
    assert r["ok"] and r["brake_high"] == {} and r["first_p0_ms"] == pytest.approx(290.0, abs=0.2)
    # EM_4 (motor 4, DIO 3) high for 2 ms, 30 ms after the 3.3 V rise.
    bad = Capture(digital=digital(cfg, 1.0, {4: [(0.01, 1.0)], 3: [(0.04, 0.042)]}, {8: p0_toggles}))
    r = commands.analyze_powerup_dio(cfg, bad)
    assert not r["ok"] and list(r["brake_high"]) == [4]
    assert r["brake_high"][4][0] == pytest.approx([30.0, 2.0], abs=0.2)


def test_loop_resume_skips_single_edges():
    t_reset = 1.0
    edges = np.concatenate(([1.0 + 1e-4, 1.03, 1.0302], 1.25 + np.arange(100) * 55e-6))
    assert restart.loop_resume(edges, t_reset) == pytest.approx(1.25)
    assert restart.loop_resume(np.array([1.03]), t_reset) is None


def test_high_spans_and_engage_lead():
    seg = digital(None, 2.0, {0: [(0.2, 0.9)], 3: [(0.2, 0.9), (1.05, 1.06)]})
    st0 = analyze.Step.from_edges(analyze.bit_edges(seg, 0))
    st3 = analyze.Step.from_edges(analyze.bit_edges(seg, 3))
    assert restart.high_spans(st0, 1.0, 1.5, 1.0) == []
    assert restart.high_spans(st3, 1.0, 1.5, 1.0) == [[50.0, 10.0]]
    lead = restart.engage_lead(st0, 1.0, 1.0)
    assert lead["engaged_at_standby"] and lead["stays_engaged"] and lead["engage_before_standby_ms"] == 100.0
    assert not restart.engage_lead(st3, 1.0, 1.0)["stays_engaged"]


def test_judge_restart():
    cfg = step400_cfg()
    lead = {m: {"engaged_at_standby": True, "stays_engaged": True, "engage_before_standby_ms": 101.0,
                "engage_ms_from_ref": -101.0} for m in range(1, 5)}
    drivers = {m: {"config": 0x2C88, "status": 0, "config_now": 0x2C8B} for m in range(1, 5)}
    res = {"brake_engage_vs_standby": lead, "brakes_high_during_boot": {}, "boot_window_ms": [-60.0, 300.0],
           "boot_drivers_raw": drivers, "booted_reset_cause": [5]}
    j = restart.judge_restart(cfg, "R", res, 100.0)
    assert all(v["ok"] for v in j.values()) and set(j) == {"engage", "boot", "drivers", "cause"}
    res2 = dict(res, brakes_high_during_boot={4: [[30.0, 2.0]]}, booted_reset_cause=[4],
                boot_drivers_raw={**drivers, 2: {"config": 0x2E88, "status": 0, "config_now": 0}})
    j = restart.judge_restart(cfg, "R", res2, 100.0)
    assert not j["boot"]["ok"] and not j["cause"]["ok"] and not j["drivers"]["ok"] and j["engage"]["ok"]
    # Watchdog: the brakes engage at the early warning, about 2 s before the standby.
    lead_w = {m: dict(v, engage_before_standby_ms=2000.0) for m, v in lead.items()}
    j = restart.judge_restart(cfg, "W", dict(res, brake_engage_vs_standby=lead_w, booted_reset_cause=[4]), 100.0)
    assert j["engage"]["ok"] and j["cause"]["ok"]
    j = restart.judge_restart(cfg, "R", dict(res, brake_engage_vs_standby=lead_w), 100.0)
    assert not j["engage"]["ok"]


def test_engage_before_reset_ignores_hiz_before_the_engage():
    # STEP400: HiZ (12.6 V) at the start, energized (0 V) from 0.5 s, the brake released from
    # 0.6 s and engaged at 1.0 s, the excitation lost (12.6 V) at 1.6 s.
    cfg = step400_cfg()
    t = np.arange(int(2.0 * RATE)) / RATE
    v = np.where((t >= 0.5) & (t < 1.6), 0.0, 12.6)
    cap = Capture(analog_rate=RATE, analog={1: v})
    released = analyze.Step(np.array([0.0, 0.6, 1.0]), np.array([False, True, False]), 2.0)
    r = commands.check_engage_before_reset(cfg, cap, 0.0, released, 1.0, "E")
    assert r.passed and "600.0 ms" in r.detail


def test_reset_before_loop_skips_the_hang():
    loop1 = np.arange(0, 0.5, 55e-6)
    hang_end = [2.5]  # early warning sets P0 high after a 2 s hang
    reset = [4.5, 4.5001]  # the pin floats at the watchdog reset
    loop2 = 5.2 + np.arange(200) * 55e-6
    p0 = np.concatenate((loop1, hang_end, reset, loop2))
    assert restart.reset_before_loop(p0) == pytest.approx(4.5001)
    assert restart.mcu_reset_time(p0) == pytest.approx(loop1[-1])  # the longest gap is the hang


def test_host_report_intervals():
    from hil.osc import LogEntry

    class Link:
        def __init__(self, log):
            self.log = log

        def entries(self, direction=None, t_from_ns=0, t_to_ns=None):
            return [e for e in self.log if e.direction == direction and t_from_ns <= e.t_ns <= t_to_ns]

    t0 = 10**12
    log = []
    for k in range(50):
        for m in (1, 2):
            jitter = 3_000_000 if (m == 2 and k == 20) else 0
            log.append(LogEntry(t0 + k * 20_000_000 + m * 100_000 + jitter, "rx", "/position", (m, k)))
    cap = Capture(host_trigger_ns=t0 - 1, host_stop_ns=t0 + 10**9)
    r = commands.host_report_intervals(Link(log), cap, {"position_report_ms": 20}, 2)
    assert r["per"][1]["off"] == 0 and r["per"][2]["off"] == 2 and r["intervals"] == 98
