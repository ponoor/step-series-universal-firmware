"""Unit tests of hil.analyze with synthetic data (no hardware)."""

import numpy as np
import pytest

from hil import analyze as a

RATE = 1e6


def encode_pairs(samples: np.ndarray) -> np.ndarray:
    """Compressed record format of the AD2: value, span - 1, value, span - 1, ..."""
    seg = a.segments_from_samples(samples, RATE)
    spans = np.diff(np.append(seg.start, seg.n_samples))
    words = np.empty(2 * seg.start.size, np.uint32)
    words[0::2] = seg.value
    words[1::2] = spans - 1
    return words


def square(n: int, highs: list[tuple[int, int]], bit: int = 0) -> np.ndarray:
    s = np.zeros(n, np.uint32)
    for lo, hi in highs:
        s[lo:hi] |= np.uint32(1 << bit)
    return s


# --- segments ---------------------------------------------------------------------------


def test_segments_from_pairs_decodes_spans():
    seg = a.segments_from_pairs(np.array([5, 2, 7, 0, 5, 9], np.uint32), RATE)
    assert seg.start.tolist() == [0, 3, 4]
    assert seg.value.tolist() == [5, 7, 5]
    assert seg.n_samples == 14


def test_segments_from_pairs_merges_repeated_values():
    # A long stable period can arrive as several pairs with the same value.
    seg = a.segments_from_pairs(np.array([1, 99, 1, 99, 0, 0], np.uint32), RATE)
    assert seg.start.tolist() == [0, 200]
    assert seg.value.tolist() == [1, 0]
    assert seg.n_samples == 201


def test_segments_from_pairs_rejects_odd_length():
    with pytest.raises(ValueError):
        a.segments_from_pairs(np.array([1, 2, 3], np.uint32), RATE)


def test_pairs_and_samples_agree():
    rng = np.random.default_rng(1)
    samples = np.repeat(rng.integers(0, 1 << 15, 300).astype(np.uint32), rng.integers(1, 50, 300))
    s1 = a.segments_from_samples(samples, RATE)
    s2 = a.segments_from_pairs(encode_pairs(samples), RATE)
    assert np.array_equal(s1.start, s2.start)
    assert np.array_equal(s1.value, s2.value)
    assert s1.n_samples == s2.n_samples == samples.size


# --- edges and pulses -------------------------------------------------------------------


def test_bit_edges_times_and_levels():
    s = square(1000, [(100, 110), (500, 750)], bit=9) | square(1000, [(0, 300)], bit=2)
    seg = a.segments_from_samples(s, RATE)
    e = a.bit_edges(seg, 9)
    assert np.allclose(e.rise, [100e-6, 500e-6])
    assert np.allclose(e.fall, [110e-6, 750e-6])
    assert e.initial == 0 and e.final == 0
    e2 = a.bit_edges(seg, 2)
    assert e2.initial == 1 and e2.rise.size == 0 and np.allclose(e2.fall, [300e-6])


def test_pulse_widths_ignores_incomplete_pulses():
    # High at the start (no rise) and high at the end (no fall).
    s = square(1000, [(0, 50), (200, 230), (400, 420), (900, 1000)])
    rise, width = a.pulse_widths(a.bit_edges(a.segments_from_samples(s, RATE), 0))
    assert np.allclose(rise, [200e-6, 400e-6])
    assert np.allclose(width, [30e-6, 20e-6])


def test_pulse_groups():
    rise = np.array([0.0, 20e-6, 40e-6, 2e-3, 2.02e-3, 5e-3])
    assert a.pulse_groups(rise, 0.5e-3) == [3, 2, 1]
    assert a.pulse_groups(np.zeros(0), 1.0) == []


# --- statistics -------------------------------------------------------------------------


def test_stats():
    s = a.stats(np.arange(1, 1001), scale=2.0)
    assert s["count"] == 1000 and s["min"] == 2 and s["max"] == 2000
    assert s["p50"] == pytest.approx(1001.0)
    assert s["p99"] == pytest.approx(np.percentile(np.arange(1, 1001) * 2.0, 99))
    assert s["mean"] == pytest.approx(1001.0)
    empty = a.stats(np.zeros(0))
    assert empty["count"] == 0 and empty["max"] is None


# --- probe self-test and Scope / Digital alignment --------------------------------------


def selftest_pulses(dio_of_ch: dict[int, int], t0: float = 0.1, noise: bool = True):
    rng = np.random.default_rng(2)
    pulses = {d: ([], []) for d in range(8, 15)}
    t = t0
    for ch in range(7):
        for n in range(ch + 1):
            pulses[dio_of_ch[ch]][0].append(t)
            pulses[dio_of_ch[ch]][1].append(10e-6)
            t += 20e-6
        t += 1e-3
    if noise:
        # Loop toggle on P0 (wider pulses) and status / OSC pulses before and after.
        for k in range(200):
            pulses[dio_of_ch[0]][0].append(t0 - 0.05 + k * 60e-6)
            pulses[dio_of_ch[0]][1].append(30e-6)
        for k in range(20):
            pulses[dio_of_ch[3]][0].append(t + 0.01 * k)
            pulses[dio_of_ch[3]][1].append(float(rng.uniform(5e-6, 15e-6)))
    out = {}
    for d, (r, w) in pulses.items():
        order = np.argsort(r)
        out[d] = (np.array(r)[order], np.array(w)[order])
    return out


def test_find_selftest_expected_wiring():
    wiring = {ch: 8 + ch for ch in range(7)}
    assert a.find_selftest(selftest_pulses(wiring)) == wiring


def test_find_selftest_swapped_wiring():
    wiring = {0: 8, 1: 9, 2: 11, 3: 10, 4: 12, 5: 14, 6: 13}
    assert a.find_selftest(selftest_pulses(wiring)) == wiring


def test_find_selftest_missing_line():
    wiring = {ch: 8 + ch for ch in range(7)}
    pulses = selftest_pulses(wiring, noise=False)
    pulses[12] = (np.zeros(0), np.zeros(0))
    assert a.find_selftest(pulses) is None


def test_estimate_offset():
    rng = np.random.default_rng(3)
    t_dig = np.cumsum(rng.uniform(20e-6, 60e-6, 2000))
    t_ana = t_dig + 37e-6 + rng.normal(0, 0.3e-6, t_dig.size)
    d, n = a.estimate_offset(t_ana, t_dig, tolerance=3e-6, max_offset=5e-3)
    assert d == pytest.approx(37e-6, abs=0.5e-6)
    assert n > 1900


# --- command matching -------------------------------------------------------------------


def test_match_commands_fit():
    rng = np.random.default_rng(4)
    tx = np.sort(rng.uniform(0.2, 5.0, 100))
    p1 = tx * (1 + 20e-6) + 0.0123 + rng.uniform(0, 0.5e-3, tx.size)  # drift + network delay
    fit = a.match_commands(tx, p1)
    assert fit.slope == pytest.approx(1.0, abs=1e-3)
    assert fit.residual_max < 0.5e-3
    assert fit.to_ad2(tx[10]) == pytest.approx(p1[10], abs=0.5e-3)


def test_match_commands_count_mismatch():
    with pytest.raises(a.MatchError):
        a.match_commands(np.array([0.1, 0.2]), np.array([0.11, 0.21, 0.3]))


def test_match_commands_single():
    fit = a.match_commands(np.array([0.5]), np.array([0.51]))
    assert fit.to_ad2(0.5) == pytest.approx(0.51)


# --- excitation -------------------------------------------------------------------------


def pwm_then_hiz(rate=2e5, t_pwm=0.5, t_hiz=0.5, f=15.6e3, vs=12.0, seed=5):
    rng = np.random.default_rng(seed)
    t = np.arange(int((t_pwm + t_hiz) * rate)) / rate
    v = np.where((t * f) % 1.0 < 0.3, vs, 0.0)
    v[t >= t_pwm] = 5.0  # floating output
    return v + rng.normal(0, 0.3, t.size)


def test_comparator_hysteresis_rejects_noise():
    rng = np.random.default_rng(6)
    v = 6.0 + rng.normal(0, 0.4, 100000)  # noise around the level, inside the hysteresis
    v[:10] = 0.0
    assert a.comparator_edges(v, 2e5, 6.0, 4.0).size == 0


def test_excitation_detects_pwm_and_hiz():
    v = pwm_then_hiz()
    step, t_win, counts = a.excitation(v, 2e5, 6.0, 2.0, 1e-3, 4)
    assert counts[:499].min() >= 25  # 15.6 kHz: about 31 edges per ms
    assert counts[501:].max() == 0
    assert step.intervals(True) == [(0.0, pytest.approx(0.5, abs=1e-3))]
    assert step.at(0.25) and not step.at(0.75)


# --- step signals and invariants ---------------------------------------------------------


def step(changes: list[tuple[float, bool]], t_end: float) -> a.Step:
    return a.Step(np.array([t for t, _ in changes]), np.array([v for _, v in changes]), t_end)


def test_step_from_edges():
    e = a.Edges(np.array([0.2, 0.6]), np.array([0.4]), initial=0, final=1, duration=1.0)
    s = a.Step.from_edges(e)
    assert s.intervals(True) == [(0.2, 0.4), (0.6, 1.0)]
    assert s.intervals(False) == [(0.0, 0.2), (0.4, 0.6)]
    assert s.at(np.array([0.1, 0.3, 0.5, 0.9])).tolist() == [False, True, False, True]
    assert a.first_change_after(s, 0.3) == 0.4


def test_intersect_subtract():
    assert a.intersect([(0, 2), (3, 5)], [(1, 4)]) == [(1, 2), (3, 4)]
    assert a.subtract([(0, 10)], [(2, 3), (5, 6)]) == [(0, 2), (3, 5), (6, 10)]
    assert a.subtract([(0, 1)], [(0, 1)]) == []


# Normal sequence: excited at 1.0, released at 1.11, engaged at 3.0, HiZ at 3.1.
RELEASED = step([(0.0, False), (1.11, True), (3.0, False)], 5.0)
EXCITED = step([(0.0, False), (1.0, True), (3.1, False)], 5.0)


def test_inv1_pass_and_fail():
    assert a.check_inv1(RELEASED, EXCITED).passed
    hiz_early = step([(0.0, False), (1.0, True), (2.5, False)], 5.0)
    r = a.check_inv1(RELEASED, hiz_early)
    assert not r.passed and r.violations == [(2.5, 3.0)]


def test_inv1_exemption_and_resolution():
    hiz_early = step([(0.0, False), (1.0, True), (2.5, False)], 5.0)
    assert a.check_inv1(RELEASED, hiz_early, exempt=[(2.4, 3.05)]).passed
    glitch = step([(0.0, False), (1.0, True), (2.0, False), (2.0005, True), (3.1, False)], 5.0)
    assert a.check_inv1(RELEASED, glitch, resolution=1e-3).passed
    assert not a.check_inv1(RELEASED, glitch).passed


def test_inv2_allowance():
    # Overlaps 1.0-1.11 (110 ms) and 3.0-3.1 (100 ms), both at brake transitions.
    assert a.check_inv2(RELEASED, EXCITED, allowance=0.13).passed
    assert not a.check_inv2(RELEASED, EXCITED, allowance=0.105).passed


def test_inv2_overlap_away_from_transition():
    # Excited for 20 ms while engaged, 1 s after the brake engaged (home switch case).
    excited = step([(0.0, False), (1.0, True), (3.1, False), (4.0, True), (4.02, False)], 5.0)
    r = a.check_inv2(RELEASED, excited, allowance=0.13)
    assert not r.passed and r.violations == [(4.0, 4.02)]


def test_inv3():
    t = np.array([0.5, 0.52, 2.0, 2.02, 3.5, 3.52])
    assert a.check_inv3(RELEASED, t, np.array([0, 0, 10, 20, 20, 20])).passed
    r = a.check_inv3(RELEASED, t, np.array([0, 0, 10, 20, 20, 25]))
    assert not r.passed and r.violations == [(3.5, 3.52)]
    # A pair that overlaps an exempt range is not checked.
    assert a.check_inv3(RELEASED, t, np.array([0, 0, 10, 20, 20, 25]), exempt=[(3.51, 4.0)]).passed
    assert not a.check_inv3(RELEASED, t, np.array([0, 0, 10, 20, 20, 25]), exempt=[(3.53, 4.0)]).passed


def test_check_constant():
    assert a.check_constant(RELEASED, 1.2, 2.9, True, "X").passed
    r = a.check_constant(RELEASED, 1.2, 4.0, True, "X")
    assert not r.passed and r.violations == [(3.0, 4.0)]


def test_first_edge_to():
    sig = a.Step(np.array([0.0, 1.0, 2.0, 3.0]), np.array([True, False, True, False]), 4.0)
    assert a.first_edge_to(sig, 0.5, False) == 1.0
    assert a.first_edge_to(sig, 1.0, False) == 3.0
    assert a.first_edge_to(sig, 0.5, True) == 2.0
    assert a.first_edge_to(sig, 3.5, True) is None


def test_check_motion():
    t = np.arange(10) * 0.02
    moving = np.arange(10) * 128
    assert a.check_motion(t, moving, 0.0, 1.0, True, "M").passed
    assert not a.check_motion(t, moving, 0.0, 1.0, False, "M").passed
    still = np.full(10, 5)
    assert a.check_motion(t, still, 0.0, 1.0, False, "M").passed
    stops = np.array([0, 1, 2, 3, 3, 3, 3, 3, 3, 3])
    r = a.check_motion(t, stops, 0.0, 1.0, True, "M")
    assert not r.passed and len(r.violations) == 6
    assert a.check_motion(t, stops, 0.07, 1.0, False, "M").passed
    # Too few readings in the window
    assert not a.check_motion(t, still, 0.0, 0.03, False, "M").passed


def test_check_engage_after_stop():
    t = np.arange(20) * 0.02
    pos = np.concatenate((np.arange(10) * 100, np.full(10, 900)))  # last change between t[8] and t[9]
    def brake(t_eng):
        return a.Step(np.array([0.0, t_eng]), np.array([True, False]), 1.0)
    assert a.check_engage_after_stop(brake(0.17), t, pos, 0.0, "E").passed
    assert a.check_engage_after_stop(brake(0.16), t, pos, 0.0, "E").passed   # at t[8]
    assert not a.check_engage_after_stop(brake(0.10), t, pos, 0.0, "E").passed
    never = a.Step(np.array([0.0]), np.array([True]), 1.0)
    assert not a.check_engage_after_stop(never, t, pos, 0.0, "E").passed


def test_check_constant_resolution():
    sig = a.Step(np.array([0.0, 1.0, 1.0005]), np.array([False, True, False]), 2.0)
    assert not a.check_constant(sig, 0.0, 2.0, False, "C").passed
    assert a.check_constant(sig, 0.0, 2.0, False, "C", resolution=0.001).passed
