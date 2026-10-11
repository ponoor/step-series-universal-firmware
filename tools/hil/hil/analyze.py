"""Signal analysis: digital segments -> edges, pulse widths and statistics, command
matching against the P1 probe, excitation detection on OUT1A and the brake invariants.

Digital captures are represented as segments: seg_start[k] is the first sample of a run
of identical samples with value seg_value[k]. The capture covers n_samples samples.
This is what the compressed record of the AD2 delivers (value, span - 1 pairs).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------------------
# Digital segments and edges
# --------------------------------------------------------------------------------------


@dataclass
class Segments:
    start: np.ndarray  # int64, first sample of each segment
    value: np.ndarray  # uint32, sample value of each segment
    n_samples: int  # total samples covered
    rate: float  # samples per second

    @property
    def duration(self) -> float:
        return self.n_samples / self.rate


def segments_from_pairs(words: np.ndarray, rate: float) -> Segments:
    """Decodes compressed record data: value0, count0, value1, count1, ...
    where each value is held for count + 1 samples. Consecutive pairs with the same
    value (e.g. after a counter overflow) are merged."""
    words = np.asarray(words, dtype=np.uint32)
    if words.size % 2:
        raise ValueError("compressed data must have an even number of words")
    values = words[0::2]
    spans = words[1::2].astype(np.int64) + 1
    starts = np.concatenate(([0], np.cumsum(spans)[:-1])) if spans.size else np.zeros(0, np.int64)
    n = int(spans.sum())
    return _merge_equal(Segments(starts.astype(np.int64), values, n, rate))


def segments_from_samples(samples: np.ndarray, rate: float) -> Segments:
    """Builds segments from uncompressed samples."""
    samples = np.asarray(samples)
    if samples.size == 0:
        return Segments(np.zeros(0, np.int64), np.zeros(0, np.uint32), 0, rate)
    change = np.flatnonzero(samples[1:] != samples[:-1]) + 1
    starts = np.concatenate(([0], change)).astype(np.int64)
    return Segments(starts, samples[starts].astype(np.uint32), int(samples.size), rate)


def _merge_equal(seg: Segments) -> Segments:
    if seg.value.size < 2:
        return seg
    keep = np.concatenate(([True], seg.value[1:] != seg.value[:-1]))
    return Segments(seg.start[keep], seg.value[keep], seg.n_samples, seg.rate)


@dataclass
class Edges:
    rise: np.ndarray  # seconds
    fall: np.ndarray  # seconds
    initial: int  # level at t = 0
    final: int  # level at the end
    duration: float  # seconds covered by the capture

    @property
    def all(self) -> np.ndarray:
        return np.sort(np.concatenate((self.rise, self.fall)))


def bit_edges(seg: Segments, bit: int) -> Edges:
    level = ((seg.value >> np.uint32(bit)) & np.uint32(1)).astype(np.int8)
    if level.size == 0:
        return Edges(np.zeros(0), np.zeros(0), 0, 0, seg.duration)
    d = np.diff(level)
    t = seg.start[1:] / seg.rate
    return Edges(t[d > 0], t[d < 0], int(level[0]), int(level[-1]), seg.duration)


def pulse_widths(edges: Edges) -> tuple[np.ndarray, np.ndarray]:
    """High pulses fully inside the capture: (rise times, widths)."""
    rise, fall = edges.rise, edges.fall
    if rise.size == 0 or fall.size == 0:
        return np.zeros(0), np.zeros(0)
    # The first fall after each rise.
    idx = np.searchsorted(fall, rise, side="right")
    ok = idx < fall.size
    rise = rise[ok]
    fall = fall[idx[ok]]
    # Drop rises that share a fall (cannot happen with clean edges, kept for safety).
    if rise.size > 1:
        unique = np.concatenate((fall[1:] != fall[:-1], [True]))
        rise, fall = rise[unique], fall[unique]
    return rise, fall - rise


def intervals(times: np.ndarray) -> np.ndarray:
    return np.diff(np.asarray(times, dtype=float))


def pulse_groups(rise: np.ndarray, gap: float) -> list[int]:
    """Number of pulses in each group of rises separated by more than gap seconds."""
    if rise.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(rise) > gap)
    bounds = np.concatenate(([0], breaks + 1, [rise.size]))
    return [int(b - a) for a, b in zip(bounds[:-1], bounds[1:])]


# --------------------------------------------------------------------------------------
# Statistics
# --------------------------------------------------------------------------------------

STAT_KEYS = ("count", "min", "p50", "p99", "p99_9", "max", "mean")


def stats(values: np.ndarray, scale: float = 1.0) -> dict:
    """count / min / p50 / p99 / p99.9 / max / mean of values * scale (None when empty)."""
    v = np.asarray(values, dtype=float) * scale
    if v.size == 0:
        return {k: (0 if k == "count" else None) for k in STAT_KEYS}
    p50, p99, p999 = np.percentile(v, [50, 99, 99.9])
    return {
        "count": int(v.size),
        "min": float(v.min()),
        "p50": float(p50),
        "p99": float(p99),
        "p99_9": float(p999),
        "max": float(v.max()),
        "mean": float(v.mean()),
    }


# --------------------------------------------------------------------------------------
# Probe self-test ('P') and Scope / Digital In alignment
# --------------------------------------------------------------------------------------


def find_selftest(pulses: dict[int, tuple[np.ndarray, np.ndarray]], n_probes: int = 7,
                  width: tuple[float, float] = (5e-6, 40e-6), cluster_gap: float = 0.5e-3,
                  group_gap: tuple[float, float] = (0.8e-3, 1.6e-3)) -> dict[int, int] | None:
    """Finds the 'P' self-test pattern: groups of 1, 2, ..., n_probes pulses of about
    10 us, about 1 ms apart, each group on one line. pulses maps a DIO number to
    (rise times, widths). Returns {probe channel: DIO} or None if not found."""
    events = []  # (rise, dio) of pulses with the self-test width
    for dio, (rise, w) in pulses.items():
        ok = (w >= width[0]) & (w <= width[1])
        events += [(float(t), dio) for t in rise[ok]]
    events.sort()
    # Clusters: consecutive pulses on the same line closer than cluster_gap.
    clusters: list[list] = []  # [first rise, last rise, dio, count]
    for t, dio in events:
        if clusters and clusters[-1][2] == dio and t - clusters[-1][1] <= cluster_gap:
            clusters[-1][1] = t
            clusters[-1][3] += 1
        else:
            clusters.append([t, t, dio, 1])
    for i in range(len(clusters) - n_probes + 1):
        seq = clusters[i:i + n_probes]
        if [c[3] for c in seq] != list(range(1, n_probes + 1)):
            continue
        gaps = [b[0] - a[1] for a, b in zip(seq[:-1], seq[1:])]
        if all(group_gap[0] <= g <= group_gap[1] for g in gaps):
            return {ch: c[2] for ch, c in enumerate(seq)}
    return None


def estimate_offset(t_a: np.ndarray, t_b: np.ndarray, tolerance: float, max_offset: float) -> tuple[float, int]:
    """Offset d such that the edges t_a - d line up with the edges t_b (same signal seen
    by two instruments). Returns (d, number of matched edges)."""
    t_a = np.sort(np.asarray(t_a, float))
    t_b = np.sort(np.asarray(t_b, float))
    if t_a.size == 0 or t_b.size == 0:
        return 0.0, 0
    candidates = []
    for t in t_a[:50]:
        near = t_b[(t_b >= t - max_offset) & (t_b <= t + max_offset)]
        candidates += list(t - near)
    best, best_n = 0.0, -1
    for d in candidates:
        shifted = t_a - d
        idx = np.clip(np.searchsorted(t_b, shifted), 1, t_b.size - 1)
        nearest = np.minimum(np.abs(t_b[idx] - shifted), np.abs(t_b[idx - 1] - shifted))
        n = int(np.sum(nearest <= tolerance))
        if n > best_n:
            best, best_n = d, n
    # Refine with the median residual of the matched edges.
    shifted = t_a - best
    idx = np.clip(np.searchsorted(t_b, shifted), 1, t_b.size - 1)
    pick = np.where(np.abs(t_b[idx] - shifted) < np.abs(t_b[idx - 1] - shifted), idx, idx - 1)
    res = shifted - t_b[pick]
    ok = np.abs(res) <= tolerance
    if ok.any():
        best += float(np.median(res[ok]))
    return float(best), best_n


# --------------------------------------------------------------------------------------
# Host time <-> AD2 time
# --------------------------------------------------------------------------------------


class MatchError(Exception):
    pass


@dataclass
class ClockFit:
    """t_ad2 = slope * (t_host - t_host0) + offset (seconds)."""

    slope: float
    offset: float
    t_host0: float
    residual_max: float
    residual_std: float

    def to_ad2(self, t_host: np.ndarray | float) -> np.ndarray | float:
        return self.slope * (np.asarray(t_host, dtype=float) - self.t_host0) + self.offset


def match_commands(tx_host: np.ndarray, p1_rise: np.ndarray) -> ClockFit:
    """Pairs the n-th sent message with the n-th P1 rise (host seconds vs AD2 seconds) and
    fits a linear clock map. The counts must be equal: an extra or missing pulse (another
    packet on the network, a lost packet) fails the measurement."""
    tx = np.asarray(tx_host, dtype=float)
    p1 = np.asarray(p1_rise, dtype=float)
    if tx.size != p1.size:
        raise MatchError(f"sent {tx.size} OSC messages but saw {p1.size} P1 pulses")
    if tx.size == 0:
        raise MatchError("no OSC messages to match")
    t0 = tx[0]
    x = tx - t0
    if tx.size >= 2 and np.ptp(x) > 0:
        slope, offset = np.polyfit(x, p1, 1)
    else:
        slope, offset = 1.0, float(np.mean(p1 - x))
    res = p1 - (slope * x + offset)
    return ClockFit(float(slope), float(offset), float(t0), float(np.max(np.abs(res))), float(np.std(res)))


# --------------------------------------------------------------------------------------
# Step signals (boolean functions of time) and interval logic
# --------------------------------------------------------------------------------------


@dataclass
class Step:
    """Boolean signal: value[k] holds from times[k] to times[k+1] (the last one to t_end)."""

    times: np.ndarray
    values: np.ndarray
    t_end: float

    @classmethod
    def from_edges(cls, edges: Edges, invert: bool = False) -> "Step":
        ev = sorted([(t, 1) for t in edges.rise] + [(t, 0) for t in edges.fall])
        times = [0.0] + [t for t, _ in ev]
        values = [edges.initial] + [v for _, v in ev]
        vals = np.array(values, dtype=bool)
        return cls(np.array(times, dtype=float), ~vals if invert else vals, edges.duration)._compact()

    @classmethod
    def from_windows(cls, t_start: np.ndarray, flags: np.ndarray, t_end: float) -> "Step":
        return cls(np.asarray(t_start, float), np.asarray(flags, bool), t_end)._compact()

    def _compact(self) -> "Step":
        if self.values.size < 2:
            return self
        keep = np.concatenate(([True], self.values[1:] != self.values[:-1]))
        return Step(self.times[keep], self.values[keep], self.t_end)

    def at(self, t: np.ndarray | float) -> np.ndarray:
        idx = np.searchsorted(self.times, np.asarray(t, float), side="right") - 1
        return self.values[np.clip(idx, 0, self.values.size - 1)]

    def intervals(self, value: bool = True) -> list[tuple[float, float]]:
        ends = np.concatenate((self.times[1:], [self.t_end]))
        return [(float(a), float(b)) for a, b, v in zip(self.times, ends, self.values) if bool(v) == value and b > a]

    def transitions(self) -> np.ndarray:
        return self.times[1:].copy()


def intersect(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> list[tuple[float, float]]:
    out = []
    i = j = 0
    while i < len(a) and j < len(b):
        lo, hi = max(a[i][0], b[j][0]), min(a[i][1], b[j][1])
        if hi > lo:
            out.append((lo, hi))
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return out


def subtract(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Parts of the intervals a that are not covered by the intervals b."""
    out = []
    for lo, hi in a:
        cur = lo
        for blo, bhi in b:
            if bhi <= cur or blo >= hi:
                continue
            if blo > cur:
                out.append((cur, blo))
            cur = max(cur, bhi)
        if cur < hi:
            out.append((cur, hi))
    return out


# --------------------------------------------------------------------------------------
# Excitation detection
# --------------------------------------------------------------------------------------


def comparator_edges(volts: np.ndarray, rate: float, level: float, hysteresis: float) -> np.ndarray:
    """Times of the output transitions of a hysteresis comparator (both directions)."""
    v = np.asarray(volts, dtype=float)
    hi = v > level + hysteresis / 2
    lo = v < level - hysteresis / 2
    state = np.where(hi, 1, np.where(lo, 0, -1))
    known = np.flatnonzero(state >= 0)
    if known.size == 0:
        return np.zeros(0)
    # Forward-fill the undecided samples with the last decided state.
    idx = np.zeros(v.size, dtype=np.int64)
    idx[known] = known
    idx = np.maximum.accumulate(idx)
    filled = state[idx]
    filled[: known[0]] = state[known[0]]
    change = np.flatnonzero(filled[1:] != filled[:-1]) + 1
    return change / rate


def window_medians(volts: np.ndarray, rate: float, window: float) -> np.ndarray:
    v = np.asarray(volts, dtype=float)
    per = max(1, int(round(window * rate)))
    n_win = max(1, int(np.ceil(v.size / per)))
    padded = np.full(n_win * per, np.nan)
    padded[:v.size] = v
    return np.nanmedian(padded.reshape(n_win, per), axis=1)


def excitation(volts: np.ndarray, rate: float, level: float, hysteresis: float, window: float,
               min_edges: int, clamp_v: float | None = None) -> tuple[Step, np.ndarray, np.ndarray]:
    """Excited where a window has at least min_edges comparator transitions (PWM), or,
    with clamp_v, where the window median is below clamp_v (the L6470 holds the
    non-switching side of a bridge low, while a HiZ output floats higher).
    Returns (step signal, window start times, edges per window)."""
    n = len(volts)
    duration = n / rate
    edges = comparator_edges(volts, rate, level, hysteresis)
    n_win = max(1, int(np.ceil(duration / window)))
    counts = np.bincount(np.minimum((edges / window).astype(np.int64), n_win - 1), minlength=n_win)
    t_win = np.arange(n_win) * window
    flags = counts >= min_edges
    if clamp_v is not None:
        flags |= window_medians(volts, rate, window)[:n_win] < clamp_v
    return Step.from_windows(t_win, flags, duration), t_win, counts


# --------------------------------------------------------------------------------------
# Brake invariants and scenario checks
# --------------------------------------------------------------------------------------


@dataclass
class CheckResult:
    id: str
    passed: bool
    violations: list[tuple[float, float]] = field(default_factory=list)
    detail: str = ""

    def to_dict(self) -> dict:
        return {"id": self.id, "passed": self.passed, "detail": self.detail,
                "violations": [[round(a, 6), round(b, 6)] for a, b in self.violations]}


def check_inv1(released: Step, excited: Step, exempt: list[tuple[float, float]] = (),
               resolution: float = 0.0) -> CheckResult:
    """INV-1: no time with the brake released while the motor is not excited.
    Intervals shorter than the resolution of the excitation detection are ignored."""
    bad = intersect(released.intervals(True), excited.intervals(False))
    bad = subtract(bad, sorted(exempt))
    bad = [iv for iv in bad if iv[1] - iv[0] > resolution]
    return CheckResult("INV-1", not bad, bad, f"{len(bad)} interval(s) released and not excited")


def check_inv2(released: Step, excited: Step, allowance: float, exempt: list[tuple[float, float]] = ()) -> CheckResult:
    """INV-2: brake engaged and motor excited only around a brake transition, for at most
    `allowance` seconds per interval. An overlap not adjacent to a brake transition
    (within allowance) is a violation regardless of its length."""
    overlaps = subtract(intersect(released.intervals(False), excited.intervals(True)), sorted(exempt))
    trans = released.transitions()
    bad = []
    for lo, hi in overlaps:
        near = trans.size and (np.min(np.abs(trans - lo)) <= allowance or np.min(np.abs(trans - hi)) <= allowance)
        if hi - lo > allowance or not near:
            bad.append((lo, hi))
    longest = max((hi - lo for lo, hi in overlaps), default=0.0)
    return CheckResult("INV-2", not bad, bad,
                       f"{len(overlaps)} overlap(s), longest {longest * 1e3:.1f} ms, allowance {allowance * 1e3:.0f} ms")


def check_inv3(released: Step, pos_t: np.ndarray, pos: np.ndarray) -> CheckResult:
    """INV-3: the position does not change while the brake is engaged (both readings of a
    changed pair taken with the brake engaged)."""
    pos_t = np.asarray(pos_t, float)
    pos = np.asarray(pos)
    bad = []
    if pos.size >= 2:
        engaged = ~released.at(pos_t)
        changed = pos[1:] != pos[:-1]
        both = engaged[1:] & engaged[:-1]
        for k in np.flatnonzero(changed & both):
            bad.append((float(pos_t[k]), float(pos_t[k + 1])))
    return CheckResult("INV-3", not bad, bad, f"{len(bad)} position change(s) while engaged")


def check_constant(signal: Step, t_from: float, t_to: float, value: bool, check_id: str) -> CheckResult:
    """The signal keeps `value` from t_from to t_to."""
    window = [(t_from, t_to)]
    bad = intersect(window, signal.intervals(not value))
    return CheckResult(check_id, not bad, bad, f"expected {'high' if value else 'low'} from {t_from:.3f} s to {t_to:.3f} s")


def first_change_after(signal: Step, t: float) -> float | None:
    """Time of the first transition of the signal after t (None if none)."""
    tr = signal.transitions()
    after = tr[tr > t]
    return float(after[0]) if after.size else None
