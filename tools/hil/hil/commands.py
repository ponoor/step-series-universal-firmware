"""Implementation of the hil.py subcommands: selftest, timing, brake, powerup, compare."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
import threading
import time
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path

import numpy as np

from . import analyze, plots
from . import scenarios as sc
from .ad2 import AD2, Capture
from .config import HIL_DIR, NUM_MOTORS, Config
from .diag import SerialDiag, parse_probe_map
from .osc import OscError, OscLink

RESULTS_DIR = HIL_DIR / "results"
PROBE_NAMES = ("loop", "oscReceive", "updateServo", "checkStatus", "getMyId", "udpSend", "brakeWrite")
EXPECTED_PROBE_MAP = {0: "D1(PA10)", 1: "A1(PB08)", 2: "D8(PA06)", 3: "D9(PA07)",
                      4: "D38(PA13)", 5: "SCL(PA23)", 6: "SDA(PA22)"}


# --------------------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------------------


def git_short_sha() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=HIL_DIR, capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "nogit"


def new_result_dir(label: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    path = RESULTS_DIR / f"{stamp}_{git_short_sha()}_{label}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def write_json(path: Path, data) -> None:
    path.write_text(json.dumps(data, indent=2, default=_json_default) + "\n")


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (set, frozenset)):
        return sorted(o)
    raise TypeError(type(o))


def write_csv(path: Path, headers: list[str], rows: list[list]) -> None:
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        w.writerows(rows)


def fmt(v, digits: int = 1) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def print_table(headers: list[str], rows: list[list]) -> None:
    cells = [[str(h) for h in headers]] + [[fmt(c) for c in r] for r in rows]
    widths = [max(len(r[i]) for r in cells) for i in range(len(headers))]
    for k, r in enumerate(cells):
        print("  ".join(c.rjust(w) if k and i else c.ljust(w) for i, (c, w) in enumerate(zip(r, widths))))
        if k == 0:
            print("  ".join("-" * w for w in widths))


def prompt(message: str) -> None:
    """Waits for Enter from the operator; without a terminal, only prints the message."""
    if sys.stdin.isatty():
        input(message)
    else:
        print(message + "(no terminal: continuing)")


def mask_of(dios) -> int:
    m = 0
    for d in dios:
        m |= 1 << int(d)
    return m


def open_ad2(cfg: Config) -> AD2:
    a = cfg.ad2
    conf = a.get("configuration", -1)
    return AD2(a.get("dwf_library", ""), a.get("serial_number", ""), float(a["scope"]["range_vpp"]),
               None if conf is None or int(conf) < 0 else int(conf))


def open_link(cfg: Config) -> OscLink:
    return OscLink(cfg.board_ip, cfg.board_in_port, cfg.board_out_port)


def connect_board(link: OscLink, timeout: float = 15.0) -> str:
    """/setDestIp with retries (the board may still be booting). Returns the local IP."""
    deadline = time.monotonic() + timeout
    while True:
        try:
            return link.set_dest_ip()
        except (OscError, OSError):  # OSError: no ARP entry yet while the board boots
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.5)


def firmware_version(link: OscLink) -> str:
    try:
        return str(link.request("/getVersion", reply="/version").args[0])
    except OscError:
        return "unknown"


def check_capture(cap: Capture) -> None:
    if not cap.ok:
        raise RuntimeError("; ".join(cap.problems()) + ". Lower the sample rate and measure again.")


def tx_in_window(link: OscLink, cap: Capture) -> list:
    return link.entries("tx", cap.host_trigger_ns, cap.host_stop_ns)


def host_s(cap: Capture, t_ns: int) -> float:
    return (t_ns - cap.host_trigger_ns) / 1e9


class ScopeGapError(RuntimeError):
    pass


def scope_offset(cfg: Config, cap: Capture) -> tuple[float, dict]:
    """Scope start minus Digital In start for this capture, from the P0 edges seen by
    both instruments (ad2.scope.p0_ref); the configured constant without a reference."""
    ref = int(cfg.ad2["scope"].get("p0_ref", 0))
    if ref and ref in cap.analog and cap.digital is not None:
        t_dig = analyze.bit_edges(cap.digital, cfg.probe_dio[0]).all
        t_ana = analyze.comparator_edges(cap.analog[ref], cap.analog_rate, 1.65, 0.6)
        tol = max(3 / cap.analog_rate, 1.5 / cap.digital.rate)
        d, n = analyze.estimate_offset(t_ana, t_dig, tolerance=tol, max_offset=5e-3)
        fraction = n / max(1, min(len(t_ana), len(t_dig)))
        # The Scope can drop a whole buffer without reporting it (Task 5: 8192 samples), which
        # shifts the rest of the recording. Every window must have the same offset.
        # Only where both recordings have data (the Scope runs a little past Digital In).
        both = (t_ana - d >= 0.0) & (t_ana - d <= cap.digital.duration - 1e-3)
        t_both = t_ana[both]
        windows = []
        for t0 in np.arange(0.0, t_both.max(initial=0.0), 0.2):
            sel = (t_both >= t0) & (t_both < t0 + 0.2)
            if sel.sum() >= 20:
                windows.append((float(t0), analyze.match_fraction(t_both[sel], t_dig, d, tol)))
        bad = [w for w in windows if w[1] < 0.9]
        if fraction < 0.95 or bad:
            where = f", from {bad[0][0]:.1f} s" if bad else ""
            raise ScopeGapError(f"Scope / Digital In alignment failed: {n} of {min(len(t_ana), len(t_dig))} "
                                f"P0 edges matched{where}; the Scope probably dropped samples without reporting it")
        return -d, {"method": "p0_ref", "scope_minus_digital_us": -d * 1e6, "matched_edges": n,
                    "matched_fraction": fraction, "windows": len(windows)}
    value = float(cfg.section("sync")["scope_minus_digital_s"])
    return value, {"method": "config", "scope_minus_digital_us": value * 1e6}


def excitation_step(cfg: Config, cap: Capture, shift: float | None = None) -> tuple[analyze.Step, np.ndarray, np.ndarray]:
    """Excitation of the test axis on the Digital In time base (shift: scope minus digital)."""
    ex = cfg.section("excitation")
    ch = int(cfg.ad2["scope"]["out1a"])
    step, t_win, counts = analyze.excitation(cap.analog[ch], cap.analog_rate, float(ex["level_v"]),
                                             float(ex["hysteresis_v"]), float(ex["window_ms"]) / 1e3,
                                             int(ex["min_edges"]), ex.get("clamp_v"))
    if shift is None:
        shift = 0.0
    return analyze.Step(step.times + shift, step.values, step.t_end + shift), t_win + shift, counts


# --------------------------------------------------------------------------------------
# selftest (Task 3)
# --------------------------------------------------------------------------------------


def cmd_selftest(cfg: Config, args) -> int:
    out = new_result_dir("selftest")
    results: dict = {"config": str(cfg.path), "axis": cfg.test_axis}
    rows = []
    with ExitStack() as stack:
        ad2 = stack.enter_context(open_ad2(cfg))
        results["waveforms"] = ad2.version()
        if ad2.supplies_enabled():
            print("WARNING: the AD2 power supplies are enabled; the bench does not use them.")
        diag = stack.enter_context(SerialDiag(cfg.serial_port))
        link = stack.enter_context(open_link(cfg))

        boot = diag.reboot()
        (out / "boot.txt").write_text(boot)
        results["local_ip"] = connect_board(link)
        results["firmware"] = firmware_version(link)

        steps = [("probes", _selftest_probes), ("brake_map", _selftest_brake_map),
                 ("home_sw", _selftest_home_sw), ("excitation", _selftest_excitation)]
        if args.sync:
            steps.append(("sync", _selftest_sync))
        for name, fn in steps:
            if args.only and name not in args.only:
                continue
            try:
                r = fn(cfg, ad2, diag, link, out, boot)
            except Exception as ex:  # keep going, report the failure
                r = {"ok": False, "error": f"{type(ex).__name__}: {ex}"}
            results[name] = r
            rows.append([name, "OK" if r.get("ok") else "NG", r.get("summary", r.get("error", ""))])
        diag.reboot()  # brake outputs back to the boot state
    link.save_log(out / "osc_log.jsonl")
    write_json(out / "summary.json", results)
    print_table(["check", "result", "detail"], rows)
    print(f"results: {out}")
    return 0 if all(r[1] == "OK" for r in rows) else 1


def _selftest_probes(cfg, ad2, diag, link, out, boot) -> dict:
    probe_dio = cfg.probe_dio
    rec = ad2.start_recording(0.8, digital_rate=1e6, sensible_mask=mask_of(probe_dio))
    time.sleep(0.2)
    pmap, text = diag.probe_selftest()
    cap = rec.wait()
    check_capture(cap)
    cap.save(out / "probes_capture.npz")
    pulses = {d: analyze.pulse_widths(analyze.bit_edges(cap.digital, d)) for d in probe_dio}
    found = analyze.find_selftest(pulses)
    expected = {ch: d for ch, d in enumerate(probe_dio)}
    ok = found == expected and pmap == EXPECTED_PROBE_MAP
    summary = ("P0-P6 on DIO " + ",".join(str(found[c]) for c in range(7))) if found else "self-test pattern not found"
    if pmap != EXPECTED_PROBE_MAP:
        summary += f"; firmware pin map {pmap}"
    return {"ok": ok, "summary": summary, "found": found, "expected": expected,
            "boot_map": parse_probe_map(boot), "serial_map": pmap}


def _selftest_brake_map(cfg, ad2, diag, link, out, boot) -> dict:
    """'b' x 8: which DIO 0-7 goes high for each motor, and the delay against P4."""
    p4 = cfg.probe_dio[4]
    brake_dios = list(range(8))
    rec = ad2.start_recording(None, digital_rate=1e6, sensible_mask=mask_of(brake_dios + [p4]))
    time.sleep(0.3)
    log = []
    for _ in range(NUM_MOTORS):
        t = time.monotonic_ns()
        off, on = diag.brake_step()
        log.append((t, off, on))
        time.sleep(0.3)
    cap = rec.stop()
    check_capture(cap)
    cap.save(out / "brake_map_capture.npz")
    edges = {d: analyze.bit_edges(cap.digital, d) for d in brake_dios}
    p4_rise = analyze.bit_edges(cap.digital, p4).rise
    mapping: dict[int, int] = {}
    details = []
    for t_ns, off, on in log:
        t_cmd = host_s(cap, t_ns)
        risen = [(e.rise[(e.rise > t_cmd) & (e.rise < t_cmd + 0.25)], d) for d, e in edges.items()]
        risen = [(r[0], d) for r, d in risen if r.size]
        if len(risen) != 1:
            details.append({"motor": on, "error": f"{len(risen)} DIO rose"})
            continue
        t_rise, dio = risen[0]
        mapping[on] = dio
        before = p4_rise[p4_rise <= t_rise + 5e-5]
        n_p4 = int(np.sum((p4_rise > t_cmd) & (p4_rise <= t_rise + 5e-5)))
        details.append({"motor": on, "dio": dio, "delay_from_command_ms": (t_rise - t_cmd) * 1e3,
                        "p4_rises_between": n_p4,
                        "from_last_p4_rise_us": (t_rise - before[-1]) * 1e6 if before.size else None})
    write_csv(out / "brake_map.csv", ["motor", "dio", "delay_ms", "p4_rises_between", "from_last_p4_rise_us"],
              [[d.get("motor"), d.get("dio"), d.get("delay_from_command_ms"), d.get("p4_rises_between"),
                d.get("from_last_p4_rise_us")] for d in details])
    ok = len(mapping) == NUM_MOTORS and len(set(mapping.values())) == NUM_MOTORS
    configured = {m: int(cfg.ad2["dio"]["brake"][m - 1]) for m in range(1, NUM_MOTORS + 1)}
    summary = "motor->DIO " + ",".join(f"{m}:{mapping.get(m, '?')}" for m in range(1, NUM_MOTORS + 1))
    if ok and mapping != configured:
        summary += " (differs from bench.toml)"
    return {"ok": ok, "summary": summary, "mapping": mapping, "configured": configured, "details": details,
            "note": "delay_from_command includes the USB serial latency"}


def _selftest_home_sw(cfg, ad2, diag, link, out, boot) -> dict:
    axis = cfg.test_axis
    is_axis = lambda a: int(a[0]) == axis  # noqa: E731
    link.send("/enableHomeSwReport", axis, 1)
    ad2.release()
    time.sleep(0.1)
    s0 = link.request("/getHomeSw", axis, reply="/homeSw", predicate=is_axis).args
    dio_released = (ad2.read_dio() >> 15) & 1
    t = time.monotonic_ns()
    ad2.press()
    rep1 = link.wait_for("/homeSw", 1.0, t, is_axis)
    time.sleep(0.1)
    dio_pressed = (ad2.read_dio() >> 15) & 1
    s1 = link.request("/getHomeSw", axis, reply="/homeSw", predicate=is_axis).args
    t = time.monotonic_ns()
    ad2.release()
    rep2 = link.wait_for("/homeSw", 1.0, t, is_axis)
    time.sleep(0.1)
    s2 = link.request("/getHomeSw", axis, reply="/homeSw", predicate=is_axis).args
    link.send("/enableHomeSwReport", axis, 0)
    states = {"get_released": int(s0[1]), "report_pressed": int(rep1.args[1]) if rep1 else None,
              "get_pressed": int(s1[1]), "report_released": int(rep2.args[1]) if rep2 else None,
              "get_released_again": int(s2[1]), "dio15_released": dio_released, "dio15_pressed": dio_pressed}
    ok = (states["get_released"] == 0 and states["report_pressed"] == 1 and states["get_pressed"] == 1
          and states["report_released"] == 0 and states["get_released_again"] == 0
          and dio_released == 1 and dio_pressed == 0)
    return {"ok": ok, "summary": " ".join(f"{k}={v}" for k, v in states.items()), "states": states}


def _selftest_excitation(cfg, ad2, diag, link, out, boot) -> dict:
    """OUT1A energized at EL_POS 0 (held low), energized at +64 microsteps (PWM), HiZ."""
    axis = cfg.test_axis
    b = cfg.section("brake")
    kval = int(b["kval"])
    link.send("/setKval", axis, kval, kval, kval, kval)
    link.send("/setMaxSpeed", axis, float(b["max_speed"]))
    time.sleep(0.1)
    ch = int(cfg.ad2["scope"]["out1a"])
    rec = ad2.start_recording(None, analog_channels=(ch,), analog_rate=float(b["analog_rate"]))
    time.sleep(0.3)
    marks = [("energized EL_POS 0 (/hardStop)", link.send("/hardStop", axis).t_ns)]
    time.sleep(1.0)
    marks.append(("energized +64 usteps (/move)", link.send("/move", axis, 64).t_ns))
    time.sleep(1.0)
    marks.append(("HiZ (/hardHiZ)", link.send("/hardHiZ", axis).t_ns))
    time.sleep(1.0)
    cap = rec.stop()
    check_capture(cap)
    cap.save(out / "excitation_capture.npz")
    step, t_win, counts = excitation_step(cfg, cap)
    shift = 0.0
    medians = analyze.window_medians(cap.analog[ch], cap.analog_rate, float(cfg.section("excitation")["window_ms"]) / 1e3)
    end = len(cap.analog[ch]) / cap.analog_rate
    starts = [host_s(cap, t) for _, t in marks] + [end]
    res: dict = {"kval_sent": [kval] * 4, "analog_rate": cap.analog_rate, "spans": {}}
    ok = True
    for (name, _), lo, hi in zip(marks, starts[:-1], starts[1:]):
        lo, hi = lo + 0.3, hi - 0.05
        sel = (t_win >= lo + shift) & (t_win + 1e-3 <= hi + shift)
        excited_frac = float(np.mean(step.at(t_win[sel] + 5e-4)))
        expect = not name.startswith("HiZ")
        ok &= excited_frac == (1.0 if expect else 0.0)
        res["spans"][name] = {"edges_per_window": analyze.stats(counts[sel]),
                              "median_v": analyze.stats(medians[:counts.size][sel]),
                              "excited_fraction": excited_frac, "expected": expect}
    plots.waveform(out / "excitation_waveform.png", f"OUT1A, KVAL {kval}", cap.analog_rate, cap.analog[ch],
                   [(name, lo + 0.3, hi) for (name, _), lo, hi in zip(marks, starts[:-1], starts[1:])],
                   snippet=0.3e-3)
    res["ok"] = bool(ok)
    res["summary"] = "; ".join(f"{n.split(' (')[0]}: excited {v['excited_fraction']:.3f}, "
                               f"edges/ms p50 {v['edges_per_window']['p50']}, median {v['median_v']['p50']:.2f} V"
                               for n, v in res["spans"].items())
    return res


def _selftest_sync(cfg, ad2, diag, link, out, boot) -> dict:
    """Per-capture Scope / Digital In alignment from P0 (Scope p0_ref on P0)."""
    ref = int(cfg.ad2["scope"].get("p0_ref", 0))
    if not ref:
        return {"ok": False, "summary": "ad2.scope.p0_ref is 0"}
    b = cfg.section("brake")
    p0 = cfg.probe_dio[0]
    runs = []
    for i in range(5):
        cap = ad2.record(0.3, digital_rate=float(b["digital_rate"]), sensible_mask=mask_of([p0]),
                         analog_channels=(ref,), analog_rate=float(b["analog_rate"]))
        check_capture(cap)
        _, info = scope_offset(cfg, cap)
        runs.append(info)
    offsets = [r["scope_minus_digital_us"] for r in runs]
    worst = min(r["matched_fraction"] for r in runs)
    return {"ok": worst >= 0.99, "runs": runs,
            "summary": f"scope - digital {min(offsets):.0f} to {max(offsets):.0f} us over {len(runs)} captures, "
                       f"edges matched >= {worst * 100:.1f} %"}


# --------------------------------------------------------------------------------------
# timing (Task 4)
# --------------------------------------------------------------------------------------


def timing_series(cfg: Config, seg: analyze.Segments) -> dict[str, np.ndarray]:
    """Durations in microseconds from the probes."""
    d = cfg.probe_dio
    e = [analyze.bit_edges(seg, d[ch]) for ch in range(7)]
    widths = [analyze.pulse_widths(x)[1] * 1e6 for x in e]
    return {
        "loop": analyze.intervals(e[0].all) * 1e6,
        "oscReceive": widths[1],
        "updateServo": widths[2],
        "checkStatus": widths[3],
        "getMyId": widths[4],
        "udpSend": widths[5],
        "servoInterval": analyze.intervals(e[2].rise) * 1e6,
        "pollInterval": analyze.intervals(e[4].rise) * 1e6,
    }


PERF_OF_SERIES = {"loop": "loop", "oscReceive": "oscReceive", "updateServo": "updateServo",
                  "checkStatus": "checkStatus", "servoInterval": "servoInterval"}


def cmd_timing(cfg: Config, args) -> int:
    t = cfg.section("timing")
    duration = float(args.duration if args.duration else t["duration"])
    rate = float(args.rate if args.rate else t["digital_rate"])
    out = new_result_dir(args.label)
    axis = cfg.test_axis
    kval = int(cfg.section("brake")["kval"])
    summary: dict = {"label": args.label, "duration_s": duration, "digital_rate": rate, "axis": axis}
    with ExitStack() as stack:
        ad2 = stack.enter_context(open_ad2(cfg))
        diag = stack.enter_context(SerialDiag(cfg.serial_port))
        link = stack.enter_context(open_link(cfg))
        if not args.no_reboot:
            diag.reboot()
        summary["local_ip"] = connect_board(link)
        summary["firmware"] = firmware_version(link)

        # Load (STATUS 7): servo mode on all motors, target list at 100 Hz, position reports.
        report_ms = int(t["position_report_ms"])
        link.send("/setKval", axis, kval, kval, kval, kval)
        pos0 = int(link.request("/getPosition", axis, reply="/position",
                                predicate=lambda a: int(a[0]) == axis).args[1])
        link.send("/enableServoMode", 255, 1)
        link.send("/setPositionReportInterval", 255, report_ms)
        summary["load"] = {"servo": "all", "target_rate_hz": t["target_rate_hz"],
                           "sine_amplitude": t["sine_amplitude"], "sine_period_s": t["sine_period"],
                           "position_report_ms": report_ms, "kval": kval}
        time.sleep(0.5)

        stop = threading.Event()

        def send_targets():
            period = 1.0 / float(t["target_rate_hz"])
            amp, sine_t = float(t["sine_amplitude"]), float(t["sine_period"])
            start = next_t = time.monotonic()
            while not stop.is_set():
                targets = [0] * NUM_MOTORS
                targets[axis - 1] = pos0 + int(round(amp * math.sin(2 * math.pi * (next_t - start) / sine_t)))
                link.send("/setTargetPositionList", *targets)
                next_t += period
                time.sleep(max(0.0, next_t - time.monotonic()))

        sender = threading.Thread(target=send_targets, daemon=True)
        sender.start()
        time.sleep(0.5)
        perf_before, text_before = diag.perf()  # resets the statistics
        rec = ad2.start_recording(duration, digital_rate=rate, sensible_mask=mask_of(cfg.probe_dio))
        cap = rec.wait()
        perf_after, text_after = diag.perf()
        stop.set()
        sender.join(1.0)
        link.send("/setPositionReportInterval", 255, 0)
        link.send("/enableServoMode", 255, 0)
        link.send("/softHiZ", axis)
        time.sleep(0.2)
        (out / "perf.txt").write_text(text_before + "\n----\n" + text_after)
    link.save_log(out / "osc_log.jsonl")
    cap.save(out / "capture.npz")
    summary["capture"] = {"ok": cap.ok, "problems": cap.problems(), "digital_samples": cap.digital.n_samples,
                          "segments": int(cap.digital.start.size)}
    if not cap.ok:
        write_json(out / "summary.json", summary)
        print("FAILED: " + "; ".join(cap.problems()) + ". Lower the sample rate (--rate).")
        print(f"results: {out}")
        return 1

    series = timing_series(cfg, cap.digital)
    tx = tx_in_window(link, cap)
    p1 = analyze.bit_edges(cap.digital, cfg.probe_dio[1]).rise
    summary["p1_vs_sent"] = {"sent": len(tx), "p1_pulses": int(p1.size)}
    summary["stats_us"] = {k: analyze.stats(v) for k, v in series.items()}
    summary["perf_before"] = perf_before
    summary["perf_after"] = perf_after
    summary["perf_vs_probe"] = {
        k: {"perf_max_us": perf_after.get(p, {}).get("max_us"), "probe_max_us": summary["stats_us"][k]["max"],
            "perf_avg_us": perf_after.get(p, {}).get("avg_us"), "probe_mean_us": summary["stats_us"][k]["mean"],
            "perf_n": perf_after.get(p, {}).get("n"), "probe_n": summary["stats_us"][k]["count"]}
        for k, p in PERF_OF_SERIES.items()}
    write_json(out / "summary.json", summary)
    rows = [[k] + [s[x] for x in analyze.STAT_KEYS] for k, s in summary["stats_us"].items()]
    write_csv(out / "stats_us.csv", ["metric"] + list(analyze.STAT_KEYS), rows)
    plots.histograms(series, out / "histograms.png", title=f"{args.label} ({duration:.0f} s)")

    print_table(["metric [us]"] + list(analyze.STAT_KEYS), rows)
    print()
    print_table(["perf vs probe", "perf max", "probe max", "perf avg", "probe mean"],
                [[k, v["perf_max_us"], v["probe_max_us"], v["perf_avg_us"], v["probe_mean_us"]]
                 for k, v in summary["perf_vs_probe"].items()])
    if len(tx) != p1.size:
        print(f"WARNING: sent {len(tx)} OSC messages but saw {p1.size} P1 pulses")
    print(f"results: {out}")
    return 0


def cmd_compare(cfg: Config, args) -> int:
    dirs = [Path(d) for d in args.result_dirs]
    sums = [json.loads((d / "summary.json").read_text()) for d in dirs]
    labels = tuple(s.get("label", d.name) for s, d in zip(sums, dirs))
    rows = []
    for metric in sums[0]["stats_us"]:
        a, b = sums[0]["stats_us"][metric], sums[1]["stats_us"].get(metric)
        if not b:
            continue
        for key in ("p50", "p99", "p99_9", "max"):
            va, vb = a[key], b[key]
            delta = (vb - va) if va is not None and vb is not None else None
            rel = (delta / va * 100) if delta is not None and va else None
            rows.append([metric, key, va, vb, delta, rel])
    out = dirs[1] / f"compare_{dirs[0].name}"
    out.mkdir(exist_ok=True)
    headers = ["metric", "stat", f"{labels[0]} [us]", f"{labels[1]} [us]", "delta [us]", "delta [%]"]
    write_csv(out / "compare.csv", headers, rows)
    series = [timing_series(cfg, Capture.load(d / "capture.npz").digital) for d in dirs]
    plots.compare_histograms(series[0], series[1], labels, out / "compare_histograms.png")
    print_table(headers, rows)
    print(f"results: {out}")
    return 0


# --------------------------------------------------------------------------------------
# brake (Task 5)
# --------------------------------------------------------------------------------------


def scenario_values(cfg: Config) -> dict:
    """Values of the placeholders of the scenarios (AXIS, KVAL, ...)."""
    b = cfg.section("brake")
    return {"axis": cfg.test_axis, "kval": int(b["kval"]), "max_speed": float(b["max_speed"]),
            "transition_ms": int(b["transition_ms"])}


def _run_steps(steps, values: dict, link: OscLink, ad2: AD2 | None, log: list | None,
               diag: SerialDiag | None = None) -> None:
    for st in steps:
        if st.kind == "osc":
            entry = link.send(st.address, *sc.resolve_args(st.args, values))
            if log is not None:
                log.append((st, entry.t_ns, entry))
        elif st.kind == "serial":
            t = time.monotonic_ns()
            diag.send(st.address)
            if log is not None:
                log.append((st, t, None))
        elif st.kind == "wait":
            time.sleep(st.seconds)
        elif st.kind in ("press", "release"):
            t = time.monotonic_ns()
            (ad2.press if st.kind == "press" else ad2.release)()
            if log is not None:
                log.append((st, t, None))
        else:
            raise ValueError(st.kind)


def run_scenario(cfg: Config, scenario: sc.Scenario, ad2: AD2, diag: SerialDiag, link: OscLink,
                 out: Path) -> dict:
    b = cfg.section("brake")
    axis = cfg.test_axis
    values = scenario_values(cfg)
    transition = int(b["transition_ms"]) / 1e3
    latency = int(b["latency_allowance_ms"]) / 1e3

    diag.reboot()
    connect_board(link)
    _run_steps(sc.COMMON_SETUP + scenario.extra_setup, values, link, ad2, None)
    time.sleep(0.3)

    pre = hiz_precheck(cfg, ad2)
    if not pre["ok"]:
        return {"id": scenario.id, "title": scenario.title, "expect": scenario.expect,
                "expected_failures": scenario.expected_failures, "outcome": "ABORTED",
                "precheck": pre, "error": pre["detail"]}

    probe = cfg.probe_dio
    brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
    out1a = int(cfg.ad2["scope"]["out1a"])
    ref = int(cfg.ad2["scope"].get("p0_ref", 0))
    # P0 is recorded only as the time reference of the Scope.
    mask = mask_of(brake_dios + (probe if ref else probe[1:]) + [15])
    rec = ad2.start_recording(None, digital_rate=float(b["digital_rate"]), sensible_mask=mask,
                              analog_channels=(out1a, ref) if ref else (out1a,),
                              analog_rate=float(b["analog_rate"]))
    stop = threading.Event()

    def poll():
        period = int(b["position_poll_ms"]) / 1e3
        while not stop.is_set():
            link.send("/getPosition", axis)
            time.sleep(period)

    poller = threading.Thread(target=poll, daemon=True)
    try:
        if scenario.poll_position:
            poller.start()
        time.sleep(0.3)
        log: list = []
        _run_steps(scenario.steps, values, link, ad2, log, diag)
        time.sleep(0.3)
    finally:
        stop.set()
        if poller.is_alive():
            poller.join(1.0)
        ad2.release()
        time.sleep(0.1)
        cap = rec.stop()
    cap.save(out / f"{scenario.id}_capture.npz")
    res = analyze_scenario(cfg, scenario, cap, link, log, transition, latency, out)
    res["precheck"] = pre
    return res


def hiz_precheck(cfg: Config, ad2: AD2) -> dict:
    """The test axis is HiZ with the brake engaged at the start of a scenario: the median
    of OUT1A must be the floating level ([excitation] hiz_median_v), else the wiring is
    suspect and the scenario must not be judged."""
    lo, hi = (float(v) for v in cfg.section("excitation")["hiz_median_v"])
    ch = int(cfg.ad2["scope"]["out1a"])
    cap = ad2.record(0.2, analog_channels=(ch,), analog_rate=float(cfg.section("brake")["analog_rate"]))
    median = float(np.median(cap.analog[ch])) if cap.analog[ch].size else float("nan")
    ok = cap.ok and lo <= median <= hi
    detail = (f"OUT1A median {median:.3f} V at the start (HiZ, brake engaged), expected {lo}-{hi} V"
              + ("" if ok else ": check the Scope 1 wiring; scenario aborted"))
    return {"ok": ok, "median_v": median, "range_v": [lo, hi], "detail": detail}


def analyze_scenario(cfg: Config, scenario: sc.Scenario, cap: Capture, link: OscLink, log: list,
                     transition: float, latency: float, out: Path) -> dict:
    res: dict = {"id": scenario.id, "title": scenario.title, "expect": scenario.expect,
                 "expected_failures": scenario.expected_failures}
    if not cap.ok:
        res.update(outcome="ERROR", error="; ".join(cap.problems()) + ". Lower the sample rate.", retryable=True)
        return res
    seg = cap.digital
    probe = cfg.probe_dio
    tx = tx_in_window(link, cap)
    p1 = analyze.bit_edges(seg, probe[1]).rise
    if scenario.reboots:
        # The probe pins float while the board restarts: keep the pulses of the messages.
        p1 = p1[:len(tx)]
    try:
        fit = analyze.match_commands(np.array([host_s(cap, e.t_ns) for e in tx]), p1)
    except analyze.MatchError as ex:
        res.update(outcome="ERROR", error=str(ex))
        return res
    res["clock_fit"] = {"slope": fit.slope, "offset_s": fit.offset, "residual_max_ms": fit.residual_max * 1e3,
                        "residual_std_ms": fit.residual_std * 1e3,
                        "note": "host -> board network delay; used only to find the DIO15 edge of press/release"}
    tx_index = {id(e): i for i, e in enumerate(tx)}

    end = seg.duration
    released = analyze.Step.from_edges(analyze.bit_edges(seg, cfg.brake_dio))
    try:
        shift, res["scope_alignment"] = scope_offset(cfg, cap)
    except ScopeGapError as ex:
        res.update(outcome="ERROR", error=str(ex), retryable=True)
        return res
    excited, _, _ = excitation_step(cfg, cap, shift)

    # AD2 time (and host time) of each logged step.
    label_t: dict[str, float] = {}
    label_host_ns: dict[str, int] = {}
    commands = []
    timing_notes = []
    dio15 = analyze.bit_edges(seg, 15)
    t_prev = 0.0
    for st, t_ns, entry in log:
        if entry is not None:
            t = float(p1[tx_index[id(entry)]])
        elif st.kind == "serial":
            # No probe pulse: the first change of the signal the command acts on.
            t = analyze.first_edge_to(excited, t_prev, False) if st.edge == sc.EXCITATION_OFF else None
            if t is None:
                t = float(fit.to_ad2(host_s(cap, t_ns)))
                timing_notes.append(f"{st.label or st.address}: no {st.edge} edge, host time used")
        else:
            # press / release: the DIO15 edge nearest to the host time.
            t = float(fit.to_ad2(host_s(cap, t_ns)))
            edges = dio15.fall if st.kind == "press" else dio15.rise
            if edges.size:
                t = float(edges[np.argmin(np.abs(edges - t))])
        t_prev = t
        if st.label:
            label_t[st.label] = t
            label_host_ns[st.label] = t_ns
        commands.append((t, st.label or st.address or st.kind, st))
    if timing_notes:
        res["step_timing_notes"] = timing_notes
    axis = cfg.test_axis
    # A position is read when the board processes the request: time it at the request's
    # P1 rise (the network delay, up to about 80 ms over Wi-Fi, does not matter then).
    req_idx, replies = match_replies(cfg, cap, tx, link, "/getPosition", "/position")
    answered = [(i, r) for i, r in zip(req_idx, replies) if r is not None]
    pos_t = np.array([float(p1[i]) for i, _ in answered])
    pos = np.array([int(r.args[1]) for _, r in answered])

    exempt: dict[str, list] = {}
    for ex in scenario.exempt:
        if ex.span is not None:
            t_to = label_t[ex.from_label] + sc.resolve_delay(ex.span, transition, latency)
        else:
            t_to = label_t[ex.to_label] + latency if ex.to_label else end
        exempt.setdefault(ex.invariant, []).append((label_t[ex.from_label], t_to))
    window = float(cfg.section("excitation")["window_ms"]) / 1e3
    checks: list[analyze.CheckResult] = []
    if "INV-1" in scenario.invariants:
        checks.append(analyze.check_inv1(released, excited, exempt.get("INV-1", []), resolution=window))
    if "INV-2" in scenario.invariants:
        checks.append(analyze.check_inv2(released, excited, transition + latency, exempt.get("INV-2", [])))
    if "INV-3" in scenario.invariants:
        checks.append(analyze.check_inv3(released, pos_t, pos))
    for c in scenario.checks:
        t_from = label_t[c.from_label] + sc.resolve_delay(c.delay, transition, latency)
        t_to = label_t[c.to_label] if c.to_label else end
        sig = released if c.signal == "brake" else excited
        checks.append(analyze.check_constant(sig, t_from, t_to, c.value, c.id))
    for m in scenario.motion:
        t_from = label_t[m.from_label]
        if isinstance(m, sc.EngageAfterStop):
            checks.append(analyze.check_engage_after_stop(released, pos_t, pos, t_from, m.id))
            continue
        t_from += sc.resolve_delay(m.delay, transition, latency)
        t_to = (label_t[m.to_label] if m.to_label else end) - sc.resolve_delay(m.to_delay, transition, latency)
        checks.append(analyze.check_motion(pos_t, pos, t_from, t_to, m.moving, m.id))
    values = scenario_values(cfg)
    for r in scenario.replies:
        checks.append(check_reply(link, r, label_host_ns[r.from_label], sc.resolve_args(r.args, values)))
    failed = {c.id for c in checks if not c.passed}
    outcome, as_expected = sc.verdict(scenario, failed)
    res.update(outcome=outcome, as_expected=as_expected, failed=failed, checks=[c.to_dict() for c in checks])

    # Characterization: command -> brake output and -> brakeStatus write (P6) delays,
    # and every engaged + excited overlap.
    p6 = analyze.bit_edges(seg, probe[6]).rise
    delays = []
    for t, name, st in commands:
        if st.kind not in ("osc", "serial"):
            continue
        nxt = analyze.first_change_after(released, t)
        ex_nxt = analyze.first_change_after(excited, t)
        p6_after = p6[(p6 > t) & (p6 < t + 1.0)]
        delays.append({"step": name, "t_s": t,
                       "brake_change_ms": (nxt - t) * 1e3 if nxt is not None and nxt - t < 1.0 else None,
                       "excited_change_ms": (ex_nxt - t) * 1e3 if ex_nxt is not None and ex_nxt - t < 2.0 else None,
                       "first_p6_ms": (p6_after[0] - t) * 1e3 if p6_after.size else None})
    overlaps = analyze.intersect(released.intervals(False), excited.intervals(True))
    res["reply_loss"] = reply_loss(cfg, cap, tx, p1, req_idx, replies, "/getPosition")
    res["delays"] = delays
    res["overlaps_ms"] = [[round(a, 4), round((b - a) * 1e3, 2)] for a, b in overlaps]

    p4 = analyze.bit_edges(seg, probe[4]).rise
    plots.timing_diagram(out / f"{scenario.id}_timing.png", f"{scenario.id}: {scenario.title}",
                         [("brake released", released), ("excited", excited)],
                         [(t, name) for t, name, _ in commands], {"P4": p4, "P6": p6}, (pos_t, pos))
    events = [[round(t, 6), "cmd", name] for t, name, _ in commands]
    events += [[round(t, 6), "brake", "released" if v else "engaged"] for t, v in zip(released.times, released.values)]
    events += [[round(t, 6), "excited", int(v)] for t, v in zip(excited.times, excited.values)]
    write_csv(out / f"{scenario.id}_events.csv", ["t_s", "signal", "value"], sorted(events))
    return res


def check_reply(link, reply: sc.Reply, t_ns: int, args: tuple) -> analyze.CheckResult:
    """The board sends `reply.address` with these leading args within the window after the
    host time t_ns of the step (present), or does not (absent)."""
    t_to = t_ns + int(reply.window_s * 1e9)

    def matches(e) -> bool:
        if e.address != reply.address or len(e.args) < len(args):
            return False
        return all((a == b) if isinstance(a, str) else (isinstance(b, (int, float)) and float(a) == float(b))
                   for a, b in zip(args, e.args))

    hits = [e for e in link.entries("rx", t_ns, t_to) if matches(e)]
    ok = bool(hits) == reply.present
    want = f"{reply.address} {' '.join(str(a) for a in args)}".strip()
    if hits:
        detail = f"{want} received {(hits[0].t_ns - t_ns) / 1e6:.1f} ms after the step"
    else:
        detail = f"no {want} within {reply.window_s * 1e3:.0f} ms"
    return analyze.CheckResult(reply.id, ok, [] if ok else [(0.0, 0.0)],
                               detail + ("" if ok else f" (expected {'present' if reply.present else 'absent'})"))


def match_replies(cfg: Config, cap: Capture, tx: list, link, request: str, reply: str,
                  timeout_ns: int = 10**9) -> tuple[list[int], list]:
    """Pairs each request of the test axis with the first unused reply received after it
    (UDP keeps the order on this link). Returns (indexes into tx of the requests, the reply
    entry or None for each)."""
    axis = cfg.test_axis
    rx = sorted((e for e in link.entries("rx", cap.host_trigger_ns, cap.host_stop_ns + timeout_ns)
                 if e.address == reply and int(e.args[0]) == axis), key=lambda e: e.t_ns)
    idx = [i for i, e in enumerate(tx) if e.address == request and e.args and int(e.args[0]) == axis]
    replies: list = []
    j = 0
    for i in idx:
        while j < len(rx) and rx[j].t_ns < tx[i].t_ns:
            j += 1
        if j < len(rx) and rx[j].t_ns - tx[i].t_ns <= timeout_ns:
            replies.append(rx[j])
            j += 1
        else:
            replies.append(None)
    return idx, replies


def reply_loss(cfg: Config, cap: Capture, tx: list, p1: np.ndarray, idx: list[int], replies: list,
               request: str) -> dict:
    """Missing replies = requests - replies received. The board side is exact: a request
    with no P5 (UDP send) pulse between its P1 rise and the next P1 rise was not answered
    by the board (listed with its time). The rest were sent by the board and lost on the
    way (only their number is known: over Wi-Fi, replies cannot be tied to requests by
    host time)."""
    p5_rise = analyze.bit_edges(cap.digital, cfg.probe_dio[5]).rise
    no_reply = []
    for i in idx:
        a_from = float(p1[i])
        a_to = float(p1[i + 1]) if i + 1 < len(p1) else cap.digital.duration
        if not np.any((p5_rise >= a_from) & (p5_rise < a_to)):
            no_reply.append({"t_s": a_from})
    missing = sum(r is None for r in replies)
    return {"request": request, "requests": len(idx), "missing": missing,
            "board_no_reply": len(no_reply), "lost_in_transit": max(0, missing - len(no_reply)),
            "board_no_reply_events": no_reply,
            "positions_may_be_misaligned": missing > 0}


def cmd_brake(cfg: Config, args) -> int:
    chosen = [sc.get(args.scenario)] if args.scenario else list(sc.SCENARIOS)
    out = new_result_dir("brake" if not args.scenario else f"brake_{chosen[0].id}")
    motor_connected = bool(cfg.raw["motor"].get("motor_connected", False))
    if motor_connected:
        print("WARNING: motor.motor_connected = true. The excitation detection was set up without a "
              "motor; review [excitation] before trusting INV-1 / INV-2.")
    results = []
    with ExitStack() as stack:
        ad2 = stack.enter_context(open_ad2(cfg))
        diag = stack.enter_context(SerialDiag(cfg.serial_port))
        link = stack.enter_context(open_link(cfg))
        for scenario in chosen:
            print(f"running {scenario.id}: {scenario.title}")
            for attempt in range(3):
                res = run_scenario(cfg, scenario, ad2, diag, link, out)
                if not res.get("retryable"):
                    break
                print(f"  {scenario.id}: {res['error']} Running again.")
            res["retries_for_capture"] = attempt
            results.append(res)
        diag.reboot()
    link.save_log(out / "osc_log.jsonl")
    write_json(out / "summary.json", {"axis": cfg.test_axis, "motor_connected": motor_connected,
                                      "brake": cfg.section("brake"), "excitation": cfg.section("excitation"),
                                      "scenarios": results})
    rows = []
    for r in results:
        exp = "-" if r.get("as_expected") is None else ("yes" if r["as_expected"] else "NO")
        detail = r.get("error") or ", ".join(sorted(r.get("failed", []))) or "all checks passed"
        if r.get("reply_loss", {}).get("missing"):
            detail += f" ({r['reply_loss']['missing']} replies missing)"
        rows.append([r["id"], r["expect"], r["outcome"], exp, detail])
    print_table(["scenario", "expect", "outcome", "as expected", "failed checks"], rows)
    print(f"results: {out}")
    return 0 if all(r.get("as_expected") is not False and r["outcome"] not in ("ERROR", "ABORTED") for r in results) else 1


# --------------------------------------------------------------------------------------
# powerup (Task 5-4)
# --------------------------------------------------------------------------------------


def wait_rail(ad2: AD2, ch: int, below: float, hold: float = 1.0, timeout: float = 600.0) -> bool:
    """Waits until the Scope channel stays below `below` volts for `hold` seconds."""
    deadline = time.monotonic() + timeout
    since = None
    while time.monotonic() < deadline:
        cap = ad2.record(0.05, analog_channels=(ch,), analog_rate=1e5)
        if cap.analog[ch].size and float(np.max(cap.analog[ch])) < below:
            since = since or time.monotonic()
            if time.monotonic() - since >= hold:
                return True
        else:
            since = None
    return False


def cmd_powerup(cfg: Config, args) -> int:
    out = new_result_dir("powerup")
    p4 = cfg.probe_dio[4]
    brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
    v33 = int(cfg.ad2["scope"]["v33"])
    runs = []
    with open_ad2(cfg) as ad2:
        prompt(f"Scope {v33}+ must be on 3.3 V (J3-2), not on P0, and the STEP800 USB cable unplugged "
               "(USB powers the 3.3 V rail). Press Enter when ready: ")
        for i in range(args.repeat):
            print(f"[{i + 1}/{args.repeat}] Switch the 12 V supply OFF.", flush=True)
            # Arm only after the rail is down, so a bounce of the switch cannot trigger.
            if not wait_rail(ad2, v33, below=0.3):
                runs.append({"run": i + 1, "error": "3.3 V did not go below 0.3 V within 10 minutes"})
                break
            rec = ad2.start_recording(args.duration, digital_rate=1e6, sensible_mask=mask_of(brake_dios + [p4]),
                                      analog_channels=(v33,), analog_rate=1e5, analog_trigger=(v33, 1.65))
            print(f"[{i + 1}/{args.repeat}] Power is off, armed. Switch the 12 V supply ON (no time limit).",
                  flush=True)
            cap = rec.wait(timeout=600.0 + args.duration)
            if rec._thread is not None and rec._thread.is_alive():
                cap = rec.stop()
                runs.append({"run": i + 1, "error": "no 3.3 V rising edge within 10 minutes"})
                continue
            cap.save(out / f"powerup_{i + 1}.npz")
            runs.append(_analyze_powerup(i + 1, cap, brake_dios, p4, v33))
            print(f"[{i + 1}/{args.repeat}] recorded.", flush=True)
    write_json(out / "summary.json", {"runs": runs})
    rows = [[r["run"], r.get("first_p4_ms"), ",".join(str(m) for m in r.get("released_before_p4", [])) or "none",
             r.get("released_ms", ""), r.get("error", "")] for r in runs]
    print_table(["run", "first P4 [ms]", "motors released before P4", "released [ms]", "error"], rows)
    print(f"results: {out}")
    return 0


def _analyze_powerup(run: int, cap: Capture, brake_dios: list[int], p4: int, v33: int) -> dict:
    """Times are from the 3.3 V rise (1.65 V crossing found in the trace). The 74HC595
    output register is not cleared at power-up (only the shift register is, by
    SHIFTOUT_ENABLE / R3) and G is tied to GND, so the outputs show their power-up state
    until the first RCK edge (the latch line)."""
    if not cap.ok:
        return {"run": run, "error": "; ".join(cap.problems())}
    v = cap.analog[v33]
    low = np.flatnonzero(v < 0.3)
    start = low[-1] if low.size else 0
    t_rise = (start + int(np.argmax(v[start:] > 1.65))) / cap.analog_rate
    seg = cap.digital
    p4_rise = analyze.bit_edges(seg, p4).rise
    p4_rise = p4_rise[p4_rise >= t_rise]
    t_p4 = float(p4_rise[0]) if p4_rise.size else seg.duration
    released, outputs, spans = [], {}, []
    for motor, d in enumerate(brake_dios, start=1):
        e = analyze.bit_edges(seg, d)
        step = analyze.Step.from_edges(e)
        high = analyze.intersect([(t_rise, t_p4)], step.intervals(True))
        if high:
            released.append(motor)
            spans += [(a - t_rise, b - t_rise) for a, b in high]
        outputs[motor] = {"high_after_rise_ms": [[round((a - t_rise) * 1e3, 2), round((b - t_rise) * 1e3, 2)] for a, b in high],
                          "edges_ms": [round((t - t_rise) * 1e3, 2) for t in e.all if t_rise <= t <= t_p4 + 0.05]}
    released_ms = (f"{min(a for a, _ in spans) * 1e3:.1f} to {max(b for _, b in spans) * 1e3:.1f}" if spans else "")
    return {"run": run, "rise_in_capture_ms": t_rise * 1e3,
            "first_p4_ms": (t_p4 - t_rise) * 1e3 if p4_rise.size else None,
            "released_before_p4": released, "released_ms": released_ms, "outputs": outputs}
