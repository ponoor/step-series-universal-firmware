"""Implementation of the hil.py subcommands: selftest, timing, brake, powerup, compare."""

from __future__ import annotations

import csv
import json
import math
import subprocess
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


def mask_of(dios) -> int:
    m = 0
    for d in dios:
        m |= 1 << int(d)
    return m


def open_ad2(cfg: Config) -> AD2:
    a = cfg.ad2
    return AD2(a.get("dwf_library", ""), a.get("serial_number", ""), float(a["scope"]["range_vpp"]))


def open_link(cfg: Config) -> OscLink:
    return OscLink(cfg.board_ip, cfg.board_in_port, cfg.board_out_port)


def connect_board(link: OscLink, timeout: float = 15.0) -> str:
    """/setDestIp with retries (the board may still be booting). Returns the local IP."""
    deadline = time.monotonic() + timeout
    while True:
        try:
            return link.set_dest_ip()
        except OscError:
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


def excitation_step(cfg: Config, cap: Capture) -> tuple[analyze.Step, np.ndarray, np.ndarray]:
    ex = cfg.section("excitation")
    ch = int(cfg.ad2["scope"]["out1a"])
    step, t_win, counts = analyze.excitation(cap.analog[ch], cap.analog_rate, float(ex["level_v"]),
                                             float(ex["hysteresis_v"]), float(ex["window_ms"]) / 1e3,
                                             int(ex["min_edges"]))
    shift = float(cfg.section("sync")["scope_minus_digital_s"])
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
    rec = ad2.start_recording(None, digital_rate=1e5, sensible_mask=mask_of(brake_dios + [p4]))
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
    axis = cfg.test_axis
    b = cfg.section("brake")
    ex = cfg.section("excitation")
    kval = int(b["kval"])
    link.send("/setKval", axis, kval, kval, kval, kval)
    time.sleep(0.1)
    ch = int(cfg.ad2["scope"]["out1a"])
    rec = ad2.start_recording(None, analog_channels=(ch,), analog_rate=float(b["analog_rate"]))
    time.sleep(0.3)
    t_stop = link.send("/hardStop", axis).t_ns
    time.sleep(1.2)
    t_hiz = link.send("/hardHiZ", axis).t_ns
    time.sleep(1.2)
    cap = rec.stop()
    check_capture(cap)
    cap.save(out / "excitation_capture.npz")
    v = cap.analog[ch]
    rate = cap.analog_rate
    window = float(ex["window_ms"]) / 1e3
    _, t_win, counts = analyze.excitation(v, rate, float(ex["level_v"]), float(ex["hysteresis_v"]), window,
                                          int(ex["min_edges"]))
    a, bnd = host_s(cap, t_stop), host_s(cap, t_hiz)
    end = len(v) / rate
    spans = {"excited": (a + 0.2, bnd - 0.05), "hiz": (bnd + 0.2, end - 0.05)}
    res: dict = {"kval_sent": [kval] * 4, "spans_s": spans}
    for name, (lo, hi) in spans.items():
        sel = (t_win >= lo) & (t_win + window <= hi)
        c = counts[sel]
        vs = v[int(lo * rate):int(hi * rate)]
        res[name] = {"edges_per_window": analyze.stats(c),
                     "volts": {"p1": float(np.percentile(vs, 1)), "p50": float(np.median(vs)),
                               "p99": float(np.percentile(vs, 99))}}
    exc_min = res["excited"]["edges_per_window"]["min"]
    hiz_max = res["hiz"]["edges_per_window"]["max"]
    res["suggested_min_edges"] = int(math.floor((exc_min + hiz_max) / 2)) + 1 if exc_min > hiz_max else None
    ok = exc_min >= int(ex["min_edges"]) > hiz_max
    plots.waveform(out / "excitation_waveform.png", f"OUT1A, KVAL {kval}", rate, v,
                   [("excited (/hardStop)", spans["excited"][0], spans["excited"][1]),
                    ("HiZ (/hardHiZ)", spans["hiz"][0], spans["hiz"][1])])
    plots.histograms({"edges/window excited": counts[(t_win >= spans["excited"][0]) & (t_win < spans["excited"][1])],
                      "edges/window HiZ": counts[(t_win >= spans["hiz"][0]) & (t_win < spans["hiz"][1])]},
                     out / "excitation_edges.png", unit="edges")
    res["ok"] = ok
    res["summary"] = (f"edges/{ex['window_ms']} ms: excited min {exc_min}, HiZ max {hiz_max}; "
                      f"min_edges={ex['min_edges']} (suggested {res['suggested_min_edges']})")
    return res


def _selftest_sync(cfg, ad2, diag, link, out, boot) -> dict:
    """Scope and Digital In start offset, with Scope 2 temporarily on P0."""
    input("Connect Scope 2+ to probe P0 (the DIO8 line) and 2- to GND, then press Enter: ")
    p0 = cfg.probe_dio[0]
    rate = 1e6
    rec = ad2.start_recording(0.3, digital_rate=rate, sensible_mask=mask_of([p0]), analog_channels=(2,),
                              analog_rate=rate)
    cap = rec.wait()
    input("Reconnect Scope 2+ to 3.3 V (J3-2), then press Enter: ")
    check_capture(cap)
    cap.save(out / "sync_capture.npz")
    t_dig = analyze.bit_edges(cap.digital, p0).all
    t_ana = analyze.comparator_edges(cap.analog[2], cap.analog_rate, 1.65, 0.6)
    d, n = analyze.estimate_offset(t_ana, t_dig, tolerance=3 / rate, max_offset=5e-3)
    scope_minus_digital = -d
    ok = n >= 0.9 * min(len(t_ana), len(t_dig)) and n > 10
    return {"ok": ok, "scope_minus_digital_s": scope_minus_digital, "matched_edges": n,
            "edges": [len(t_ana), len(t_dig)],
            "summary": f"scope - digital = {scope_minus_digital * 1e6:.2f} us ({n} edges matched)"}


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


def _run_steps(steps, values: dict, link: OscLink, ad2: AD2 | None, log: list | None) -> None:
    for st in steps:
        if st.kind == "osc":
            entry = link.send(st.address, *sc.resolve_args(st.args, values))
            if log is not None:
                log.append((st, entry.t_ns, entry))
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
    values = {"axis": axis, "kval": int(b["kval"]), "max_speed": float(b["max_speed"]),
              "transition_ms": int(b["transition_ms"])}
    transition = int(b["transition_ms"]) / 1e3
    latency = int(b["latency_allowance_ms"]) / 1e3

    diag.reboot()
    connect_board(link)
    _run_steps(sc.COMMON_SETUP + scenario.extra_setup, values, link, ad2, None)
    time.sleep(0.3)

    probe = cfg.probe_dio
    brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
    mask = mask_of(brake_dios + probe[1:] + [15])  # P0 left out: it toggles every loop
    out1a = int(cfg.ad2["scope"]["out1a"])
    rec = ad2.start_recording(None, digital_rate=float(b["digital_rate"]), sensible_mask=mask,
                              analog_channels=(out1a,), analog_rate=float(b["analog_rate"]))
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
        _run_steps(scenario.steps, values, link, ad2, log)
        time.sleep(0.3)
    finally:
        stop.set()
        if poller.is_alive():
            poller.join(1.0)
        ad2.release()
        time.sleep(0.1)
        cap = rec.stop()
    cap.save(out / f"{scenario.id}_capture.npz")
    return analyze_scenario(cfg, scenario, cap, link, log, transition, latency, out)


def analyze_scenario(cfg: Config, scenario: sc.Scenario, cap: Capture, link: OscLink, log: list,
                     transition: float, latency: float, out: Path) -> dict:
    res: dict = {"id": scenario.id, "title": scenario.title, "expect": scenario.expect,
                 "expected_failures": scenario.expected_failures}
    if not cap.ok:
        res.update(outcome="ERROR", error="; ".join(cap.problems()) + ". Lower the sample rate.")
        return res
    seg = cap.digital
    probe = cfg.probe_dio
    tx = tx_in_window(link, cap)
    p1 = analyze.bit_edges(seg, probe[1]).rise
    try:
        fit = analyze.match_commands(np.array([host_s(cap, e.t_ns) for e in tx]), p1)
    except analyze.MatchError as ex:
        res.update(outcome="ERROR", error=str(ex))
        return res
    res["clock_fit"] = {"slope": fit.slope, "offset_s": fit.offset, "residual_max_ms": fit.residual_max * 1e3}
    tx_index = {id(e): i for i, e in enumerate(tx)}

    # AD2 time of each logged step.
    label_t: dict[str, float] = {}
    commands = []
    for st, t_ns, entry in log:
        t = float(p1[tx_index[id(entry)]]) if entry is not None else float(fit.to_ad2(host_s(cap, t_ns)))
        if st.label:
            label_t[st.label] = t
        commands.append((t, st.label or st.address or st.kind, st))

    end = seg.duration
    released = analyze.Step.from_edges(analyze.bit_edges(seg, cfg.brake_dio))
    excited, _, _ = excitation_step(cfg, cap)
    axis = cfg.test_axis
    rx = [e for e in link.entries("rx", cap.host_trigger_ns, cap.host_stop_ns)
          if e.address == "/position" and int(e.args[0]) == axis]
    pos_t = np.array([float(fit.to_ad2(host_s(cap, e.t_ns))) for e in rx])
    pos = np.array([int(e.args[1]) for e in rx])

    exempt: dict[str, list] = {}
    for ex in scenario.exempt:
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
    failed = {c.id for c in checks if not c.passed}
    outcome, as_expected = sc.verdict(scenario, failed)
    res.update(outcome=outcome, as_expected=as_expected, failed=failed, checks=[c.to_dict() for c in checks])

    # Characterization: command -> brake output and -> brakeStatus write (P6) delays,
    # and every engaged + excited overlap.
    p6 = analyze.bit_edges(seg, probe[6]).rise
    delays = []
    for t, name, st in commands:
        if st.kind != "osc":
            continue
        nxt = analyze.first_change_after(released, t)
        p6_after = p6[(p6 > t) & (p6 < t + 1.0)]
        delays.append({"step": name, "t_s": t,
                       "brake_change_ms": (nxt - t) * 1e3 if nxt is not None and nxt - t < 1.0 else None,
                       "first_p6_ms": (p6_after[0] - t) * 1e3 if p6_after.size else None})
    overlaps = analyze.intersect(released.intervals(False), excited.intervals(True))
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


def cmd_brake(cfg: Config, args) -> int:
    chosen = [sc.get(args.scenario)] if args.scenario else list(sc.SCENARIOS)
    out = new_result_dir("brake" if not args.scenario else f"brake_{chosen[0].id}")
    results = []
    with ExitStack() as stack:
        ad2 = stack.enter_context(open_ad2(cfg))
        diag = stack.enter_context(SerialDiag(cfg.serial_port))
        link = stack.enter_context(open_link(cfg))
        for scenario in chosen:
            print(f"running {scenario.id}: {scenario.title}")
            results.append(run_scenario(cfg, scenario, ad2, diag, link, out))
        diag.reboot()
    link.save_log(out / "osc_log.jsonl")
    write_json(out / "summary.json", {"axis": cfg.test_axis, "brake": cfg.section("brake"), "scenarios": results})
    rows = []
    for r in results:
        exp = "-" if r.get("as_expected") is None else ("yes" if r["as_expected"] else "NO")
        detail = r.get("error") or ", ".join(sorted(r.get("failed", []))) or "all checks passed"
        rows.append([r["id"], r["expect"], r["outcome"], exp, detail])
    print_table(["scenario", "expect", "outcome", "as expected", "failed checks"], rows)
    print(f"results: {out}")
    return 0 if all(r.get("as_expected") is not False and r["outcome"] != "ERROR" for r in results) else 1


# --------------------------------------------------------------------------------------
# powerup (Task 5-4)
# --------------------------------------------------------------------------------------


def cmd_powerup(cfg: Config, args) -> int:
    out = new_result_dir("powerup")
    p4 = cfg.probe_dio[4]
    brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
    v33 = int(cfg.ad2["scope"]["v33"])
    runs = []
    with open_ad2(cfg) as ad2:
        for i in range(args.repeat):
            input(f"[{i + 1}/{args.repeat}] Turn off the STEP800 power supply and unplug its USB cable "
                  "(USB can power the 3.3 V rail). Press Enter when it is off: ")
            rec = ad2.start_recording(args.duration, digital_rate=1e5, sensible_mask=mask_of(brake_dios + [p4]),
                                      analog_channels=(v33,), analog_rate=1e5, analog_trigger=(v33, 1.65))
            print("Armed. Turn on the STEP800 power supply now (USB stays unplugged).")
            cap = rec.wait(timeout=120.0 + args.duration)
            if rec._thread is not None and rec._thread.is_alive():
                cap = rec.stop()
                runs.append({"run": i + 1, "error": "no 3.3 V rising edge within 120 s"})
                continue
            cap.save(out / f"powerup_{i + 1}.npz")
            runs.append(_analyze_powerup(i + 1, cap, brake_dios, p4))
    write_json(out / "summary.json", {"runs": runs})
    rows = [[r["run"], r.get("first_p4_ms"), ",".join(str(m) for m in r.get("released_before_p4", [])) or "none",
             r.get("error", "")] for r in runs]
    print_table(["run", "first P4 [ms]", "motors released before P4", "error"], rows)
    print(f"results: {out}")
    return 0


def _analyze_powerup(run: int, cap: Capture, brake_dios: list[int], p4: int) -> dict:
    if not cap.ok:
        return {"run": run, "error": "; ".join(cap.problems())}
    seg = cap.digital
    p4_rise = analyze.bit_edges(seg, p4).rise
    t_p4 = float(p4_rise[0]) if p4_rise.size else seg.duration
    released = []
    for motor, d in enumerate(brake_dios, start=1):
        step = analyze.Step.from_edges(analyze.bit_edges(seg, d))
        if analyze.intersect([(0.0, t_p4)], step.intervals(True)):
            released.append(motor)
    return {"run": run, "first_p4_ms": t_p4 * 1e3 if p4_rise.size else None, "released_before_p4": released}
