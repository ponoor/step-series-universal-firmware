"""HIL checks of the OSC handler fixes (TASK_osc_fixes.md): network and USB serial only, no AD2.

    .venv/bin/python osc_fixes.py [--config bench.step400.toml] [--sections s1,s2,s3,s4] [--label x]

The board runs a HIL_PROBE or a plain build; no motor is needed (the driver counts steps
without one). Every check reads the state back with the get commands (and 'G' for the saved
variables). The motors 1 and 2 are used; the others are left alone except for 255 cases.
Results: results/<time>_<sha>_oscfix_<label>/summary.json, printed table.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

HIL = Path(__file__).resolve().parent
sys.path.insert(0, str(HIL))
from hil import config  # noqa: E402
from hil.commands import open_link, connect_board, preflight, firmware_version  # noqa: E402
from hil.diag import SerialDiag  # noqa: E402

SETTLE = 0.15


class Bench:
    def __init__(self, cfg, link):
        self.cfg, self.link = cfg, link
        self.n = cfg.board.num_motors
        self.is400 = cfg.board.name == "step400"
        self.results: list[dict] = []

    # --- recording ---
    def case(self, name: str, ok: bool, detail=None) -> bool:
        self.results.append({"name": name, "ok": bool(ok), "detail": detail})
        print(("PASS " if ok else "FAIL ") + name + ("" if ok else f"   {detail}"))
        return bool(ok)

    # --- OSC ---
    def send(self, addr, *args, wait=SETTLE):
        t = time.monotonic_ns()
        self.link.send(addr, *args)
        time.sleep(wait)
        return t

    def errors(self, t):
        return [(e.address, [str(a) for a in e.args]) for e in self.link.entries("rx", t) if e.address.startswith("/error")]

    def ask(self, addr, *args, reply, motor=None):
        m = args[0] if motor is None else motor
        e = self.link.request(addr, *args, reply=reply, predicate=lambda a: a[0] == m)
        return list(e.args)[1:]

    def kval(self, m): return self.ask("/getKval", m, reply="/kval")
    def tval(self, m): return self.ask("/getTval", m, reply="/tval")
    def sp(self, m): return self.ask("/getSpeedProfile", m, reply="/speedProfile")
    def spraw(self, m): return self.ask("/getSpeedProfileRaw", m, reply="/speedProfileRaw")
    def microstep(self, m): return self.ask("/getMicrostepMode", m, reply="/microstepMode")[0]
    def homesw_mode(self, m): return self.ask("/getHomeSwMode", m, reply="/homeSwMode")[0]
    def status(self, m): return self.ask("/getStatus", m, reply="/status")[0]
    def elpos(self, m): return self.ask("/getElPos", m, reply="/elPos")
    def position(self, m): return self.ask("/getPosition", m, reply="/position")[0]
    def servo_param(self, m): return self.ask("/getServoParam", m, reply="/servoParam")
    def timeout_go(self, m): return self.ask("/getGoUntilTimeout", m, reply="/goUntilTimeout")[0]
    def timeout_rel(self, m): return self.ask("/getReleaseSwTimeout", m, reply="/releaseSwTimeout")[0]

    def config_json(self) -> dict:
        with SerialDiag(self.cfg.serial_port) as d:
            text = d.command("G", idle=1.0, timeout=8)
        line = [l for l in text.splitlines() if l.startswith("{")][-1]
        return json.loads(line)

    # --- states ---
    def hiz(self, m=255):
        self.send("/hardHiZ", m, wait=0.3)

    def energize(self, m):
        """Holding current on (not HiZ), stopped."""
        self.send("/hardStop", m, wait=0.3)

    def run_motor(self, m, speed=800.0):
        self.send("/run", m, speed, wait=0.4)


def arrays_close(a, b, tol=0.5):
    return len(a) == len(b) and all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


# ---------------------------------------------------------------------------------
# section 1: argument validation (feature/osc-arg-validation)
# ---------------------------------------------------------------------------------
def section1(b: Bench):
    n = b.n
    b.send("/reportError", 1)
    b.hiz()
    # normal commands still work, without an error
    t = b.send("/setKval", 2, 11, 12, 13, 14)
    b.case("s1 /setKval complete: applied, no error", b.kval(2) == [11, 12, 13, 14] and not b.errors(t), (b.kval(2), b.errors(t)))

    # missing arguments: error, state unchanged
    t = b.send("/setKval", 2)
    b.case("s1 /setKval 2 (missing): MissingArgument once, KVAL unchanged",
           b.errors(t) == [("/error/osc", ["MissingArgument"])] and b.kval(2) == [11, 12, 13, 14], (b.errors(t), b.kval(2)))
    t = b.send("/setKval", 2, 1, 2, 3)
    b.case("s1 /setKval 2 1 2 3 (one missing): MissingArgument, unchanged",
           b.errors(t) == [("/error/osc", ["MissingArgument"])] and b.kval(2) == [11, 12, 13, 14], (b.errors(t), b.kval(2)))
    t = b.send("/setKval")
    b.case("s1 /setKval (no argument): MissingArgument", b.errors(t) == [("/error/osc", ["MissingArgument"])], b.errors(t))

    b.send("/setSpeedProfile", 2, 1000.0, 1000.0, 400.0)
    sp0 = b.sp(2)
    t = b.send("/setSpeedProfile", 2, 1.0)
    b.case("s1 /setSpeedProfile 2 1.0 (missing): MissingArgument, acc / dec / max unchanged",
           b.errors(t) == [("/error/osc", ["MissingArgument"])] and arrays_close(b.sp(2), sp0), (b.errors(t), b.sp(2), sp0))

    # wrong types
    t = b.send("/setKval", 2, "a", 5, 5, 5)
    b.case("s1 /setKval 2 'a' 5 5 5 (string): WrongDataType, unchanged",
           b.errors(t) == [("/error/osc", ["WrongDataType"])] and b.kval(2) == [11, 12, 13, 14], (b.errors(t), b.kval(2)))
    t = b.send("/setKval", "2", 5, 5, 5, 5)
    b.case("s1 /setKval '2' ... (motor ID as a string): WrongDataType, unchanged",
           b.errors(t) == [("/error/osc", ["WrongDataType"])] and b.kval(2) == [11, 12, 13, 14], (b.errors(t), b.kval(2)))
    t = b.send("/setMaxSpeed", 2, float("nan"))
    b.case("s1 /setMaxSpeed 2 NaN: WrongDataType, unchanged",
           b.errors(t) == [("/error/osc", ["WrongDataType"])] and arrays_close(b.sp(2), sp0), (b.errors(t), b.sp(2)))

    # motor ID out of the byte range
    k1 = b.kval(1)
    for mid in (257, 511, 256, -1, 65537):
        t = b.send("/setKval", mid, 99, 99, 99, 99)
        e = b.errors(t)
        ok = e == [("/error/command", ["MotorIdNotMatch", str(mid)])] and b.kval(1) == k1 and b.kval(2) == [11, 12, 13, 14]
        b.case(f"s1 /setKval {mid} 99 99 99 99: MotorIdNotMatch {mid}, no motor changed", ok, (e, b.kval(1)))
    t = b.send("/getKval", 261)
    b.case("s1 /getKval 261: MotorIdNotMatch, no reply", b.errors(t) == [("/error/command", ["MotorIdNotMatch", "261"])]
           and not [e for e in b.link.entries("rx", t) if e.address == "/kval"], b.errors(t))
    t = b.send("/setKval", 0, 1, 1, 1, 1)
    b.case("s1 /setKval 0: MotorIdNotMatch 0", b.errors(t) == [("/error/command", ["MotorIdNotMatch", "0"])], b.errors(t))
    t = b.send("/setKval", n + 1, 1, 1, 1, 1)
    b.case(f"s1 /setKval {n + 1}: MotorIdNotMatch", b.errors(t) == [("/error/command", ["MotorIdNotMatch", str(n + 1)])], b.errors(t))
    b.send("/setKval", 255, 21, 22, 23, 24)
    allk = [b.kval(i) for i in range(1, n + 1)]
    b.case("s1 /setKval 255: all motors", all(k == [21, 22, 23, 24] for k in allk), allk)

    # /setTargetPositionList with fewer than NUM_OF_MOTOR positions: nothing applied
    b.send("/setServoParam", 1, 0.5, 0.0, 0.0)
    b.send("/setPosition", 1, 1000)
    b.send("/enableServoMode", 1, 1, wait=0.4)
    p0 = b.position(1)
    t = b.send("/setTargetPositionList", *([0] * (n - 1)), wait=0.6)
    p1 = b.position(1)
    b.case("s1 /setTargetPositionList with one position missing: MissingArgument, the servo target stays",
           b.errors(t) == [("/error/osc", ["MissingArgument"])] and abs(p1 - p0) < 20, (b.errors(t), p0, p1))
    t = b.send("/setTargetPositionList", *([2000] + [0] * (n - 1)), wait=1.5)
    p2 = b.position(1)
    b.case("s1 /setTargetPositionList complete: the servo moves toward the target", p2 > p1 + 20 and not b.errors(t), (p1, p2, b.errors(t)))
    t = b.send("/setTargetPositionList", 0, "x", *([0] * (n - 2)))
    b.case("s1 /setTargetPositionList with a string: WrongDataType", b.errors(t) == [("/error/osc", ["WrongDataType"])], b.errors(t))
    b.send("/enableServoMode", 1, 0, wait=0.5)
    b.hiz(1)

    # /setConfigName
    cfgname0 = b.link.request("/getConfigName", reply="/configName").args[0]
    t = b.send("/setConfigName", 5)
    name1 = b.link.request("/getConfigName", reply="/configName").args[0]
    b.case("s1 /setConfigName 5 (not a string): WrongDataType, name unchanged",
           b.errors(t) == [("/error/osc", ["WrongDataType"])] and name1 == cfgname0, (b.errors(t), cfgname0, name1))
    t = b.send("/setConfigName")
    b.case("s1 /setConfigName (no argument): MissingArgument", b.errors(t) == [("/error/osc", ["MissingArgument"])], b.errors(t))
    b.send("/setConfigName", "oscfix")
    name2 = b.link.request("/getConfigName", reply="/configName").args[0]
    b.case("s1 /setConfigName 'oscfix': applied", name2 == "oscfix", name2)
    b.send("/setConfigName", cfgname0)

    # position report intervals: out of range rejected, a negative value disables
    t = b.send("/setPositionReportInterval", 1, 70000)
    b.case("s1 /setPositionReportInterval 1 70000: CommandIgnored", b.errors(t) == [("/error/command", ["CommandIgnored", "1"])], b.errors(t))
    t = b.send("/setPositionReportInterval", 1, 32768, wait=0.2)  # was a negative int16
    b.case("s1 /setPositionReportInterval 1 32768: accepted (it was read as -32768 and disabled)", not b.errors(t), b.errors(t))
    t = b.send("/setPositionReportInterval", 1, 50, wait=0.5)
    reports = [e for e in b.link.entries("rx", t) if e.address == "/position"]
    b.case("s1 /setPositionReportInterval 1 50: reports arrive", len(reports) >= 4, len(reports))
    b.send("/setPositionReportInterval", 1, -5, wait=0.1)
    t = b.send("/getBusy", 1, wait=0.5)
    reports = [e for e in b.link.entries("rx", t) if e.address == "/position"]
    b.case("s1 /setPositionReportInterval 1 -5: disables (kept)", len(reports) == 0, len(reports))
    t = b.send("/setPositionListReportInterval", 70000)
    b.case("s1 /setPositionListReportInterval 70000: CommandIgnored", b.errors(t) == [("/error/command", ["CommandIgnored", "255"])], b.errors(t))
    t = b.send("/setPositionListReportInterval", 50, wait=0.5)
    reports = [e for e in b.link.entries("rx", t) if e.address == "/positionList"]
    b.case("s1 /setPositionListReportInterval 50: reports arrive", len(reports) >= 4, len(reports))
    b.send("/setPositionListReportInterval", 0)
    b.send("/setPositionReportInterval", 255, 0)

    # timeouts: negative rejected
    b.send("/setGoUntilTimeout", 1, 4321)
    t = b.send("/setGoUntilTimeout", 1, -1)
    b.case("s1 /setGoUntilTimeout 1 -1: CommandIgnored, unchanged",
           b.errors(t) == [("/error/command", ["CommandIgnored", "1"])] and b.timeout_go(1) == 4321, (b.errors(t), b.timeout_go(1)))
    b.send("/setReleaseSwTimeout", 1, 8765)
    t = b.send("/setReleaseSwTimeout", 1, -100)
    b.case("s1 /setReleaseSwTimeout 1 -100: CommandIgnored, unchanged",
           b.errors(t) == [("/error/command", ["CommandIgnored", "1"])] and b.timeout_rel(1) == 8765, (b.errors(t), b.timeout_rel(1)))
    t = b.send("/setGoUntilTimeout", 255, -1)
    b.case("s1 /setGoUntilTimeout 255 -1: CommandIgnored 255", b.errors(t) == [("/error/command", ["CommandIgnored", "255"])], b.errors(t))

    # /setElPos
    b.hiz(1)
    b.send("/setElPos", 1, 1, 7)
    el0 = b.elpos(1)
    for args in ((4, 0), (-1, 0), (0, 128), (0, -3)):
        t = b.send("/setElPos", 1, *args)
        b.case(f"s1 /setElPos 1 {args[0]} {args[1]}: CommandIgnored, EL_POS unchanged",
               b.errors(t) == [("/error/command", ["CommandIgnored", "1"])] and b.elpos(1) == el0, (b.errors(t), b.elpos(1), el0))
    b.send("/setElPos", 1, 2, 5)
    b.case("s1 /setElPos 1 2 5: applied", b.elpos(1) == [2, 5], b.elpos(1))

    # /setServoParam
    b.send("/setServoParam", 1, 1.0, 2.0, 3.0)
    for args in ((-1.0, 0.0, 0.0), (1.0, float("nan"), 1.0), (1.0, 1.0, -0.001)):
        t = b.send("/setServoParam", 1, *args)
        e = b.errors(t)
        ok = e in ([("/error/command", ["CommandIgnored", "1"])], [("/error/osc", ["WrongDataType"])]) and arrays_close(b.servo_param(1), [1.0, 2.0, 3.0], 1e-3)
        b.case(f"s1 /setServoParam 1 {args}: rejected, unchanged", ok, (e, b.servo_param(1)))
    b.send("/setServoParam", 1, 0.0, 0.0, 0.0)
    b.case("s1 /setServoParam 1 0 0 0: accepted", arrays_close(b.servo_param(1), [0, 0, 0], 1e-3), b.servo_param(1))

    # slow-path pattern addresses still validate
    t = b.send("/enable*Report", 1)
    errs = b.errors(t)
    b.case("s1 /enable*Report 1 (pattern, one argument): every matching handler reports MissingArgument",
           len(errs) >= 8 and all(e == ("/error/osc", ["MissingArgument"]) for e in errs), len(errs))
    # restore
    b.send("/setKval", 255, 0, 0, 0, 0)


SECTIONS = {"s1": section1}


def git_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=HIL, text=True).strip()
    except Exception:
        return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--sections", default="s1")
    ap.add_argument("--label", default="")
    a = ap.parse_args()
    cfg = config.load(a.config)
    pre = preflight(cfg)
    names = [s.strip() for s in a.sections.split(",") if s.strip()]
    with open_link(cfg) as link:
        connect_board(link)
        b = Bench(cfg, link)
        version = firmware_version(link)
        print("board", cfg.board.name, "firmware", version)
        for s in names:
            SECTIONS[s](b)
        out = HIL / "results" / f"{time.strftime('%Y%m%d-%H%M%S')}_{git_sha()}_oscfix{('_' + a.label) if a.label else ''}"
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps({"board": cfg.board.name, "firmware": version, "preflight": pre,
                                                       "sections": names, "results": b.results}, indent=1, default=str))
        link.save_log(out / "osc_log.jsonl")
    fails = [r for r in b.results if not r["ok"]]
    print(f"\n{len(b.results) - len(fails)}/{len(b.results)} passed; results in {out}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
