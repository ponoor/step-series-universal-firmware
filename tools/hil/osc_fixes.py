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
from hil.commands import open_ad2, open_link, connect_board, preflight, firmware_version  # noqa: E402
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


# ---------------------------------------------------------------------------------
# section 2: write conditions of the driver registers (feature/driver-register-write-guard)
# ---------------------------------------------------------------------------------
def section2(b: Bench):
    n = b.n
    NOT_HIZ, NOT_STOPPED = "MotorNotHiZ", "MotorNotStopped"

    def only(t, motors, name):
        """The errors since t are exactly this error (name) for these motors (one each)."""
        return sorted(b.errors(t)) == sorted(("/error/command", [name, str(m)]) for m in motors)

    def var(section, key, m):
        return b.config_json()[section][key][m - 1]

    b.send("/reportError", 1)
    b.hiz()
    b.send("/setSpeedProfile", 255, 1000.0, 1000.0, 400.0)
    b.send("/setMinSpeed", 255, 0.0)
    b.send("/setMicrostepMode", 255, 7)
    b.send("/setHomeSwMode", 255, 1)

    # --- WH: STEP_MODE, CONFIG (SW_MODE), BEMF / decay parameters: energized and stopped -> rejected
    b.energize(1)
    ms0, hs0 = b.microstep(1), b.homesw_mode(1)
    t = b.send("/setMicrostepMode", 1, 3)
    b.case("s2 /setMicrostepMode 1 3 while energized: MotorNotHiZ only", only(t, [1], NOT_HIZ), b.errors(t))
    b.case("s2   register and saved variable unchanged", b.microstep(1) == ms0 and var("driverSettings", "stepMode", 1) == ms0,
           (b.microstep(1), var("driverSettings", "stepMode", 1), ms0))
    t = b.send("/setHomeSwMode", 1, 0)
    b.case("s2 /setHomeSwMode 1 0 while energized: MotorNotHiZ only", only(t, [1], NOT_HIZ), b.errors(t))
    b.case("s2   register and saved variable unchanged", b.homesw_mode(1) == hs0 and var("driverSettings", "homeSwMode", 1) == bool(hs0),
           (b.homesw_mode(1), var("driverSettings", "homeSwMode", 1), hs0))
    bemf0 = b.ask("/getBemfParam", 1, reply="/bemfParam")
    t = b.send("/setBemfParam", 1, 500, 10, 20, 30)
    b.case("s2 /setBemfParam 1 ... while energized: MotorNotHiZ only, nothing written",
           only(t, [1], NOT_HIZ) and b.ask("/getBemfParam", 1, reply="/bemfParam") == bemf0
           and var("voltageMode", "INT_SPEED", 1) == bemf0[0], (b.errors(t), b.ask("/getBemfParam", 1, reply="/bemfParam"), bemf0))
    if b.is400:
        dec0 = b.ask("/getDecayModeParam", 1, reply="/decayModeParam")
        t = b.send("/setDecayModeParam", 1, 5, 6, 7)
        b.case("s2 /setDecayModeParam 1 5 6 7 while energized: MotorNotHiZ only, unchanged",
               only(t, [1], NOT_HIZ) and b.ask("/getDecayModeParam", 1, reply="/decayModeParam") == dec0, b.errors(t))

        t2 = b.send("/setCurrentMode", 1)
        b.send("/setKval", 1, 40, 41, 42, 43)
        b.case("s2 /setCurrentMode 1 while energized: MotorNotHiZ, the mode stays voltage (/setKval still writes the registers)",
               only(t2, [1], NOT_HIZ) and b.kval(1) == [40, 41, 42, 43], (b.errors(t2), b.kval(1)))
    # while running: WH is rejected as well
    b.run_motor(1)
    t = b.send("/setMicrostepMode", 1, 3)
    b.case("s2 /setMicrostepMode 1 3 while running: MotorNotHiZ only", only(t, [1], NOT_HIZ), b.errors(t))
    b.send("/softStop", 1, wait=1.5)

    # --- HiZ: applied
    b.hiz(1)
    t = b.send("/setMicrostepMode", 1, 3)
    b.case("s2 /setMicrostepMode 1 3 while HiZ: applied, no error",
           not b.errors(t) and b.microstep(1) == 3 and var("driverSettings", "stepMode", 1) == 3, (b.errors(t), b.microstep(1)))
    t = b.send("/setHomeSwMode", 1, 0)
    b.case("s2 /setHomeSwMode 1 0 while HiZ: applied, no error",
           not b.errors(t) and b.homesw_mode(1) == 0 and var("driverSettings", "homeSwMode", 1) is False, (b.errors(t), b.homesw_mode(1)))
    t = b.send("/setBemfParam", 1, 500, 10, 20, 30)
    b.case("s2 /setBemfParam 1 500 10 20 30 while HiZ: applied", not b.errors(t) and b.ask("/getBemfParam", 1, reply="/bemfParam") == [500, 10, 20, 30],
           (b.errors(t), b.ask("/getBemfParam", 1, reply="/bemfParam")))
    b.send("/setMicrostepMode", 1, 7)
    b.send("/setHomeSwMode", 1, 1)

    # --- WS: ACC, DEC, MIN_SPEED, EL_POS while running -> rejected; stopped (even energized) -> applied
    b.send("/setSpeedProfile", 1, 1000.0, 1000.0, 400.0)
    b.send("/setMinSpeed", 1, 10.0)
    sp0, raw0 = b.sp(1), b.spraw(1)
    min0 = b.ask("/getMinSpeed", 1, reply="/minSpeed")
    b.run_motor(1)
    cases = [
        ("/setAcc 1 2000.0", ("/setAcc", 1, 2000.0)),
        ("/setDec 1 2000.0", ("/setDec", 1, 2000.0)),
        ("/setMinSpeed 1 50.0", ("/setMinSpeed", 1, 50.0)),
        ("/setSpeedProfile 1 2000 2000 500", ("/setSpeedProfile", 1, 2000.0, 2000.0, 500.0)),
        ("/setAccRaw 1 300", ("/setAccRaw", 1, 300)),
        ("/setDecRaw 1 300", ("/setDecRaw", 1, 300)),
        ("/setMinSpeedRaw 1 40", ("/setMinSpeedRaw", 1, 40)),
        ("/setSpeedProfileRaw 1 300 300 50", ("/setSpeedProfileRaw", 1, 300, 300, 50)),
        ("/enableLowSpeedOptimize 1 1", ("/enableLowSpeedOptimize", 1, 1)),
    ]
    for name, msg in cases:
        t = b.send(*msg)
        same = b.sp(1) == sp0 and b.spraw(1) == raw0 and b.ask("/getMinSpeed", 1, reply="/minSpeed") == min0
        want = NOT_STOPPED
        b.case(f"s2 {name} while running: {want} only, registers unchanged", only(t, [1], want) and same, (b.errors(t), same))
    # the variables did not change either (G)
    cj = b.config_json()
    b.case("s2   saved acc / dec / minSpeed unchanged by the rejected commands",
           abs(cj["speedProfile"]["acc"][0] - 1000.0) < 1e-3 and abs(cj["speedProfile"]["dec"][0] - 1000.0) < 1e-3
           and abs(cj["speedProfile"]["minSpeed"][0] - 10.0) < 1e-3 and cj["voltageMode"]["lowSpeedOptimizeEnable"][0] in (False, 0),
           {k: cj["speedProfile"][k][0] for k in ("acc", "dec", "minSpeed")})
    # WR registers are written while running
    t = b.send("/setMaxSpeed", 1, 300.0)
    b.case("s2 /setMaxSpeed 1 300 while running: applied (WR)", not b.errors(t) and abs(b.sp(1)[2] - 300.0) < 16, (b.errors(t), b.sp(1)))
    t = b.send("/setKval", 1, 30, 31, 32, 33)
    b.case("s2 /setKval 1 ... while running: applied (WR)", not b.errors(t) and b.kval(1) == [30, 31, 32, 33], (b.errors(t), b.kval(1)))
    t = b.send("/setStallThreshold", 1, 12)
    b.case("s2 /setStallThreshold 1 12 while running: no error", not b.errors(t), b.errors(t))
    b.send("/softStop", 1, wait=2.0)
    st = b.status(1)
    b.case("s2   the motor stopped (MOT_STATUS 0)", (st & 0x60) == 0, hex(st))
    # stopped but energized -> WS applied
    t = b.send("/setAcc", 1, 2000.0)
    b.case("s2 /setAcc 1 2000 stopped and energized: applied, no error", not b.errors(t) and abs(b.sp(1)[0] - 2000.0) < 10, (b.errors(t), b.sp(1)))
    t = b.send("/setSpeedProfile", 1, 1500.0, 1500.0, 400.0)
    b.case("s2 /setSpeedProfile 1 1500 1500 400 stopped: applied", not b.errors(t) and abs(b.sp(1)[1] - 1500.0) < 10, (b.errors(t), b.sp(1)))
    t = b.send("/setMinSpeed", 1, 20.0)
    b.case("s2 /setMinSpeed 1 20 stopped: applied", not b.errors(t) and abs(b.ask("/getMinSpeed", 1, reply="/minSpeed")[0] - 20.0) < 1.0, (b.errors(t),))
    t = b.send("/enableLowSpeedOptimize", 1, 1)
    lso = b.ask("/getLowSpeedOptimizeThreshold", 1, reply="/lowSpeedOptimizeThreshold")
    b.case("s2 /enableLowSpeedOptimize 1 1 stopped and energized: applied (LSPD_OPT)", not b.errors(t) and bool(lso[1]), (b.errors(t), lso))
    t = b.send("/enableLowSpeedOptimize", 1, 0)
    lso = b.ask("/getLowSpeedOptimizeThreshold", 1, reply="/lowSpeedOptimizeThreshold")
    b.case("s2 /enableLowSpeedOptimize 1 0 stopped: applied", not b.errors(t) and not lso[1], (b.errors(t), lso))
    t = b.send("/setAccRaw", 1, 200)
    b.case("s2 /setAccRaw 1 200 stopped: applied", not b.errors(t) and b.spraw(1)[0] == 200, (b.errors(t), b.spraw(1)))
    # HiZ after running: applied
    b.hiz(1)
    t = b.send("/setAcc", 1, 900.0)
    b.case("s2 /setAcc 1 900 while HiZ: applied", not b.errors(t) and abs(b.sp(1)[0] - 900.0) < 10, (b.errors(t), b.sp(1)))

    # --- 255: per motor
    b.hiz()
    b.send("/setMicrostepMode", 255, 7)
    b.energize(2)
    t = b.send("/setMicrostepMode", 255, 4)
    ms = [b.microstep(i) for i in range(1, n + 1)]
    want = [7 if i == 2 else 4 for i in range(1, n + 1)]
    b.case("s2 /setMicrostepMode 255 4 with motor 2 energized: MotorNotHiZ 2 only, the others applied", only(t, [2], NOT_HIZ) and ms == want, (b.errors(t), ms))
    b.run_motor(3)
    t = b.send("/setAcc", 255, 777.0)
    acc = [b.sp(i)[0] for i in range(1, n + 1)]
    ok = only(t, [3], NOT_STOPPED) and all(abs(a - 777.0) < 10 for i, a in enumerate(acc, 1) if i != 3) and abs(acc[2] - 777.0) > 10
    b.case("s2 /setAcc 255 777 with motor 3 running: MotorNotStopped 3 only, the others applied (motor 2 energized, stopped)", ok, (b.errors(t), acc))
    b.send("/hardHiZ", 3)
    # --- all or nothing for a motor: /setSpeedProfile on a running motor writes none of acc, dec, max
    b.run_motor(4)
    sp4 = b.sp(4)
    t = b.send("/setSpeedProfile", 4, 3000.0, 3000.0, 100.0)
    b.case("s2 /setSpeedProfile on a running motor: none of acc / dec / max is written", only(t, [4], NOT_STOPPED) and b.sp(4) == sp4, (b.errors(t), b.sp(4), sp4))
    # --- the re-application after /resetMotorDriver (HiZ right after the reset) is not blocked
    b.hiz()
    b.send("/setMicrostepMode", 255, 5)
    b.send("/setSpeedProfile", 255, 1200.0, 1300.0, 350.0)
    b.send("/setBemfParam", 255, 600, 11, 22, 33)
    b.send("/setKval", 255, 50, 51, 52, 53)
    b.energize(2)
    t = b.send("/resetMotorDriver", 255, wait=0.8)
    spall = [b.sp(i) for i in range(1, n + 1)]
    ok = (not b.errors(t) and all(b.microstep(i) == 5 for i in range(1, n + 1)) and all(b.kval(i) == [50, 51, 52, 53] for i in range(1, n + 1))
          and all(abs(s[0] - 1200.0) < 10 and abs(s[1] - 1300.0) < 10 for s in spall)
          and all(b.ask("/getBemfParam", i, reply="/bemfParam") == [600, 11, 22, 33] for i in range(1, n + 1)))
    b.case("s2 /resetMotorDriver 255 (motor 2 energized): every setting re-applied, no error", ok, (b.errors(t), spall[:2]))
    b.hiz()
    b.send("/setSpeedProfile", 255, 1000.0, 1000.0, 400.0)
    b.send("/setMinSpeed", 255, 0.0)
    b.send("/setMicrostepMode", 255, 7)
    b.send("/setBemfParam", 255, 1032, 25, 41, 41)
    b.send("/setKval", 255, 0, 0, 0, 0)


# ---------------------------------------------------------------------------------
# section 3: the direction of /goTo, /goHome, /goMark (feature/goto-direction)
# ---------------------------------------------------------------------------------
def section3(b: Bench):
    """The home switch (DIO15 of the AD2) is pressed; motion towards homingDirection is prohibited.
    No motor is needed: the driver counts the steps."""
    ad2 = b.ad2
    m = 1
    HOME, ERR = "HomeSwActivating", "/error/command"

    def attempt(label, send, start, sign, target, allowed):
        """allowed: the command starts the motion (direction sign), moving to target when it is near,
        else HomeSwActivating and no motion."""
        b.hiz(m)
        b.send("/setPosition", m, start)
        t = b.send(*send, wait=0.3)
        errs = b.errors(t)
        if allowed and target is not None and abs(target - start) <= 3000:
            time.sleep(1.0)
            pos = b.position(m)
            ok = not errs and abs(pos - target) <= 2
        elif allowed:
            pos = b.position(m)
            ok = not errs and (pos - start) * sign > 0  # moving the right way
        else:
            time.sleep(0.2)
            pos = b.position(m)
            ok = errs == [(ERR, [HOME, str(m)])] and pos == start
        b.case(label + (": allowed" if allowed else ": HomeSwActivating"), ok,
               {"errors": errs, "position": pos, "start": start, "target": target})
        b.send("/softStop", m, wait=0.4)

    b.send("/reportError", 1)
    b.hiz()
    b.send("/setSpeedProfile", m, 20000.0, 20000.0, 15000.0)
    b.send("/setProhibitMotionOnHomeSw", m, 1)
    ad2.release()
    time.sleep(0.3)
    try:
        for homing_dir in (0, 1):  # 0 = REV, 1 = FWD
            b.send("/setHomingDirection", m, homing_dir)
            ad2.press("home")
            time.sleep(0.4)
            sw = b.ask("/getHomeSw", m, reply="/homeSw")
            b.case(f"s3 homingDirection {homing_dir}: the pressed home switch reads as active", sw[0] == 1, sw)
            toward = +1 if homing_dir == 1 else -1  # sign of the motion towards the switch
            for dist in (100, 2047, 2049, 100000, 2097151):
                for sign, name in ((toward, "towards the switch"), (-toward, "away from the switch")):
                    target = sign * dist
                    attempt(f"s3 homingDirection {homing_dir}: /goTo {target:+d} from 0 ({name})",
                            ("/goTo", m, target), 0, sign, target, sign != toward)
            # the shortest way wraps: from 0, +3,000,000 is -1,194,304 in 22 bits
            for target, sign in ((3000000, -1), (-3000000, +1)):
                attempt(f"s3 homingDirection {homing_dir}: /goTo {target:+d} from 0 (the shortest way is {'REV' if sign < 0 else 'FWD'})",
                        ("/goTo", m, target), 0, sign, None, sign != toward)
            # /goHome: from +200 the way is REV, from -200 FWD
            for start in (200, -200):
                sign = -1 if start > 0 else +1
                attempt(f"s3 homingDirection {homing_dir}: /goHome from {start:+d} ({'REV' if sign < 0 else 'FWD'})",
                        ("/goHome", m), start, sign, 0, sign != toward)
            # /goMark: mark at +300, from 0 the way is FWD; mark at -300 REV
            for mark in (300, -300):
                sign = +1 if mark > 0 else -1
                b.send("/setMark", m, mark)
                attempt(f"s3 homingDirection {homing_dir}: /goMark (mark {mark:+d}) from 0 ({'FWD' if sign > 0 else 'REV'})",
                        ("/goMark", m), 0, sign, mark, sign != toward)
            ad2.release()
            time.sleep(0.3)
        # the switch released: everything is allowed
        b.send("/setHomingDirection", m, 0)
        attempt("s3 switch released: /goTo -100 from 0", ("/goTo", m, -100), 0, -1, -100, True)
    finally:
        ad2.release()
        b.hiz()
        b.send("/setMark", m, 0)


SECTIONS = {"s1": section1, "s2": section2, "s3": section3}


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
    ad2_needed = "s3" in names
    from contextlib import ExitStack
    with ExitStack() as stack:
        link = stack.enter_context(open_link(cfg))
        connect_board(link)
        b = Bench(cfg, link)
        if ad2_needed:
            b.ad2 = stack.enter_context(open_ad2(cfg))
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
