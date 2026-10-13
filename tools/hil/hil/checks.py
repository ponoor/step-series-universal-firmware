"""hil.py check: the remaining bench checks C1-C11 of the phase 1 / brake fix / 5 ms
slot changes (pattern matching, /getMinSpeed, /goUntil errors, packed transfers, the USB
serial diagnosis, an unreachable destination IP, the reboot from the config tool, the
first /setDestIp after a restart, the reset cause, the DIP switch and the SD save).

Every check reboots the board first and records the USB serial output for its whole
duration (serial_log.txt; every line with "Warning:" goes to summary.json). Checks that
need a human (power cycle, reset button, DIP switch, SD card) print a line starting with
"OPERATOR:" after the recording has started, then wait for Enter on a terminal, or, without
a terminal, for the effect of the operation (a /booted message, a new ID, ...).

Verdicts: PASS / FAIL, or RECORD (recorded only, no expectation to judge).
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import analyze
from .config import NUM_MOTORS, Config
from .diag import DiagError, find_port, parse_perf

PASS, FAIL, RECORD = "PASS", "FAIL", "RECORD"

ABS_POS_BITS = 22
ABS_POS_MIN = -(1 << (ABS_POS_BITS - 1))
ABS_POS_MAX = (1 << (ABS_POS_BITS - 1)) - 1

# alarmAndReport keys of the 'G' JSON set by the ten STEP800 routes matching
# /enable*Report (src/oscRoutes.h; /enableLimitSwReport exists on the STEP400 only).
REPORT_KEYS = ("reportBUSY", "reportHiZ", "reportHomeSwStatus", "reportDir", "reportMotorStatus",
               "reportSwEvn", "reportUVLO", "reportThermalStatus", "reportOCD", "reportStall")

# Expected results of other address patterns (src/OSCMatch.c, CNMAT semantics; dispatch
# order of src/oscRoutes.h): (pattern, replies in order, slow path count, not matched count).
# A received address followed by "/..." matches the route it starts with (oscRouteMatches).
PATTERN_CASES = (
    ("/getPositio?", ("/position",), 1, 0),
    ("/get[PD]osition", ("/position",), 1, 0),
    ("/{getPosition,getDir}", ("/position", "/dir"), 1, 0),
    ("/getPosition/extra", ("/position",), 1, 0),
    ("/noSuchCommand", ("/error/osc",), 0, 1),
)

HOMING_UNDEFINED, HOMING_GOUNTIL, HOMING_RELEASESW, HOMING_COMPLETED, HOMING_TIMEOUT = range(5)
MIN_SPEED_LSB = 2 ** -24 / 250e-9  # steps/s per MIN_SPEED LSB (L6470 datasheet), about 0.238
SPEED_RAW_PER_STEP_S = 2 ** 28 * 250e-9  # SPEED register units per step/s, about 67.1

# --------------------------------------------------------------------------------------
# Pure helpers (tested with synthetic data in tests/test_checks.py)
# --------------------------------------------------------------------------------------


def wrap22(x: int) -> int:
    """Value of the 22-bit ABS_POS register (two's complement) for x."""
    return ((int(x) - ABS_POS_MIN) % (1 << ABS_POS_BITS)) + ABS_POS_MIN


def servo_error(target: int, current: int) -> int:
    """Position error of the servo mode as computeServoSpeed() (src/main.cpp) computes it:
    target - current wrapped into -2^21 .. 2^21-1, so the motor takes the short way."""
    error = int(target) - int(current)
    if error > 0x1FFFFF:
        error -= 0x400000
    elif error < -0x200000:
        error += 0x400000
    return error


def osc_type_tag(value) -> str:
    """OSC type tag of an argument as python-osc decodes it: i -> int, f -> float,
    T / F -> bool, s -> str."""
    if isinstance(value, bool):
        return "T" if value else "F"
    if isinstance(value, int):
        return "i"
    if isinstance(value, float):
        return "f"
    if isinstance(value, str):
        return "s"
    return type(value).__name__


def json_line(text: str, prefix: str) -> dict | None:
    """The first line of text that starts with prefix and parses as JSON."""
    for line in text.splitlines():
        line = line.strip()
        if line.startswith(prefix):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return None


def warning_lines(text: str) -> list[str]:
    return [ln.strip() for ln in text.splitlines() if "Warning:" in ln]


def report_flags(config: dict, value: bool) -> dict[str, list[int]]:
    """Report flags of the 'G' JSON that differ from value: {key: [motor, ...]}."""
    ar = config.get("alarmAndReport", {})
    out = {}
    for key in REPORT_KEYS:
        flags = ar.get(key)
        if not isinstance(flags, list) or len(flags) != NUM_MOTORS:
            out[key] = ["missing"]
            continue
        bad = [m for m, v in enumerate(flags, start=1) if bool(v) != value]
        if bad:
            out[key] = bad
    return out


def json_diff(a, b, path: str = "", ignore: tuple[str, ...] = ()) -> list[str]:
    """Paths where two JSON values differ (keys named in ignore are skipped)."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) | set(b)):
            if k in ignore:
                continue
            if k not in a or k not in b:
                out.append(f"{path}/{k}")
            else:
                out += json_diff(a[k], b[k], f"{path}/{k}", ignore)
        return out
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += json_diff(x, y, f"{path}[{i}]", ignore)
        return out
    return [] if a == b else [path or "/"]


def longest_gap(times: np.ndarray, t_from: float = -np.inf, t_to: float = np.inf) -> tuple[float, float]:
    """Longest interval between consecutive edges within [t_from, t_to]: (length, start)."""
    t = np.asarray(times, float)
    t = t[(t >= t_from) & (t <= t_to)]
    if t.size < 2:
        return 0.0, float("nan")
    d = np.diff(t)
    k = int(np.argmax(d))
    return float(d[k]), float(t[k])


def long_pulses(edges: analyze.Edges, min_width: float) -> list[tuple[float, float]]:
    """High pulses at least min_width long: [(rise, width)]. A pulse still high at the end
    of the capture counts with the width up to the end."""
    out = []
    for r in edges.rise:
        f = edges.fall[edges.fall > r]
        width = (f[0] if f.size else edges.duration) - r
        if width >= min_width:
            out.append((float(r), float(width)))
    return out


def within(value: float | None, lo: float, hi: float) -> bool:
    return value is not None and lo <= value <= hi


# --- USB serial diagnosis 's' (C5) ---

# L6470 CONFIG fields, positions from the datasheet. The library (Ponoor_L6470Constants.h)
# masks OSC_SEL and EXT_CLK together (CONFIG_OSC_SEL 0x000F); the datasheet splits them
# into OSC_SEL [2:0] and EXT_CLK [3]. The other masks are those of the library.
L6470_CONFIG_FIELDS = {
    "OSC_SEL": 0x0007,
    "EXT_CLK": 0x0008,
    "SW_MODE": 0x0010,  # CONFIG_SW_MODE
    "EN_VSCOMP": 0x0020,  # CONFIG_EN_VSCOMP
    "OC_SD": 0x0080,  # CONFIG_OC_SD
    "POW_SR": 0x0300,  # CONFIG_POW_SR
    "F_PWM_DEC": 0x1C00,  # CONFIG_F_PWM_DEC
    "F_PWM_INT": 0xE000,  # CONFIG_F_PWM_INT
}
# STATUS bits (Ponoor_L6470Constants.h STATUS_*).
ST_HIZ, ST_BUSY, ST_SW_F, ST_DIR = 0x0001, 0x0002, 0x0004, 0x0010
ST_MOT_STATUS, ST_UVLO, ST_TH_WRN, ST_TH_SD = 0x0060, 0x0200, 0x0400, 0x0800
ST_OCD, ST_STEP_LOSS_A, ST_STEP_LOSS_B = 0x1000, 0x2000, 0x4000
MOT_STATUS_TEXT = ("Stopped", "Acceleration", "Deceleration", "Constant speed")
THERMAL_TEXT = ("Normal", "Warning", "Bridge shutdown", "Device shutdown")


def field_value(raw: int, mask: int) -> int:
    shift = (mask & -mask).bit_length() - 1
    return (raw & mask) >> shift


def expected_status_fields(status: int) -> dict[str, str]:
    """The texts 's' should print for a STATUS value, from the datasheet bit meanings."""
    yes = lambda b: "Yes" if b else "No"  # noqa: E731
    th = (~((status & (ST_TH_WRN | ST_TH_SD)) >> 10)) & 3  # both active low
    return {
        "High impedance state": yes(status & ST_HIZ),
        "BUSY": yes(not status & ST_BUSY),  # active low
        "Motor direction": "Forward" if status & ST_DIR else "Reverse",
        "Motor status": MOT_STATUS_TEXT[field_value(status, ST_MOT_STATUS)],
        "UVLO (Undervoltage lock out)": yes(not status & ST_UVLO),  # active low
        "Thermal status": THERMAL_TEXT[th],
        "OCD (Overcurrent detection)": yes(not status & ST_OCD),  # active low
        "Stalled": yes((status & (ST_STEP_LOSS_A | ST_STEP_LOSS_B)) != (ST_STEP_LOSS_A | ST_STEP_LOSS_B)),
        "SW_F": "1" if status & ST_SW_F else "0",
    }


_DRIVER_HEAD = re.compile(r"^Driver ID#(\d+)\s*$")
_HEX_LINE = re.compile(r"^\t(\w+): 0x([0-9A-Fa-f]+)(?: \((.*)\))?\s*$")
_SUB_LINE = re.compile(r"^\t  (\w+): (\d+)\s*$")
_BOOL_LINE = re.compile(r"^\t  (.+?) : (.+?)\s*$")
_SWF_LINE = re.compile(r"^\t  SW_F: (\d) -HOME sensor input (closed|open)\.\s*$")


def parse_status_dump(text: str) -> dict:
    """Parses the 's' output: {"drivers": {motor: {...}}, "dip_bin": str, "dip_dec": int}."""
    out: dict = {"drivers": {}}
    m = re.search(r"BIN : ([01]{4} [01]{4})", text)
    if m:
        out["dip_bin"] = m.group(1)
    m = re.search(r"DEC : (\d+)", text)
    if m:
        out["dip_dec"] = int(m.group(1))
    cur: dict | None = None
    section = None
    for line in text.splitlines():
        line = line.rstrip("\r")
        h = _DRIVER_HEAD.match(line)
        if h:
            cur = {"hex": {}, "hex_detail": {}, "config_fields": {}, "status_fields": {}}
            out["drivers"][int(h.group(1))] = cur
            section = None
            continue
        if cur is None:
            continue
        if line.startswith("----"):
            cur = None
            continue
        m = _HEX_LINE.match(line)
        if m:
            cur["hex"][m.group(1)] = int(m.group(2), 16)
            cur["hex_detail"][m.group(1)] = m.group(3)
            section = m.group(1)
            continue
        m = _SWF_LINE.match(line)
        if m:
            cur["status_fields"]["SW_F"] = m.group(1)
            cur["status_fields"]["SW_F_text"] = m.group(2)
            continue
        m = _SUB_LINE.match(line)
        if m and section == "CONFIG":
            cur["config_fields"][m.group(1)] = int(m.group(2))
            continue
        m = _BOOL_LINE.match(line)
        if m and section == "STATUS":
            cur["status_fields"][m.group(1)] = m.group(2)
    return out


def check_status_dump(dump: dict) -> list[str]:
    """Mismatches between the raw values and the fields printed by 's'."""
    bad = []
    if "dip_bin" in dump and "dip_dec" in dump:
        if int(dump["dip_bin"].replace(" ", ""), 2) != dump["dip_dec"]:
            bad.append(f"DIP BIN {dump['dip_bin']} != DEC {dump['dip_dec']}")
    for motor, d in sorted(dump["drivers"].items()):
        cfg = d["hex"].get("CONFIG")
        if cfg is None:
            bad.append(f"#{motor}: no CONFIG")
        else:
            for name, mask in L6470_CONFIG_FIELDS.items():
                shown = d["config_fields"].get(name)
                if shown != field_value(cfg, mask):
                    bad.append(f"#{motor} CONFIG 0x{cfg:04X} {name}: shown {shown}, expected {field_value(cfg, mask)}")
        st = d["hex"].get("STATUS")
        if st is None:
            bad.append(f"#{motor}: no STATUS")
        else:
            for name, exp in expected_status_fields(st).items():
                shown = d["status_fields"].get(name)
                if shown != exp:
                    bad.append(f"#{motor} STATUS 0x{st:04X} {name}: shown {shown!r}, expected {exp!r}")
            text = d["status_fields"].get("SW_F_text")
            if text != ("closed" if st & ST_SW_F else "open"):
                bad.append(f"#{motor} SW_F text {text!r}")
        el = d["hex"].get("EL_POS")
        detail = d["hex_detail"].get("EL_POS") or ""
        m = re.match(r"Fullstep:(\d+), Microstep:(\d+)", detail)
        # EL_POS: STEP [8:7], MICROSTEP [6:0] (datasheet)
        if el is None or not m or (int(m.group(1)), int(m.group(2))) != ((el >> 7) & 3, el & 0x7F):
            bad.append(f"#{motor} EL_POS 0x{el if el is not None else 0:X} shown {detail!r}")
        al = d["hex"].get("ALARM_EN")
        detail = (d["hex_detail"].get("ALARM_EN") or "").replace(" ", "")
        if al is None or detail != f"{al:08b}":
            bad.append(f"#{motor} ALARM_EN shown {d['hex_detail'].get('ALARM_EN')!r}")
    return bad


# --------------------------------------------------------------------------------------
# Result items
# --------------------------------------------------------------------------------------


@dataclass
class Item:
    id: str
    verdict: str
    detail: str
    data: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"id": self.id, "verdict": self.verdict, "detail": self.detail, "data": self.data}


def judged(item_id: str, ok: bool, detail: str, **data) -> Item:
    return Item(item_id, PASS if ok else FAIL, detail, data)


def recorded(item_id: str, detail: str, **data) -> Item:
    return Item(item_id, RECORD, detail, data)


# --------------------------------------------------------------------------------------
# USB serial with a log of everything received
# --------------------------------------------------------------------------------------


class SerialLog:
    """USB serial port of the board, read continuously by a background thread. Every
    received chunk is kept with its arrival time (time.monotonic_ns()), across reconnects
    (a restart re-enumerates the USB port)."""

    def __init__(self, port: str = ""):
        self._configured = port
        self.port = port
        self._ser = None
        self._chunks: list[tuple[int, str]] = []
        self._cond = threading.Condition()
        self._thread: threading.Thread | None = None
        self._running = False

    # --- lifecycle ---
    def open(self) -> "SerialLog":
        import serial

        self.port = self._configured or find_port()
        self._ser = serial.Serial(self.port, 115200, timeout=0.05)
        self._running = True
        self._thread = threading.Thread(target=self._read, name="serial-log", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # the port may be gone already
                pass
            self._ser = None

    def __enter__(self) -> "SerialLog":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    def _read(self) -> None:
        while self._running:
            try:
                data = self._ser.read(4096)
            except Exception:  # port gone (restart, power cycle)
                self._running = False
                self._mark("port lost")
                return
            if data:
                with self._cond:
                    self._chunks.append((time.monotonic_ns(), data.decode(errors="replace")))
                    self._cond.notify_all()

    def _mark(self, what: str) -> None:
        with self._cond:
            self._chunks.append((time.monotonic_ns(), f"\n[bench: {what}]\n"))
            self._cond.notify_all()

    @property
    def alive(self) -> bool:
        return self._running

    # --- text ---
    def text(self, since_ns: int = 0) -> str:
        with self._cond:
            return "".join(c for t, c in self._chunks if t >= since_ns)

    def wait_for(self, pattern: str, since_ns: int, timeout: float) -> re.Match | None:
        rx = re.compile(pattern)
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                m = rx.search("".join(c for t, c in self._chunks if t >= since_ns))
                if m:
                    return m
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(min(remaining, 0.1))

    def write(self, data: str) -> int:
        t = time.monotonic_ns()
        self._ser.write(data.encode())
        self._ser.flush()
        return t

    def command(self, data: str, until: str, timeout: float = 5.0) -> str:
        """Sends data and returns the text received after it once `until` matches."""
        t = self.write(data)
        m = self.wait_for(until, t, timeout)
        if m is None:
            raise DiagError(f"timeout waiting for {until!r} after {data[:20]!r}; got {self.text(t)[-300:]!r}")
        return self.text(t)

    # --- restart ---
    def reconnect(self, timeout: float = 15.0) -> None:
        """Waits for the USB port of a restarted board and reopens it."""
        self.close()
        self._mark("reconnecting")
        deadline = time.monotonic() + timeout
        time.sleep(0.5)
        while time.monotonic() < deadline:
            try:
                self.open()
                self._mark(f"reconnected to {self.port}")
                return
            except Exception:
                self._ser = None
                time.sleep(0.2)
        raise DiagError(f"USB serial port did not come back within {timeout} s")

    def wait_gone(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while self._running and time.monotonic() < deadline:
            time.sleep(0.02)
        return not self._running

    def save(self, path: Path, t0_ns: int) -> None:
        """Writes the log; each line is prefixed with the time (s) its first chunk arrived."""
        lines = []
        cur, t_line = "", None
        with self._cond:
            chunks = list(self._chunks)
        for t, c in chunks:
            for part in re.split(r"(\n)", c):
                if part == "\n":
                    lines.append(f"{((t_line or t) - t0_ns) / 1e9:10.3f}  {cur.rstrip(chr(13))}")
                    cur, t_line = "", None
                elif part:
                    if t_line is None:
                        t_line = t
                    cur += part
        if cur:
            lines.append(f"{((t_line or 0) - t0_ns) / 1e9:10.3f}  {cur}")
        path.write_text("\n".join(lines) + "\n")


# --------------------------------------------------------------------------------------
# Bench context
# --------------------------------------------------------------------------------------


class Bench:
    """AD2, logged USB serial and OSC link for one check."""

    def __init__(self, cfg: Config, out: Path, args):
        from . import commands

        self.cfg = cfg
        self.out = out
        self.args = args
        self.axis = cfg.test_axis
        self.other = self.axis % NUM_MOTORS + 1
        self.commands = commands
        self.t0_ns = time.monotonic_ns()
        self.ad2 = None
        self.ser = SerialLog(cfg.serial_port)
        self.link = None
        self.extra_links: list = []
        self.notes: list[str] = []

    def __enter__(self) -> "Bench":
        self.ad2 = self.commands.open_ad2(self.cfg).open()
        self.ser.open()
        self.link = self.commands.open_link(self.cfg).open()
        return self

    def __exit__(self, *exc) -> None:
        for link in [self.link] + self.extra_links:
            if link is not None:
                link.close()
        self.ser.close()
        if self.ad2 is not None:
            self.ad2.close()

    # --- board ---
    def reboot(self, connect: bool = True) -> None:
        """'R', waits for the port to come back and sends /setDestIp."""
        try:
            self.ser.write("R")
        except Exception:
            pass
        self.ser.wait_gone(3.0)
        self.ser.reconnect()
        if connect:
            self.connect()

    def connect(self, timeout: float = 15.0) -> str:
        return self.commands.connect_board(self.link, timeout)

    def perf(self) -> dict:
        text = self.ser.command("p", until=r"reportClockRealign\s+\d+")
        return parse_perf(text)

    def config_json(self) -> dict:
        text = self.ser.command("G", until=r"\{\"board\".*\}\r?\n")
        d = json_line(text, '{"board"')
        if d is None:
            raise DiagError("no config JSON from 'G'")
        return d

    def config_json_text(self) -> str:
        text = self.ser.command("G", until=r"\{\"board\".*\}\r?\n")
        for line in text.splitlines():
            if line.startswith('{"board"'):
                return line.strip()
        raise DiagError("no config JSON from 'G'")

    def status_json(self) -> dict:
        text = self.ser.command("T", until=r"\{\"board\".*\]\}\r?\n")
        d = json_line(text, '{"board"')
        if d is None:
            raise DiagError("no status JSON from 'T'")
        return d

    def send(self, address: str, *args):
        return self.link.send(address, *args)

    def request(self, address: str, *args, reply: str, predicate=None, timeout: float = 0.5):
        return self.link.request(address, *args, reply=reply, predicate=predicate, timeout=timeout)

    def position(self, motor: int | None = None) -> int:
        m = motor or self.axis
        return int(self.request("/getPosition", m, reply="/position", predicate=lambda a: int(a[0]) == m).args[1])

    def collect(self, address: str, since_ns: int, window: float = 0.3, predicate=None) -> list:
        """Messages with this address received within window seconds after since_ns."""
        time.sleep(max(0.0, since_ns / 1e9 + window - time.monotonic_ns() / 1e9))
        return [e for e in self.link.entries("rx", since_ns, since_ns + int(window * 1e9))
                if e.address == address and (predicate is None or predicate(e.args))]

    def all_motors(self, request: str, reply: str, window: float = 0.3) -> dict[int, tuple]:
        """Sends request 255 and returns {motor: args} of the replies within window."""
        t = self.send(request, 255).t_ns
        return {int(e.args[0]): tuple(e.args) for e in self.collect(reply, t, window)}

    def setup_excitation(self, motor: int | None = None) -> None:
        """KVAL of the excitation detection (as the brake scenarios)."""
        b = self.cfg.section("brake")
        kval = int(b["kval"])
        self.send("/setKval", motor or self.axis, kval, kval, kval, kval)

    # --- recording ---
    def start(self, digital: list[int], digital_rate: float, analog: tuple[int, ...] = (),
              analog_rate: float | None = None):
        kwargs = dict(digital_rate=digital_rate, sensible_mask=self.commands.mask_of(digital),
                      analog_channels=analog, analog_rate=analog_rate)
        try:
            return self.ad2.start_recording(None, **kwargs)
        except RuntimeError as ex:
            # Seen once after a power cycle of the board ("did not arm"): reopen the AD2.
            self.notes.append(f"AD2 reopened after: {ex}")
            self.ad2.close()
            time.sleep(1.0)
            self.ad2 = self.commands.open_ad2(self.cfg).open()
            return self.ad2.start_recording(None, **kwargs)

    def host_s(self, cap, t_ns: int) -> float:
        return (t_ns - cap.host_trigger_ns) / 1e9

    def save_capture(self, cap, name: str) -> None:
        cap.save(self.out / f"{name}.npz")

    def excited_fraction(self, cap, t_from: float, t_to: float) -> float:
        """Fraction of the excitation windows of OUT1A within [t_from, t_to] (AD2 s)
        classified as excited. The Scope and Digital In start within a few hundred us,
        far below the windows used here, so no alignment is applied."""
        step, t_win, _ = self.commands.excitation_step(self.cfg, cap, 0.0)
        sel = (t_win >= t_from) & (t_win + 1e-3 <= t_to)
        return float(np.mean(step.at(t_win[sel] + 5e-4))) if sel.any() else float("nan")

    # --- operator ---
    def operator(self, message: str, done=None, timeout: float = 300.0) -> bool:
        """Asks for a human operation. With a terminal, waits for Enter; otherwise polls
        done() (if given) until it returns True, up to timeout seconds."""
        print(f"OPERATOR: {message}", flush=True)
        if sys.stdin.isatty():
            input("Press Enter when done. ")
            return True if done is None else bool(done())
        if done is None:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if done():
                return True
            time.sleep(0.2)
        return False


# --------------------------------------------------------------------------------------
# C1: pattern matching
# --------------------------------------------------------------------------------------


def check_c1(b: Bench) -> list[Item]:
    items = []
    b.perf()  # resets the counters
    g0 = b.config_json()
    servo0 = [m["isServoMode"] for m in b.status_json()["motors"]]
    b.send("/enable*Report", 255, 0)
    time.sleep(0.3)
    g1 = b.config_json()
    b.send("/enable*Report", 255, 1)
    time.sleep(0.3)
    g2 = b.config_json()
    servo2 = [m["isServoMode"] for m in b.status_json()["motors"]]
    perf = b.perf()
    off = report_flags(g1, False)
    items.append(judged("C1-2", not off, "/enable*Report 255 0: all ten report flags false" if not off
                        else f"not false: {off}", not_false=off))
    on = report_flags(g2, True)
    others = json_diff(g0, g2, ignore=REPORT_KEYS)
    ok = not on and not others and servo0 == servo2
    items.append(judged("C1-3", ok, f"/enable*Report 255 1: flags not true {on or 'none'}; other config "
                        f"changes {others or 'none'}; servo mode {servo2}", not_true=on, other_changes=others))
    slow, nm = perf.get("oscSlowPath"), perf.get("oscNotMatched")
    items.append(judged("C1-4", slow == 2 and nm == 0, f"oscSlowPath {slow} (expected 2), oscNotMatched {nm} (expected 0)",
                        perf=perf))
    rows = []
    all_ok = True
    for pattern, replies, exp_slow, exp_nm in PATTERN_CASES:
        b.perf()
        t = b.send(pattern, b.axis).t_ns
        time.sleep(0.3)
        got = [e.address for e in b.link.entries("rx", t, t + int(0.3e9))]
        p = b.perf()
        row = {"pattern": f"{pattern} {b.axis}", "replies": got, "expected_replies": list(replies),
               "oscSlowPath": p.get("oscSlowPath"), "oscNotMatched": p.get("oscNotMatched"),
               "expected": [exp_slow, exp_nm]}
        row["as_expected"] = (got == list(replies) and p.get("oscSlowPath") == exp_slow
                              and p.get("oscNotMatched") == exp_nm)
        all_ok &= row["as_expected"]
        rows.append(row)
    items.append(recorded("C1-5", ("all as expected" if all_ok else "DIFFERS from the expectation: "
                                   + "; ".join(f"{r['pattern']} -> {r['replies']} slow {r['oscSlowPath']} "
                                               f"nm {r['oscNotMatched']}" for r in rows if not r["as_expected"])),
                          cases=rows))
    return items


# --------------------------------------------------------------------------------------
# C2: /getMinSpeed and boolean type tags
# --------------------------------------------------------------------------------------


def check_c2(b: Bench) -> list[Item]:
    items = []
    ax, ot = b.axis, b.other
    is_m = lambda m: (lambda a: int(a[0]) == m)  # noqa: E731

    def single(m):
        return float(b.request("/getMinSpeed", m, reply="/minSpeed", predicate=is_m(m)).args[1])

    def all_():
        return {m: float(a[1]) for m, a in b.all_motors("/getMinSpeed", "/minSpeed").items()}

    tol = MIN_SPEED_LSB
    b.send("/setMinSpeed", ax, 10.0)
    b.send("/setMinSpeed", ot, 5.0)
    time.sleep(0.1)
    s_ax, s_ot, a1 = single(ax), single(ot), all_()
    ok = (len(a1) == NUM_MOTORS and a1.get(ax) == s_ax and a1.get(ot) == s_ot
          and abs(s_ax - 10.0) <= tol and abs(s_ot - 5.0) <= tol)
    items.append(judged("C2-1", ok, f"set {ax}: 10.0 -> single {s_ax:.4f}, 255 {a1.get(ax)}; "
                        f"set {ot}: 5.0 -> single {s_ot:.4f}, 255 {a1.get(ot)}", all=a1))
    b.send("/setMinSpeed", 255, 20.0)
    time.sleep(0.1)
    a2 = all_()
    singles = {m: single(m) for m in (ax, ot)}
    vals = set(a2.values())
    ok = (len(a2) == NUM_MOTORS and len(vals) == 1 and abs(next(iter(vals)) - 20.0) <= tol
          and all(singles[m] == a2[m] for m in singles))
    items.append(judged("C2-2", ok, f"/setMinSpeed 255 20.0 -> 255 values {sorted(vals)}, singles {singles}", all=a2))
    b.send("/enableLowSpeedOptimize", ax, 1)
    time.sleep(0.1)
    s3, a3 = single(ax), all_()
    lso_on = b.request("/getLowSpeedOptimizeThreshold", ax, reply="/lowSpeedOptimizeThreshold", predicate=is_m(ax)).args
    unchanged = all(a3.get(m) == a2.get(m) for m in a2 if m != ax)
    ok = s3 == 0.0 and a3.get(ax) == 0.0 and unchanged
    items.append(judged("C2-3", ok, f"/enableLowSpeedOptimize {ax} 1 -> single {s3}, 255 {a3.get(ax)}, "
                        f"other motors unchanged: {unchanged}", all=a3))
    b.send("/enableLowSpeedOptimize", ax, 0)
    time.sleep(0.1)
    s4, a4 = single(ax), all_()
    lso_off = b.request("/getLowSpeedOptimizeThreshold", ax, reply="/lowSpeedOptimizeThreshold", predicate=is_m(ax)).args
    items.append(recorded("C2-3r", f"after /enableLowSpeedOptimize {ax} 0: single {s4:.4f}, 255 {a4.get(ax)} "
                          "(minSpeed setting 20.0 written back to MIN_SPEED)", all=a4))
    tags_on = [osc_type_tag(v) for v in lso_on]
    tags_off = [osc_type_tag(v) for v in lso_off]
    uvlo = b.all_motors("/getUvlo", "/uvlo")
    uvlo_tags = {m: osc_type_tag(a[1]) for m, a in uvlo.items()}
    ok = (len(tags_on) == 3 and tags_on[2] == "i" and lso_on[2] == 1 and tags_off[2] == "i" and lso_off[2] == 0
          and len(uvlo_tags) == NUM_MOTORS and set(uvlo_tags.values()) == {"i"})
    items.append(judged("C2-4", ok, f"/lowSpeedOptimizeThreshold tags {''.join(tags_on)} args {list(lso_on)} "
                        f"(enabled), {''.join(tags_off)} {list(lso_off)} (disabled); /uvlo arg 2 tags "
                        f"{sorted(set(uvlo_tags.values()))}", uvlo=uvlo_tags))
    return items


# --------------------------------------------------------------------------------------
# C3: /goUntil errors
# --------------------------------------------------------------------------------------


def check_c3(b: Bench) -> list[Item]:
    items = []
    ax = b.axis
    probe = b.cfg.probe_dio
    out1a = int(b.cfg.ad2["scope"]["out1a"])
    b.setup_excitation()
    b.send("/setKval", 255, 16, 16, 16, 16)
    time.sleep(0.2)
    rec = b.start([probe[1], probe[5], 15], 1e6, (out1a,), float(b.cfg.section("brake")["analog_rate"]))
    marks = []
    try:
        time.sleep(0.3)
        b.ad2.press()
        time.sleep(0.1)
        sw = int(b.request("/getHomeSw", ax, reply="/homeSw", predicate=lambda a: int(a[0]) == ax).args[1])
        items.append(judged("C3-1", sw == 1, f"/getHomeSw {ax} with DIO15 pressed: {sw}"))
        speed_raw = int(round(100.0 * SPEED_RAW_PER_STEP_S))
        cmds = [("/goUntil", 100.0), ("/goUntil", -100.0), ("/goUntilRaw", speed_raw), ("/goUntilRaw", -speed_raw)]
        for report in (1, 0):
            b.send("/reportError", report)
            time.sleep(0.05)
            for addr, speed in cmds:
                p0 = b.position()
                t = b.send(addr, ax, 0, speed).t_ns
                errs = b.collect("/error/command", t, 0.3)
                time.sleep(0.2)
                p1 = b.position()
                marks.append({"report": report, "cmd": f"{addr} {ax} 0 {speed}", "t_ns": t,
                              "errors": [list(e.args) for e in errs], "pos": [p0, p1]})
        b.send("/reportError", 1)
        # 4: released, /goUntil starts; pressed, it stops
        b.ad2.release()
        time.sleep(0.1)
        t = b.send("/goUntil", ax, 0, -100.0).t_ns
        time.sleep(0.1)
        run = [b.position() for _ in range(4)]
        t_press = time.monotonic_ns()
        b.ad2.press()
        time.sleep(0.5)
        stop = [b.position() for _ in range(4)]
        hs = int(b.request("/getHomingStatus", ax, reply="/homingStatus", predicate=lambda a: int(a[0]) == ax).args[1])
        b.send("/hardHiZ", ax)
        time.sleep(0.3)
        # 5: all motors, the test axis pressed
        before = {m: int(a[1]) for m, a in b.all_motors("/getPosition", "/position").items()}
        t5 = b.send("/goUntil", 255, 0, 100.0).t_ns
        errs5 = b.collect("/error/command", t5, 0.3)
        time.sleep(0.2)
        after = {m: int(a[1]) for m, a in b.all_motors("/getPosition", "/position").items()}
        b.send("/hardHiZ", 255)
        time.sleep(0.3)
    finally:
        b.ad2.release()
        time.sleep(0.1)
        cap = rec.stop()
    b.save_capture(cap, "c3_capture")
    exc = lambda t_ns: b.excited_fraction(cap, b.host_s(cap, t_ns) + 0.05, b.host_s(cap, t_ns) + 0.45)  # noqa: E731
    for mk in marks:
        mk["excited"] = exc(mk["t_ns"])
    rows = [m for m in marks if m["report"] == 1]
    ok = all(m["errors"] == [["HomeSwActivating", ax]] and m["pos"][0] == m["pos"][1] and m["excited"] == 0.0
             for m in rows)
    items.append(judged("C3-2", ok, "; ".join(f"{m['cmd']}: errors {m['errors']}, pos {m['pos']}, excited "
                                              f"{m['excited']:.2f}" for m in rows), runs=rows))
    rows = [m for m in marks if m["report"] == 0]
    ok = all(not m["errors"] and m["pos"][0] == m["pos"][1] and m["excited"] == 0.0 for m in rows)
    items.append(judged("C3-3", ok, "/reportError 0: " + "; ".join(
        f"{m['cmd']}: errors {m['errors']}, pos {m['pos']}, excited {m['excited']:.2f}" for m in rows), runs=rows))
    moving = len(set(run)) == len(run)
    stopped = len(set(stop)) == 1
    ex_run = exc(t)
    ok = moving and stopped and ex_run == 1.0 and hs == HOMING_COMPLETED
    items.append(judged("C3-4", ok, f"released: /goUntil {ax} 0 -100.0 positions {run} (excited {ex_run:.2f}); "
                        f"pressed: positions {stop} (ACT 0 resets ABS_POS at the switch), /homingStatus {hs}",
                        press_host_s=(t_press - b.t0_ns) / 1e9))
    moved = {m: after.get(m, 0) != before.get(m, 0) for m in before}
    errs = [list(e.args) for e in errs5]
    ok = (errs == [["HomeSwActivating", ax]] and not moved.get(ax, True)
          and all(moved.get(m) for m in range(1, NUM_MOTORS + 1) if m != ax) and exc(t5) == 0.0)
    items.append(judged("C3-5", ok, f"/goUntil 255 0 100.0: errors {errs}; moved {moved}; test axis excited "
                        f"{exc(t5):.2f}", before=before, after=after))
    return items


# --------------------------------------------------------------------------------------
# C4: packed transfers (warnings, homing, positions around +/-2^21, servo wrap)
# --------------------------------------------------------------------------------------


def scan_warnings(sha: str, results_dir: Path) -> dict:
    """'Warning:' lines in the serial logs of every check run with this git sha."""
    out = {}
    for d in sorted(results_dir.glob(f"*_{sha}_check_*")):
        log = d / "serial_log.txt"
        if log.exists():
            out[d.name] = warning_lines(log.read_text(errors="replace"))
    return out


def check_c4(b: Bench) -> list[Item]:
    items = []
    ax = b.axis
    is_ax = lambda a: int(a[0]) == ax  # noqa: E731
    b.setup_excitation()
    time.sleep(0.1)
    # --- 2. homing ---
    settings = {}
    for get, reply in (("/getHomingDirection", "/homingDirection"), ("/getHomingSpeed", "/homingSpeed"),
                       ("/getGoUntilTimeout", "/goUntilTimeout"), ("/getReleaseSwTimeout", "/releaseSwTimeout"),
                       ("/getHomeSwMode", "/homeSwMode")):
        settings[reply[1:]] = b.request(get, ax, reply=reply, predicate=is_ax).args[1]
    b.ad2.release()
    time.sleep(0.1)
    t = b.send("/homing", ax).t_ns
    p = [b.position()]
    deadline = time.monotonic() + 3.0
    while p[-1] == p[0] and time.monotonic() < deadline:
        time.sleep(0.05)
        p.append(b.position())
    started = p[-1] != p[0]
    t_press = time.monotonic_ns()
    b.ad2.press()
    rs = b.link.wait_for("/homingStatus", 1.5, t_press, lambda a: is_ax(a) and int(a[1]) == HOMING_RELEASESW)
    time.sleep(0.3)
    t_rel = time.monotonic_ns()
    b.ad2.release()
    done = b.link.wait_for("/homingStatus", 3.0, t_rel, lambda a: is_ax(a) and int(a[1]) == HOMING_COMPLETED)
    time.sleep(0.3)
    final = b.position()
    seq = [int(e.args[1]) for e in b.link.entries("rx", t) if e.address == "/homingStatus" and is_ax(e.args)]
    errs = [list(e.args) for e in b.link.entries("rx", t) if e.address == "/error/command"]
    ok = (started and seq == [HOMING_GOUNTIL, HOMING_RELEASESW, HOMING_COMPLETED] and rs is not None
          and done is not None and final == 0 and not errs)
    items.append(judged("C4-2", ok, f"settings {settings}; moving before the press: {started}; /homingStatus "
                        f"{seq} (expected [1, 2, 3]); position after completion {final} (expected 0: ReleaseSW "
                        f"with ACT 0 resets ABS_POS at the release); errors {errs}", settings=settings))
    b.send("/hardHiZ", ax)
    time.sleep(0.2)
    b.send("/setGoUntilTimeout", ax, 1000)
    time.sleep(0.05)
    t = b.send("/homing", ax).t_ns
    err = b.link.wait_for("/error/command", 2.0, t, lambda a: tuple(a) == ("GoUntilTimeout", ax))
    hs = b.link.wait_for("/homingStatus", 2.0, t, lambda a: is_ax(a) and int(a[1]) == HOMING_TIMEOUT)
    dt = (err.t_ns - t) / 1e6 if err else None
    ok = err is not None and hs is not None and within(dt, 1000, 1300)
    items.append(judged("C4-2t", ok, f"/setGoUntilTimeout {ax} 1000, /homing, switch not pressed: GoUntilTimeout "
                        f"after {fmt_ms(dt)} ms (host time), /homingStatus 4: {hs is not None}"))
    b.send("/hardHiZ", ax)
    time.sleep(0.2)
    # --- 3. positions around +/-2^21 ---
    b.send("/setMaxSpeed", ax, 50.0)
    rows = []
    for start, step in ((ABS_POS_MAX - 199, 50), (ABS_POS_MIN + 200, -50)):
        b.send("/setPosition", ax, start)
        time.sleep(0.1)
        for k in range(9):
            single = b.position()
            lst = b.request("/getPositionList", reply="/positionList").args
            rows.append({"expected": wrap22(start + step * k), "single": single, "list": int(lst[ax - 1])})
            if k < 8:
                b.send("/move", ax, step)
                time.sleep(0.4)
    b.send("/hardHiZ", ax)
    ok = all(r["single"] == r["list"] == r["expected"] for r in rows)
    items.append(judged("C4-3", ok, "stops every 50 microsteps from 2^21-200 up and from -2^21+200 down: " + (
        "single, list and expected equal at all 18 stops" if ok else
        "; ".join(f"exp {r['expected']} single {r['single']} list {r['list']}" for r in rows
                  if not r["single"] == r["list"] == r["expected"])), stops=rows))
    # --- 4. servo across the wrap ---
    runs = []
    aborted = None
    b.send("/setPosition", ax, ABS_POS_MAX - 51)
    time.sleep(0.1)
    b.send("/enableServoMode", ax, 1)
    time.sleep(0.2)
    for target in (ABS_POS_MIN + 52, ABS_POS_MAX - 51):
        start = b.position()
        expect_dir = 1 if servo_error(target, start) > 0 else -1
        b.send("/setTargetPosition", ax, target)
        trace = []
        t_end = time.monotonic() + 2.5
        while time.monotonic() < t_end:
            pos = b.position()
            trace.append(pos)
            if abs(servo_error(target, pos)) > abs(servo_error(target, start)) + 500:
                aborted = f"moving away from {target}: {pos}"
                break
            time.sleep(0.05)
        if aborted:
            b.send("/enableServoMode", ax, 0)
            b.send("/hardHiZ", ax)
            break
        steps = [servo_error(trace[i + 1], trace[i]) for i in range(len(trace) - 1)]
        wrong = [s for s in steps if s * expect_dir < 0]
        runs.append({"target": target, "start": start, "expected_error": servo_error(target, start),
                     "expected_dir": expect_dir, "final": trace[-1], "final_error": servo_error(target, trace[-1]),
                     "steps_against": len(wrong), "trace": trace})
    b.send("/enableServoMode", ax, 0)
    time.sleep(0.1)
    b.send("/hardHiZ", ax)
    ok = aborted is None and len(runs) == 2 and all(abs(r["final_error"]) <= 2 and r["steps_against"] == 0
                                                     for r in runs)
    items.append(judged("C4-4", ok, aborted or "; ".join(
        f"target {r['target']} from {r['start']}: expected error {r['expected_error']} (dir {r['expected_dir']:+d}), "
        f"final {r['final']} (error {r['final_error']}), steps against the direction {r['steps_against']}"
        for r in runs), runs=runs, expectation="computeServoSpeed() wraps target - current into -2^21..2^21-1: "
                                               "the motor takes the short way across the ABS_POS wrap"))
    # --- 1. warnings (all check runs of this firmware so far, this one included) ---
    items.append(Item("C4-1", RECORD, "judged at the end of the run from every serial log of this sha"))
    return items


def fmt_ms(v) -> str:
    return "-" if v is None else f"{v:.1f}"


# --------------------------------------------------------------------------------------
# C5: USB serial diagnosis 's' / 'c'
# --------------------------------------------------------------------------------------


def check_c5(b: Bench) -> list[Item]:
    items = []
    ax = b.axis
    s1 = b.ser.command("s", until=r"Homing status : .*\n", timeout=10)
    c1 = b.ser.command("c", until=r"kD : .*\n", timeout=10)
    (b.out / "s_before.txt").write_text(s1)
    (b.out / "c_before.txt").write_text(c1)
    d1 = parse_status_dump(s1)
    bad1 = check_status_dump(d1)
    n = len(d1["drivers"])
    items.append(judged("C5-1", n == NUM_MOTORS and not bad1, f"'s': {n} drivers, mismatches: {bad1 or 'none'}",
                        mismatches=bad1, configs={m: f"0x{d['hex'].get('CONFIG', 0):04X}" for m, d in d1['drivers'].items()}))
    old = d1["drivers"].get(ax, {}).get("config_fields", {}).get("SW_MODE")
    new = 0 if old else 1
    b.send("/setHomeSwMode", ax, new)
    b.setup_excitation()
    b.send("/move", ax, 1000)  # changes STATUS (BUSY, DIR, MOT_STATUS) and EL_POS
    time.sleep(0.05)
    s2 = b.ser.command("s", until=r"Homing status : .*\n", timeout=10)
    (b.out / "s_after.txt").write_text(s2)
    b.send("/hardHiZ", ax)
    d2 = parse_status_dump(s2)
    bad2 = check_status_dump(d2)
    shown = d2["drivers"].get(ax, {}).get("config_fields", {}).get("SW_MODE")
    raw = d2["drivers"].get(ax, {}).get("hex", {})
    ok = shown == new and not bad2
    items.append(judged("C5-2", ok, f"/setHomeSwMode {ax} {new}: SW_MODE shown {shown} (was {old}), CONFIG "
                        f"0x{raw.get('CONFIG', 0):04X}, STATUS 0x{raw.get('STATUS', 0):04X}; mismatches {bad2 or 'none'}",
                        mismatches=bad2))
    return items


# --------------------------------------------------------------------------------------
# C6: destination IP that does not answer ARP
# --------------------------------------------------------------------------------------


def address_unused(ip: str) -> tuple[bool, str]:
    """No answer to ping, and no ARP entry with a MAC address afterwards."""
    ping = subprocess.run(["ping", "-c", "2", "-t", "2", ip], capture_output=True, text=True)
    arp = subprocess.run(["arp", "-n", ip], capture_output=True, text=True).stdout.strip()
    unused = ping.returncode != 0 and ("incomplete" in arp or "no entry" in arp)
    return unused, f"ping rc {ping.returncode}; arp: {arp}"


def load_capture_stats(cfg: Config, cap, t_from: float = 0.0) -> dict:
    probe = cfg.probe_dio
    p5r, p5w = analyze.pulse_widths(analyze.bit_edges(cap.digital, probe[5]))
    sel = p5r >= t_from
    p0 = analyze.bit_edges(cap.digital, probe[0]).all
    gap, at = longest_gap(p0, t_from)
    early = long_pulses(analyze.bit_edges(cap.digital, probe[2]), 1e-3)
    return {"udp_send_us": analyze.stats(p5w[sel], 1e6), "loop_max_ms": gap * 1e3, "loop_max_at_s": at,
            "p2_long_pulses": early, "p1_count": int(np.sum(analyze.bit_edges(cap.digital, probe[1]).rise >= t_from))}


C6_LOADS = ("a", "b", "c", "d")
C6_LOAD_TEXT = {"a": "/getPosition axis at 2 Hz", "b": "/setPositionListReportInterval 20",
                "c": "/setPositionReportInterval 255 20", "d": "/getPosition 255 once"}


def run_c6_load(b: Bench, load: str, duration: float) -> dict:
    probe = b.cfg.probe_dio
    b.perf()
    rec = b.start(probe, 1e6)
    t_start = time.monotonic_ns()
    try:
        time.sleep(0.3)
        if load == "a":
            end = time.monotonic() + duration
            while time.monotonic() < end:
                b.send("/getPosition", b.axis)
                time.sleep(0.5)
        elif load == "b":
            b.send("/setPositionListReportInterval", 20)
            time.sleep(duration)
            b.send("/setPositionListReportInterval", 0)
        elif load == "c":
            b.send("/setPositionReportInterval", 255, 20)
            time.sleep(duration)
            b.send("/setPositionReportInterval", 255, 0)
        else:
            b.send("/getPosition", 255)
            time.sleep(3.0)
        time.sleep(1.5)  # the board may still be sending
    finally:
        cap = rec.stop()
    b.save_capture(cap, f"c6_{load}")
    res = load_capture_stats(b.cfg, cap)
    perf = b.perf()
    res["slotLate"] = perf.get("slotLate")
    res["perf_udp_send_max_us"] = (perf.get("udpSend") or {}).get("max_us")
    res["booted"] = [list(e.args) for e in b.link.entries("rx", t_start) if e.address == "/booted"]
    res["capture_ok"] = cap.ok
    return res


def check_c6(b: Bench) -> list[Item]:
    items = []
    dead = b.args.dead_ip
    cfgname = b.request("/getConfigName", reply="/configName").args
    if int(cfgname[1]) != 0:
        return [judged("C6-pre", False, f"sdInitializeSucceeded is {cfgname[1]}: remove the SD card first (stopped)")]
    unused, why = address_unused(dead)
    if not unused:
        return [judged("C6-pre", False, f"{dead} is not unused ({why}); choose another --dead-ip (stopped)")]
    items.append(recorded("C6-pre", f"no SD card (sdInitializeSucceeded 0); {dead}: {why}"))
    status0 = b.status_json()["board"]
    duration = float(b.args.duration or 10.0)
    normal = {load: run_c6_load(b, load, duration) for load in C6_LOADS}
    # destination: the dead address, through the config tool command '{' (no SD card: the
    # values are applied, the save fails with "No SD card")
    g_before = b.config_json()
    t = b.ser.write('{"network":{"destIp":[' + dead.replace(".", ",") + "]}}")
    m = b.ser.wait_for(r"\{\"result\".*\}\r?\n", t, 5.0)
    reply = m.group(0).strip() if m else None
    g_after = b.config_json()
    changed = json_diff(g_before, g_after)
    items.append(judged("C6-cfg", bool(changed) and all(c.startswith("/network/destIp[") for c in changed)
                        and g_after["network"]["destIp"] == [int(x) for x in dead.split(".")],
                        f"'{{' reply {reply}; config changes {changed}", reply=reply))
    dead_res = {load: run_c6_load(b, load, duration) for load in C6_LOADS}
    # back to this Mac
    local = b.connect(5.0)
    status1 = b.status_json()["board"]
    items.append(recorded("C6-restore", f"/setDestIp from {local}: reply received; resetCause "
                          f"{status0['resetCause']} -> {status1['resetCause']}"))
    rows = []
    for load in C6_LOADS:
        for dest, r in (("mac", normal[load]), ("dead", dead_res[load])):
            rows.append({"load": load, "dest": dest, "p5_max_ms": (r["udp_send_us"]["max"] or 0) / 1e3,
                         "p5_p50_ms": (r["udp_send_us"]["p50"] or 0) / 1e3, "p5_count": r["udp_send_us"]["count"],
                         "loop_max_ms": r["loop_max_ms"], "slotLate": r["slotLate"],
                         "early_warning": len(r["p2_long_pulses"]), "booted": r["booted"]})
    p5_dead_max = max(r["p5_max_ms"] for r in rows if r["dest"] == "dead")
    # Expected at most about 60 ms (RTR 20 ms x (RCR 2 + 1), the bound of utils.cpp), not much more.
    items.append(judged("C6-p5", p5_dead_max <= 70.0,
                        f"P5 width max with the dead destination {p5_dead_max:.2f} ms (bound about 60 ms = "
                        f"20 ms x 3)", rows=rows))
    ab = [r for r in rows if r["dest"] == "dead" and r["load"] in ("a", "b")]
    items.append(judged("C6-loop-ab", all(r["loop_max_ms"] <= 70.0 for r in ab),
                        "; ".join(f"{r['load']}: loop max {r['loop_max_ms']:.1f} ms, slotLate {r['slotLate']}" for r in ab)))
    cd = [r for r in rows if r["dest"] == "dead" and r["load"] in ("c", "d")]
    items.append(recorded("C6-loop-cd", "; ".join(f"{r['load']}: loop max {r['loop_max_ms']:.1f} ms, slotLate "
                                                  f"{r['slotLate']}, P5 count {r['p5_count']}" for r in cd)))
    no_reset = (all(r["early_warning"] == 0 and not r["booted"] for r in rows)
                and status1["resetCause"] == status0["resetCause"])
    items.append(judged("C6-reset", no_reset, f"P2 early warning pulses {[r['early_warning'] for r in rows]}, "
                        f"/booted {[r['booted'] for r in rows if r['booted']]}, resetCause "
                        f"{status0['resetCause']} -> {status1['resetCause']}"))
    items.append(recorded("C6-table", "\n" + "\n".join(
        f"  {r['load']} {r['dest']:4s} P5 n {r['p5_count']:4d} p50 {r['p5_p50_ms']:7.3f} max {r['p5_max_ms']:7.3f} ms, "
        f"loop max {r['loop_max_ms']:8.1f} ms, slotLate {r['slotLate']}" for r in rows)))
    return items


# --------------------------------------------------------------------------------------
# Restart helpers (C7-C9, C11)
# --------------------------------------------------------------------------------------

RESTART_RATE = 1e4  # USB re-enumeration stalls the AD2 stream at higher rates


def restart_recording(b: Bench):
    probe = b.cfg.probe_dio
    brakes = [int(d) for d in b.cfg.ad2["dio"]["brake"]]
    out1a = int(b.cfg.ad2["scope"]["out1a"])
    return b.start(brakes + probe, RESTART_RATE, (out1a,), RESTART_RATE)


def restart_events(b: Bench, cap) -> dict:
    """Times (AD2 s) of the driver standby entry (OUT1A jump), the last P0 edge before it,
    the brake output fall of the test axis before it, the resumption of P0, and the
    longest P0 gap of the running firmware afterwards."""
    from . import restart

    probe = b.cfg.probe_dio
    out1a = int(b.cfg.ad2["scope"]["out1a"])
    p0 = analyze.bit_edges(cap.digital, probe[0]).all
    t_sb = restart.standby_entry(cap.analog[out1a], cap.analog_rate)
    t_reset = restart.mcu_reset_time(p0)
    ref = t_sb if t_sb is not None else t_reset
    res = {"standby_s": t_sb, "mcu_reset_s": t_reset, "ref": "standby" if t_sb is not None else "p0"}
    if ref is None:
        return res
    res.update(restart_events_from(b, cap, ref))
    return res


def restart_events_from(b: Bench, cap, ref: float) -> dict:
    """restart_events() for a given reference time (AD2 s)."""
    probe = b.cfg.probe_dio
    p0 = analyze.bit_edges(cap.digital, probe[0]).all
    res: dict = {}
    before = p0[p0 <= ref + 1e-4]
    # safeSystemReset() sets P0 high just before the reset: an edge there (if P0 was low)
    # is the marker, not a loop pass. The last loop pass is the edge before it.
    if before.size >= 2 and ref - before[-1] < 1e-3:
        res["p0_reset_marker_ms"] = (before[-1] - ref) * 1e3
        before = before[:-1]
    res["last_p0_before_ms"] = (before[-1] - ref) * 1e3 if before.size else None
    brake = analyze.bit_edges(cap.digital, b.cfg.brake_dio)
    falls = brake.fall[brake.fall <= ref + 1e-4]
    res["brake_fall_before_ms"] = (falls[-1] - ref) * 1e3 if falls.size else None
    res["brake_edges_ms"] = sorted([round(float(t - ref) * 1e3, 1), 1] for t in brake.rise) + \
        sorted([round(float(t - ref) * 1e3, 1), 0] for t in brake.fall)
    after = p0[p0 > ref + 0.02]
    res["p0_resume_ms"] = (after[0] - ref) * 1e3 if after.size else None
    if after.size > 1:
        gap, at = longest_gap(after)
        res["p0_longest_gap_after_resume_ms"] = gap * 1e3
        res["p0_longest_gap_at_ms"] = (at - ref) * 1e3
    return res


def wait_booted(b: Bench, since_ns: int, timeout: float = 10.0):
    return b.link.wait_for("/booted", timeout, since_ns)


# --------------------------------------------------------------------------------------
# C7: reboot from the config tool ('R')
# --------------------------------------------------------------------------------------

C7_CASES = {
    "a": ("100 ms", {1: 100}),
    "b": ("1000 ms", {1: 1000}),
    "c": ("5000 ms (limited to 1000 ms)", {1: 5000}),
    "d": ("brake disabled", {}),
    "e": ("all motors, 100-1000 ms", {1: 100, 2: 300, 3: 1000, 4: 200, 5: 700, 6: 400, 7: 900, 8: 500}),
}
C7_EXPECTED_WAIT = {"a": 100, "b": 1000, "c": 1000, "d": 0, "e": 1000}


def c7_case(b: Bench, case: str, first: bool) -> dict:
    title, durations = C7_CASES[case]
    ax = b.axis
    if not first:  # run_test() rebooted before the first case
        b.reboot()
    b.setup_excitation()
    if case == "e":
        motors = {m: d for m, d in durations.items()}
    else:
        motors = {ax: d for d in durations.values()}
    for m, d in motors.items():
        b.send("/enableElectromagnetBrake", m, 1)
        b.send("/setBrakeTransitionDuration", m, d)
        b.send("/setKval", m, 16, 16, 16, 16)
    time.sleep(0.1)
    if motors:
        for m in motors:
            b.send("/activate", m, 1)
        time.sleep(min(max(motors.values()), 65535) / 1e3 + 0.5)
        released = {m: int(b.request("/getBrakeStatus", m, reply="/brakeStatus",
                                      predicate=lambda a, m=m: int(a[0]) == m).args[1]) for m in motors}
    else:
        b.send("/hardStop", ax)  # energized, so that the standby entry shows on OUT1A
        time.sleep(0.3)
        released = {}
    rec = restart_recording(b)
    try:
        time.sleep(0.3)
        t_w = b.ser.write("R")
        m = b.ser.wait_for(r"Rebooting\.\.\.", t_w, 3.0)
        t_reply = time.monotonic_ns() if m else None
        booted = wait_booted(b, t_w, 10.0)
        time.sleep(0.5)
    finally:
        cap = rec.stop()
    b.save_capture(cap, f"c7_{case}")
    b.ser.reconnect()
    b.connect()
    ev = restart_events(b, cap)
    reply_chunk = [t for t, c in b.ser._chunks if t >= t_w and "Rebooting" in c]
    reply_ms = (reply_chunk[0] - t_w) / 1e6 if reply_chunk else None
    status = b.status_json()["board"]
    exp = C7_EXPECTED_WAIT[case]
    wait = (-ev["brake_fall_before_ms"]) if ev.get("brake_fall_before_ms") is not None else None
    silent = -ev["last_p0_before_ms"] if ev.get("last_p0_before_ms") is not None else None
    if case == "d":
        wait_ok = wait is None and within(silent, -1.0, 6.0)
    else:
        wait_ok = within(wait, exp - 1.0, exp + 6.0)
    res = {"case": case, "title": title, "durations": durations, "brake_status_before": released,
           "reply_ms_host": reply_ms, "booted_ms_host": (booted.t_ns - t_w) / 1e6 if booted else None,
           "booted": list(booted.args) if booted else None, "brake_fall_to_standby_ms": wait,
           "p0_silent_before_standby_ms": silent, "events": ev, "resetCause_T": status["resetCause"],
           "resetCauseRaw_T": status["resetCauseRaw"], "capture_ok": cap.ok}
    res["ok"] = bool(within(reply_ms, 0, 100) and wait_ok and booted is not None and int(booted.args[1]) == 5
                     and cap.ok)
    return res


def check_c7(b: Bench) -> list[Item]:
    items = []
    first = True
    n = b.args.repeat or 1
    for case in (b.args.cases or "abcde"):
        for run in range(n):
            r = c7_case(b, case, first)
            first = False
            items.append(judged(
                f"C7-{case}" + (f"{run + 1}" if n > 1 else ""), r["ok"],
                f"{r['title']}: reply {fmt_ms(r['reply_ms_host'])} ms (host), brake fall -> standby "
                f"{fmt_ms(r['brake_fall_to_standby_ms'])} ms (expected {C7_EXPECTED_WAIT[case]}), last loop P0 edge "
                f"-> standby {fmt_ms(r['p0_silent_before_standby_ms'])} ms, R -> /booted {fmt_ms(r['booted_ms_host'])} "
                f"ms (host), /booted {r['booted']}", **{k: v for k, v in r.items() if k != "ok"}))
    return items


# --------------------------------------------------------------------------------------
# C8: /setDestIp right after a restart
# --------------------------------------------------------------------------------------


class DestIpSender:
    """Sends /setDestIp every period until a /destIp reply arrives (or stop()).

    after_silence: the board is still running when the sender starts (power cycle: with
    the USB cable unplugged first, the 12 V supply keeps it running). Replies are counted
    only after at least after_silence seconds of sends without a reply (the board was
    off); the first reply after that ends the sending."""

    def __init__(self, b: Bench, period: float = 0.05, timeout: float = 30.0, after_silence: float = 0.0):
        self.b, self.period, self.timeout, self.after_silence = b, period, timeout, after_silence
        self.sent: list[int] = []
        self.reply = None
        self.silent_from_ns: int | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "DestIpSender":
        self._t0 = time.monotonic_ns()
        self._thread.start()
        return self

    def _run(self) -> None:
        end = time.monotonic() + self.timeout
        last_reply_ns = self._t0
        armed = self.after_silence <= 0
        while not self._stop.is_set() and time.monotonic() < end:
            try:
                self.sent.append(self.b.send("/setDestIp").t_ns)
            except OSError:  # no route while the link is down
                pass
            e = self.b.link.wait_for("/destIp", self.period, last_reply_ns + 1)
            now = time.monotonic_ns()
            if e is not None:
                if armed:
                    self.reply = e
                    return
                last_reply_ns = e.t_ns
                time.sleep(max(0.0, self.period - (now - e.t_ns) / 1e9))
            elif not armed and (now - last_reply_ns) / 1e9 >= self.after_silence:
                armed = True
                self.silent_from_ns = last_reply_ns
                self.sent = []  # count the sends to the restarting board only

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(2.0)

    def join(self, timeout: float) -> None:
        self._thread.join(timeout)


def c8_analyze(b: Bench, cap, t_trigger_ns: int, sender_sent: list[int], reply, booted: list,
               power_cycle: bool = False) -> dict:
    probe = b.cfg.probe_dio
    ev = restart_events(b, cap)
    ref_name = ev.get("ref")
    ref = ev.get("standby_s") if ref_name == "standby" else ev.get("mcu_reset_s")
    if power_cycle:
        # No standby jump to rely on (the 12 V goes off too): the first P0 activity after
        # the longest gap (the bootloader drives P0 about 30 ms after the reset).
        p0 = analyze.bit_edges(cap.digital, probe[0]).all
        t_off = ev.get("mcu_reset_s")
        nxt = p0[p0 > t_off] if t_off is not None else np.zeros(0)
        ref, ref_name = (float(nxt[0]), "first P0 edge after power-on") if nxt.size else (None, "none")
        ev = {**restart_events_from(b, cap, ref), "power_off_last_p0_s": t_off} if ref is not None else ev
    res = {"events": ev, "ref": ref_name}
    if ref is None:
        res["error"] = "no restart found in the capture"
        return res
    p1 = analyze.bit_edges(cap.digital, probe[1]).rise
    p5r, p5w = analyze.pulse_widths(analyze.bit_edges(cap.digital, probe[5]))
    a1 = p1[p1 > ref]
    a5 = p5r[p5r > ref]
    host = lambda t_ns: (t_ns - cap.host_trigger_ns) / 1e9 - ref  # noqa: E731 (host time, approximate)
    res["first_p1_ms"] = (a1[0] - ref) * 1e3 if a1.size else None
    res["first_p5_ms"] = (a5[0] - ref) * 1e3 if a5.size else None
    res["p5_max_ms"] = float(p5w[p5r > ref].max() * 1e3) if a5.size else None
    res["first_destIp_ms_host"] = host(reply.t_ns) * 1e3 if reply else None
    res["booted_ms_host"] = [host(e.t_ns) * 1e3 for e in booted]
    res["booted"] = [list(e.args) for e in booted]
    if reply:
        n_sent = sum(1 for t in sender_sent if t <= reply.t_ns)
        res["setDestIp_sent_until_reply"] = n_sent
        t_reply_ad2 = host(reply.t_ns) + ref
        res["p1_until_reply"] = int(np.sum((a1 > ref) & (a1 <= t_reply_ad2)))
        sent_after_reset = [t for t in sender_sent if host(t) > 0 and t <= reply.t_ns]
        res["setDestIp_sent_after_reset_until_reply"] = len(sent_after_reset)
    res["p0_longest_gap_ms"] = ev.get("p0_longest_gap_after_resume_ms")
    return res


def c8_run(b: Bench, mode: str, run: int) -> dict:
    """mode a: R, /setDestIp every 50 ms; b: R, /setDestIp on /booted (again after 300 ms);
    c: power cycle by the operator, /setDestIp every 50 ms."""
    ax = b.axis
    b.setup_excitation()
    b.send("/hardStop", ax)  # energized: the standby entry shows on OUT1A
    time.sleep(0.3)
    rec = restart_recording(b)
    sender = None
    booted_list = []
    try:
        time.sleep(0.3)
        t_start = time.monotonic_ns()
        if mode == "a":
            b.ser.write("R")
            sender = DestIpSender(b).start()
            sender.join(20.0)
        elif mode == "b":
            b.ser.write("R")
            booted = wait_booted(b, t_start, 15.0)
            sender = DestIpSender(b, period=0.3, timeout=3.0)
            if booted is not None:
                sender.start()
                sender.join(5.0)
        else:
            sender = DestIpSender(b, timeout=600.0, after_silence=1.0).start()
            b.operator(f"C8c run {run}: turn off the board (USB cable AND 12 V motor supply), wait 10 s, "
                       "turn both on again", done=lambda: sender.reply is not None, timeout=600.0)
            sender.stop()
        time.sleep(1.5)
        booted_list = [e for e in b.link.entries("rx", t_start) if e.address == "/booted"]
    finally:
        if sender is not None:
            sender.stop()
        cap = rec.stop()
    b.save_capture(cap, f"c8{mode}_{run}")
    b.ser.reconnect(30.0)
    res = c8_analyze(b, cap, t_start, sender.sent if sender else [], sender.reply if sender else None, booted_list,
                     power_cycle=mode == "c")
    res["capture_ok"] = cap.ok
    res["setDestIp_sent_total"] = len(sender.sent) if sender else 0
    status = b.status_json()["board"]
    res["resetCause_T"], res["resetCauseRaw_T"] = status["resetCause"], status["resetCauseRaw"]
    # One restart only: one /booted, and its cause is the one 'T' reports afterwards. The
    # expected cause (5 after 'R'; 1 after a power cycle, judged as C9a) is a separate item.
    res["no_second_restart"] = len(booted_list) == 1 and int(booted_list[0].args[1]) == status["resetCause"]
    res["cause_as_expected"] = status["resetCause"] == (1 if mode == "c" else 5)
    if sender is None or sender.reply is None:
        b.connect()
    return res


def check_c8(b: Bench) -> list[Item]:
    items = []
    modes = b.args.cases or "ab"
    for mode in modes:
        n = b.args.repeat or (3 if mode == "c" else 10)
        runs = [c8_run(b, mode, i + 1) for i in range(n)]
        ok = all(r["no_second_restart"] for r in runs)
        causes = [(r.get("booted"), r["resetCause_T"], f"0x{r['resetCauseRaw_T']:02X}") for r in runs]
        items.append(judged(f"C8{mode}", ok, f"{n} runs, one /booted each (no second restart): {ok}; "
                            f"/booted, T resetCause, raw: {causes}", runs=runs))
        if mode == "c":
            items.append(judged("C9a", all(r["cause_as_expected"] for r in runs),
                                f"power-on (expected 1), from the C8c power cycles: {causes}"))
        items.append(recorded(f"C8{mode}-table", "\n" + "\n".join(
            f"  run {i + 1}: first P1 {fmt_ms(r.get('first_p1_ms'))} ms, first P5 {fmt_ms(r.get('first_p5_ms'))} ms, "
            f"/destIp {fmt_ms(r.get('first_destIp_ms_host'))} ms (host), /booted "
            f"{[round(x, 1) for x in r.get('booted_ms_host', [])]} ms (host), sent until reply "
            f"{r.get('setDestIp_sent_until_reply')} (after the reset {r.get('setDestIp_sent_after_reset_until_reply')}), "
            f"P1 until reply {r.get('p1_until_reply')}, P5 max {fmt_ms(r.get('p5_max_ms'))} ms, longest P0 gap "
            f"{fmt_ms(r.get('p0_longest_gap_ms'))} ms, booted {r.get('booted')}" for i, r in enumerate(runs))))
    return items


# --------------------------------------------------------------------------------------
# C9: resetCause of /booted
# --------------------------------------------------------------------------------------


def c9_run(b: Bench, how: str, run: int) -> dict:
    ax = b.axis
    rec = None
    if how == "button":
        b.setup_excitation()
        b.send("/enableElectromagnetBrake", ax, 1)
        b.send("/setBrakeTransitionDuration", ax, 100)
        time.sleep(0.1)
        b.send("/activate", ax, 1)
        time.sleep(0.6)
        rec = restart_recording(b)
        time.sleep(0.3)
    t0 = time.monotonic_ns()
    detail = {}
    try:
        if how == "power":
            ok = b.operator(f"C9a run {run}: turn off the board (USB cable AND 12 V), wait 3 s, turn both on",
                            done=lambda: wait_booted(b, t0, 0.0) is not None, timeout=600.0)
        elif how == "button":
            ok = b.operator(f"C9b run {run}: press the reset button of the STEP800 once",
                            done=lambda: wait_booted(b, t0, 0.0) is not None, timeout=600.0)
        elif how == "upload":
            b.ser.close()
            root = Path(__file__).resolve().parents[3]
            r = subprocess.run([shutil.which("pio") or str(Path.home() / ".platformio/penv/bin/pio"), "run", "-e", "step800_devlib_probe", "-t", "upload"], cwd=root,
                               capture_output=True, text=True)
            detail["upload_rc"] = r.returncode
            (b.out / f"c9_upload_{run}.txt").write_text(r.stdout[-3000:] + r.stderr[-3000:])
            ok = r.returncode == 0
        else:
            b.send("/resetDevice")
            ok = True
        booted = wait_booted(b, t0, 20.0) if ok else None
        time.sleep(0.5)
    finally:
        cap = rec.stop() if rec is not None else None
    if cap is not None:
        b.save_capture(cap, f"c9_{how}_{run}")
        detail["events"] = restart_events(b, cap)
        detail["capture_ok"] = cap.ok
    b.ser.reconnect(30.0)
    status = b.status_json()["board"]
    b.connect()
    return {"how": how, "booted": list(booted.args) if booted else None, "resetCause_T": status["resetCause"],
            "resetCauseRaw_T": status["resetCauseRaw"], **detail}


C9_EXPECTED = {"power": 1, "button": 3, "upload": None, "resetDevice": 5}


def check_c9(b: Bench) -> list[Item]:
    items = []
    names = {"a": "power", "b": "button", "c": "upload", "d": "resetDevice"}
    for case in (b.args.cases or "abcd"):
        how = names[case]
        n = b.args.repeat or 3
        runs = [c9_run(b, how, i + 1) for i in range(n)]
        exp = C9_EXPECTED[how]
        summary = "; ".join(f"/booted {r['booted']}, T resetCause {r['resetCause_T']} raw 0x{r['resetCauseRaw_T']:02X}"
                            for r in runs)
        if exp is None:
            items.append(recorded(f"C9{case}", f"{how}: {summary}", runs=runs))
        else:
            ok = all(r["booted"] is not None and int(r["booted"][1]) == exp and r["resetCause_T"] == exp for r in runs)
            items.append(judged(f"C9{case}", ok, f"{how} (expected {exp}): {summary}", runs=runs))
        if how == "button":
            items.append(recorded("C9b-brake", "; ".join(
                f"run {i + 1}: brake edges {r.get('events', {}).get('brake_edges_ms')} ms from the standby entry "
                f"(ref {r.get('events', {}).get('ref')}), P0 resumes {fmt_ms(r.get('events', {}).get('p0_resume_ms'))} ms"
                for i, r in enumerate(runs))))
    return items


# --------------------------------------------------------------------------------------
# C10: DIP switch other than 0
# --------------------------------------------------------------------------------------


def c10_check_id(b: Bench, dip: int, t_since: int, cap_brakes: dict | None) -> dict:
    from .osc import OscLink

    res: dict = {"dip": dip}
    st = b.status_json()["board"]
    res["T_myId"], res["T_myIp"] = st["myId"], st["myIp"]
    log = b.ser.text(t_since)
    res["w5500_line"] = "W5500 RTR:200 RCR:2" in log
    ips = re.findall(r"New IP: (\d+\.\d+\.\d+\.\d+)", log)
    res["new_ip_lines"] = ips
    ip = f"10.0.0.{(100 + dip) & 0xFF}"
    res["ip"] = ip
    own = b.link.listen_port == 50100 + dip and b.link.board_ip == ip  # --id already set to dip
    link = b.link if own else OscLink(ip, b.cfg.board_in_port, 50100 + dip).open()
    try:
        r = link.request("/setDestIp", reply="/destIp", timeout=0.5, attempts=3)
        res["destIp_reply"] = list(r.args)
    except Exception as ex:
        res["destIp_reply"] = None
        res["destIp_error"] = str(ex)
    finally:
        if not own:
            link.close()
    if cap_brakes is not None:
        res.update(cap_brakes)
    res["ok"] = bool(st["myId"] == dip and res["w5500_line"] and ip in ips and res["destIp_reply"] is not None
                     and (cap_brakes is None or cap_brakes["brake_edges"] == 0 and cap_brakes["brake_levels"] == [0] * 8))
    return res


def brake_lines(b: Bench, cap) -> dict:
    brakes = [int(d) for d in b.cfg.ad2["dio"]["brake"]]
    edges = {d: analyze.bit_edges(cap.digital, d) for d in brakes}
    return {"brake_edges": int(sum(e.rise.size + e.fall.size for e in edges.values())),
            "brake_levels": [edges[d].initial for d in brakes]}


def check_c10(b: Bench) -> list[Item]:
    items = []
    brakes = [int(d) for d in b.cfg.ad2["dio"]["brake"]]
    rows = []
    values = [int(v) for v in b.args.dip_values.split(",")] if b.args.dip_values else [1, 2, 4, 8, 16, 32, 64, 128]
    for dip in values:
        rec = b.start(brakes, 1e6)
        t = time.monotonic_ns()
        try:
            time.sleep(0.2)

            def done(dip=dip):
                try:
                    return b.status_json()["board"]["myId"] == dip
                except Exception:
                    return False

            ok = b.operator(f"C10: set the DIP switch to {dip} (binary {dip:08b}: only this switch on)", done=done,
                            timeout=600.0)
            time.sleep(1.0)
        finally:
            cap = rec.stop()
        b.save_capture(cap, f"c10_dip{dip}")
        r = c10_check_id(b, dip, t, brake_lines(b, cap)) if ok else {"dip": dip, "ok": False, "error": "not detected"}
        rows.append(r)
    items.append(judged("C10-1", all(r["ok"] for r in rows), "; ".join(
        f"{r['dip']}: myId {r.get('T_myId')}, W5500 line {r.get('w5500_line')}, New IP {r.get('new_ip_lines')}, "
        f"/destIp {'ok' if r.get('destIp_reply') else 'none'}, brake edges {r.get('brake_edges')}" for r in rows),
                        rows=rows))
    return items


def check_c10_boot(b: Bench) -> list[Item]:
    """C10-2: the DIP switch is already at --dip (e.g. 85); restart with 'R' and read it at boot."""
    from .osc import OscLink

    dip = int(b.args.dip)
    brakes = [int(d) for d in b.cfg.ad2["dio"]["brake"]]
    rec = b.start(brakes, RESTART_RATE)
    t = time.monotonic_ns()
    try:
        time.sleep(0.2)
        b.ser.write("R")
        b.ser.wait_gone(3.0)
        b.ser.reconnect()
        time.sleep(1.5)
    finally:
        cap = rec.stop()
    b.save_capture(cap, f"c10_boot_dip{dip}")
    edges = brake_lines(b, cap)
    r = c10_check_id(b, dip, t, None)
    r.update(edges)
    # The boot messages are printed before the port is opened; only the reconnect is logged.
    r["ok"] = bool(r["T_myId"] == dip and r["destIp_reply"] is not None)
    return [judged("C10-2", r["ok"], f"after 'R' with DIP {dip}: myId {r['T_myId']}, myIp {r['T_myIp']}, /destIp from "
                   f"{r['ip']}:{50100 + dip} {r['destIp_reply']}, W5500 line seen {r['w5500_line']} (boot output "
                   f"is usually printed before the port opens), brake edges across the restart {r['brake_edges']}",
                   **{k: v for k, v in r.items() if k != "ok"})]


# --------------------------------------------------------------------------------------
# C11: SD save and the watchdog
# --------------------------------------------------------------------------------------


def check_c11(b: Bench) -> list[Item]:
    items = []
    probe = b.cfg.probe_dio
    cfgname = b.request("/getConfigName", reply="/configName").args
    if int(cfgname[1]) != 1:
        return [judged("C11-1", False, f"sdInitializeSucceeded {cfgname[1]}: insert the test SD card (stopped)")]
    items.append(recorded("C11-1", f"/configName {list(cfgname)}"))
    reply = b.ser.command("F/hil_save_test.json\n", until=r"\{\"result\".*\}\r?\n")
    items.append(recorded("C11-F", json_line(reply, '{"result"').__repr__()))
    status0 = b.status_json()["board"]
    runs = []
    for how in ("serial", "osc"):
        for i in range(b.args.repeat or 5):
            text = b.config_json_text()
            rec = b.start(probe, 1e6)
            try:
                time.sleep(0.3)
                t = time.monotonic_ns()
                if how == "serial":
                    b.ser.write(text)
                    m = b.ser.wait_for(r"\{\"result\".*\}\r?\n", t, 15.0)
                    resp = m.group(0).strip() if m else None
                else:
                    e = b.link.wait_for("/saveConfig", 15.0, b.send("/saveConfig").t_ns)
                    err = b.link.entries("rx", t)
                    resp = list(e.args) if e else [x.address for x in err if x.address.startswith("/error")]
                time.sleep(0.5)
            finally:
                cap = rec.stop()
            b.save_capture(cap, f"c11_{how}_{i + 1}")
            p0 = analyze.bit_edges(cap.digital, probe[0]).all
            gap, at = longest_gap(p0)
            booted = [e for e in b.link.entries("rx", t) if e.address == "/booted"]
            runs.append({"how": how, "run": i + 1, "response": resp, "save_ms": gap * 1e3, "at_s": at,
                         "booted": len(booted), "early_warning": len(long_pulses(
                             analyze.bit_edges(cap.digital, probe[2]), 1e-3)), "capture_ok": cap.ok})
    status1 = b.status_json()["board"]
    ok_serial = all(r["booted"] == 0 and r["early_warning"] == 0 and isinstance(r["response"], str)
                    and '"result":"ok"' in r["response"] for r in runs if r["how"] == "serial")
    ok_osc = all(r["booted"] == 0 and r["early_warning"] == 0 and isinstance(r["response"], list)
                 and r["response"] and "Saved" in str(r["response"][0]) for r in runs if r["how"] == "osc")
    same = status1["resetCause"] == status0["resetCause"]
    items.append(judged("C11-2", ok_serial and same, "'{' x5: " + "; ".join(
        f"{r['save_ms']:.1f} ms" for r in runs if r["how"] == "serial") + f"; resetCause unchanged {same}",
                        runs=[r for r in runs if r["how"] == "serial"]))
    items.append(judged("C11-3", ok_osc and same, "/saveConfig x5: " + "; ".join(
        f"{r['save_ms']:.1f} ms" for r in runs if r["how"] == "osc"), runs=[r for r in runs if r["how"] == "osc"]))
    # 4: the watchdog still works after a save: save, then hang ('W')
    text = b.config_json_text()
    t = b.ser.write(text)
    b.ser.wait_for(r"\{\"result\".*\}\r?\n", t, 15.0)
    rec = b.start(probe, RESTART_RATE)
    try:
        time.sleep(0.3)
        t_w = b.ser.write("W")
        booted = wait_booted(b, t_w, 15.0)
        time.sleep(0.5)
    finally:
        cap = rec.stop()
    b.save_capture(cap, "c11_watchdog")
    b.ser.reconnect(30.0)
    status2 = b.status_json()["board"]
    b.connect()
    p4 = analyze.bit_edges(cap.digital, probe[4]).rise
    early = long_pulses(analyze.bit_edges(cap.digital, probe[2]), 1e-3)
    t_hang = None
    if early:
        before = p4[p4 < early[0][0]]
        t_hang = float(before[-1]) if before.size else None
    dt = (early[0][0] - t_hang) if early and t_hang is not None else None
    ok = within(dt, 1.95, 2.06) and status2["resetCause"] == 4 and booted is not None and int(booted.args[1]) == 4
    items.append(judged("C11-4", ok, f"'W' after a save: early warning (P2) {fmt_ms(dt * 1e3 if dt else None)} ms after "
                        f"the last getMyId/watchdog reset (P4); /booted {list(booted.args) if booted else None}; "
                        f"T resetCause {status2['resetCause']} raw 0x{status2['resetCauseRaw']:02X}"))
    return items


# --------------------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------------------

TESTS = {
    "C1": ("pattern matching", check_c1),
    "C2": ("/getMinSpeed and boolean type tags", check_c2),
    "C3": ("/goUntil errors", check_c3),
    "C4": ("packed transfers: homing, positions around +/-2^21, servo wrap", check_c4),
    "C5": ("USB serial diagnosis CONFIG / STATUS fields", check_c5),
    "C6": ("destination IP that does not answer ARP", check_c6),
    "C7": ("reboot from the config tool ('R')", check_c7),
    "C8": ("/setDestIp right after a restart", check_c8),
    "C9": ("resetCause of /booted", check_c9),
    "C10": ("DIP switch other than 0 (while running)", check_c10),
    "C10boot": ("DIP switch read at boot (--dip)", check_c10_boot),
    "C11": ("SD save and the watchdog", check_c11),
}
AUTO_TESTS = ("C1", "C2", "C3", "C4", "C5", "C6", "C7")


def run_test(cfg: Config, test: str, args) -> dict:
    from . import commands

    title, fn = TESTS[test]
    out = commands.new_result_dir(f"check_{test}")
    res: dict = {"test": test, "title": title, "board_id": cfg.board_id, "axis": cfg.test_axis}
    with Bench(cfg, out, args) as b:
        try:
            b.reboot()
            res["firmware"] = commands.firmware_version(b.link)
            items = fn(b)
        except Exception as ex:  # keep the logs
            items = [Item(f"{test}-error", FAIL, f"{type(ex).__name__}: {ex}")]
        finally:
            b.ser.save(out / "serial_log.txt", b.t0_ns)
            b.link.save_log(out / "osc_log.jsonl")
        res["warnings"] = warning_lines(b.ser.text())
        res["notes"] = b.notes
    res["items"] = [i.to_dict() for i in items]
    commands.write_json(out / "summary.json", res)
    res["dir"] = out
    return res


def cmd_check(cfg: Config, args) -> int:
    from . import commands

    tests = list(AUTO_TESTS) if args.all_auto else args.test
    if not tests:
        print("give --test or --all-auto")
        return 2
    rows = []
    any_fail = False
    for test in tests:
        res = run_test(cfg, test, args)
        for it in res["items"]:
            rows.append([it["id"], it["verdict"], it["detail"]])
            any_fail |= it["verdict"] == FAIL
        if res["warnings"]:
            rows.append([f"{test}-warnings", RECORD, "; ".join(res["warnings"])])
        print(f"{test}: {res['dir']}")
    if "C4" in tests:
        found = scan_warnings(commands.git_short_sha(), commands.RESULTS_DIR)
        lines = {k: v for k, v in found.items() if v}
        ok = not lines
        rows.append(["C4-1", PASS if ok else FAIL, f"'Warning:' lines in {len(found)} check serial logs of this "
                     f"sha: {lines or 'none'}"])
        any_fail |= not ok
    for r in rows:
        detail = str(r[2])
        print(f"{r[0]:12s} {r[1]:6s} {detail}")
    return 1 if any_fail else 0
