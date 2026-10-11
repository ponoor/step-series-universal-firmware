"""USB serial diagnosis commands of the firmware (src/diagnosis.cpp):
p (timing stats, then reset), P (probe self-test, HIL_PROBE builds), b (brake output
test), s (status), R (reboot)."""

from __future__ import annotations

import glob
import re
import time

import serial

PERF_LINE = re.compile(r"^(\w+)\s+max\s+(\d+) us\s+avg\s+(\d+) us\s+n (\d+)\s*$")
PERF_COUNTER = re.compile(r"^(oscSlowPath|oscNotMatched)\s+(\d+)\s*$")
BRAKE_LINE = re.compile(r"Turn off #(\d+), Turn on #(\d+)")
PROBE_MAP = re.compile(r"HIL probe:((?: P\d=\w+\(P[AB]\d+\))+)")


class DiagError(Exception):
    pass


def find_port(pattern: str = "/dev/cu.usbmodem*") -> str:
    ports = sorted(glob.glob(pattern))
    if not ports:
        raise DiagError(f"no USB serial port matches {pattern}; is the STEP800 connected?")
    if len(ports) > 1:
        raise DiagError(f"several ports match {pattern}: {', '.join(ports)}; set serial.port in bench.toml")
    return ports[0]


def parse_perf(text: str) -> dict:
    """Parses the output of 'p' into {name: {max_us, avg_us, n}} plus the counters."""
    out: dict = {}
    for line in text.splitlines():
        m = PERF_LINE.match(line.strip())
        if m:
            out[m.group(1)] = {"max_us": int(m.group(2)), "avg_us": int(m.group(3)), "n": int(m.group(4))}
            continue
        m = PERF_COUNTER.match(line.strip())
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


def parse_probe_map(text: str) -> dict[int, str] | None:
    """'HIL probe: P0=D1(PA10) ...' -> {0: 'D1(PA10)', ...}"""
    m = PROBE_MAP.search(text)
    if not m:
        return None
    return {int(k): v for k, v in re.findall(r"P(\d)=(\w+\(P[AB]\d+\))", m.group(1))}


class SerialDiag:
    def __init__(self, port: str = ""):
        self.port = port or find_port()
        self._ser: serial.Serial | None = None

    def open(self) -> "SerialDiag":
        self._ser = serial.Serial(self.port, 115200, timeout=0.05)
        return self

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "SerialDiag":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    def read_available(self, idle: float = 0.3, until: str | None = None, timeout: float = 5.0) -> str:
        """Reads until nothing arrives for `idle` seconds, or until the regex `until`
        matches the text read so far, or timeout."""
        buf = bytearray()
        start = last = time.monotonic()
        pattern = re.compile(until) if until else None
        while True:
            chunk = self._ser.read(4096)
            now = time.monotonic()
            if chunk:
                buf += chunk
                last = now
                if pattern and pattern.search(buf.decode(errors="replace")):
                    break
            elif not pattern and now - last >= idle:
                break
            if now - start >= timeout:
                if pattern:
                    raise DiagError(f"timeout waiting for {until!r}; got {buf.decode(errors='replace')[-200:]!r}")
                break
        return buf.decode(errors="replace")

    def command(self, ch: str, **kwargs) -> str:
        self._ser.reset_input_buffer()
        self._ser.write(ch.encode())
        self._ser.flush()
        return self.read_available(**kwargs)

    # --- commands ---
    def perf(self) -> tuple[dict, str]:
        """'p': prints and resets the timing statistics."""
        text = self.command("p", until=r"oscNotMatched\s+\d+")
        return parse_perf(text), text

    def probe_selftest(self) -> tuple[dict[int, str] | None, str]:
        """'P': probe map, then ch + 1 pulses on each probe."""
        text = self.command("P", until=r"HIL probe self-test done")
        return parse_probe_map(text), text

    def brake_step(self) -> tuple[int, int]:
        """'b': turns off brake output #a and turns on #b (1-based). Returns (a, b)."""
        text = self.command("b", until=r"Turn on #\d+")
        m = BRAKE_LINE.search(text)
        return int(m.group(1)), int(m.group(2))

    def status(self) -> str:
        return self.command("s", idle=0.5)

    def reboot(self, timeout: float = 15.0) -> str:
        """'R': reboots the board, waits for the USB port to come back and returns what
        was printed in the first second (the boot messages may be gone already)."""
        try:
            self.command("R", until=r"Rebooting")
        except (serial.SerialException, OSError, DiagError):
            pass  # the port can disappear before the reply is read
        self.close()
        deadline = time.monotonic() + timeout
        time.sleep(0.5)
        while time.monotonic() < deadline:
            try:
                self.open()
                return self.read_available(idle=1.0, timeout=3.0)
            except (serial.SerialException, OSError):
                time.sleep(0.2)
        raise DiagError(f"{self.port} did not come back within {timeout} s after reboot")
