"""Tests of hil.py swd (SWD through OpenOCD) without hardware: OpenOCD output parsing, the
Tcl client against a fake server, and the dump of a synthetic halted CPU."""

import socket
import threading
from pathlib import Path

import pytest

from hil import swd

PROBE_ELF = Path(__file__).resolve().parents[3] / ".pio/build/step400_devlib_probe/firmware.elf"


def test_parse_mdw_and_reg():
    text = "0x20001f7c: 48464c54 00000003 00000002 \n0x20001f88: deadbeef \n"
    assert swd.parse_mdw(text) == {0x20001F7C: 0x48464C54, 0x20001F80: 3, 0x20001F84: 2, 0x20001F88: 0xDEADBEEF}
    assert swd.parse_reg("pc (/32): 0x00009d2c\n") == 0x9D2C
    assert swd.parse_reg("primask (/1): 0x01\n") == 1
    assert swd.parse_reg("garbage") is None


def test_exception_name():
    assert swd.exception_name(0) == "thread"
    assert swd.exception_name(3) == "HardFault"
    assert swd.exception_name(23) == "IRQ 7 (USB)"
    assert swd.exception_name(18) == "IRQ 2 (WDT)"


def test_exception_frames():
    sp = 0x20007F00
    frame = [1, 2, 3, 4, 12, 0x00012345, 0x00018600, 0x21000017]  # interrupted the USB handler
    stack = [0x11, 0x22, 0xFFFFFFF9] + frame + [0xFFFFFFF9, 0, 0, 0, 0, 0, 0x5, 0x20000000, 0]
    fr = swd.exception_frames(stack, sp)
    assert len(fr) == 1  # the second candidate has no code PC / Thumb bit
    assert fr[0]["pc"] == 0x18600 and fr[0]["interrupted"] == "IRQ 7 (USB)" and fr[0]["frame_addr"] == sp + 12


def test_nvic_priorities_and_flash_diff():
    # USB (IRQ 7) at 0, WDT (IRQ 2) at 0, SERCOM4 (IRQ 13) at 3.
    ipr = [0, 0, 0, 0xC0 << 8, 0, 0, 0, 0]
    pr = swd.nvic_priorities(ipr)
    assert pr["USB"] == 0 and pr["WDT"] == 0 and pr["SERCOM4"] == 3
    image = bytes(range(16))
    read = {0x2000: int.from_bytes(image[0:4], "little"), 0x2004: 0, 0x2008: int.from_bytes(image[8:12], "little")}
    assert swd.flash_diff(read, image) == [0x2004]
    assert swd.rcause_name(1 << 5) == "WDT" and swd.rcause_name(0) == "none"


class FakeTcl:
    """Answers OpenOCD Tcl commands (0x1a terminated) from a memory dict."""

    def __init__(self, mem: dict[int, int], regs: dict[str, int]):
        self.mem, self.regs = mem, regs
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.commands: list[str] = []
        threading.Thread(target=self._serve, daemon=True).start()

    def _answer(self, c: str) -> str:
        self.commands.append(c)
        inner = c[len("capture {"):-1] if c.startswith("capture {") else c
        parts = inner.split()
        if parts[0] == "mdw":
            a, n = int(parts[1], 16), int(parts[2])
            return f"0x{a:08x}: " + " ".join(f"{self.mem.get(a + 4 * k, 0):08x}" for k in range(n)) + " \n"
        if parts[0] == "reg":
            return f"{parts[1]} (/32): 0x{self.regs.get(parts[1], 0):08x}\n"
        return ""

    def _serve(self):
        conn, _ = self.srv.accept()
        buf = b""
        while True:
            data = conn.recv(4096)
            if not data:
                return
            buf += data
            while b"\x1a" in buf:
                c, buf = buf.split(b"\x1a", 1)
                conn.sendall(self._answer(c.decode()).encode() + b"\x1a")


def fake_ocd(mem, regs) -> swd.OpenOcd:
    srv = FakeTcl(mem, regs)
    ocd = swd.OpenOcd()
    ocd.sock = socket.create_connection(("127.0.0.1", srv.port))
    ocd.server = srv
    return ocd


def test_client_words_and_info():
    mem = {0x41002018: 0x10010305, 0x40000438: 0x20, 0xE000EDF0: 0x00030003, 0x2000: 0x20008000, 0x2004: 0x195A1}
    ocd = fake_ocd(mem, {})
    assert ocd.words(0x2000, 2) == [0x20008000, 0x195A1]
    image = (0x20008000).to_bytes(4, "little") + (0x195A1).to_bytes(4, "little")
    r = swd.info(ocd, image, n_words=2)
    assert r["rcause"] == "WDT" and r["halted"] and r["flash_words_differ"] == []
    assert not any(c.startswith(("reset", "flash", "halt")) or "reset" in c for c in ocd.server.commands)


def test_dump_halted_in_usb_handler():
    sp = 0x20007F00
    frame = [0, 0, 0, 0, 0, 0x9B7F, 0x9C00, 0x21000000]  # thread code interrupted by USB
    mem = {sp + 4 * k: w for k, w in enumerate([0x7, 0xFFFFFFF9] + frame)}
    mem[0xE000EDF0] = 1 << 17
    mem[0xE000ED04] = 23
    mem[0xE000E100] = (1 << 7) | (1 << 2)
    mem[0x41005000 + 0x100 + 0x20 * 0 + 4] = 0x10 << 24  # EP0 EPINTFLAG RXSTP
    regs = {"pc": 0x18600, "lr": 0xFFFFFFF9, "sp": sp, "msp": sp, "psp": 0, "xPSR": 0x21000017, "primask": 0}
    d = swd.dump(fake_ocd(mem, regs), None)
    assert d["executing"] == "IRQ 7 (USB)" and d["ipsr"] == 23
    assert d["nvic_enabled"] == ["WDT", "USB"] and d["nvic_priority"] == {"WDT": 0, "USB": 0}
    assert d["exception_frames"][0]["interrupted"] == "thread"
    assert d["usb_ep"] == {0: {"EPCFG": "0x00", "EPSTATUS": "0x00", "EPINTFLAG": "0x10", "EPINTENSET": "0x00"}}


@pytest.mark.skipif(not PROBE_ELF.exists(), reason="needs the step400_devlib_probe build")
def test_symbols_of_probe_build():
    syms = swd.nm_symbols(PROBE_ELF)
    assert "hilFaultRecordStorage" in syms and "usbIsrCount" in syms
    lines = swd.addr2line(PROBE_ELF, [syms["hilFaultRecordStorage"]])
    assert len(lines) == 1
