"""hil.py swd: read where the MCU is through SWD (Atmel-ICE, OpenOCD), TASK_usb_unplug Task 3.

OpenOCD runs as a server (cmsis-dap, at91samdXX.cfg) and is driven over its Tcl port. The
connection does not reset the board: at91samdXX.cfg resets only on an explicit `reset`
(there is no connect_assert_srst), nothing here sends one, nor any flash command, and
`reset_config none` keeps OpenOCD off nRESET (wired on the 10-pin connector).

Steps (each run is one step; the human approves each):
- --info: no halt. Chip ID (DSU DID), reset cause (PM RCAUSE), debug state (DHCSR), and
  the first words of the application in flash compared with the given firmware.bin.
- --halt: halts the CPU now, reads the state (dump below), then resumes it (or not, with
  --stay-halted). While halted nothing runs: the brake outputs and the drivers keep their
  state (a turning motor keeps turning).
- --on-hang: sends /getVersion every 100 ms and halts as soon as no reply came for 0.3 s
  (the USB cable is unplugged by the operator meanwhile), then as --halt. --setup u1 (a
  hil.py usb step) applies that step's setup and its messages before the unplug first.

The watchdog stops while the CPU is halted (measured 2026-10-16: no early warning flag
after a 2.5 s halt, no restart after the resume): a halt does not shorten the time to the
reset, the count goes on after the resume.

The dump: PC, LR, SP, MSP, PSP, xPSR (the exception number in IPSR: 0 thread, 3 HardFault,
16 + n IRQ n, USB = 23, WDT = 18), PRIMASK, ICSR, the NVIC enable / pending / priority
registers, WDT, USB (device and endpoint registers), PM RCAUSE, 128 words of the stack, the
exception frames found on the stack (after an EXC_RETURN word), and, with a HIL_PROBE ELF,
the fault record and the USB interrupt counters. Code addresses are turned into functions
and lines with addr2line; with --gdb also a gdb backtrace (gdb unwinds exception frames).

Peripheral registers are read only where a read has no side effect (no SERCOM DATA).
"""

from __future__ import annotations

import json
import re
import shutil
import socket
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from .config import Config

TOOLCHAIN = Path.home() / ".platformio/packages/toolchain-gccarmnoneeabi@1.70201.0/bin"
OPENOCD_TCL_PORT = 6666
OPENOCD_GDB_PORT = 3333
APP_START = 0x2000  # after the 8 KB bootloader
FLASH_END = 0x40000
RAM_START, RAM_END = 0x20000000, 0x20008000
EXC_RETURN = (0xFFFFFFF1, 0xFFFFFFF9, 0xFFFFFFFD)

REG = {
    "DSU_DID": 0x41002018,
    "PM_RCAUSE": 0x40000438,  # 8 bit: POR 0, BOD12 1, BOD33 2, EXT 4, WDT 5, SYST 6
    "DHCSR": 0xE000EDF0,  # S_HALT 17, S_SLEEP 18, S_LOCKUP 19
    "ICSR": 0xE000ED04,
    "NVIC_ISER": 0xE000E100,
    "NVIC_ISPR": 0xE000E200,
    "WDT_CTRL_CONFIG_EWCTRL": 0x40001000,  # CTRL, CONFIG, EWCTRL, -
    "WDT_INTEN_INTFLAG_STATUS": 0x40001004,  # INTENCLR, INTENSET, INTFLAG, STATUS
    "USB_CTRLA": 0x41005000,  # CTRLA, SYNCBUSY, QOSCTRL
    "USB_CTRLB_DADD": 0x41005008,
    "USB_STATUS_FSMSTATUS": 0x4100500C,
    "USB_FNUM": 0x41005010,
    "USB_INTENSET": 0x41005018,
    "USB_INTFLAG": 0x4100501C,
    "USB_EPINTSMRY": 0x41005020,
}
NVIC_IPR = 0xE000E400  # 8 words, 2 priority bits per IRQ in bits 7:6 of each byte
USB_EP = 0x41005100  # + n * 0x20: EPCFG; +4: -, -, EPSTATUS, EPINTFLAG; +8: EPINTENCLR, EPINTENSET
IRQ_NAMES = ["PM", "SYSCTRL", "WDT", "RTC", "EIC", "NVMCTRL", "DMAC", "USB", "EVSYS", "SERCOM0", "SERCOM1",
             "SERCOM2", "SERCOM3", "SERCOM4", "SERCOM5", "TCC0", "TCC1", "TCC2", "TC3", "TC4", "TC5", "TC6",
             "TC7", "ADC", "AC", "DAC", "PTC", "I2S"]
EXC_NAMES = {0: "thread", 2: "NMI", 3: "HardFault", 11: "SVCall", 14: "PendSV", 15: "SysTick"}
# HIL_PROBE live counters (src/hilFault.cpp, anonymous namespace)
HIL_SYMBOLS = ("hilFaultRecordStorage", "usbIsrCount", "usbInIsr", "loopMillis", "loopUsbCount", "usbLastIndex",
               "usbLast")


class SwdError(RuntimeError):
    pass


# --------------------------------------------------------------------------------------
# Parsing (pure, tested in tests/test_swd.py)
# --------------------------------------------------------------------------------------

MDW_LINE = re.compile(r"^0x([0-9a-fA-F]+):((?:\s+[0-9a-fA-F]{8})+)\s*$")
REG_LINE = re.compile(r"^\s*(\w+)\s*\(/\d+\):\s*0x([0-9a-fA-F]+)")


def parse_mdw(text: str) -> dict[int, int]:
    """'mdw' output -> {address: word}."""
    out: dict[int, int] = {}
    for line in text.splitlines():
        m = MDW_LINE.match(line.strip())
        if m:
            addr = int(m.group(1), 16)
            for k, w in enumerate(m.group(2).split()):
                out[addr + 4 * k] = int(w, 16)
    return out


def parse_reg(text: str) -> int | None:
    m = REG_LINE.search(text)
    return int(m.group(2), 16) if m else None


def exception_name(n: int) -> str:
    if n >= 16:
        k = n - 16
        return f"IRQ {k} ({IRQ_NAMES[k] if k < len(IRQ_NAMES) else '?'})"
    return EXC_NAMES.get(n, f"exception {n}")


def is_code(addr: int) -> bool:
    return APP_START <= (addr & ~1) < FLASH_END


def exception_frames(stack: list[int], sp: int) -> list[dict]:
    """Exception frames on the stack: the 8 words after each EXC_RETURN word (a handler
    that calls functions pushes LR = EXC_RETURN last, right below the frame the hardware
    stacked). Kept only when the stacked xPSR has the Thumb bit and the PC is code."""
    out = []
    for i, w in enumerate(stack):
        if w in EXC_RETURN:
            f = stack[i + 1:i + 9]
            if len(f) < 8:
                continue
            r0, r1, r2, r3, r12, lr, pc, xpsr = f
            if (xpsr & 0x01000000) and is_code(pc):
                out.append({"exc_return": w, "frame_addr": sp + 4 * (i + 1), "r0": r0, "r1": r1, "r2": r2,
                            "r3": r3, "r12": r12, "lr": lr, "pc": pc, "xpsr": xpsr,
                            "interrupted": exception_name(xpsr & 0x3F)})
    return out


def nvic_priorities(ipr_words: list[int]) -> dict[str, int]:
    """IPR0-7 -> {IRQ name: priority 0-3} (bits 7:6 of each byte)."""
    out = {}
    for n in range(min(len(IRQ_NAMES), 4 * len(ipr_words))):
        b = (ipr_words[n // 4] >> (8 * (n % 4))) & 0xFF
        out[IRQ_NAMES[n]] = b >> 6
    return out


def flash_diff(read: dict[int, int], image: bytes, base: int = APP_START) -> list[int]:
    """Addresses whose word read from flash differs from the image (firmware.bin at base)."""
    diff = []
    for addr, w in sorted(read.items()):
        off = addr - base
        if 0 <= off and off + 4 <= len(image):
            if int.from_bytes(image[off:off + 4], "little") != w:
                diff.append(addr)
    return diff


def rcause_name(v: int) -> str:
    names = {0: "POR", 1: "BOD12", 2: "BOD33", 4: "EXT", 5: "WDT", 6: "SYST"}
    return "+".join(n for b, n in names.items() if v & (1 << b)) or "none"


# --------------------------------------------------------------------------------------
# OpenOCD
# --------------------------------------------------------------------------------------


class OpenOcd:
    """OpenOCD server with the Atmel-ICE (CMSIS-DAP) on SWD, driven over the Tcl port."""

    def __init__(self, executable: str = "", log_path: Path | None = None, speed_khz: int = 1000):
        self.exe = executable or shutil.which("openocd") or "openocd"
        self.log_path = log_path
        self.speed = speed_khz
        self.proc: subprocess.Popen | None = None
        self.sock: socket.socket | None = None

    def start(self, timeout: float = 10.0) -> "OpenOcd":
        args = [self.exe, "-f", "interface/cmsis-dap.cfg", "-c", "transport select swd",
                "-f", "target/at91samdXX.cfg", "-c", "reset_config none",
                "-c", f"adapter speed {self.speed}",
                "-c", f"tcl_port {OPENOCD_TCL_PORT}", "-c", f"gdb_port {OPENOCD_GDB_PORT}",
                "-c", "telnet_port disabled", "-c", "init"]
        log = open(self.log_path, "w") if self.log_path else subprocess.DEVNULL
        self.proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise SwdError(f"openocd exited ({self.proc.returncode}); see {self.log_path}")
            try:
                self.sock = socket.create_connection(("127.0.0.1", OPENOCD_TCL_PORT), timeout=1.0)
                self.sock.settimeout(5.0)
                return self
            except OSError:
                time.sleep(0.2)
        raise SwdError("openocd did not open its Tcl port")

    def stop(self) -> None:
        if self.sock is not None:
            try:
                self.cmd("shutdown")
            except (OSError, SwdError):
                pass
            self.sock.close()
            self.sock = None
        if self.proc is not None:
            try:
                self.proc.wait(5.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            self.proc = None

    def __enter__(self) -> "OpenOcd":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def cmd(self, command: str) -> str:
        """One Tcl command; returns its result. 'capture' keeps the printed output."""
        self.sock.sendall(command.encode() + b"\x1a")
        buf = bytearray()
        while not buf.endswith(b"\x1a"):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise SwdError("openocd closed the Tcl connection")
            buf += chunk
        return buf[:-1].decode(errors="replace")

    def capture(self, command: str) -> str:
        return self.cmd(f"capture {{{command}}}")

    def words(self, addr: int, n: int) -> list[int]:
        got = parse_mdw(self.capture(f"mdw 0x{addr:08x} {n}"))
        try:
            return [got[addr + 4 * k] for k in range(n)]
        except KeyError:
            raise SwdError(f"mdw 0x{addr:08x} {n}: no data") from None

    def word(self, addr: int) -> int:
        return self.words(addr, 1)[0]

    def reg(self, name: str) -> int | None:
        return parse_reg(self.capture(f"reg {name}"))

    def halt(self) -> None:
        self.capture("halt 500")
        if not self.word(REG["DHCSR"]) & (1 << 17):
            raise SwdError("the CPU did not halt")

    def resume(self) -> None:
        self.capture("resume")


# --------------------------------------------------------------------------------------
# Symbols
# --------------------------------------------------------------------------------------


def nm_symbols(elf: Path) -> dict[str, int]:
    """Data symbols of the HIL fault record (C++ names demangled by nm -C)."""
    text = subprocess.run([str(TOOLCHAIN / "arm-none-eabi-nm"), "-C", str(elf)], capture_output=True,
                          text=True, check=True).stdout
    out = {}
    for line in text.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3:
            name = parts[2].split("::")[-1]
            if name in HIL_SYMBOLS:
                out[name] = int(parts[0], 16)
    return out


def addr2line(elf: Path, addrs: list[int]) -> dict[int, str]:
    if not addrs:
        return {}
    a = [f"0x{x & ~1:x}" for x in addrs]
    text = subprocess.run([str(TOOLCHAIN / "arm-none-eabi-addr2line"), "-f", "-C", "-e", str(elf), *a],
                          capture_output=True, text=True, check=True).stdout.splitlines()
    out = {}
    for k, x in enumerate(addrs):
        fn, loc = (text[2 * k:2 * k + 2] + ["?", "?"])[:2]
        out[x] = fn if loc.startswith("??") else f"{fn} {Path(loc).name}"
    return out


# --------------------------------------------------------------------------------------
# Steps
# --------------------------------------------------------------------------------------


def info(ocd: OpenOcd, image: bytes | None, n_words: int | None = None, chunk: int = 1024) -> dict:
    """No halt. n_words None: the whole image is compared with the flash (read while the
    CPU runs: a debugger read of the flash does not disturb it)."""
    dhcsr = ocd.word(REG["DHCSR"])
    rcause = ocd.word(REG["PM_RCAUSE"]) & 0xFF
    res = {"dsu_did": f"0x{ocd.word(REG['DSU_DID']):08X}", "rcause": rcause_name(rcause),
           "dhcsr": f"0x{dhcsr:08X}", "halted": bool(dhcsr & (1 << 17)), "lockup": bool(dhcsr & (1 << 19))}
    if image is not None:
        n_words = n_words or len(image) // 4
        read: dict[int, int] = {}
        for k in range(0, n_words, chunk):
            n = min(chunk, n_words - k)
            a = APP_START + 4 * k
            read.update(zip(range(a, a + 4 * n, 4), ocd.words(a, n)))
        d = flash_diff(read, image)
        res["flash_words_compared"] = n_words
        res["flash_words_differ"] = [f"0x{a:08X}" for a in d]
    return res


def dump(ocd: OpenOcd, elf: Path | None) -> dict:
    """Reads the state of a halted CPU."""
    regs = {r: ocd.reg(r) for r in ("pc", "lr", "sp", "msp", "psp", "xPSR", "primask")}
    xpsr = regs["xPSR"] or 0
    res: dict = {"regs": {k: (None if v is None else f"0x{v:08X}") for k, v in regs.items()},
                 "ipsr": xpsr & 0x3F, "executing": exception_name(xpsr & 0x3F)}
    raw = {name: ocd.word(addr) for name, addr in REG.items()}
    res["raw"] = {k: f"0x{v:08X}" for k, v in raw.items()}
    res["rcause"] = rcause_name(raw["PM_RCAUSE"] & 0xFF)
    res["lockup"] = bool(raw["DHCSR"] & (1 << 19))
    icsr = raw["ICSR"]
    res["icsr"] = {"vectactive": icsr & 0x3F, "vectpending": (icsr >> 12) & 0x3F,
                   "isrpending": bool(icsr & (1 << 22))}
    res["nvic_enabled"] = [IRQ_NAMES[n] for n in range(len(IRQ_NAMES)) if raw["NVIC_ISER"] >> n & 1]
    res["nvic_pending"] = [IRQ_NAMES[n] for n in range(len(IRQ_NAMES)) if raw["NVIC_ISPR"] >> n & 1]
    res["nvic_priority"] = {k: v for k, v in nvic_priorities(ocd.words(NVIC_IPR, 8)).items()
                            if k in res["nvic_enabled"] or k in ("USB", "WDT")}
    eps = {}
    for n in range(8):
        w0, w1, w2 = ocd.words(USB_EP + 0x20 * n, 3)
        eps[n] = {"EPCFG": w0 & 0xFF, "EPSTATUS": (w1 >> 16) & 0xFF, "EPINTFLAG": (w1 >> 24) & 0xFF,
                  "EPINTENSET": (w2 >> 8) & 0xFF}
    res["usb_ep"] = {n: {k: f"0x{v:02X}" for k, v in e.items()} for n, e in eps.items() if any(e.values())}
    sp = regs["sp"] or regs["msp"] or RAM_END
    n = max(0, min(128, (RAM_END - sp) // 4))
    stack = ocd.words(sp, n) if n else []
    res["stack"] = [f"0x{w:08X}" for w in stack]
    frames = exception_frames(stack, sp)
    res["exception_frames"] = [{k: (f"0x{v:08X}" if isinstance(v, int) and k != "frame_addr" else v)
                                for k, v in f.items()} for f in frames]
    if elf is not None:
        syms = nm_symbols(elf)
        if "hilFaultRecordStorage" in syms:
            res["hil_record_words"] = [f"0x{w:08X}" for w in ocd.words(syms["hilFaultRecordStorage"], 64)]
        res["hil_live"] = {k: ocd.word(a) for k, a in syms.items() if k in ("usbIsrCount", "usbInIsr", "loopMillis",
                                                                          "loopUsbCount")}
        code = [regs["pc"], regs["lr"]] + [f["pc"] for f in frames] + [f["lr"] for f in frames]
        code += [w for w in stack if is_code(w) and w & 1]
        code = [c for c in dict.fromkeys(code) if c is not None and is_code(c)]
        lines = addr2line(elf, code)
        res["symbols"] = {f"0x{a:08X}": s for a, s in lines.items()}
    return res


def gdb_backtrace(elf: Path) -> str:
    """gdb on the OpenOCD gdb port: bt (gdb unwinds the Cortex-M exception frames)."""
    r = subprocess.run([str(TOOLCHAIN / "arm-none-eabi-gdb"), "-batch", "-nx", "-ex", "set pagination off",
                        "-ex", f"target extended-remote localhost:{OPENOCD_GDB_PORT}", "-ex", "bt",
                        "-ex", "info registers", "-ex", "disconnect", str(elf)],
                       capture_output=True, text=True, timeout=60)
    return r.stdout + r.stderr


def setup_messages(cfg: Config, step_id: str) -> list[tuple]:
    """--setup: the setup of a hil.py usb step and its OSC messages / waits before the
    unplug (the host serial port is not used here). ("wait", seconds) for a wait."""
    from . import motor, usb

    step = usb.STEPS[step_id]
    out: list[tuple] = list(usb.setup_actions(cfg, step))
    for a in step.actions:
        if a.kind == "unplug":
            break
        if a.kind == "osc":
            out.append((a.address, *motor._resolve(a.args, cfg.test_axis)))
        elif a.kind == "wait":
            out.append(("wait", a.seconds))
    return out


def wait_hang(cfg: Config, timeout: float, gap: float = 0.3, setup: str | None = None) -> dict:
    """Heartbeat /getVersion every 100 ms; returns when no reply came for gap seconds."""
    from . import commands
    from .usb import Heartbeat, wait_heartbeat_lost

    with commands.open_link(cfg) as link:
        commands.connect_board(link)
        sent = []
        for m in setup_messages(cfg, setup) if setup else []:
            if m[0] == "wait":
                time.sleep(m[1])
            else:
                link.send(*m)
                sent.append(list(m))
                time.sleep(0.02)
        brake = None
        if setup:
            try:
                brake = list(link.request("/getBrakeStatus", cfg.test_axis, reply="/brakeStatus", timeout=0.3).args)
            except Exception as ex:  # OscError
                brake = str(ex)
            print(f"setup {setup}: {len(sent)} messages, /brakeStatus {brake}", flush=True)
        hb = Heartbeat(link).start()
        try:
            print("OPERATOR: unplug the USB cable of the board now (the AD2 and the Atmel-ICE stay connected)",
                  flush=True)
            t0 = time.monotonic_ns()
            last = wait_heartbeat_lost(link, t0, timeout, gap)
        finally:
            hb.stop()
    if last is None:
        raise SwdError(f"the replies did not stop within {timeout} s")
    return {"last_reply_ns": last, "setup": sent, "brake_before": brake}


def cmd_swd(cfg: Config, args) -> int:
    from . import commands

    elf = Path(args.elf) if args.elf else None
    image = Path(args.bin).read_bytes() if args.bin else (elf.with_suffix(".bin").read_bytes()
                                                          if elf and elf.with_suffix(".bin").exists() else None)
    out = commands.new_result_dir("swd_" + ("info" if args.info else "halt" if args.halt else "onhang")
                                  + (f"_{args.label}" if args.label else ""))
    res: dict = {"elf": str(elf) if elf else None}
    with OpenOcd(log_path=out / "openocd.log") as ocd:
        res["before"] = info(ocd, image)
        print(json.dumps(res["before"], indent=2))
        if args.info:
            commands.write_json(out / "summary.json", res)
            print(f"results: {out}")
            return 0
        if args.on_hang:
            hang = wait_hang(cfg, args.timeout, setup=args.setup)
            ocd.halt()
            res["halt_after_last_reply_ms"] = round((time.monotonic_ns() - hang["last_reply_ns"]) / 1e6, 1)
            res["setup"], res["brake_before"] = hang["setup"], hang["brake_before"]
        else:
            ocd.halt()
        t_halt = time.monotonic()
        res["dump"] = dump(ocd, elf)
        if args.gdb and elf is not None:
            res["gdb"] = gdb_backtrace(elf)
            if not ocd.word(REG["DHCSR"]) & (1 << 17):  # gdb left it running: halt again
                ocd.halt()
        res["halted_s"] = round(time.monotonic() - t_halt, 2)
        if args.stay_halted:
            res["after"] = "left halted"
        else:
            ocd.resume()
            res["after"] = "resumed"
    commands.write_json(out / "summary.json", res)
    d = res["dump"]
    print(f"executing: {d['executing']}  pc {d['regs']['pc']}  lr {d['regs']['lr']}  primask {d['regs']['primask']}"
          f"  lockup {d['lockup']}")
    for f in d["exception_frames"]:
        print(f"frame: interrupted {f['interrupted']} at pc {f['pc']} lr {f['lr']}")
    for a, s in d.get("symbols", {}).items():
        print(f"  {a}  {s}")
    print(f"nvic pending {d['nvic_pending']}  priority {d['nvic_priority']}  usb ep {d['usb_ep']}")
    if "gdb" in res:
        print(res["gdb"])
    print(f"CPU {res['after']} after {res['halted_s']} s; results: {out}")
    return 0
