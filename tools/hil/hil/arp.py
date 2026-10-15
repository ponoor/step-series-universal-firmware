"""hil.py arp: does the board keep sending to an old MAC address when the client is
replaced by another one with the same IP address?

Two USB-LAN adapters of the Mac (NIC A and NIC B, [arp] nic_a / nic_b) are both set to
the client address ([arp] client_ip). Only one is connected at a time: a human swaps
them (T1: the cables at an unmanaged switch, the board's link stays up; T2: the direct
cable to the board moves from NIC A to NIC B, the board's link goes down once). The
bench captures both adapters with tcpdump (filter: ether host <board MAC> or arp),
polls their link status, records the probes P0-P6 with the AD2 in consecutive segments
and logs the OSC messages per adapter (sockets scoped with IP_BOUND_IF). The verdicts
are RECORD only, judged from the capture; what the OSC sockets receive is recorded too.

Steps (--step):
  check  NIC A: restart, /setDestIp, position list reports, /getVersion
  e1     E1 1-6: NIC A, swap to NIC B, wait, /getVersion and /setDestIp from NIC B, back
  e2a    E1 1-5, then /setDestIp from an alias address of NIC B, then from client_ip
  e2b    E1 1-5, then restart ('R') and /setDestIp from NIC B
  c1/c2  E1 1-5, then the HIL serial command of the candidate fix (HIL_PROBE builds),
         a /getVersion burst during it; E1 6 and the command again (back on NIC A the
         board keeps the MAC address of NIC B)
  n1/n2  E1 1 and the command three times while the client is reachable (position
         list reports every 20 ms), no swap
"""

from __future__ import annotations

import re
import signal
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import analyze
from .checks import Bench, Item, recorded
from .config import Config
from .diag import DiagError, parse_perf
from .osc import OscLink

BROADCAST_MAC = "ff:ff:ff:ff:ff:ff"
ETH_ARP, ETH_IPV4 = 0x0806, 0x0800
ARP_REQUEST, ARP_REPLY = 1, 2
STEPS = ("check", "e1", "e2a", "e2b", "c1", "c2", "n1", "n2", "gap", "fix")
# USB serial commands of the candidate fixes (HIL_PROBE builds, src/diagnosis.cpp).
FIX_COMMANDS = {"c1": "N", "c2": "U"}
FIX_DONE = re.compile(r"HIL: (.+?) in (\d+) us")
# A board send is paired with a captured frame within this time (after the offset).
MATCH_TOL_S = 0.005

# --------------------------------------------------------------------------------------
# Pure helpers (tested with synthetic data in tests/test_arp.py)
# --------------------------------------------------------------------------------------


def mac_str(b: bytes) -> str:
    return ":".join(f"{x:02x}" for x in b)


def ip_str(b: bytes) -> str:
    return ".".join(str(x) for x in b)


@dataclass
class Frame:
    t: float  # seconds; wall clock as captured, see to_monotonic()
    src: str
    dst: str
    ethertype: int
    iface: str = ""
    arp_op: int | None = None
    arp_sha: str | None = None
    arp_spa: str | None = None
    arp_tha: str | None = None
    arp_tpa: str | None = None
    ip_src: str | None = None
    ip_dst: str | None = None
    udp_sport: int | None = None
    udp_dport: int | None = None
    osc: str | None = None  # OSC address (or "#bundle")


def parse_pcap(data: bytes, iface: str = "") -> list[Frame]:
    """Ethernet frames of a classic pcap file (as written by tcpdump -w)."""
    if len(data) < 24:
        return []
    magic = data[:4]
    if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
        end = "<"
    elif magic in (b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"):
        end = ">"
    else:
        raise ValueError("not a pcap file (pcapng is not supported)")
    nano = magic in (b"\x4d\x3c\xb2\xa1", b"\xa1\xb2\x3c\x4d")
    linktype = struct.unpack(end + "I", data[20:24])[0]
    if linktype != 1:
        raise ValueError(f"link type {linktype}, expected Ethernet (1)")
    frames = []
    pos = 24
    while pos + 16 <= len(data):
        sec, frac, incl, _orig = struct.unpack(end + "IIII", data[pos:pos + 16])
        pkt = data[pos + 16:pos + 16 + incl]
        pos += 16 + incl
        if len(pkt) < incl or len(pkt) < 14:
            break  # truncated at the end (capture stopped while writing)
        t = sec + frac * (1e-9 if nano else 1e-6)
        frames.append(parse_ethernet(pkt, t, iface))
    return frames


def parse_ethernet(pkt: bytes, t: float, iface: str = "") -> Frame:
    dst, src = mac_str(pkt[0:6]), mac_str(pkt[6:12])
    etype = struct.unpack(">H", pkt[12:14])[0]
    f = Frame(t, src, dst, etype, iface)
    body = pkt[14:]
    if etype == ETH_ARP and len(body) >= 28:
        f.arp_op = struct.unpack(">H", body[6:8])[0]
        f.arp_sha, f.arp_spa = mac_str(body[8:14]), ip_str(body[14:18])
        f.arp_tha, f.arp_tpa = mac_str(body[18:24]), ip_str(body[24:28])
    elif etype == ETH_IPV4 and len(body) >= 20:
        ihl = (body[0] & 0x0F) * 4
        f.ip_src, f.ip_dst = ip_str(body[12:16]), ip_str(body[16:20])
        if body[9] == 17 and len(body) >= ihl + 8:
            f.udp_sport, f.udp_dport = struct.unpack(">HH", body[ihl:ihl + 4])
            payload = body[ihl + 8:]
            if payload[:1] in (b"/", b"#"):
                f.osc = payload.split(b"\0", 1)[0].decode(errors="replace")
    return f


def to_monotonic(frames: list[Frame], clock: list[tuple[float, float]] | float) -> list[Frame]:
    """Frame times moved from the wall clock (pcap) to time.monotonic() seconds. clock:
    (monotonic s, wall - monotonic s) samples taken during the capture, interpolated
    (the wall clock is slewed by NTP: seen 0.9 ms/s after the Mac changed networks),
    or one constant offset."""
    if not isinstance(clock, list):
        clock = [(0.0, float(clock))]
    mono = np.array([c[0] for c in clock])
    off = np.array([c[1] for c in clock])
    for f in frames:
        # the offset at the frame's time; mono ~ wall - offset, refined once
        o = float(np.interp(f.t - off[0], mono, off))
        f.t -= float(np.interp(f.t - o, mono, off))
    return frames


def parse_ifconfig(text: str) -> dict:
    """{'mac', 'status', 'inet': [addresses]} of `ifconfig <if>`."""
    mac = re.search(r"\bether ([0-9a-f:]{17})", text)
    status = re.search(r"\bstatus: (\w+)", text)
    return {"mac": mac.group(1) if mac else None, "status": status.group(1) if status else None,
            "inet": re.findall(r"\binet (\d+\.\d+\.\d+\.\d+)", text)}


def link_transitions(samples: list[tuple[float, str, str]]) -> list[tuple[float, str, str]]:
    """(t, iface, status) polls -> the first sample and every change, per interface."""
    last: dict[str, str] = {}
    out = []
    for t, iface, status in samples:
        if last.get(iface) != status:
            out.append((t, iface, status))
            last[iface] = status
    return out


def capture_nics(step: str, nic: dict[str, str]) -> dict[str, str]:
    """The adapters captured by a step: `gap` uses NIC A only, so it also runs when NIC B
    is not attached (its tcpdump would fail); the other steps capture both."""
    return {"A": nic["A"]} if step == "gap" else dict(nic)


def nic_names(info: dict[str, dict]) -> dict[str, str]:
    """MAC address -> "NIC A" / "NIC B" of the `ifconfig` results; an adapter that is not
    attached has no MAC address and is left out."""
    return {i["mac"]: f"NIC {k}" for k, i in info.items() if i.get("mac")}


def mac_name(mac: str | None, names: dict[str, str]) -> str:
    if mac is None:
        return "?"
    if mac == BROADCAST_MAC:
        return "broadcast"
    return names.get(mac, mac)


def frame_summary(frames: list[Frame], board_mac: str, board_ip: str, names: dict[str, str],
                  t0: float, t1: float, nic_b: str = "") -> dict:
    """What the capture shows between t0 and t1 (monotonic s). names: MAC -> name
    (e.g. NIC A / NIC B). Times in the lists are ms from t0."""
    sel = [f for f in frames if t0 <= f.t < t1]
    ms = lambda f: round((f.t - t0) * 1e3, 1)  # noqa: E731
    out: dict = {"board_ipv4_by_dst": {}, "board_osc_by_dst": {}, "board_arp_requests": [],
                 "board_arp_replies": [], "arp_to_board": [], "arp_other": [], "udp_to_board_by_src": {}}
    for f in sel:
        if f.src == board_mac:
            if f.ethertype == ETH_IPV4:
                d = mac_name(f.dst, names)
                out["board_ipv4_by_dst"][d] = out["board_ipv4_by_dst"].get(d, 0) + 1
                if f.osc:
                    key = f"{d} {f.osc}"
                    out["board_osc_by_dst"][key] = out["board_osc_by_dst"].get(key, 0) + 1
            elif f.arp_op == ARP_REQUEST:
                out["board_arp_requests"].append({"ms": ms(f), "who_has": f.arp_tpa, "iface": f.iface})
            elif f.arp_op == ARP_REPLY:
                out["board_arp_replies"].append({"ms": ms(f), "to": mac_name(f.arp_tha, names), "iface": f.iface})
        elif f.arp_op is not None:
            entry = {"ms": ms(f), "op": "request" if f.arp_op == ARP_REQUEST else "reply",
                     "from": mac_name(f.arp_sha, names), "spa": f.arp_spa, "tpa": f.arp_tpa, "iface": f.iface}
            if f.arp_spa == f.arp_tpa:
                entry["gratuitous"] = True
            if f.arp_tpa == board_ip or f.arp_tha == board_mac:
                out["arp_to_board"].append(entry)
            else:
                out["arp_other"].append(entry)
        elif f.dst == board_mac and f.udp_dport is not None:
            s = mac_name(f.src, names)
            out["udp_to_board_by_src"][s] = out["udp_to_board_by_src"].get(s, 0) + 1
    if nic_b:
        mac_b = next((m for m, n in names.items() if n == "NIC B"), None)
        # UDP sends of the board that reached NIC B (not its ARP replies)
        out["nic_b_frames_to_nic_b"] = sum(1 for f in sel if f.iface == nic_b and f.src == board_mac
                                           and f.dst == mac_b and f.ethertype == ETH_IPV4)
    return out


def board_send_times(frames: list[Frame], board_mac: str) -> np.ndarray:
    """Times of the IPv4 frames sent by the board (one per UDP send, i.e. per P5 pulse)."""
    return np.array(sorted(f.t for f in frames if f.src == board_mac and f.ethertype == ETH_IPV4))


def match_times(a: np.ndarray, b: np.ndarray, tol: float = MATCH_TOL_S) -> dict:
    """Pairs the events a (e.g. captured board frames) with b (e.g. P5 rises) one to one.
    The clock offset (median of b - a over the nearest neighbours) is removed first;
    then each a is paired with the first unused b within tol, in time order."""
    a, b = np.sort(np.asarray(a, float)), np.sort(np.asarray(b, float))
    res = {"a": int(a.size), "b": int(b.size), "matched": 0, "offset_ms": None}
    if a.size == 0 or b.size == 0:
        return res
    idx = np.clip(np.searchsorted(b, a), 1, b.size - 1) if b.size > 1 else np.zeros(a.size, int)
    near = b[idx]
    if b.size > 1:
        left = b[idx - 1]
        near = np.where(np.abs(left - a) < np.abs(near - a), left, near)
    offset = float(np.median(near - a))
    res["offset_ms"] = offset * 1e3
    j = 0
    used = np.zeros(b.size, bool)
    for t in a + offset:
        while j < b.size and b[j] < t - tol:
            j += 1
        k = j
        while k < b.size and b[k] <= t + tol and used[k]:
            k += 1
        if k < b.size and abs(b[k] - t) <= tol and not used[k]:
            used[k] = True
            res["matched"] += 1
    return res


def covered(t: np.ndarray, coverage: list[tuple[float, float]]) -> np.ndarray:
    """Mask of the times inside one of the (start, end) intervals."""
    t = np.asarray(t, float)
    m = np.zeros(t.size, bool)
    for s, e in coverage:
        m |= (t >= s) & (t <= e)
    return m


def phase_windows(events: list[dict], t_end: float) -> list[tuple[str, float, float]]:
    """Phases: each event with "phase" starts one, until the next one (or t_end). A
    repeated name gets " #2", " #3", ... (the phases are keyed by name)."""
    marks = [(e["t"], e["phase"]) for e in events if e.get("phase")]
    out = []
    seen: dict[str, int] = {}
    for i, (t, name) in enumerate(marks):
        seen[name] = seen.get(name, 0) + 1
        if seen[name] > 1:
            name = f"{name} #{seen[name]}"
        out.append((name, t, marks[i + 1][0] if i + 1 < len(marks) else t_end))
    return out


def h1_verdict(after: dict, client_ip: str) -> dict:
    """H1 from the merged frame summary of the phases after the swap (before any fix):
    no ARP request of the board for client_ip and no frame of the board to NIC B."""
    arp = [r for r in after["board_arp_requests"] if r["who_has"] == client_ip]
    to_b = after["board_ipv4_by_dst"].get("NIC B", 0)
    to_a = after["board_ipv4_by_dst"].get("NIC A", 0)
    return {"board_arp_requests_for_client": len(arp), "frames_to_nic_b": to_b, "frames_to_nic_a_seen": to_a,
            "h1": len(arp) == 0 and to_b == 0}


ARP_REFRESH_GAP_S = 1.5  # src/arpRefresh.h
DISCARD_PORT = 9


def is_refresh_dummy(f: Frame, board_mac: str) -> bool:
    """The board's dummy broadcast of the ARP refresh (1 byte to 255.255.255.255:9)."""
    return f.src == board_mac and f.ip_dst == "255.255.255.255" and f.udp_dport == DISCARD_PORT


def expected_refreshes(packets: list[tuple[float, str, str | None]], dest: str | None = None,
                       dest_set: bool = False) -> list[float]:
    """The firmware rule (src/arpRefresh.cpp) applied to the packets received by the board,
    (time s, source IP, OSC address) in time order. /setDestIp sets destIp and
    isDestIpSet after the rule is applied. Returns the times of the packets that refresh."""
    out = []
    rec_ip, rec_t = None, None
    for t, src, osc in packets:
        if src == dest:
            gap = rec_ip != src or t - rec_t >= ARP_REFRESH_GAP_S
            rec_ip, rec_t = src, t
            if gap and dest_set:
                out.append(t)
        if osc == "/setDestIp":
            dest, dest_set = src, True
    return out


def refresh_sequences(frames: list[Frame], board_mac: str, board_ip: str, client_ip: str) -> list[dict]:
    """For each dummy broadcast of the board: the packet to the board just before it (the
    trigger) and the times (ms from the trigger) of the dummy, the board's next ARP
    request for client_ip, the ARP reply to the board and the board's next unicast frame
    to client_ip (the reply). order_ok: trigger < dummy < ARP request < ARP reply < reply,
    with no other board frame between the dummy and the ARP request."""
    out = []
    for i, f in enumerate(frames):
        if not is_refresh_dummy(f, board_mac):
            continue
        trig = next((g for g in reversed(frames[:i]) if g.dst == board_mac and g.ip_dst == board_ip
                     and g.udp_dport is not None), None)
        rest = frames[i + 1:]
        req = next((k for k, g in enumerate(rest) if g.src == board_mac and g.arp_op == ARP_REQUEST
                    and g.arp_tpa == client_ip), None)
        rep = next((g for g in rest if g.arp_op == ARP_REPLY and g.arp_tha == board_mac
                    and g.arp_spa == client_ip), None)
        uni = next((g for g in rest if g.src == board_mac and g.ip_dst == client_ip), None)
        t0 = trig.t if trig else f.t
        ms = lambda g: None if g is None else round((g.t - t0) * 1e3, 3)  # noqa: E731
        between = [] if req is None else [g for g in rest[:req] if g.src == board_mac]
        times = [t0 if trig else None, f.t, None if req is None else rest[req].t,
                 None if rep is None else rep.t, None if uni is None else uni.t]
        ok = None not in times and all(a < b for a, b in zip(times, times[1:])) and not between
        out.append({"t": f.t, "trigger": trig.osc if trig else None, "dummy_ms": ms(f),
                    "arp_request_ms": ms(rest[req]) if req is not None else None, "arp_reply_ms": ms(rep),
                    "reply_ms": ms(uni), "reply": uni.osc if uni else None, "order_ok": ok})
    return out


def swap_recovery(frames: list[Frame], board_mac: str, board_ip: str, client_ip: str, new_mac: str,
                  t0: float, t1: float) -> dict:
    """After a swap to the adapter new_mac (from t0, when it has the address, to t1): the
    first packet from it to the board, then (ms from that packet) the board's first dummy
    broadcast, ARP request for client_ip and IPv4 frame to new_mac; the dummies in the
    window and the board's OSC frames to new_mac by address."""
    sel = [f for f in frames if t0 <= f.t < t1]
    rx = next((f for f in sel if f.src == new_mac and f.dst == board_mac and f.ip_dst == board_ip
               and f.udp_dport is not None), None)
    out: dict = {"first_rx": rx.osc if rx else None, "first_rx_s": None if rx is None else round(rx.t - t0, 3),
                 "dummies": sum(1 for f in sel if is_refresh_dummy(f, board_mac)), "osc_to_new": {}}
    for f in sel:
        if f.src == board_mac and f.dst == new_mac and f.ethertype == ETH_IPV4:
            out["osc_to_new"][f.osc] = out["osc_to_new"].get(f.osc, 0) + 1
    if rx is None:
        out.update(dummy_ms=None, arp_request_ms=None, first_to_new_ms=None, first_to_new=None)
        return out
    after = [f for f in sel if f.t >= rx.t]
    ms = lambda f: None if f is None else round((f.t - rx.t) * 1e3, 3)  # noqa: E731
    out["dummy_ms"] = ms(next((f for f in after if is_refresh_dummy(f, board_mac)), None))
    out["arp_request_ms"] = ms(next((f for f in after if f.src == board_mac and f.arp_op == ARP_REQUEST
                                     and f.arp_tpa == client_ip), None))
    first = next((f for f in after if f.src == board_mac and f.dst == new_mac and f.ethertype == ETH_IPV4), None)
    out["first_to_new_ms"] = ms(first)
    out["first_to_new"] = first.osc if first else None
    return out


def nearest_pulse(rise: np.ndarray, width: np.ndarray, t: float, tol: float = 0.003) -> tuple[int | None, float | None]:
    """Index and width of the P5 pulse nearest to t (within tol)."""
    if rise.size == 0:
        return None, None
    k = int(np.argmin(np.abs(rise - t)))
    return (k, float(width[k])) if abs(rise[k] - t) <= tol else (None, None)


def merge_summaries(parts: list[dict]) -> dict:
    out: dict = {"board_ipv4_by_dst": {}, "board_osc_by_dst": {}, "board_arp_requests": []}
    for p in parts:
        for k in ("board_ipv4_by_dst", "board_osc_by_dst"):
            for kk, v in p[k].items():
                out[k][kk] = out[k].get(kk, 0) + v
        out["board_arp_requests"] += p["board_arp_requests"]
    return out


# --------------------------------------------------------------------------------------
# Capture, link status and the AD2 in the background
# --------------------------------------------------------------------------------------


def ifconfig(iface: str) -> dict:
    return parse_ifconfig(subprocess.run(["ifconfig", iface], capture_output=True, text=True).stdout)


def wall_minus_mono() -> float:
    """time.time() - time.monotonic() in seconds (the pair read closest together)."""
    best = None
    for _ in range(5):
        m0 = time.monotonic_ns()
        w = time.time_ns()
        m1 = time.monotonic_ns()
        if best is None or m1 - m0 < best[0]:
            best = (m1 - m0, w - (m0 + m1) // 2)
    return best[1] / 1e9


class Tcpdump:
    """tcpdump -i iface -e -n -U --immediate-mode -w path filter, as a subprocess (no
    sudo: the BPF devices must be readable, e.g. with Wireshark's ChmodBPF). Without
    --immediate-mode the frames of the last second (BPF buffer) were missing after stop."""

    def __init__(self, iface: str, bpf_filter: str, path: Path):
        self.iface, self.filter, self.path = iface, bpf_filter, path
        self.err_path = path.with_suffix(".log")
        self._p: subprocess.Popen | None = None

    def start(self) -> "Tcpdump":
        self._err = open(self.err_path, "w")
        self._p = subprocess.Popen(["tcpdump", "-i", self.iface, "-e", "-n", "-U", "--immediate-mode",
                                    "-w", str(self.path),
                                    self.filter], stdout=subprocess.DEVNULL, stderr=self._err)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self._p.poll() is not None:
                raise DiagError(f"tcpdump on {self.iface} exited: {self.err_path.read_text().strip()}")
            if "listening on" in self.err_path.read_text():
                return self
            time.sleep(0.05)
        raise DiagError(f"tcpdump on {self.iface} did not start: {self.err_path.read_text().strip()}")

    def stop(self) -> None:
        if self._p is not None and self._p.poll() is None:
            self._p.send_signal(signal.SIGINT)
            try:
                self._p.wait(5.0)
            except subprocess.TimeoutExpired:
                self._p.kill()
        if self._p is not None:
            self._err.close()
            self._p = None

    def frames(self) -> list[Frame]:
        return parse_pcap(self.path.read_bytes(), self.iface) if self.path.exists() else []


class LinkMonitor:
    """Polls the link status of the interfaces every `period` s, and samples the wall
    clock against the monotonic clock (for the pcap time stamps)."""

    def __init__(self, ifaces: list[str], period: float = 0.1):
        self.ifaces, self.period = ifaces, period
        self.samples: list[tuple[float, str, str]] = []
        self.clock: list[tuple[float, float]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="link-monitor", daemon=True)
        self._lock = threading.Lock()

    def start(self) -> "LinkMonitor":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                self.clock.append((time.monotonic(), wall_minus_mono()))
            for i in self.ifaces:
                st = ifconfig(i)["status"] or "?"
                with self._lock:
                    self.samples.append((time.monotonic(), i, st))
            self._stop.wait(self.period)

    def status(self, iface: str) -> str | None:
        with self._lock:
            for t, i, s in reversed(self.samples):
                if i == iface:
                    return s
        return ifconfig(iface)["status"]

    def transitions(self) -> list[tuple[float, str, str]]:
        with self._lock:
            return link_transitions(list(self.samples))


class SegmentRecorder:
    """P0-P6 at 1 MHz in consecutive AD2 recordings of `segment_s`, saved as
    ad2_<k>.npz. Keeps the P1 rises and the P5 pulses in time.monotonic() seconds.
    Stop it across a restart of the board: the USB re-enumeration drops part of the
    AD2 stream without reporting it (seen: 0.53 s of 10.5 s). A segment whose data are
    shorter than its host time by more than MAX_SHORT_S is reported and not used."""

    MAX_SHORT_S = 0.005

    def __init__(self, b: Bench, segment_s: float = 30.0):
        self.b, self.segment_s = b, segment_s
        self.segments: list[dict] = []
        self.problems: list[str] = []
        self.error: BaseException | None = None
        self._k = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> "SegmentRecorder":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ad2-segments", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        if self._thread is not None:
            self._stop.set()
            self._thread.join(self.segment_s + 10.0)
            self._thread = None

    def _run(self) -> None:
        probe = self.b.cfg.probe_dio
        try:
            while not self._stop.is_set():
                rec = self.b.start(probe, 1e6)
                try:
                    self._stop.wait(self.segment_s)
                finally:
                    cap = rec.stop()
                name = f"ad2_{self._k}"
                self._k += 1
                self.b.save_capture(cap, name)
                t0 = cap.host_trigger_ns / 1e9
                short = (cap.host_stop_ns - cap.host_trigger_ns) / 1e9 - cap.digital.duration
                if not cap.ok or short > self.MAX_SHORT_S:
                    self.problems.append(f"{name}: " + "; ".join(cap.problems() + [
                        f"data {cap.digital.duration:.3f} s for {short + cap.digital.duration:.3f} s host time"
                        ] * (short > self.MAX_SHORT_S)) + " (not used)")
                    continue
                r, w = analyze.pulse_widths(analyze.bit_edges(cap.digital, probe[5]))
                self.segments.append({"name": name, "t0": t0, "t1": t0 + cap.digital.duration,
                                      "p1": analyze.bit_edges(cap.digital, probe[1]).rise + t0,
                                      "p5_rise": r + t0, "p5_width": w})
        except BaseException as ex:  # reported in the summary
            self.error = ex

    @property
    def coverage(self) -> list[tuple[float, float]]:
        return [(g["t0"], g["t1"]) for g in self.segments]

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cat = lambda k: np.concatenate([g[k] for g in self.segments]) if self.segments else np.zeros(0)  # noqa: E731
        return cat("p1"), cat("p5_rise"), cat("p5_width")


# --------------------------------------------------------------------------------------
# The run
# --------------------------------------------------------------------------------------


class Heartbeat:
    """/setDestIp every `period` s through `lk` (replaced by the run when the adapters are
    swapped). Sends that fail (adapter unplugged, socket closed) are ignored."""

    def __init__(self, lk: OscLink, period: float = 1.0):
        self.lk, self.period = lk, period
        self.sent: list[tuple[float, str]] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="heartbeat", daemon=True)

    def start(self) -> "Heartbeat":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(2.0)

    def _run(self) -> None:
        t = time.monotonic()
        while not self._stop.is_set():
            lk = self.lk
            try:
                if lk is None:
                    raise RuntimeError("paused")
                lk.send("/setDestIp")
                self.sent.append((time.monotonic(), lk.bound_if))
            except Exception:
                pass
            t += self.period
            self._stop.wait(max(0.0, t - time.monotonic()))


class ArpBench(Bench):
    """Bench with the OSC links scoped to NIC A / NIC B instead of the default link."""

    def __enter__(self) -> "ArpBench":
        self.ad2 = self.commands.open_ad2(self.cfg).open()
        self.ser.open()
        return self


class ArpRun:
    def __init__(self, b: ArpBench, args):
        a = b.cfg.section("arp")
        self.b, self.args = b, args
        self.topology = args.topology
        self.nic = {"A": a["nic_a"], "B": a["nic_b"]}
        self.client_ip = a["client_ip"]
        self.alias_ip = a["alias_ip"]
        self.report_ms = int(a["report_ms"])
        self.events: list[dict] = []
        self.links: list[tuple[str, OscLink]] = []
        self.serial_out: dict[str, str] = {}
        self.perf: dict[str, dict] = {}
        self.mono0 = time.monotonic()

    # --- helpers ---
    def event(self, name: str, phase: str | None = None, **detail) -> float:
        t = time.monotonic()
        self.events.append({"t": t, "s": round(t - self.mono0, 3), "name": name, "phase": phase, **detail})
        print(f"  [{t - self.mono0:7.1f} s] {name}" + (f" {detail}" if detail else ""), flush=True)
        return t

    def link(self, nic: str, local_ip: str | None = None) -> OscLink:
        """Opens a socket pair scoped to the NIC; earlier ones on the same address are
        closed first (one receive socket per address and port; their logs are kept)."""
        cfg = self.b.cfg
        for _, old in self.links:
            if old.local_ip == (local_ip or self.client_ip):
                old.close()
        lk = OscLink(cfg.board_ip, cfg.board_in_port, cfg.board_out_port, local_ip=local_ip or self.client_ip,
                     bound_if=self.nic[nic]).open()
        self.links.append((f"{nic}_{local_ip or self.client_ip}", lk))
        self.b.extra_links.append(lk)
        return lk

    def read_perf(self, key: str) -> dict:
        t = self.b.ser.write("p")
        if self.b.ser.wait_for(r"udpSendDropped\s+\d+", t, 5.0) is None:
            raise DiagError("no 'p' output")
        self.b.ser.wait_for(r"arpRefreshFailed\s+\d+", t, 0.3)  # firmware with the ARP refresh
        self.perf[key] = parse_perf(self.b.ser.text(t))
        return self.perf[key]

    def wait_link(self, active: str) -> bool:
        """NIC `active` is up and the other one is not."""
        other = "B" if active == "A" else "A"
        return (self.mon.status(self.nic[active]) == "active"
                and self.mon.status(self.nic[other]) != "active")

    def swap(self, to: str, step: str) -> None:
        frm = "B" if to == "A" else "A"
        if self.topology == "T1":
            how = (f"unplug the cable of NIC {frm} ({self.nic[frm]}) from the switch, then plug the cable of "
                   f"NIC {to} ({self.nic[to]}) into the switch")
        else:
            how = (f"unplug the direct cable from NIC {frm} ({self.nic[frm]}) and plug it into NIC {to} "
                   f"({self.nic[to]})")
        self.event(f"{step} swap to NIC {to} requested", phase=f"{step} swap to {to}")
        if not self.b.operator(f"{step}: {how}", done=lambda: self.wait_link(to), timeout=600.0):
            raise DiagError(f"NIC {to} did not become the only active adapter")
        # The Enter (or the poll) can come before the link change is seen.
        deadline = time.monotonic() + 10.0
        while not self.wait_link(to) and time.monotonic() < deadline:
            time.sleep(0.1)
        if not self.wait_link(to):
            raise DiagError(f"NIC {to} is not the only active adapter: A {self.mon.status(self.nic['A'])}, "
                            f"B {self.mon.status(self.nic['B'])}")
        self.wait_address(to)
        self.event(f"{step} {self.client_ip} on NIC {to}")

    def wait_address(self, nic: str) -> None:
        """macOS assigns the address some time after the link comes up."""
        deadline = time.monotonic() + 30.0
        while self.client_ip not in ifconfig(self.nic[nic])["inet"]:
            if time.monotonic() > deadline:
                raise DiagError(f"{self.client_ip} not on {self.nic[nic]} 30 s after its link came up")
            time.sleep(0.1)

    def query(self, lk: OscLink, address: str, reply: str, step: str, *args, window: float = 2.0) -> dict:
        t = self.event(f"{step} {address} from {lk.bound_if} {lk.local_ip}", phase=f"{step} {address}")
        sent = lk.send(address, *args)
        r = lk.wait_for(reply, window, sent.t_ns)
        res = {"reply": list(r.args) if r else None, "reply_ms": (r.t_ns - sent.t_ns) / 1e6 if r else None}
        self.events[-1]["result"] = res
        time.sleep(max(0.0, t + window - time.monotonic()))
        return res

    # --- E1 1-6 ---
    def establish(self, heartbeat: bool = False) -> OscLink:
        """E1 1: NIC A connected, restart, /setDestIp and position list reports from NIC A.
        heartbeat: starts self.hb (/setDestIp every 1 s) right after the /setDestIp."""
        if not self.wait_link("A"):
            if not self.b.operator(f"connect NIC A ({self.nic['A']}) only (NIC B unplugged)",
                                   done=lambda: self.wait_link("A"), timeout=600.0):
                raise DiagError("NIC A is not the only active adapter")
        self.wait_address("A")
        lk = self.link("A")
        self.b.link = lk
        self.event("1 restart ('R')", phase="1 restart")
        self.rec.stop()
        self.b.reboot(connect=False)
        self.b.connect()
        if heartbeat:
            self.hb = Heartbeat(lk).start()
        self.rec.start()
        time.sleep(0.5)
        self.event("1 /setDestIp from NIC A answered; position list reports on", phase="1 NIC A")
        lk.send("/setPositionListReportInterval", self.report_ms)
        time.sleep(3.0)
        return lk

    def old_state(self, wait_s: float, getversion: bool = True) -> OscLink:
        """E1 2-5: swap to NIC B, wait, /getVersion and /setDestIp from NIC B."""
        self.read_perf("before_swap")
        self.swap("B", "2")
        lk_b = self.link("B")
        self.event(f"3 wait {wait_s:.0f} s on NIC B", phase="3 wait")
        time.sleep(wait_s)
        self.read_perf("after_wait")
        if getversion:
            self.query(lk_b, "/getVersion", "/version", "4")
        self.query(lk_b, "/setDestIp", "/destIp", "5", window=self.args.observe)
        self.read_perf("after_5")
        return lk_b

    def back_to_a(self) -> None:
        """E1 6: back to NIC A; reports without a command for 10 s, then /getVersion."""
        self.swap("A", "6")
        lk_a = self.link("A")
        self.event("6 on NIC A, no command for 10 s", phase="6 back on A")
        time.sleep(10.0)
        self.query(lk_a, "/getVersion", "/version", "6")
        lk_a.send("/setPositionListReportInterval", 0)
        self.b.link = lk_a

    # --- steps ---
    def run_check(self) -> None:
        lk = self.establish()
        self.query(lk, "/getVersion", "/version", "check")
        lk.send("/setPositionListReportInterval", 0)

    def run_e1(self) -> None:
        self.establish()
        self.old_state(self.args.wait if self.args.wait is not None else 300.0)
        self.back_to_a()

    def run_e2a(self) -> None:
        self.establish()
        lk_b = self.old_state(self.args.wait if self.args.wait is not None else 30.0)
        iface = self.nic["B"]
        add = f"sudo ifconfig {iface} alias {self.alias_ip} 255.255.255.255"
        remove = f"sudo ifconfig {iface} -alias {self.alias_ip}"
        present = lambda: self.alias_ip in ifconfig(iface)["inet"]  # noqa: E731
        self.event("a alias requested", phase="a alias add")
        if not present() and not self.b.operator(f"a: add the alias address: {add}", done=present, timeout=600.0):
            raise DiagError(f"{self.alias_ip} was not added to {iface}")
        lk_alias = self.link("B", self.alias_ip)
        self.query(lk_alias, "/setDestIp", "/destIp", "a", window=10.0)
        self.query(lk_b, "/setDestIp", "/destIp", "a", window=self.args.observe)
        self.event("a alias removal requested", phase="a alias remove")
        if not self.b.operator(f"a: remove the alias address: {remove}", done=lambda: not present(),
                               timeout=600.0):
            raise DiagError(f"{self.alias_ip} was not removed from {iface}")
        self.back_to_a()

    def run_e2b(self) -> None:
        self.establish()
        lk_b = self.old_state(self.args.wait if self.args.wait is not None else 30.0)
        self.b.link = lk_b
        self.event("b restart ('R') on NIC B", phase="b restart")
        self.rec.stop()
        self.b.reboot(connect=False)
        self.b.connect()
        self.rec.start()
        time.sleep(0.5)
        self.event("b /setDestIp from NIC B answered; position list reports on", phase="b after restart")
        lk_b.send("/setPositionListReportInterval", self.report_ms)
        time.sleep(self.args.observe)
        self.back_to_a()

    def run_gap(self) -> None:
        """Task 3 of the ARP refresh: packets from the client with and without gaps of
        1.5 s, NIC A only. 'p' is read (and reset) at the end of each pattern."""
        if not self.wait_link("A"):
            raise DiagError("NIC A must be the only active adapter")
        self.wait_address("A")
        lk = self.link("A")
        self.b.link = lk
        self.event("restart ('R')", phase="restart")
        self.rec.stop()
        self.b.reboot(connect=False)
        self.rec.start()
        time.sleep(1.0)
        self.read_perf("restart")
        net = self.b.config_json()["network"]
        self.initial = {"destIp": ".".join(str(int(x)) for x in net["destIp"]),
                        "canSendMsgBeforeDestIp": bool(net.get("canSendMsgBeforeDestIp", False))}
        self.event("0 /setDestIp (first)", phase="gap 0")
        self.b.connect()
        lk.send("/setPositionListReportInterval", self.report_ms)
        self.read_perf("gap 0")

        def every(address: str, period: float, n: int, first_after: float) -> None:
            t = time.monotonic() + first_after
            for _ in range(n):
                time.sleep(max(0.0, t - time.monotonic()))
                lk.send(address)
                t += period

        self.event("a /setDestIp every 1.0 s for 30 s", phase="gap a")
        every("/setDestIp", 1.0, 30, 0.5)
        time.sleep(0.5)
        self.read_perf("gap a")
        self.event("b /getVersion every 2.0 s for 20 s", phase="gap b")
        every("/getVersion", 2.0, 10, 1.5)
        time.sleep(0.5)
        self.read_perf("gap b")
        self.event("c 3 s silence, then /getVersion every 10 ms x 10", phase="gap c")
        every("/getVersion", 0.01, 10, 3.0)
        time.sleep(0.5)
        self.read_perf("gap c")
        self.event("d 3 s silence, then /getVersion (another source: none on this Mac)", phase="gap d")
        every("/getVersion", 0.0, 1, 3.0)
        time.sleep(0.5)
        self.read_perf("gap d")
        lk.send("/setPositionListReportInterval", 0)

    def read_dest_mac(self, label: str) -> dict:
        """Destination MAC address from 's' and 'T' (firmware with destMac)."""
        text = self.b.ser.command("s", until=r"Destination MAC address : ([0-9A-Fa-f:]{17})")
        m = re.search(r"Destination MAC address : ([0-9A-Fa-f:]{17})", text).group(1).lower()
        j = self.b.status_json()["board"].get("destMac")
        t = ":".join(f"{int(x):02x}" for x in j) if j else None
        d = {"label": label, "s": m, "T": t, "t_s": round(time.monotonic() - self.mono0, 3)}
        self.dest_macs.append(d)
        self.event(f"{label}: destination MAC s {m}, T {t}")
        return d

    def run_fix_swap(self, heartbeat: bool) -> None:
        """Task 4 of the ARP refresh (T1). F1 (heartbeat): /setDestIp every 1.0 s from the
        connected adapter, 30 s on NIC A, swap to NIC B, 30 s, swap back, 30 s. F2: no
        heartbeat; after each swap 10 s without a command, the destination MAC, /getVersion
        from the new adapter, 10 s, the destination MAC."""
        tag = "F1" if heartbeat else "F2"
        self.dest_macs = []
        self.hb = None
        self.establish(heartbeat)
        hb = self.hb
        self.read_perf(f"{tag} start")
        if heartbeat:
            self.event(f"{tag} heartbeat on NIC A, 30 s", phase=f"{tag} heartbeat on A")
            time.sleep(30.0)
            self.read_perf(f"{tag} heartbeat on A")
        try:
            for to in ("B", "A"):
                if hb is not None:
                    # Sent only from the connected adapter: a socket of the unplugged one
                    # sent through the other adapter before it had its address (no ARP
                    # reply yet: failed send, send pause).
                    hb.lk = None
                self.swap(to, f"{tag} {to}")
                new = self.link(to)
                self.b.link = new
                if hb is not None:
                    hb.lk = new
                    self.event(f"{tag} heartbeat on NIC {to}, 30 s", phase=f"{tag} on {to}")
                    time.sleep(30.0)
                    self.read_dest_mac(f"{tag} on {to} after 30 s")
                else:
                    self.event(f"{tag} on NIC {to}, no command for 10 s", phase=f"{tag} on {to}")
                    time.sleep(10.0)
                    self.read_dest_mac(f"{tag} on {to} before the first packet")
                    self.query(new, "/getVersion", "/version", f"{tag} {to}", window=10.0)
                    self.read_dest_mac(f"{tag} on {to} after /getVersion")
                self.read_perf(f"{tag} on {to}")
        finally:
            if hb is not None:
                hb.stop()
        self.b.link.send("/setPositionListReportInterval", 0)

    def fix_command(self, cmd: str, step: str, lk: OscLink | None, burst: bool) -> dict:
        """Sends the serial command; with burst, /getVersion 20 times every 2 ms around it."""
        res: dict = {}
        sender = None
        sent: list[int] = []
        if burst and lk is not None:
            def run():
                for _ in range(20):
                    sent.append(lk.send("/getVersion").t_ns)
                    time.sleep(0.002)
            sender = threading.Thread(target=run, daemon=True)
            sender.start()
            time.sleep(0.01)
        t_cmd = self.b.ser.write(cmd)
        self.events.append({"t": t_cmd / 1e9, "s": round(t_cmd / 1e9 - self.mono0, 3), "name": f"{step} '{cmd}' written",
                            "phase": None})
        m = self.b.ser.wait_for(FIX_DONE.pattern, t_cmd, 2.0)
        res["serial"] = self.b.ser.text(t_cmd)[-300:]
        res["command_us"] = int(m.group(2)) if m else None
        if sender is not None:
            sender.join(2.0)
            time.sleep(0.5)
            res["burst_sent"] = len(sent)
            res["burst_t"] = [sent[0] / 1e9, sent[-1] / 1e9 + 0.05] if sent else None
            res["burst_replies"] = len([e for e in lk.entries("rx", sent[0]) if e.address == "/version"]) if sent else 0
        return res

    def run_fix(self, which: str) -> None:
        cmd = FIX_COMMANDS[which]
        self.establish()
        lk_b = self.old_state(self.args.wait if self.args.wait is not None else 30.0)
        # A new AD2 recording, so that the command is not in the gap between two (0.3 s).
        self.rec.stop()
        self.rec.start()
        time.sleep(1.0)
        self.event(f"{which} '{cmd}' with the old MAC, /getVersion burst", phase=f"{which} fix")
        self.serial_out["fix"] = self.fix_command(cmd, which, lk_b, burst=True)
        self.events[-2]["result"] = {k: v for k, v in self.serial_out["fix"].items() if k != "serial"}
        time.sleep(self.args.observe)
        self.back_to_a()
        # Back on NIC A the board keeps the MAC address of NIC B: the command again.
        self.b.link.send("/setPositionListReportInterval", self.report_ms)
        self.event(f"{which} '{cmd}' back on NIC A", phase=f"{which} fix on A")
        self.serial_out["fix_on_a"] = self.fix_command(cmd, which, None, burst=False)
        time.sleep(5.0)
        self.b.link.send("/setPositionListReportInterval", 0)

    def run_normal(self, which: str) -> None:
        """The command while the client is reachable (no swap): reports every 20 ms."""
        cmd = FIX_COMMANDS[which]
        lk_a = self.establish()
        lk_a.send("/setPositionListReportInterval", 20)
        self.event(f"{which} reachable, reports every 20 ms", phase=f"{which} normal before")
        time.sleep(3.0)
        self.rec.stop()
        self.rec.start()
        time.sleep(1.0)
        for k in range(3):
            self.event(f"{which} '{cmd}' while reachable", phase=f"{which} normal")
            self.serial_out[f"normal_{k}"] = self.fix_command(cmd, which, None, burst=False)
            time.sleep(3.0)
        lk_a.send("/setPositionListReportInterval", 0)


# --------------------------------------------------------------------------------------
# Summary
# --------------------------------------------------------------------------------------


def report_intervals(entries, t0: float, t1: float) -> dict:
    """Intervals (ms) between the /positionList messages received from t0 to t1."""
    t = np.sort([e.t_ns / 1e9 for e in entries if e.address == "/positionList" and t0 <= e.t_ns / 1e9 < t1])
    return analyze.stats(np.diff(t), 1e3) if t.size > 1 else analyze.stats(np.zeros(0))


def summarize(run: ArpRun, frames: list[Frame], names: dict[str, str], board_mac: str,
              rec: SegmentRecorder, t_end: float) -> dict:
    cfg = run.b.cfg
    p1, p5r, p5w = rec.arrays()
    board_tx = board_send_times(frames, board_mac)
    phases = {}
    rx = [(name, e) for name, lk in run.links for e in lk.entries("rx")]
    for name, t0, t1 in phase_windows(run.events, t_end):
        fs = frame_summary(frames, board_mac, cfg.board_ip, names, t0, t1, run.nic["B"])
        cov = [(max(s, t0), min(e, t1)) for s, e in rec.coverage if e > t0 and s < t1]
        sel5 = (p5r >= t0) & (p5r < t1)
        tx = board_tx[(board_tx >= t0) & (board_tx < t1)]
        tx_cov = tx[covered(tx, cov)]
        app: dict[str, int] = {}
        for lname, e in rx:
            if t0 <= e.t_ns / 1e9 < t1:
                key = f"{lname} {e.address}"
                app[key] = app.get(key, 0) + 1
        first5 = np.argmax(sel5) if sel5.any() else None
        phases[name] = {
            "t0_s": round(t0 - run.mono0, 3), "duration_s": round(t1 - t0, 3),
            "ad2_covered_s": round(sum(e - s for s, e in cov), 3),
            "p1": int(np.sum((p1 >= t0) & (p1 < t1))), "p5": int(sel5.sum()),
            "p5_ms": analyze.stats(p5w[sel5], 1e3),
            "first_p5_after_ms": None if first5 is None else round((p5r[first5] - t0) * 1e3, 2),
            "first_p5_width_ms": None if first5 is None else round(float(p5w[first5]) * 1e3, 3),
            "capture": fs,
            "p5_vs_capture": match_times(tx_cov, p5r[sel5]),
            "app_rx": app,
            "app_report_ms": report_intervals([e for _, e in rx], t0, t1),
        }
    links = [{"t_s": round(t - run.mono0, 3), "iface": i, "status": s} for t, i, s in run.mon.transitions()]
    # Time mapping check: the n-th OSC packet sent to the board starts a P1 pulse.
    tx = np.array(sorted(e.t_ns / 1e9 for _, lk in run.links for e in lk.entries("tx")))
    segs = []
    for g in rec.segments:
        m = match_times(tx[(tx >= g["t0"]) & (tx < g["t1"])], g["p1"], tol=0.01)
        segs.append({"name": g["name"], "t0_s": round(g["t0"] - run.mono0, 3),
                     "duration_s": round(g["t1"] - g["t0"], 3), "p1_vs_osc_tx": m})
    return {"phases": phases, "link_changes": links, "ad2_segments": segs}


def phase_line(name: str, ph: dict) -> str:
    c = ph["capture"]
    arp = ", ".join(f"{r['who_has']}@{r['ms']:.0f}ms" for r in c["board_arp_requests"]) or "none"
    m = ph["p5_vs_capture"]
    return (f"{ph['duration_s']:.1f} s: board ARP requests {arp}; board frames by dst {c['board_ipv4_by_dst']}; "
            f"NIC B got {c.get('nic_b_frames_to_nic_b')}; ARP to board "
            f"{[(r['op'], r['from'], r['ms']) for r in c['arp_to_board']][:6]}; P1 {ph['p1']}, P5 {ph['p5']} "
            f"(max {ph['p5_ms']['max'] if ph['p5_ms']['max'] is None else round(ph['p5_ms']['max'], 2)} ms; "
            f"matched {m['matched']} of {m['a']} frames seen in the AD2 time); app {ph['app_rx']}")


GAP_NOMINAL = {"gap 0": 0, "gap a": 0, "gap b": 10, "gap c": 1, "gap d": 1}


def gap_items(res: dict, frames: list[Frame], rec: SegmentRecorder) -> list[Item]:
    """Judges the gap step: per pattern, the dummy broadcasts in the capture against the
    rule applied to the packets actually sent (and the nominal count), and 'p'."""
    board_mac, board_ip, client = res["board_mac"], res["board_ip"], res["client_ip"]
    packets = [(f.t, f.ip_src, f.osc) for f in frames
               if f.dst == board_mac and f.ip_dst == board_ip and f.udp_dport is not None]
    init = res.get("gap_initial") or {}
    expected = np.array(expected_refreshes(packets, init.get("destIp"), init.get("canSendMsgBeforeDestIp", False)))
    dummies = np.array([f.t for f in frames if is_refresh_dummy(f, board_mac)])
    seqs = refresh_sequences(frames, board_mac, board_ip, client)
    _, p5r, p5w = rec.arrays()
    tx = board_send_times(frames, board_mac)
    off = (match_times(tx[covered(tx, rec.coverage)], p5r).get("offset_ms") or 0.0) / 1e3
    for sq in seqs:
        k, w = nearest_pulse(p5r, p5w, sq["t"] + off)
        sq["dummy_p5_ms"] = None if w is None else round(w * 1e3, 3)
        sq["next_p5_ms"] = round(float(p5w[k + 1]) * 1e3, 3) if k is not None and k + 1 < p5w.size else None
    res["refresh_sequences"] = seqs
    items = []
    ev = {e["phase"]: e["t"] for e in res["events"] if e.get("phase")}
    names = [n for n in GAP_NOMINAL if n in ev]
    bounds = [ev[n] for n in names] + [np.inf]
    for i, n in enumerate(names):
        t0, t1 = bounds[i], bounds[i + 1]
        n_cap = int(np.sum((dummies >= t0) & (dummies < t1)))
        n_exp = int(np.sum((expected >= t0) & (expected < t1)))
        n_p = res["perf"].get(n, {}).get("arpRefresh")
        sent = [p for p in packets if t0 <= p[0] < t1]
        gaps = np.diff([p[0] for p in sent]) if len(sent) > 1 else np.zeros(0)
        ok = n_cap == n_exp and n_p == n_cap
        items.append(Item(f"arp {n}", "PASS" if ok else "FAIL", (
            f"dummies in the capture {n_cap}, expected from the packets sent {n_exp} (nominal {GAP_NOMINAL[n]}), "
            f"'p' arpRefresh {n_p}; {len(sent)} packets to the board, intervals "
            f"{'-' if gaps.size == 0 else f'{gaps.min():.3f}-{gaps.max():.3f} s'}"),
            {"capture": n_cap, "expected": n_exp, "nominal": GAP_NOMINAL[n], "p": n_p}))
    bad = [sq for sq in seqs if not sq["order_ok"]]
    items.append(Item("arp gap-order", "PASS" if seqs and not bad else "FAIL", (
        f"{len(seqs)} refreshes, order trigger < dummy < ARP request < ARP reply < reply in "
        f"{len(seqs) - len(bad)}; ms from the trigger: dummy "
        f"{rng([sq['dummy_ms'] for sq in seqs])}, ARP request {rng([sq['arp_request_ms'] for sq in seqs])}, reply "
        f"{rng([sq['reply_ms'] for sq in seqs])}" + (f"; out of order: {bad[:3]}" if bad else ""))))
    items.append(recorded("arp gap-p5", f"P5 of the dummy {rng([sq['dummy_p5_ms'] for sq in seqs])} ms, of the "
                          f"next send {rng([sq['next_p5_ms'] for sq in seqs])} ms"))
    return items


def fix_items(res: dict, frames: list[Frame], names: dict[str, str]) -> list[Item]:
    """Judges the fix step: each swap is repaired by the first packet from the new adapter
    (one refresh, the board's frames go to the new MAC address), none while the heartbeat
    runs on one adapter, no failed send; the destination MAC of 's' / 'T'."""
    board_mac, board_ip, client = res["board_mac"], res["board_ip"], res["client_ip"]
    mac_of = {n: m for m, n in names.items()}
    ev = res["events"]
    # up to the last event: the command that turns the reports off comes after it (and
    # refreshes once more after the 10 s without a packet of F2)
    t_end = ev[-1]["t"] + 0.01
    items = []
    hb = next((e for e in ev if e.get("phase", "") and "heartbeat on A" in e["phase"]), None)
    if hb is not None:
        nxt = next(e["t"] for e in ev if e["t"] > hb["t"] and e.get("phase"))
        n = sum(1 for f in frames if hb["t"] <= f.t < nxt and is_refresh_dummy(f, board_mac))
        p = res["perf"].get(next(k for k in res["perf"] if k.endswith("heartbeat on A")), {}).get("arpRefresh")
        items.append(Item("arp fix-heartbeat", "PASS" if n == 0 and p == 0 else "FAIL",
                          f"heartbeat on NIC A for 30 s: dummies {n}, 'p' arpRefresh {p}"))
    done = [e for e in ev if re.search(r" 10\.0\.0\.\d+ on NIC [AB]$", e["name"])]
    res["swaps"] = []
    for e in done:
        to = e["name"][-1]
        # up to the next swap request
        nxt = next((x["t"] for x in ev if x["t"] > e["t"] and "swap to NIC" in x["name"]), t_end)
        r = swap_recovery(frames, board_mac, board_ip, client, mac_of[f"NIC {to}"], e["t"], nxt)
        tag = e["name"].split(" ")[0]
        perf = res["perf"].get(f"{tag} on {to}", {})
        r.update(swap=f"{tag} to NIC {to}", arpRefresh=perf.get("arpRefresh"), udpSendFailed=perf.get("udpSendFailed"))
        res["swaps"].append(r)
        reply = "/destIp" if tag == "F1" else "/version"
        ok = (r["dummies"] == 1 and r["arpRefresh"] == 1 and r["osc_to_new"].get(reply, 0) > 0
              and r["osc_to_new"].get("/positionList", 0) > 0 and r["dummy_ms"] is not None
              and r["arp_request_ms"] is not None and r["first_to_new_ms"] is not None and not r["udpSendFailed"])
        items.append(Item(f"arp {r['swap']}", "PASS" if ok else "FAIL", (
            f"first packet from NIC {to} ({r['first_rx']}) {r['first_rx_s']} s after it had the address; from it: "
            f"dummy {r['dummy_ms']} ms, ARP request {r['arp_request_ms']} ms, first frame to NIC {to} "
            f"{r['first_to_new_ms']} ms ({r['first_to_new']}); dummies {r['dummies']}, 'p' arpRefresh "
            f"{r['arpRefresh']}, udpSendFailed {r['udpSendFailed']}; board OSC to NIC {to} {r['osc_to_new']}")))
    for d in res.get("dest_macs", []):
        to = d["label"].split(" ")[2]
        old = mac_of["NIC B" if to == "A" else "NIC A"]
        want = old if "before" in d["label"] else mac_of[f"NIC {to}"]
        ok = d["s"] == want and d["T"] == want
        items.append(Item(f"arp {d['label']}", "PASS" if ok else "FAIL",
                          f"s {d['s']}, T {d['T']}, expected {want} ({names.get(want)})"))
    return items


def rng(values) -> str:
    v = [x for x in values if x is not None]
    return "-" if not v else f"{min(v):.2f}-{max(v):.2f}"


def cmd_arp(cfg: Config, args) -> int:
    from . import commands

    a = cfg.section("arp")
    out = commands.new_result_dir(f"arp_{args.topology}_{args.step}" + (f"_{args.label}" if args.label else ""))
    res: dict = {"step": args.step, "topology": args.topology, "board_id": cfg.board_id, "board_ip": cfg.board_ip,
                 "client_ip": a["client_ip"]}
    items: list[Item] = []
    nic = {"A": a["nic_a"], "B": a["nic_b"]}
    info = {k: ifconfig(v) for k, v in nic.items()}
    res["nics"] = {k: {"iface": nic[k], **info[k]} for k in nic}
    names = nic_names(info)
    with ArpBench(cfg, out, args) as b:
        run = ArpRun(b, args)
        caps: list[Tcpdump] = []
        rec = None
        run.mon = LinkMonitor(list(nic.values())).start()
        try:
            text = b.ser.command("s", until=r"Current MAC address : ([0-9A-Fa-f:]{17})")
            board_mac = re.search(r"Current MAC address : ([0-9A-Fa-f:]{17})", text).group(1).lower()
            names[board_mac] = "board"
            res["board_mac"] = board_mac
            res["wall_minus_mono_s"] = wall_minus_mono()
            flt = f"ether host {board_mac} or arp"
            for k, i in capture_nics(args.step, nic).items():
                caps.append(Tcpdump(i, flt, out / f"nic_{k}_{i}.pcap").start())
            rec = run.rec = SegmentRecorder(b, float(a["segment_s"]))  # started by establish()
            try:
                {"check": run.run_check, "e1": run.run_e1, "e2a": run.run_e2a, "e2b": run.run_e2b,
                 "c1": lambda: run.run_fix("c1"), "c2": lambda: run.run_fix("c2"),
                 "n1": lambda: run.run_normal("c1"), "n2": lambda: run.run_normal("c2"),
                 "gap": run.run_gap,
                 "fix": lambda: run.run_fix_swap(args.heartbeat)}[args.step]()
            except Exception as ex:  # keep the logs and summarize what was recorded
                items.append(Item(f"arp-{args.step}-error", "FAIL", f"{type(ex).__name__}: {ex}"))
            t_end = time.monotonic()
            time.sleep(1.5)
        finally:
            if rec is not None:
                rec.stop()
            for c in caps:
                c.stop()
            run.mon.stop()
            b.ser.save(out / "serial_log.txt", b.t0_ns)
            for k, (name, lk) in enumerate(run.links):
                lk.save_log(out / f"osc_log_{k}_{name}.jsonl")
        frames = []
        for c in caps:
            frames += c.frames()
        res["clock_wall_minus_mono_s"] = {"first": run.mon.clock[0][1], "last": run.mon.clock[-1][1],
                                          "samples": len(run.mon.clock)}
        np.save(out / "clock_wall_minus_mono.npy", np.array(run.mon.clock))  # for a new analysis of the pcaps
        to_monotonic(frames, run.mon.clock)
        frames.sort(key=lambda f: f.t)
        res.update(summarize(run, frames, names, res["board_mac"], rec, t_end))
        res["events"] = run.events
        res["perf"] = {k: {kk: v.get(kk) for kk in ("udpSendFailed", "udpSendDropped", "slotLate", "arpRefresh",
                                                    "arpRefreshFailed")}
                       for k, v in run.perf.items()}
        res["serial_fix"] = run.serial_out
        res["gap_initial"] = getattr(run, "initial", None)
        res["dest_macs"] = getattr(run, "dest_macs", [])
        res["ad2_problems"] = rec.problems + ([f"recorder: {rec.error!r}"] if rec.error else [])
        res["frames_captured"] = {c.iface: sum(1 for f in frames if f.iface == c.iface) for c in caps}
    for name, ph in res["phases"].items():
        items.append(recorded(f"arp {name}", phase_line(name, ph)))
    if args.step != "check" and any(n.startswith("3 ") for n in res["phases"]):
        after = merge_summaries([ph["capture"] for n, ph in res["phases"].items()
                                 if n.split(" ")[0] in ("3", "4", "5")])
        v = h1_verdict(after, res["client_ip"])
        res["h1"] = v
        items.append(recorded("arp H1", f"after the swap (3-5): board ARP requests for {res['client_ip']} "
                              f"{v['board_arp_requests_for_client']}, board frames to NIC B {v['frames_to_nic_b']}, "
                              f"to NIC A (seen) {v['frames_to_nic_a_seen']}; H1 {'observed' if v['h1'] else 'not observed'}; "
                              f"udpSendFailed {res['perf']}"))
    if args.step == "gap":
        items += gap_items(res, frames, rec)
    if args.step == "fix":
        items += fix_items(res, frames, names)
    items.append(recorded("arp AD2", "; ".join(
        f"{g['name']} {g['duration_s']:.0f} s: P1 {g['p1_vs_osc_tx']['b']} for {g['p1_vs_osc_tx']['a']} OSC sent, "
        f"matched {g['p1_vs_osc_tx']['matched']}" for g in res["ad2_segments"]) + (
        f"; problems: {res['ad2_problems']}" if res["ad2_problems"] else "")))
    items.append(recorded("arp links", "; ".join(f"{x['t_s']:.1f} s {x['iface']} {x['status']}"
                                                 for x in res["link_changes"])))
    res["items"] = [i.to_dict() for i in items]
    commands.write_json(out / "summary.json", res)
    for it in res["items"]:
        print(f"{it['id']:24s} {it['verdict']:6s} {it['detail']}")
    print(out)
    return 1 if any(it["verdict"] == "FAIL" for it in res["items"]) else 0
