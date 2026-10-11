"""OSC link to the STEP800: send and receive with python-osc, every message logged with
a time.monotonic_ns() timestamp."""

from __future__ import annotations

import ipaddress
import json
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from pythonosc.osc_message_builder import OscMessageBuilder
from pythonosc.osc_packet import OscPacket


class OscError(Exception):
    pass


@dataclass
class LogEntry:
    t_ns: int
    direction: str  # "tx" or "rx"
    address: str
    args: tuple

    def to_dict(self) -> dict:
        return {"t_ns": self.t_ns, "dir": self.direction, "address": self.address, "args": list(self.args)}


def local_ip_for(remote_ip: str, port: int) -> str:
    """Local address the OS uses to reach remote_ip (no packet is sent)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((remote_ip, port))
        return s.getsockname()[0]


class OscLink:
    def __init__(self, board_ip: str, board_in_port: int, listen_port: int):
        self.board_ip = board_ip
        self.board_in_port = board_in_port
        self.listen_port = listen_port
        self.log: list[LogEntry] = []
        self._log_lock = threading.Lock()
        self._cond = threading.Condition(self._log_lock)
        self._rx_sock: socket.socket | None = None
        self._tx_sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False

    # --- lifecycle ---
    def open(self) -> "OscLink":
        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind(("0.0.0.0", self.listen_port))
        self._rx_sock.settimeout(0.1)
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = True
        self._thread = threading.Thread(target=self._receive, name="osc-rx", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(1.0)
        for s in (self._rx_sock, self._tx_sock):
            if s is not None:
                s.close()
        self._rx_sock = self._tx_sock = None

    def __enter__(self) -> "OscLink":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # --- send / receive ---
    def send(self, address: str, *args) -> LogEntry:
        """Sends one message (one UDP packet). Use int for the firmware's integer and
        boolean arguments: python-osc sends Python bools as T/F."""
        builder = OscMessageBuilder(address=address)
        for a in args:
            if isinstance(a, bool):
                raise TypeError("send booleans as 0 / 1")
            builder.add_arg(a)
        data = builder.build().dgram
        entry = LogEntry(0, "tx", address, tuple(args))
        with self._log_lock:
            entry.t_ns = time.monotonic_ns()
            self._tx_sock.sendto(data, (self.board_ip, self.board_in_port))
            self.log.append(entry)
        return entry

    def _receive(self) -> None:
        while self._running:
            try:
                data, _ = self._rx_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            t = time.monotonic_ns()
            try:
                packet = OscPacket(data)
            except Exception:
                continue
            with self._cond:
                for m in packet.messages:
                    self.log.append(LogEntry(t, "rx", m.message.address, tuple(m.message.params)))
                self._cond.notify_all()

    def wait_for(self, address: str, timeout: float = 1.0, since_ns: int = 0, predicate=None) -> LogEntry | None:
        """First received message with this address after since_ns (and matching the
        predicate on its args), waiting up to timeout seconds."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                for e in self.log:
                    if (e.direction == "rx" and e.t_ns >= since_ns and e.address == address
                            and (predicate is None or predicate(e.args))):
                        return e
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(remaining)

    def request(self, address: str, *args, reply: str, timeout: float = 1.0, predicate=None) -> LogEntry:
        sent = self.send(address, *args)
        e = self.wait_for(reply, timeout, sent.t_ns, predicate)
        if e is None:
            raise OscError(f"no {reply} reply to {address} {list(args)} within {timeout} s")
        return e

    def entries(self, direction: str | None = None, t_from_ns: int = 0, t_to_ns: int | None = None) -> list[LogEntry]:
        with self._log_lock:
            return [e for e in self.log
                    if (direction is None or e.direction == direction)
                    and e.t_ns >= t_from_ns and (t_to_ns is None or e.t_ns <= t_to_ns)]

    def save_log(self, path: Path) -> None:
        with self._log_lock:
            lines = [json.dumps(e.to_dict()) for e in self.log]
        path.write_text("\n".join(lines) + "\n")

    # --- start-up ---
    def set_dest_ip(self, timeout: float = 1.0) -> str:
        """Sends /setDestIp and checks that the reply (/destIp a b c d updated) names the
        local address used to reach the board. Returns that address."""
        local = local_ip_for(self.board_ip, self.board_in_port)
        net = ipaddress.ip_network(f"{self.board_ip}/24", strict=False)
        if ipaddress.ip_address(local) not in net:
            raise OscError(f"this Mac reaches {self.board_ip} from {local}, outside {net}. "
                           "Give the Mac an address in the board's subnet.")
        try:
            reply = self.request("/setDestIp", reply="/destIp", timeout=timeout)
        except OscError:
            raise OscError(f"no /destIp reply from {self.board_ip}:{self.board_in_port} on port "
                           f"{self.listen_port}. Check board.id / ip / ports in bench.toml, the "
                           "network cable, and the macOS firewall (incoming UDP for Python).") from None
        dest = ".".join(str(int(x)) for x in reply.args[:4])
        if dest != local:
            raise OscError(f"board destIp is {dest}, expected {local}")
        return local
