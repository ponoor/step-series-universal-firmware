"""Bench configuration (bench.toml, falling back to bench.example.toml defaults)."""

from __future__ import annotations

import copy
import ipaddress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tomllib

HIL_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = HIL_DIR / "bench.toml"
EXAMPLE_CONFIG = HIL_DIR / "bench.example.toml"

# Firmware defaults, see src/loadConfig.cpp (loadConfig) and src/utils.cpp (initEthernet).
FW_IP_BASE = "10.0.0.100"
FW_IN_PORT = 50000
FW_OUT_PORT_BASE = 50100


def fw_ip_for_id(board_id: int, base: str = FW_IP_BASE) -> str:
    """The board address with the ID added (src/idOffset.cpp): the last byte is base + id
    if that is 254 or less, otherwise the base address (ID not added; firmware after
    c1787dc, before it the byte wrapped around)."""
    b = ipaddress.IPv4Address(base).packed
    last = b[3] + board_id
    return str(ipaddress.IPv4Address(bytes(b[:3]) + bytes([last if last <= 254 else b[3]])))


def fw_out_port_for_id(board_id: int, base: int = FW_OUT_PORT_BASE) -> int:
    return base + board_id if base + board_id <= 0xFFFF else base

NUM_MOTORS = 8
NUM_PROBES = 7
HOME_SW_DIO = 15


class ConfigError(Exception):
    pass


def _merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


@dataclass
class Config:
    raw: dict[str, Any]
    path: Path | None

    # --- board / network ---
    @property
    def board_id(self) -> int:
        return int(self.raw["board"]["id"])

    @property
    def board_ip(self) -> str:
        ip = self.raw["board"].get("ip")
        if ip:
            return str(ip)
        return fw_ip_for_id(self.board_id)

    @property
    def board_in_port(self) -> int:
        return int(self.raw["board"].get("in_port", FW_IN_PORT))

    @property
    def board_out_port(self) -> int:
        port = self.raw["board"].get("out_port")
        return int(port) if port else fw_out_port_for_id(self.board_id)

    # --- serial / motor ---
    @property
    def serial_port(self) -> str:
        return str(self.raw["serial"].get("port", ""))

    @property
    def test_axis(self) -> int:
        return int(self.raw["motor"]["test_axis"])

    # --- AD2 ---
    @property
    def ad2(self) -> dict:
        return self.raw["ad2"]

    @property
    def brake_dio(self) -> int:
        """DIO of the brake output of the test axis."""
        return int(self.ad2["dio"]["brake"][self.test_axis - 1])

    @property
    def probe_dio(self) -> list[int]:
        return [int(d) for d in self.ad2["dio"]["probes"]]

    def override_board_id(self, board_id: int) -> None:
        """--id: the DIP switch ID instead of board.id. The firmware defaults derive the IP
        address and the send port from it, so explicit board.ip / out_port are an error."""
        if not 0 <= board_id <= 255:
            raise ConfigError("--id must be 0-255")
        if self.raw["board"].get("ip") or self.raw["board"].get("out_port"):
            raise ConfigError("--id cannot be used with board.ip or board.out_port in bench.toml")
        self.raw["board"]["id"] = board_id

    def section(self, name: str) -> dict:
        return self.raw[name]

    def validate(self) -> None:
        if not 1 <= self.test_axis <= NUM_MOTORS:
            raise ConfigError(f"motor.test_axis must be 1-{NUM_MOTORS}")
        dio = self.ad2["dio"]
        if len(dio["brake"]) != NUM_MOTORS:
            raise ConfigError(f"ad2.dio.brake must have {NUM_MOTORS} entries")
        if len(dio["probes"]) != NUM_PROBES:
            raise ConfigError(f"ad2.dio.probes must have {NUM_PROBES} entries")
        if int(dio["home_sw"]) != HOME_SW_DIO:
            raise ConfigError("ad2.dio.home_sw must be 15 (the only pin the bench may drive)")
        inputs = [int(d) for d in dio["brake"]] + [int(d) for d in dio["probes"]]
        if len(set(inputs)) != len(inputs) or any(not 0 <= d <= 14 for d in inputs):
            raise ConfigError("ad2.dio.brake and ad2.dio.probes must be distinct DIO 0-14")
        for key in ("out1a", "v33"):
            if int(self.ad2["scope"][key]) not in (1, 2):
                raise ConfigError(f"ad2.scope.{key} must be 1 or 2")


def load(path: str | Path | None = None) -> Config:
    """Loads the example config as defaults, then bench.toml (or the given file) over it."""
    with open(EXAMPLE_CONFIG, "rb") as f:
        raw = tomllib.load(f)
    used: Path | None = None
    candidate = Path(path) if path else DEFAULT_CONFIG
    if candidate.exists():
        with open(candidate, "rb") as f:
            raw = _merge(raw, tomllib.load(f))
        used = candidate
    elif path:
        raise ConfigError(f"config file not found: {candidate}")
    cfg = Config(raw, used)
    cfg.validate()
    return cfg
