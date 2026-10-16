"""Bench configuration (bench.toml, falling back to bench.example.toml defaults).

A file with `[board] model = "step400"` takes the defaults of bench.step400.example.toml
over bench.example.toml (the STEP800 defaults), then its own values."""

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

NUM_MOTORS = 8  # STEP800; use Config.num_motors
NUM_PROBES = 7  # probe channels P0-P6 in the firmware (src/hilProbe.h); a board may wire fewer
HOME_SW_DIO = 15
LIMIT_SW_DIO = 14
# The only AD2 pins the bench may drive (open-drain, low or high impedance, never high):
# the switch inputs of the test axis. A board model allows a subset (BoardModel.switches).
SWITCH_DIO = {"home": HOME_SW_DIO, "limit": LIMIT_SW_DIO}


@dataclass(frozen=True)
class BoardModel:
    name: str
    firmware_name: str  # FIRMWARE_NAME of src/boardsDef.h ('s': "Firmware name : ...")
    num_motors: int
    probe_map: dict  # HIL probe channel -> firmware pin map entry ('P', src/hilProbe.cpp)
    switches: tuple  # switch inputs of the test axis the bench may drive (SWITCH_DIO keys)
    config_default: int  # driver CONFIG after power-on / standby
    example: str  # defaults over bench.example.toml ("" for none)

    @property
    def probe_channels(self) -> int:
        return len(self.probe_map)


BOARDS = {
    "step800": BoardModel("step800", "STEP800_R1_UNIVERSAL", 8,
                          {0: "D1(PA10)", 1: "A1(PB08)", 2: "D8(PA06)", 3: "D9(PA07)",
                           4: "D38(PA13)", 5: "SCL(PA23)", 6: "SDA(PA22)"},
                          ("home",), 0x2E88, ""),  # L6470
    "step400": BoardModel("step400", "STEP400_R1_UNIVERSAL", 4,
                          {0: "D38(PA13)", 1: "SCL(PA23)", 2: "SDA(PA22)"},
                          ("home", "limit"), 0x2C88, "bench.step400.example.toml"),  # PowerSTEP01
}


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
    def local_ip(self) -> str:
        """[network] local_ip: this Mac's address to the board (e.g. the wired adapter when
        Wi-Fi is on the same subnet); default: any (the OS route)."""
        return str(self.raw.get("network", {}).get("local_ip", "0.0.0.0"))

    @property
    def bound_if(self) -> str:
        """[network] bound_if: the interface of local_ip (e.g. "en9"); default: none."""
        return str(self.raw.get("network", {}).get("bound_if", ""))

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

    @property
    def model(self) -> str:
        return str(self.raw["board"].get("model", "step800"))

    @property
    def board(self) -> BoardModel:
        return BOARDS[self.model]

    @property
    def num_motors(self) -> int:
        return self.board.num_motors

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
        """DIO of each probe channel P0-P6; -1 for a channel without a pin on this board."""
        return [int(d) for d in self.ad2["dio"]["probes"]]

    def probe(self, ch: int) -> int | None:
        """DIO of probe channel ch, None if the board has no pin for it."""
        d = self.probe_dio[ch]
        return d if d >= 0 else None

    @property
    def probe_dios(self) -> list[int]:
        """DIO of the probe channels that have a pin."""
        return [d for d in self.probe_dio if d >= 0]

    @property
    def v33_dio(self) -> int | None:
        """DIO on the board's 3.3 V (checked high before a recording), None if not wired."""
        d = int(self.ad2["dio"].get("v33", -1))
        return d if d >= 0 else None

    @property
    def switch_dios(self) -> dict[str, int]:
        """Switch inputs of the test axis the bench may drive: {"home": 15, "limit": 14}."""
        dio = self.ad2["dio"]
        out = {"home": int(dio["home_sw"])}
        if int(dio.get("limit_sw", -1)) >= 0:
            out["limit"] = int(dio["limit_sw"])
        return out

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
        if self.model not in BOARDS:
            raise ConfigError(f"board.model must be one of {', '.join(BOARDS)}")
        n = self.num_motors
        if not 1 <= self.test_axis <= n:
            raise ConfigError(f"motor.test_axis must be 1-{n}")
        dio = self.ad2["dio"]
        if len(dio["brake"]) != n:
            raise ConfigError(f"ad2.dio.brake must have {n} entries")
        if len(dio["probes"]) != NUM_PROBES:
            raise ConfigError(f"ad2.dio.probes must have {NUM_PROBES} entries (-1: no pin)")
        wired = self.board.probe_channels
        if any(int(d) < 0 for d in dio["probes"][:wired]) or any(int(d) >= 0 for d in dio["probes"][wired:]):
            raise ConfigError(f"ad2.dio.probes: P0-P{wired - 1} need a DIO on {self.model}, "
                              f"P{wired}-P{NUM_PROBES - 1} must be -1")
        if int(dio["home_sw"]) != HOME_SW_DIO:
            raise ConfigError("ad2.dio.home_sw must be 15 (the bench drives only DIO 14 and 15)")
        limit = int(dio.get("limit_sw", -1))
        if "limit" in self.board.switches:
            if limit not in (-1, LIMIT_SW_DIO):
                raise ConfigError("ad2.dio.limit_sw must be 14 or -1 (the bench drives only DIO 14 and 15)")
        elif limit != -1:
            raise ConfigError(f"ad2.dio.limit_sw must be -1 on {self.model} (no limit switch input)")
        driven = set(self.switch_dios.values())
        inputs = [int(d) for d in dio["brake"]] + self.probe_dios
        if self.v33_dio is not None:
            inputs.append(self.v33_dio)
        if len(set(inputs)) != len(inputs) or any(not 0 <= d <= 15 or d in driven for d in inputs):
            raise ConfigError("ad2.dio.brake, ad2.dio.probes and ad2.dio.v33 must be distinct DIO 0-15, "
                              "other than the switch outputs")
        for key in ("out1a", "v33"):
            ch = int(self.ad2["scope"][key])
            if ch not in (1, 2) and not (key == "v33" and ch == 0 and self.v33_dio is not None):
                raise ConfigError(f"ad2.scope.{key} must be 1 or 2" + (" (or 0 with ad2.dio.v33)" if key == "v33" else ""))
        if int(self.ad2["scope"].get("p0_ref", 0)) == int(self.ad2["scope"]["out1a"]):
            raise ConfigError("ad2.scope.p0_ref must not be the OUT1A channel")


def load(path: str | Path | None = None) -> Config:
    """Loads the example config as defaults, then the model's example (board.model of the
    file), then bench.toml (or the given file) over it."""
    with open(EXAMPLE_CONFIG, "rb") as f:
        raw = tomllib.load(f)
    used: Path | None = None
    candidate = Path(path) if path else DEFAULT_CONFIG
    if path and not candidate.exists() and not candidate.is_absolute() and (HIL_DIR / candidate).exists():
        candidate = HIL_DIR / candidate
    if candidate.exists():
        with open(candidate, "rb") as f:
            own = tomllib.load(f)
        model = BOARDS.get(str(own.get("board", {}).get("model", "step800")))
        if model is not None and model.example and candidate.resolve() != (HIL_DIR / model.example).resolve():
            with open(HIL_DIR / model.example, "rb") as f:
                raw = _merge(raw, tomllib.load(f))
        raw = _merge(raw, own)
        used = candidate
    elif path:
        raise ConfigError(f"config file not found: {candidate}")
    cfg = Config(raw, used)
    cfg.validate()
    return cfg
