"""Analog Discovery 2: open/close, synchronized Digital In + Scope recording, and the
open-drain home switch output on DIO15.

The device is opened with dwfpy. Recording uses the low-level WaveForms API through
dwfpy.bindings, because dwfpy has no wrapper for the compressed record
(FDwfDigitalInStatusCompress / FDwfDigitalInStatusCompressed) or for a Scope and
Digital In recording started together by FDwfDeviceTriggerPC.

Output safety: the only pin this module drives is DIO15, and only low (open-drain).
The waveform generators (W1/W2), the power supplies (V+/V-) and the pattern generator
are never enabled.
"""

from __future__ import annotations

import ctypes
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import analyze
from .config import HOME_SW_DIO

DWFPY_LIBRARY = "/Library/Frameworks/dwf.framework/dwf"
APP_BUNDLE_LIBRARY = "/Applications/WaveForms.app/Contents/Frameworks/dwf.framework/dwf"
HOME_SW_MASK = 1 << HOME_SW_DIO

_dwfpy = None


def load_dwfpy(library: str = ""):
    """Imports dwfpy. dwfpy loads the runtime from /Library/Frameworks only; recent
    WaveForms installs keep it inside the app bundle, so the load is redirected."""
    global _dwfpy
    if _dwfpy is not None:
        return _dwfpy
    path = os.environ.get("DWF_LIBRARY") or library
    if not path:
        path = DWFPY_LIBRARY if os.path.exists(DWFPY_LIBRARY) else APP_BUNDLE_LIBRARY
    if not os.path.exists(path):
        raise RuntimeError(f"WaveForms runtime not found: {path}. Install WaveForms or set DWF_LIBRARY.")
    original = ctypes.cdll.LoadLibrary

    def load(name, *args, **kwargs):
        return original(path if name == DWFPY_LIBRARY else name, *args, **kwargs)

    ctypes.cdll.LoadLibrary = load
    try:
        import dwfpy
    finally:
        ctypes.cdll.LoadLibrary = original
    _dwfpy = dwfpy
    return dwfpy


def _api():
    return load_dwfpy().bindings


@dataclass
class Capture:
    """Result of one recording. Times are seconds from the trigger."""

    digital: analyze.Segments | None = None
    digital_lost: int = 0
    digital_corrupted: int = 0
    analog_rate: float | None = None
    analog: dict[int, np.ndarray] = field(default_factory=dict)  # scope channel (1, 2) -> volts
    analog_lost: int = 0
    analog_corrupted: int = 0
    host_trigger_ns: int = 0  # time.monotonic_ns() at the trigger
    host_stop_ns: int = 0

    @property
    def ok(self) -> bool:
        return (self.digital_lost == 0 and self.digital_corrupted == 0
                and self.analog_lost == 0 and self.analog_corrupted == 0)

    def problems(self) -> list[str]:
        out = []
        if self.digital_lost or self.digital_corrupted:
            out.append(f"Digital In lost {self.digital_lost} / corrupted {self.digital_corrupted} samples")
        if self.analog_lost or self.analog_corrupted:
            out.append(f"Scope lost {self.analog_lost} / corrupted {self.analog_corrupted} samples")
        return out

    def save(self, path: Path) -> None:
        arrays: dict[str, np.ndarray] = {
            "host_trigger_ns": np.array(self.host_trigger_ns, dtype=np.int64),
            "host_stop_ns": np.array(self.host_stop_ns, dtype=np.int64),
            "lost": np.array([self.digital_lost, self.digital_corrupted, self.analog_lost, self.analog_corrupted]),
        }
        if self.digital is not None:
            arrays.update(dig_start=self.digital.start, dig_value=self.digital.value,
                          dig_n=np.array(self.digital.n_samples), dig_rate=np.array(self.digital.rate))
        if self.analog_rate:
            arrays["analog_rate"] = np.array(self.analog_rate)
            for ch, v in self.analog.items():
                arrays[f"scope{ch}"] = v.astype(np.float32)
        np.savez_compressed(path, **arrays)

    @classmethod
    def load(cls, path: Path) -> "Capture":
        z = np.load(path)
        cap = cls(host_trigger_ns=int(z["host_trigger_ns"]), host_stop_ns=int(z["host_stop_ns"]))
        cap.digital_lost, cap.digital_corrupted, cap.analog_lost, cap.analog_corrupted = (int(x) for x in z["lost"])
        if "dig_start" in z:
            cap.digital = analyze.Segments(z["dig_start"], z["dig_value"], int(z["dig_n"]), float(z["dig_rate"]))
        if "analog_rate" in z:
            cap.analog_rate = float(z["analog_rate"])
            for ch in (1, 2):
                if f"scope{ch}" in z:
                    cap.analog[ch] = z[f"scope{ch}"].astype(float)
        return cap


class Recording:
    """A recording running in a background thread. Use AD2.start_recording()."""

    def __init__(self, ad2: "AD2", duration: float | None):
        self._ad2 = ad2
        self._duration = duration
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: BaseException | None = None
        self.capture = Capture()
        self._words: list[np.ndarray] = []
        self._carry = np.zeros(0, np.uint32)
        self._dig_samples = 0
        self._dig_raw: list[np.ndarray] = []
        self._analog_parts: dict[int, list[np.ndarray]] = {}
        self._analog_samples = 0

    def stop(self) -> Capture:
        """Stops the recording (if not finished yet) and returns the capture."""
        self._stop.set()
        return self.wait()

    def wait(self, timeout: float | None = None) -> Capture:
        if self._thread is not None:
            self._thread.join(timeout)
        if self._error is not None:
            raise self._error
        return self.capture

    @property
    def elapsed(self) -> float:
        return (time.monotonic_ns() - self.capture.host_trigger_ns) / 1e9 if self.capture.host_trigger_ns else 0.0

    # The loop below runs in the recording thread.
    def _run(self) -> None:
        try:
            self._loop()
        except BaseException as ex:  # reported by wait()
            self._error = ex
        finally:
            self._ad2._stop_instruments()
            self.capture.host_stop_ns = time.monotonic_ns()
            self._finish()

    def _loop(self) -> None:
        a = self._ad2
        api = _api()
        h = a.handle
        while True:
            if a._dig is not None:
                self._read_digital(api, h)
            if a._ana is not None:
                self._read_analog(api, h)
            done = self._stop.is_set()
            if self._duration is not None:
                dig_done = a._dig is None or self._dig_samples >= self._duration * a._dig["rate"]
                ana_done = a._ana is None or self._analog_samples >= self._duration * a._ana["rate"]
                done = done or (dig_done and ana_done)
            if done:
                return
            time.sleep(0.0005)

    def _read_digital(self, api, h) -> None:
        cfg = self._ad2._dig
        api.dwf_digital_in_status(h, 1)
        if cfg["compress"]:
            avail, lost, corrupt = api.dwf_digital_in_status_compress(h)
            if avail > 0:
                buf = np.empty(avail, dtype=np.uint32)
                api.dwf_digital_in_status_compressed(h, buf.ctypes.data_as(ctypes.c_void_p), 4 * avail)
                words = np.concatenate((self._carry, buf))
                even = words.size - (words.size % 2)
                self._carry = words[even:]
                words = words[:even]
                self._words.append(words)
                self._dig_samples += int(words[1::2].astype(np.int64).sum()) + words.size // 2
        else:
            avail, lost, corrupt = api.dwf_digital_in_status_record(h)
            if avail > 0:
                buf = np.empty(avail, dtype=np.uint32)
                api.dwf_digital_in_status_data(h, buf.ctypes.data_as(ctypes.c_void_p), 4 * avail)
                self._dig_raw.append(buf)
                self._dig_samples += avail
        self.capture.digital_lost += lost
        self.capture.digital_corrupted += corrupt

    def _read_analog(self, api, h) -> None:
        cfg = self._ad2._ana
        api.dwf_analog_in_status(h, 1)
        avail, lost, corrupt = api.dwf_analog_in_status_record(h)
        self.capture.analog_lost += lost
        self.capture.analog_corrupted += corrupt
        if avail > 0:
            for ch in cfg["channels"]:
                buf = np.empty(avail, dtype=np.float64)
                api.dwf_analog_in_status_data(h, ch - 1, buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), avail)
                self._analog_parts.setdefault(ch, []).append(buf)
            self._analog_samples += avail

    def _finish(self) -> None:
        a = self._ad2
        if a._dig is not None:
            rate = a._dig["rate"]
            if a._dig["compress"]:
                words = np.concatenate(self._words) if self._words else np.zeros(0, np.uint32)
                self.capture.digital = analyze.segments_from_pairs(words, rate)
            else:
                raw = np.concatenate(self._dig_raw) if self._dig_raw else np.zeros(0, np.uint32)
                self.capture.digital = analyze.segments_from_samples(raw, rate)
        if a._ana is not None:
            self.capture.analog_rate = a._ana["rate"]
            for ch in a._ana["channels"]:
                parts = self._analog_parts.get(ch, [])
                self.capture.analog[ch] = np.concatenate(parts) if parts else np.zeros(0)


class AD2:
    """Analog Discovery 2. Use as a context manager: the home switch output is always
    released before the device is closed, also on exceptions."""

    def __init__(self, library: str = "", serial_number: str = "", range_vpp: float = 50.0,
                 configuration: int | None = None):
        self._library = library
        self._serial = serial_number or None
        self._configuration = configuration
        self._range = range_vpp
        self._device = None
        self._dig: dict | None = None
        self._ana: dict | None = None
        self._lock = threading.Lock()

    # --- open / close ---
    def open(self) -> "AD2":
        dwfpy = load_dwfpy(self._library)
        if dwfpy.bindings.dwf_enum(dwfpy.bindings.ENUMFILTER_ALL) == 0 and not os.path.exists(DWFPY_LIBRARY):
            # The runtime finds the FTDI library and its data directory relative to the
            # executable or under /Library/Frameworks/dwf.framework, so it sees no device
            # when loaded from the app bundle by another program.
            raise RuntimeError(
                "no WaveForms device found. The runtime needs /Library/Frameworks/dwf.framework; "
                "link it with: sudo ln -s /Applications/WaveForms.app/Contents/Frameworks/dwf.framework "
                "/Library/Frameworks/dwf.framework")
        self._device = dwfpy.AnalogDiscovery2(configuration=self._configuration, serial_number=self._serial)
        self._device.open()
        api = _api()
        h = self.handle
        api.dwf_device_auto_configure_set(h, 0)
        # Pattern generator off, home switch released (high impedance).
        api.dwf_digital_out_reset(h)
        api.dwf_digital_out_configure(h, 0)
        self.release()
        return self

    def close(self) -> None:
        if self._device is None:
            return
        try:
            self.release()
        finally:
            self._device.close()
            self._device = None

    def __enter__(self) -> "AD2":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def handle(self):
        if self._device is None:
            raise RuntimeError("AD2 is not open")
        return self._device.handle

    def version(self) -> str:
        return load_dwfpy(self._library).Application().get_version()

    def supplies_enabled(self) -> bool:
        """True if the power supplies (V+/V-) are on. The bench never turns them on."""
        return bool(_api().dwf_analog_io_enable_get(self.handle))

    # --- home switch (DIO15, open-drain) ---
    def press(self) -> None:
        """Pulls the home switch input low: output value 0 first, then output enable."""
        api = _api()
        with self._lock:
            api.dwf_digital_io_output_set(self.handle, 0)
            api.dwf_digital_io_output_enable_set(self.handle, HOME_SW_MASK)
            api.dwf_digital_io_configure(self.handle)

    def release(self) -> None:
        """Output disabled (high impedance); the L6470 pulls the input up."""
        api = _api()
        with self._lock:
            api.dwf_digital_io_output_enable_set(self.handle, 0)
            api.dwf_digital_io_output_set(self.handle, 0)
            api.dwf_digital_io_configure(self.handle)

    def read_dio(self) -> int:
        api = _api()
        with self._lock:
            api.dwf_digital_io_status(self.handle)
            return int(api.dwf_digital_io_input_status(self.handle))

    # --- recording ---
    def start_recording(self, duration: float | None, digital_rate: float | None = None,
                        sensible_mask: int = 0xFFFF, compress: bool = True,
                        analog_channels: tuple[int, ...] = (), analog_rate: float | None = None,
                        analog_trigger: tuple[int, float] | None = None) -> Recording:
        """Starts Digital In and/or Scope in record mode and returns the running recording.

        Trigger: by default both instruments wait for the PC trigger and are started
        together with FDwfDeviceTriggerPC. With analog_trigger = (channel, level), the
        Scope triggers on a rising edge of that channel and Digital In is triggered by
        the Scope trigger. duration None records until Recording.stop()."""
        api = _api()
        h = self.handle
        self._dig = self._ana = None
        if digital_rate:
            hz = api.dwf_digital_in_internal_clock_info(h)
            div = max(1, int(round(hz / digital_rate)))
            api.dwf_digital_in_reset(h)
            api.dwf_digital_in_acquisition_mode_set(h, api.ACQMODE_RECORD)
            api.dwf_digital_in_divider_set(h, div)
            api.dwf_digital_in_sample_format_set(h, 32)
            api.dwf_digital_in_sample_sensible_set(h, sensible_mask if compress else 0)
            api.dwf_digital_in_trigger_source_set(
                h, api.TRIGSRC_ANALOG_IN if analog_trigger else api.TRIGSRC_PC)
            api.dwf_digital_in_trigger_position_set(h, 0)  # record until stopped
            api.dwf_digital_in_buffer_size_set(h, api.dwf_digital_in_buffer_size_info(h))
            api.dwf_digital_in_configure(h, 1, 0)
            self._dig = {"rate": hz / div, "compress": compress}
        if analog_channels:
            api.dwf_analog_in_reset(h)
            for idx in range(2):
                enabled = (idx + 1) in analog_channels
                api.dwf_analog_in_channel_enable_set(h, idx, int(enabled))
                if enabled:
                    api.dwf_analog_in_channel_range_set(h, idx, self._range)
                    api.dwf_analog_in_channel_offset_set(h, idx, 0.0)
            api.dwf_analog_in_acquisition_mode_set(h, api.ACQMODE_RECORD)
            api.dwf_analog_in_frequency_set(h, analog_rate)
            api.dwf_analog_in_record_length_set(h, 0.0)  # record until stopped
            api.dwf_analog_in_buffer_size_set(h, api.dwf_analog_in_buffer_size_info(h)[1])
            if analog_trigger:
                ch, level = analog_trigger
                api.dwf_analog_in_trigger_source_set(h, api.TRIGSRC_DETECTOR_ANALOG_IN)
                api.dwf_analog_in_trigger_type_set(h, api.TRIGTYPE_EDGE)
                api.dwf_analog_in_trigger_channel_set(h, ch - 1)
                api.dwf_analog_in_trigger_level_set(h, level)
                api.dwf_analog_in_trigger_condition_set(h, api.TRIGGER_SLOPE_RISE)
            else:
                api.dwf_analog_in_trigger_source_set(h, api.TRIGSRC_PC)
            api.dwf_analog_in_configure(h, 1, 0)
            self._ana = {"rate": api.dwf_analog_in_frequency_get(h), "channels": tuple(analog_channels)}
        if self._dig is None and self._ana is None:
            raise ValueError("nothing to record")

        time.sleep(0.1)  # let the Scope offsets and relays settle
        # Arm: Digital In first, so it is ready when the Scope (trigger source) starts.
        if self._dig is not None:
            api.dwf_digital_in_configure(h, 0, 1)
        if self._ana is not None:
            api.dwf_analog_in_configure(h, 0, 1)
        self._wait_armed(api, h)

        rec = Recording(self, duration)
        if analog_trigger is None:
            t0 = time.monotonic_ns()
            api.dwf_device_trigger_pc(h)
            t1 = time.monotonic_ns()
            rec.capture.host_trigger_ns = (t0 + t1) // 2
        else:
            rec.capture.host_trigger_ns = time.monotonic_ns()  # armed; the real trigger is later
        rec._thread = threading.Thread(target=rec._run, name="ad2-record", daemon=True)
        rec._thread.start()
        return rec

    def record(self, duration: float, **kwargs) -> Capture:
        return self.start_recording(duration, **kwargs).wait()

    def _wait_armed(self, api, h, timeout: float = 2.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            dig_ok = self._dig is None or api.dwf_digital_in_status(h, 0) == api.STATE_ARMED
            ana_ok = self._ana is None or api.dwf_analog_in_status(h, 0) == api.STATE_ARMED
            if dig_ok and ana_ok:
                return
            time.sleep(0.005)
        raise RuntimeError("AD2 instruments did not arm")

    def _stop_instruments(self) -> None:
        api = _api()
        h = self.handle
        if self._dig is not None:
            api.dwf_digital_in_configure(h, 0, 0)
        if self._ana is not None:
            api.dwf_analog_in_configure(h, 0, 0)
