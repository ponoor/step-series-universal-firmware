"""Judgement helpers of hil.py check (hil/checks.py), with synthetic data."""

import numpy as np
import pytest

from hil import analyze, checks


def test_wrap22():
    assert checks.wrap22(2097151) == 2097151
    assert checks.wrap22(2097152) == -2097152
    assert checks.wrap22(-2097153) == 2097151
    assert checks.wrap22(2096952 + 400) == -2097152 + 200


def test_servo_error_takes_the_short_way():
    # computeServoSpeed(): error wrapped into -2^21 .. 2^21-1
    assert checks.servo_error(-2097100, 2097100) == 104
    assert checks.servo_error(2097100, -2097100) == -104
    assert checks.servo_error(100, 0) == 100
    assert checks.servo_error(0x1FFFFF, 0) == 0x1FFFFF
    assert checks.servo_error(-0x200000, 0) == -0x200000
    assert checks.servo_error(0x200000, 0) == -0x200000  # 2^21 wraps to -2^21


def test_osc_type_tag():
    assert [checks.osc_type_tag(v) for v in (1, 1.0, True, False, "x")] == ["i", "f", "T", "F", "s"]


def test_json_line_and_warnings():
    text = 'noise\r\n{"board":{"myId":3},"motors":[]}\r\nWarning: packed transfer failed\r\n'
    assert checks.json_line(text, '{"board"') == {"board": {"myId": 3}, "motors": []}
    assert checks.json_line("{broken\n", "{") is None
    assert checks.warning_lines(text) == ["Warning: packed transfer failed"]


def _config(value: bool) -> dict:
    return {"alarmAndReport": {k: [value] * 8 for k in checks.REPORT_KEYS}}


def test_report_flags():
    assert checks.report_flags(_config(True), True) == {}
    cfg = _config(True)
    cfg["alarmAndReport"]["reportStall"][2] = False
    del cfg["alarmAndReport"]["reportDir"]
    assert checks.report_flags(cfg, True) == {"reportStall": [3], "reportDir": ["missing"]}


def test_json_diff_ignores_keys():
    a = {"x": {"reportBUSY": [1], "kP": [0.06, 0.06]}, "y": 1}
    b = {"x": {"reportBUSY": [0], "kP": [0.06, 0.07]}, "y": 1}
    assert checks.json_diff(a, b) == ["/x/kP[1]", "/x/reportBUSY[0]"]
    assert checks.json_diff(a, b, ignore=("reportBUSY",)) == ["/x/kP[1]"]


def test_longest_gap_and_long_pulses():
    t = np.array([0.0, 0.001, 0.002, 0.5, 0.501])
    assert checks.longest_gap(t) == pytest.approx((0.498, 0.002))
    assert checks.longest_gap(t, t_to=0.1) == pytest.approx((0.001, 0.0))
    edges = analyze.Edges(np.array([0.1, 0.3, 0.9]), np.array([0.10002, 0.35]), 0, 1, 1.0)
    pulses = checks.long_pulses(edges, 1e-3)
    assert [r for r, _ in pulses] == [0.3, 0.9]
    assert [w for _, w in pulses] == pytest.approx([0.05, 0.1])


def _status_dump(config: int, status: int, shown: dict | None = None, el_pos: int = 0x105) -> str:
    f = {name: checks.field_value(config, mask) for name, mask in checks.L6470_CONFIG_FIELDS.items()}
    exp = checks.expected_status_fields(status)
    exp.update(shown or {})
    lines = ["-------------- DIP Switch --------------", "BIN : 0000 0101", "DEC : 5",
             "-------------- Motor Driver --------------", "Driver ID#1",
             f"\tEL_POS: 0x{el_pos:02X} (Fullstep:{(el_pos >> 7) & 3}, Microstep:{el_pos & 0x7F})",
             "\tALARM_EN: 0xEF (1110 1111)", f"\tCONFIG: 0x{config:02X}"]
    lines += [f"\t  {k}: {v}" for k, v in f.items()]
    lines.append(f"\tSTATUS: 0x{status:02X}")
    for k, v in exp.items():
        if k == "SW_F":
            lines.append(f"\t  SW_F: {v} -HOME sensor input {'closed' if v == '1' else 'open'}.")
        else:
            lines.append(f"\t  {k} : {v}")
    lines.append("-------------- Modes --------------")
    return "\r\n".join(lines) + "\r\n"


def test_status_dump_consistent():
    dump = checks.parse_status_dump(_status_dump(0x2E88, 0x7E03))
    assert dump["dip_dec"] == 5 and set(dump["drivers"]) == {1}
    assert checks.check_status_dump(dump) == []


def test_status_dump_mismatches():
    # A field taken with % instead of & (the phase 1b bug) shows a wrong value.
    text = _status_dump(0x078D, 0x7E30, el_pos=0x1FF).replace("\t  POW_SR: 3", "\t  POW_SR: 1")
    bad = checks.check_status_dump(checks.parse_status_dump(text))
    assert len(bad) == 1 and "POW_SR" in bad[0]
    text = _status_dump(0x2E88, 0x7E03, shown={"BUSY": "Yes"})
    bad = checks.check_status_dump(checks.parse_status_dump(text))
    assert len(bad) == 1 and "BUSY" in bad[0]


def test_expected_status_fields():
    # HiZ, not busy, reverse, stopped, no UVLO / thermal / OCD / stall flags (active low)
    f = checks.expected_status_fields(0x7E03)
    assert f["High impedance state"] == "Yes" and f["BUSY"] == "No" and f["Motor direction"] == "Reverse"
    assert f["UVLO (Undervoltage lock out)"] == "No" and f["Thermal status"] == "Normal"
    assert f["OCD (Overcurrent detection)"] == "No" and f["Stalled"] == "No"
    f = checks.expected_status_fields(0x0070 & ~0x0002)  # busy, forward, constant speed, all flags low
    assert f["BUSY"] == "Yes" and f["Motor direction"] == "Forward" and f["Motor status"] == "Constant speed"
    assert f["Thermal status"] == "Device shutdown" and f["Stalled"] == "Yes" and f["UVLO (Undervoltage lock out)"] == "Yes"


def test_within():
    assert checks.within(100.0, 99, 106) and not checks.within(None, 0, 1) and not checks.within(107, 99, 106)
