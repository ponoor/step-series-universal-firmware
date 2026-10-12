"""Tests of the restart analysis helpers (hil/restart.py)."""

import numpy as np

from hil import restart


def test_parse_boot_drivers():
    text = ("HIL boot driver 1: CONFIG 0x2E88 STATUS 0x7E03 now CONFIG 0x2E8B\r\n"
            "HIL boot driver 2: CONFIG 0x1C8B STATUS 0x7E01 now CONFIG 0x1C8B\r\nHIL boot drivers done\r\n")
    d = restart.parse_boot_drivers(text)
    assert d[1] == {"config": 0x2E88, "status": 0x7E03, "config_now": 0x2E8B}
    assert d[2]["config"] == 0x1C8B and len(d) == 2


def test_mcu_reset_time():
    # a 100 ms blocking wait, then the reset (1.9 s gap)
    t = np.concatenate((np.arange(0, 0.5, 55e-6), np.arange(0.6, 1.0, 55e-6), [1.0 + 1e-4],
                        np.arange(3.0, 3.1, 55e-6)))
    assert restart.mcu_reset_time(t) == 1.0 + 1e-4
    assert restart.mcu_reset_time(np.arange(0, 1.0, 1e-3)) is None
    assert restart.mcu_reset_time(np.array([0.5])) is None


def test_level_segments():
    rate = 1e4
    v = np.concatenate((np.full(1000, 0.0), np.full(500, 1.8), np.full(200, 0.1)))
    segs = restart.level_segments(v, rate, t0=0.1)
    assert [(s["from_ms"], s["to_ms"]) for s in segs] == [(-100.0, 0.0), (0.0, 50.0), (50.0, 70.0)]
    assert [s["median_v"] for s in segs] == [0.0, 1.8, 0.1]


def test_long_pulse():
    from hil import analyze
    e = analyze.Edges(rise=np.array([0.01, 0.02, 0.5]), fall=np.array([0.011, 0.021, 0.8]), initial=0, final=0, duration=1.0)
    assert restart.long_pulse(e, 0.05) == (0.5, 0.8)
    assert restart.long_pulse(e, 0.5) is None
