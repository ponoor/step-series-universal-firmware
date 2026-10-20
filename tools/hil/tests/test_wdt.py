from hil import wdt


def test_cases_have_runners():
    assert set(wdt.CASES) == set(wdt.RUNNERS)
    # The default (no --cases) runs only the cases without a human.
    assert [c for c in wdt.CASES if not wdt.CASES[c].human] == ["w1", "w2", "w8", "w9", "w10"]


def test_resolve_without_elf():
    assert wdt.resolve(None, ["0x00002100"]) == {}
    assert wdt.resolve("missing.elf", ["0x00000000"]) == {}


def test_watchdog_early_warning_from_globals(tmp_path):
    from hil import config

    # The checkout's value, rounded as WatchdogTimer::enable() does.
    assert config.watchdog_early_warning_s() == 0.25
    h = tmp_path / "globals.h"
    for ms, s in ((2000, 2.0), (250, 0.25), (300, 0.25), (100, 0.0625), (500, 0.5)):
        h.write_text(f"#define WATCHDOG_TIMEOUT_MS  {ms} // comment\n")
        assert config.watchdog_early_warning_s(h) == s


def test_usb_classifies_cpu_fault():
    from hil import usb

    assert usb.classify({"loop_resumes_ms": 3000.0, "resetCause": 6, "early_warning_ms": 0.1}).startswith("CPU fault")
    assert usb.classify_gap(3.5, True, 6).startswith("CPU fault")
