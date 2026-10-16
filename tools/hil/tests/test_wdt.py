from hil import wdt


def test_cases_have_runners():
    assert set(wdt.CASES) == set(wdt.RUNNERS)
    # The default (no --cases) runs only the cases without a human.
    assert [c for c in wdt.CASES if not wdt.CASES[c].human] == ["w1", "w2", "w8", "w9", "w10"]


def test_resolve_without_elf():
    assert wdt.resolve(None, ["0x00002100"]) == {}
    assert wdt.resolve("missing.elf", ["0x00000000"]) == {}
