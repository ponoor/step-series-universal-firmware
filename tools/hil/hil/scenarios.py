"""Brake scenarios as data (TASK Task 5).

A scenario is a list of steps (OSC messages, waits, home switch operations) run while
the AD2 records, plus the checks to apply and the expected outcome:

- "PASS":   every check passes.
- "FAIL":   reproduces a known defect; the checks in expected_failures fail and the
            others pass.
- "RECORD": characterization only; the checks are reported, no expectation.

Placeholders in OSC arguments are resolved when the scenario runs:
AXIS (test axis), KVAL, MAX_SPEED, TRANSITION_MS (from bench.toml [brake]).
Times in checks are AD2 times of labelled steps; delays are added to them.
"""

from __future__ import annotations

from dataclasses import dataclass

AXIS = "{axis}"
KVAL = "{kval}"
MAX_SPEED = "{max_speed}"
TRANSITION_MS = "{transition_ms}"
ALL = 255

# Microsteps of a short move: 100 full steps at the default 1/128 step mode.
MOVE = 12800

# Delay symbols for checks, resolved from bench.toml [brake]:
#   "latency":    brake output delay allowance (latency_allowance_ms)
#   "transition": transition_ms + latency_allowance_ms
LATENCY = "latency"
TRANSITION = "transition"


@dataclass(frozen=True)
class Step:
    kind: str  # "osc", "wait", "press", "release"
    address: str = ""
    args: tuple = ()
    seconds: float = 0.0
    label: str = ""


def osc(address: str, *args, label: str = "") -> Step:
    return Step("osc", address, tuple(args), label=label)


def wait(seconds: float) -> Step:
    return Step("wait", seconds=seconds)


def press(label: str = "") -> Step:
    return Step("press", label=label)


def release(label: str = "") -> Step:
    return Step("release", label=label)


@dataclass(frozen=True)
class Check:
    """Signal of the test axis keeps a value over a window of the scenario.
    signal: "brake" (True = released) or "excited". to_label None = end of capture."""

    id: str
    signal: str
    value: bool
    from_label: str
    to_label: str | None = None
    delay: str | float = 0.0  # added to the from time


@dataclass(frozen=True)
class Exempt:
    """An invariant is not checked from one step to another (plus the latency allowance)."""

    invariant: str
    from_label: str
    to_label: str | None = None


@dataclass(frozen=True)
class Scenario:
    id: str
    title: str
    expect: str  # "PASS" / "FAIL" / "RECORD"
    steps: tuple[Step, ...]
    expected_failures: frozenset[str] = frozenset()
    invariants: tuple[str, ...] = ("INV-1", "INV-2", "INV-3")
    checks: tuple[Check, ...] = ()
    exempt: tuple[Exempt, ...] = ()
    extra_setup: tuple[Step, ...] = ()
    poll_position: bool = True
    notes: str = ""

    def labels(self) -> list[str]:
        return [s.label for s in self.steps if s.label]


# Common setup (Task 5-1), sent after a reboot and before the recording starts.
# After a reboot the brake status is BRAKE_ENGAGED and the motor is HiZ; /activate 255 0
# keeps it so (it does nothing while engaged).
COMMON_SETUP: tuple[Step, ...] = (
    osc("/enableElectromagnetBrake", ALL, 1),
    osc("/setBrakeTransitionDuration", ALL, TRANSITION_MS),
    osc("/setKval", AXIS, KVAL, KVAL, KVAL, KVAL),
    osc("/setMaxSpeed", AXIS, MAX_SPEED),
    osc("/activate", ALL, 0),
    wait(0.5),
)

_ACTIVATE = (osc("/activate", AXIS, 1, label="activate"), wait(0.5))

SCENARIOS: tuple[Scenario, ...] = (
    Scenario(
        "B1", "Normal sequence: activate, move, softHiZ, activate, free, activate 0", "PASS",
        steps=(
            osc("/activate", AXIS, 1, label="activate1"), wait(0.5),
            osc("/move", AXIS, MOVE, label="move"), wait(2.0),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(0.5),
            osc("/activate", AXIS, 1, label="activate2"), wait(0.5),
            osc("/free", AXIS, label="free"), wait(0.5),
            osc("/activate", AXIS, 0, label="activate0"), wait(0.5),
        ),
        # /free releases the brake and makes the motor HiZ on purpose.
        exempt=(Exempt("INV-1", "free", "activate0"),),
        checks=(
            Check("ENGAGED-AFTER-SOFTHIZ", "brake", False, "softHiZ", "activate2", LATENCY),
            Check("RELEASED-AFTER-ACTIVATE2", "brake", True, "activate2", "free", TRANSITION),
            Check("ENGAGED-AFTER-ACTIVATE0", "brake", False, "activate0", None, LATENCY),
        ),
        notes="Also records the command -> brake output delay and the engaged + excited overlaps.",
    ),
    Scenario(
        "B2", "Servo mode enabled while the brake is engaged (STATUS 5.1)", "FAIL",
        steps=(
            osc("/enableServoMode", AXIS, 1, label="servo"), wait(0.3),
            osc("/setTargetPosition", AXIS, MOVE, label="target"), wait(1.0),
            osc("/enableServoMode", AXIS, 0, label="servoOff"), wait(0.3),
        ),
        expected_failures=frozenset({"INV-2", "INV-3"}),
    ),
    Scenario(
        "B3", "softHiZ while running, then /free during the deceleration (STATUS 5.2)", "FAIL",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(0.05),
            osc("/free", AXIS, label="free"), wait(1.0),
        ),
        exempt=(Exempt("INV-1", "free"),),
        checks=(Check("RELEASED-AFTER-FREE", "brake", True, "free", None, LATENCY),),
        expected_failures=frozenset({"RELEASED-AFTER-FREE"}),
        notes="Known defect: bBrakeDecWaiting stays set, so the brake engages when the motor stops after /free.",
    ),
    Scenario(
        "B4", "softHiZ while running, then a motion command before the stop (STATUS 5.3)", "FAIL",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(0.05),
            # /run is accepted by the L6470 while decelerating (/move is not: BUSY).
            osc("/run", AXIS, MAX_SPEED, label="run2"), wait(1.0),
            osc("/softStop", AXIS, label="softStop"), wait(1.0),
        ),
        checks=(Check("RELEASED-AFTER-RUN2", "brake", True, "run2", None, LATENCY),),
        expected_failures=frozenset({"RELEASED-AFTER-RUN2"}),
        notes="Known defect: bBrakeDecWaiting stays set after the new motion command, so the brake "
              "engages when the motor stops after /softStop. A /move sent while decelerating is "
              "rejected by the L6470 (CommandIgnored), so /run is used.",
    ),
    Scenario(
        "B5", "/activate 1 within 10 ms after /hardHiZ (STATUS 5.3)", "RECORD",
        steps=_ACTIVATE + (
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.005),
            osc("/activate", AXIS, 1, label="activate2"), wait(1.0),
        ),
    ),
    Scenario(
        "B6", "Home switch pressed with prohibitMotionOnHomeSw while engaged and HiZ (STATUS 5.5)", "FAIL",
        steps=_ACTIVATE + (
            # A move in the homing direction (REV by default) so that dir == homingDirection.
            osc("/move", AXIS, -MOVE, label="moveRev"), wait(2.0),
            osc("/activate", AXIS, 0, label="activate0"), wait(0.5),
            osc("/setProhibitMotionOnHomeSw", AXIS, 1), wait(0.1),
            press(label="press"), wait(0.5),
            release(label="release"), wait(0.3),
        ),
        expected_failures=frozenset({"INV-2"}),
        notes="The task text says 'after /free (brake engaged, HiZ)', but /free releases the brake; "
              "this uses /activate 0 to get the engaged + HiZ state.",
    ),
    Scenario(
        "B7", "/activate 1 while released and running (STATUS 5.6)", "RECORD",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/activate", AXIS, 1, label="activate2"), wait(1.0),
        ),
        notes="Records whether the motor stops abruptly (position and OUT1A).",
    ),
    Scenario(
        "B8a", "/hardHiZ with the brake disabled (STATUS 7)", "PASS",
        extra_setup=(osc("/enableElectromagnetBrake", AXIS, 0),),
        steps=(
            osc("/hardStop", AXIS, label="hardStop"), wait(0.3),
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(0.5),
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.5),
        ),
        # The brake output stays low (engaged) while the brake is disabled, so the
        # invariants do not apply; check the excitation only.
        invariants=(),
        checks=(Check("HIZ-AFTER-HARDHIZ", "excited", False, "hardHiZ", None, LATENCY),),
    ),
    Scenario(
        "B8b", "/hardHiZ with the brake enabled, stopped (STATUS 7)", "PASS",
        steps=_ACTIVATE + (
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.5),
        ),
        checks=(
            Check("ENGAGED-AFTER-HARDHIZ", "brake", False, "hardHiZ", None, LATENCY),
            Check("HIZ-AFTER-TRANSITION", "excited", False, "hardHiZ", None, TRANSITION),
        ),
    ),
    Scenario(
        "B8c", "/hardHiZ with the brake enabled, running (STATUS 7)", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.5),
        ),
        checks=(
            Check("ENGAGED-AFTER-HARDHIZ", "brake", False, "hardHiZ", None, LATENCY),
            Check("HIZ-AFTER-TRANSITION", "excited", False, "hardHiZ", None, TRANSITION),
        ),
    ),
)


def get(scenario_id: str) -> Scenario:
    for s in SCENARIOS:
        if s.id.lower() == scenario_id.lower():
            return s
    raise KeyError(f"unknown scenario {scenario_id}; known: {', '.join(s.id for s in SCENARIOS)}")


def resolve_args(args: tuple, values: dict) -> tuple:
    """Replaces the placeholders (AXIS, KVAL, ...) with values[name]."""
    out = []
    for a in args:
        if isinstance(a, str) and a.startswith("{") and a.endswith("}"):
            out.append(values[a[1:-1]])
        else:
            out.append(a)
    return tuple(out)


def resolve_delay(delay: str | float, transition_s: float, latency_s: float) -> float:
    if delay == LATENCY:
        return latency_s
    if delay == TRANSITION:
        return transition_s + latency_s
    return float(delay)


def verdict(scenario: Scenario, failed: set[str]) -> tuple[str, bool | None]:
    """(outcome, matches expectation). outcome is PASS / FAIL; RECORD has no expectation."""
    outcome = "FAIL" if failed else "PASS"
    if scenario.expect == "RECORD":
        return outcome, None
    if scenario.expect == "PASS":
        return outcome, not failed
    return outcome, failed == set(scenario.expected_failures)
