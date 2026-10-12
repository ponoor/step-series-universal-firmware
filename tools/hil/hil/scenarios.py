"""Brake scenarios as data (TASK Task 5).

A scenario is a list of steps (OSC messages, USB serial commands, waits, home switch
operations) run while the AD2 records, plus the checks to apply and the expected outcome:

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
#   "poll":       one status poll of the firmware (10 ms) + latency_allowance_ms
LATENCY = "latency"
TRANSITION = "transition"
POLL = "poll"
STATUS_POLL_S = 0.010

# Time of a serial step on the AD2: the serial command has no probe pulse, so it is
# taken from the signal it changes.
EXCITATION_OFF = "excitation_off"  # first excited -> not excited edge after the previous step
BRAKE_ENGAGE = "brake_engage"  # first brake engage edge after the previous step


@dataclass(frozen=True)
class Step:
    kind: str  # "osc", "serial", "wait", "press", "release"
    address: str = ""  # OSC address, or the serial command character
    args: tuple = ()
    seconds: float = 0.0
    label: str = ""
    edge: str = ""  # serial: how its AD2 time is found (EXCITATION_OFF)


def osc(address: str, *args, label: str = "") -> Step:
    return Step("osc", address, tuple(args), label=label)


def serial(command: str, label: str = "", edge: str = EXCITATION_OFF) -> Step:
    """USB serial diagnosis command (e.g. 'Z' of HIL_PROBE builds)."""
    return Step("serial", command, label=label, edge=edge)


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
    """An invariant is not checked from one step to another (plus the latency allowance),
    or, with span, for a time after the step."""

    invariant: str
    from_label: str
    to_label: str | None = None
    span: str | float | None = None  # delay symbol or seconds; replaces to_label


@dataclass(frozen=True)
class Reply:
    """An OSC message from the board arrives (present) or not within window_s after the
    labelled step, by host time (network delay included, so the window is wide).
    args are the leading arguments to match; placeholders are resolved."""

    id: str
    from_label: str
    address: str
    args: tuple = ()
    present: bool = True
    window_s: float = 0.3


@dataclass(frozen=True)
class Motion:
    """The polled position (/getPosition) of the test axis changes between every two
    readings (moving) or stays the same (not moving) over a window."""

    id: str
    moving: bool
    from_label: str
    to_label: str | None = None
    delay: str | float = 0.0  # added to the from time
    to_delay: str | float = 0.0  # subtracted from the to time


@dataclass(frozen=True)
class EngageBeforeReset:
    """The brake engaged at the labelled step is engaged before the drivers enter standby
    at the MCU reset (OUT1A jump, see hil/restart.py) and stays engaged to the end."""

    id: str
    from_label: str


@dataclass(frozen=True)
class EngageAfterStop:
    """After the labelled step, the brake engages only once the position has stopped
    changing: the first engage edge is not before a reading that is followed by a change."""

    id: str
    from_label: str


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
    replies: tuple[Reply, ...] = ()
    motion: tuple[Motion | EngageAfterStop | EngageBeforeReset, ...] = ()
    extra_setup: tuple[Step, ...] = ()
    poll_position: bool = True
    # The board restarts during the steps (/resetDevice): only the first P1 pulses (one
    # per OSC message sent) are matched; the probe pins float during the restart.
    reboots: bool = False
    # Scope sample rate instead of [brake] analog_rate, and whether P0 on the Scope aligns
    # the recording (it needs a rate that resolves the P0 toggles).
    analog_rate: float | None = None
    p0_ref: bool = True
    notes: str = ""

    def labels(self) -> list[str]:
        return [s.label for s in self.steps if s.label]

    def check_ids(self) -> set[str]:
        return (set(self.invariants) | {c.id for c in self.checks} | {r.id for r in self.replies}
                | {m.id for m in self.motion})


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


def _brake_error(check_id: str, label: str, error: str, present: bool = True) -> Reply:
    return Reply(check_id, label, "/error/command", (error, AXIS), present)


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
            Check("ENGAGED-AFTER-SOFTHIZ", "brake", False, "softHiZ", "activate2", POLL),
            Check("RELEASED-AFTER-ACTIVATE2", "brake", True, "activate2", "free", TRANSITION),
            Check("ENGAGED-AFTER-ACTIVATE0", "brake", False, "activate0", None, LATENCY),
        ),
        notes="Also records the command -> brake output delay and the engaged + excited overlaps. "
              "/softHiZ on a stopped motor engages at the next status poll (up to 10 ms).",
    ),
    Scenario(
        "B2", "Servo mode enabled while the brake is engaged (STATUS 5.1)", "PASS",
        steps=(
            osc("/enableServoMode", AXIS, 1, label="servo"), wait(0.3),
            osc("/setTargetPosition", AXIS, MOVE, label="target"), wait(1.0),
            osc("/enableServoMode", AXIS, 0, label="servoOff"), wait(0.3),
        ),
        checks=(Check("NOT-EXCITED", "excited", False, "servo", None),),
        replies=(_brake_error("BRAKE-ENGAGED-REPLY", "servo", "BrakeEngaged"),),
        motion=(Motion("POSITION-UNCHANGED", False, "servo"),),
    ),
    Scenario(
        "B3", "softHiZ while running, then /free during the deceleration (STATUS 5.2)", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(0.05),
            osc("/free", AXIS, label="free"), wait(1.0),
        ),
        exempt=(Exempt("INV-1", "free"),),
        checks=(Check("RELEASED-AFTER-FREE", "brake", True, "free", None, LATENCY),),
        notes="Fixed: /free cancels the engaging plan of /softHiZ (K2).",
    ),
    Scenario(
        "B4", "softHiZ while running, then a motion command before the stop (STATUS 5.3)", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(0.05),
            # /run is accepted by the L6470 while decelerating (/move is not: BUSY).
            osc("/run", AXIS, MAX_SPEED, label="run2"), wait(1.0),
            osc("/softStop", AXIS, label="softStop"), wait(1.0),
        ),
        checks=(Check("RELEASED-AFTER-RUN2", "brake", True, "run2", None, LATENCY),),
        notes="Fixed: a motion command during the deceleration of /softHiZ cancels the engaging "
              "plan (K3). A /move sent while decelerating is rejected by the L6470 "
              "(CommandIgnored), so /run is used.",
    ),
    Scenario(
        "B5", "/activate 1 within 10 ms after /hardHiZ (STATUS 5.3)", "RECORD",
        steps=_ACTIVATE + (
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.005),
            osc("/activate", AXIS, 1, label="activate2"), wait(1.0),
        ),
    ),
    Scenario(
        "B6", "Home switch pressed with prohibitMotionOnHomeSw while engaged and HiZ (STATUS 5.5)", "PASS",
        steps=_ACTIVATE + (
            # A move in the homing direction (REV by default) so that dir == homingDirection.
            osc("/move", AXIS, -MOVE, label="moveRev"), wait(2.0),
            osc("/activate", AXIS, 0, label="activate0"), wait(0.5),
            osc("/setProhibitMotionOnHomeSw", AXIS, 1), wait(0.1),
            press(label="press"), wait(0.5),
            release(label="release"), wait(0.3),
        ),
        checks=(Check("NOT-EXCITED-ON-PRESS", "excited", False, "press", None),),
        notes="The task text says 'after /free (brake engaged, HiZ)', but /free releases the brake; "
              "this uses /activate 0 to get the engaged + HiZ state. Fixed: no HardStop to a HiZ "
              "motor (K5).",
    ),
    Scenario(
        "B7", "/activate 1 while released and running (STATUS 5.6)", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/activate", AXIS, 1, label="activate2"), wait(1.0),
        ),
        checks=(Check("RELEASED-AFTER-ACTIVATE2", "brake", True, "activate2", None),),
        motion=(Motion("KEEPS-RUNNING", True, "activate2"),),
        notes="Fixed: /activate 1 does nothing while released (K6).",
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
    Scenario(
        "B9", "softHiZ while running, hardHiZ 50 ms later, /activate 1 5 ms later", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(0.05),
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.005),
            osc("/activate", AXIS, 1, label="activate2"), wait(1.0),
        ),
        checks=(
            Check("RELEASED-AFTER-ACTIVATE2", "brake", True, "activate2", None, TRANSITION),
            Check("EXCITED-TO-END", "excited", True, "hardHiZ", None),
        ),
    ),
    Scenario(
        "B10", "/hardStop, /softStop, /enableServoMode 0 while engaged and HiZ", "PASS",
        steps=(
            osc("/hardStop", AXIS, label="hardStop"), wait(0.3),
            osc("/softStop", AXIS, label="softStop"), wait(0.3),
            osc("/enableServoMode", AXIS, 0, label="servoOff"), wait(0.3),
        ),
        checks=(Check("NOT-EXCITED", "excited", False, "hardStop", None),),
        notes="N1: these commands energized the HiZ motor and left the state BRAKE_ENGAGED.",
    ),
    Scenario(
        "B11", "/homing rejected while engaged, then accepted after /activate 1", "PASS",
        steps=(
            osc("/setGoUntilTimeout", AXIS, 500), wait(0.1),
            osc("/homing", AXIS, label="homing1"), wait(1.0),
            osc("/activate", AXIS, 1, label="activate"), wait(0.5),
            osc("/homing", AXIS, label="homing2"), wait(0.3),
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(0.5),
        ),
        checks=(Check("NOT-EXCITED-AFTER-HOMING1", "excited", False, "homing1", "activate"),),
        replies=(
            _brake_error("HOMING1-BRAKE-ENGAGED", "homing1", "BrakeEngaged"),
            _brake_error("HOMING2-NO-ERROR", "homing2", "BrakeEngaged", present=False),
            Reply("HOMING2-STARTED", "homing2", "/homingStatus", (AXIS, 1)),
        ),
        motion=(Motion("HOMING2-RUNS", True, "homing2", "hardHiZ", delay=0.05),),
        notes="N2: the rejected /homing left bHoming set; the goUntil timeout then sent HardStop. "
              "A 0.5 s wait after /activate 1 lets the brake release. The home switch is not "
              "pressed; /hardHiZ comes before the 500 ms timeout.",
    ),
    Scenario(
        "B12a", "/goUntil sent directly, /hardHiZ, then the goUntil timeout", "PASS",
        steps=_ACTIVATE + (
            osc("/setGoUntilTimeout", AXIS, 500), wait(0.1),
            osc("/goUntil", AXIS, 0, MAX_SPEED, label="goUntil"), wait(0.2),
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(1.0),
        ),
        checks=(Check("HIZ-AFTER-TRANSITION", "excited", False, "hardHiZ", None, TRANSITION),),
        notes="N3: homingStatus stayed GOUNTIL, and the timeout sent HardStop to the HiZ motor.",
    ),
    Scenario(
        "B12b", "B12a with the brake disabled", "PASS",
        extra_setup=(osc("/enableElectromagnetBrake", AXIS, 0),),
        steps=(
            osc("/setGoUntilTimeout", AXIS, 500), wait(0.1),
            osc("/goUntil", AXIS, 0, MAX_SPEED, label="goUntil"), wait(0.2),
            osc("/hardHiZ", AXIS, label="hardHiZ"), wait(1.0),
        ),
        invariants=(),
        checks=(Check("HIZ-AFTER-HARDHIZ", "excited", False, "hardHiZ", None, LATENCY),),
    ),
    Scenario(
        "B13", "/goTo during the acceleration of /run, then /softHiZ", "RECORD",
        extra_setup=(osc("/setAcc", AXIS, 50.0),),
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(0.5),
            osc("/goTo", AXIS, 2000000, label="goTo"), wait(0.005),
            osc("/softHiZ", AXIS, label="softHiZ"), wait(1.0),
        ),
        checks=(
            Check("ENGAGED-AFTER-STOP", "brake", False, "softHiZ", None, 0.5),
            Check("HIZ-AFTER-STOP", "excited", False, "softHiZ", None, 0.7),
        ),
        motion=(Motion("NOT-MOVING-AFTER-STOP", False, "softHiZ", delay=0.4),),
        notes="N4. RECORD: whether /goTo waited for BUSY (clearBusyForGoTo) cannot be observed "
              "from outside; the checks pass whether it waited or not with the fixed firmware.",
    ),
    Scenario(
        "B14", "/activate 0 while running", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/activate", AXIS, 0, label="activate0"), wait(1.0),
        ),
        checks=(
            Check("ENGAGED-AT-END", "brake", False, "activate0", None, 0.5),
            Check("HIZ-AT-END", "excited", False, "activate0", None, 0.7),
        ),
        motion=(EngageAfterStop("ENGAGE-AFTER-STOP", "activate0"),),
        notes="N5: /activate 0 engaged the brake without stopping the motor. Deceleration from "
              "MAX_SPEED at the default 1000 steps/s^2 takes about 0.2 s.",
    ),
    Scenario(
        "B15", "Driver HiZ by itself while released (serial Z)", "PASS",
        steps=_ACTIVATE + (
            serial("Z", label="Z"), wait(0.5),
        ),
        # The brake engages at the next status poll.
        exempt=(Exempt("INV-1", "Z", span=POLL),),
        checks=(Check("ENGAGED-AFTER-Z", "brake", False, "Z", None, POLL),),
        replies=(_brake_error("ENGAGED-ON-HIZ-REPLY", "Z", "BrakeEngagedOnHiZ"),),
        notes="N6. The time of Z is the excitation-off edge on OUT1A (the serial command has "
              "no probe pulse). Needs a HIL_PROBE build.",
    ),
    Scenario(
        "B16", "/resetMotorDriver while released", "PASS",
        steps=_ACTIVATE + (
            osc("/resetMotorDriver", AXIS, label="reset"), wait(0.5),
        ),
        checks=(
            Check("RELEASED", "brake", True, "reset", None),
            Check("EXCITED", "excited", True, "reset", None),
        ),
        replies=(_brake_error("IGNORED-REPLY", "reset", "CommandIgnored"),),
    ),
    Scenario(
        "B17", "/enableElectromagnetBrake 0 while released and running", "PASS",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/enableElectromagnetBrake", AXIS, 0, label="disable"), wait(0.5),
            osc("/softStop", AXIS, label="softStop"), wait(0.5),
        ),
        checks=(Check("RELEASED", "brake", True, "disable", None),),
        replies=(_brake_error("IGNORED-REPLY", "disable", "CommandIgnored"),),
        motion=(Motion("KEEPS-RUNNING", True, "disable", "softStop"),),
    ),
    Scenario(
        "B18", "/resetDevice while released and running", "RECORD",
        steps=_ACTIVATE + (
            osc("/run", AXIS, MAX_SPEED, label="run"), wait(1.0),
            osc("/resetDevice", label="resetDevice"), wait(2.0),
        ),
        # At 10 kS/s the window median misreads the stepping waveform while running.
        exempt=(Exempt("INV-1", "run", "resetDevice"),),
        poll_position=False,
        reboots=True,
        # The USB re-enumeration of the STEP800 stalls the AD2 stream for about 0.5 s: at
        # 200 kS/s the Scope lost 98303 samples every time. 10 kS/s fits the stall in the
        # device buffer; the excitation is then judged by the window median (clamp_v), and
        # the Scope is aligned by [sync] (+/-0.4 ms) instead of P0.
        analog_rate=1e4,
        p0_ref=False,
        notes="Records the brake engage and the excitation stop against /resetDevice. Run last: "
              "the board restarts. The MCU reset puts the drivers in standby: OUT1A then sits at "
              "about 0.13 V, which the excitation detection reads as excited (below clamp_v).",
    ),
    Scenario(
        "B19", "USB serial 'R' while released and stopped", "PASS",
        steps=_ACTIVATE + (
            serial("R", label="R", edge=BRAKE_ENGAGE), wait(3.0),
        ),
        invariants=(),
        motion=(EngageBeforeReset("ENGAGED-BEFORE-RESET", "R"),),
        poll_position=False,
        reboots=True,
        analog_rate=1e4,
        p0_ref=False,
        notes="'R' goes through safeSystemReset(): brake engaged, wait, then the MCU reset, which "
              "puts the drivers in standby. The step is timed by the brake engage edge.",
    ),
    Scenario(
        "B20", "Watchdog: USB serial 'W' (hang) while released and stopped", "PASS",
        steps=_ACTIVATE + (
            serial("W", label="W", edge=BRAKE_ENGAGE), wait(4.0),
        ),
        invariants=(),
        motion=(EngageBeforeReset("ENGAGED-BEFORE-RESET", "W"),),
        poll_position=False,
        reboots=True,
        analog_rate=1e4,
        p0_ref=False,
        notes="The watchdog early warning (2 s after the last clear) engages every brake output "
              "without SPI and waits the longest transition time, then the watchdog resets the "
              "MCU. The step is timed by the brake engage edge, 2 s after 'W'.",
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
    if delay == POLL:
        return STATUS_POLL_S + latency_s
    return float(delay)


def verdict(scenario: Scenario, failed: set[str]) -> tuple[str, bool | None]:
    """(outcome, matches expectation). outcome is PASS / FAIL; RECORD has no expectation."""
    outcome = "FAIL" if failed else "PASS"
    if scenario.expect == "RECORD":
        return outcome, None
    if scenario.expect == "PASS":
        return outcome, not failed
    return outcome, failed == set(scenario.expected_failures)
