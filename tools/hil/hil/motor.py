"""hil.py motor: tests with a real motor, brake and load (STEP400_R1, TASK_step400_motor).

Record only: OUT1A is not on the Scope (a motor is connected, motor.motor_connected = true),
so nothing is judged from the excitation. Each step is a list of actions (OSC, USB serial,
waits, switch operations, operator requests) run while the AD2 Digital In records the
brake outputs, the probes, the switches and the 3.3 V. The result is one timeline on the
AD2 time base:

- OSC commands sent: the P1 rise (packet processing) that each one caused;
- OSC messages received: host receive time mapped to the AD2 time with the command fit
  (network and host latency included, so a few ms late; marked "host");
- USB serial commands, switch operations and operator requests: host time mapped the same way;
- brake outputs (J7), switch inputs and the 3.3 V: their edges;
- MCU stops: gaps in the P0 loop toggle (a restart), and the first P0 edge after them.

The human observations are entered after the run with --note and stored in summary.json.
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import analyze
from .config import Config
from .diag import DiagError, SerialDiag
from .osc import OscError, OscLink

AXIS = "{axis}"
# Microsteps per full step at the default step mode (1/128).
USTEP = 128
# P0 toggles every loop() pass (about 50 us); a gap longer than this is an MCU stop.
P0_GAP_S = 0.05
# Received messages left out of the timeline (written to positions.csv / counted).
TIMELINE_SKIP_RX = ("/position", "/busy", "/brakeStatus")
# Received messages listed only when their arguments change (replies to wait_reply polling).
TIMELINE_CHANGES_RX = ("/uvlo",)


@dataclass(frozen=True)
class Act:
    kind: str  # osc, serial, wait, idle, wait_osc, press, release, operator, reconnect, query, sine
    address: str = ""  # OSC address, serial character, switch name, operator text
    args: tuple = ()
    seconds: float = 0.0
    label: str = ""


def osc(address: str, *args, label: str = "") -> Act:
    return Act("osc", address, tuple(args), label=label)


def serial(ch: str, label: str = "") -> Act:
    return Act("serial", ch, label=label)


def wait(seconds: float) -> Act:
    return Act("wait", seconds=seconds)


def idle(timeout: float = 10.0) -> Act:
    """Polls /getBusy of the test axis every 50 ms until it is not busy."""
    return Act("idle", seconds=timeout)


def wait_osc(address: str, timeout: float, label: str = "") -> Act:
    return Act("wait_osc", address, seconds=timeout, label=label)


def press(switch: str = "home", label: str = "") -> Act:
    return Act("press", switch, label=label)


def release(switch: str = "home", label: str = "") -> Act:
    return Act("release", switch, label=label)


def operator(text: str, label: str = "") -> Act:
    """Prints 'OPERATOR: text'. With a terminal, waits for Enter; otherwise goes on (the
    operator was told before the run, and the next action waits for the result)."""
    return Act("operator", text, label=label)


def reconnect(timeout: float = 30.0) -> Act:
    """After a restart: reopen the USB serial, /setDestIp, and apply the setup again."""
    return Act("reconnect", seconds=timeout)


def query(address: str, reply: str, *args, label: str = "") -> Act:
    """Request with reply, stored in summary.json queries (only for side-effect-free gets)."""
    return Act("query", address, (reply,) + tuple(args), label=label)


def wait_reply(address: str, reply: str, value: int, timeout: float, label: str = "") -> Act:
    """Polls address (test axis) every 0.3 s until the reply's second argument is value."""
    return Act("wait_reply", address, (reply, value), seconds=timeout, label=label)


def close_serial(label: str = "") -> Act:
    """Closes the USB serial port on the host (as with no terminal open on the board)."""
    return Act("close_serial", label=label)


def ocd_sweep(start: int = 14, stop: int = 0, dwell_s: float = 1.5) -> Act:
    """Lowers OCD_TH of the test axis one step at a time (raw start..stop), dwell_s per step,
    until /overCurrent or /HiZ (test axis, HiZ) arrives."""
    return Act("ocd_sweep", args=(start, stop), seconds=dwell_s)


def sine(amplitude: int, period_s: float, duration_s: float, rate_hz: float = 100.0) -> Act:
    """/setTargetPositionList at rate_hz: the test axis follows center + amplitude *
    sin(2 pi t / period), center = its position at the start; the others keep theirs."""
    return Act("sine", args=(amplitude, period_s, duration_s, rate_hz))


@dataclass(frozen=True)
class MotorStep:
    id: str
    title: str
    observe: str  # what the operator watches (shown before the run)
    actions: tuple
    expect: str = ""
    brake: bool = True  # enable the brake of the test axis in the setup
    restart: bool = False  # the step restarts the board: 10 kS/s


def _activate_cycle(n: int, off: str = "/activate", gap: float = 1.5) -> tuple:
    acts: tuple = ()
    for k in range(n):
        acts += (osc("/activate", AXIS, 1, label=f"activate1_{k + 1}"), wait(gap),
                 osc(off, AXIS, *((0,) if off == "/activate" else ()), label=f"{off[1:]}_{k + 1}"), wait(gap))
    return acts


def _moving_stop(n: int, stop: str, span: int = 25 * USTEP, speed: float = 20.0) -> tuple:
    """From -span, run towards + at speed and send stop at the horizontal position (0)."""
    acts: tuple = ()
    for k in range(n):
        acts += (osc("/activate", AXIS, 1), wait(0.3), osc("/goTo", AXIS, -span), idle(), wait(0.3),
                 osc("/run", AXIS, speed, label=f"run_{k + 1}"), wait(span / USTEP / speed),
                 osc(stop, AXIS, label=f"{stop[1:]}_{k + 1}"), wait(1.5),
                 query("/getPosition", "/position", AXIS, label=f"position_{k + 1}"))
    return acts


def _restart(how: str, n: int = 3) -> tuple:
    acts: tuple = ()
    for k in range(n):
        acts += (osc("/activate", AXIS, 1), wait(1.0))
        if how == "R":
            acts += (serial("R", label=f"R_{k + 1}"),)
        elif how == "resetDevice":
            acts += (osc("/resetDevice", label=f"resetDevice_{k + 1}"),)
        elif how == "W":
            acts += (serial("W", label=f"W_{k + 1}"),)
        elif how == "button":
            acts += (operator("press the reset button once", label=f"button_{k + 1}"),)
        elif how == "power":
            acts += (operator("switch off USB and both 24 V inlets (STEP400, Brake Board), wait 5 s, switch "
                              "them on", label=f"power_{k + 1}"),)
        acts += (wait_osc("/booted", 180.0 if how in ("button", "power") else 20.0, label=f"booted_{k + 1}"),
                 reconnect(), query("/getBrakeStatus", "/brakeStatus", AXIS, label=f"brake_after_{k + 1}"),
                 wait(1.0))
    return acts


# The rod falls to hanging down (/free), then is turned +50 full steps to horizontal (the
# direction found with m0); the position is 0 there, the brake engaged.
HORIZONTAL = (osc("/activate", AXIS, 1), wait(1.0), osc("/free", AXIS), wait(3.0), osc("/resetPos", AXIS),
              osc("/activate", AXIS, 1), wait(1.0), osc("/move", AXIS, 50 * USTEP), idle(20.0), wait(0.5),
              osc("/activate", AXIS, 0), wait(0.5), osc("/resetPos", AXIS), wait(1.0))

STEPS: dict[str, MotorStep] = {s.id: s for s in [
    MotorStep("m0", "first excitation: brake on, /activate 1, +/-200 full steps, /activate 0",
              "rod tip: does it move smoothly, any noise or vibration; brake clicks at the release and the "
              "engage (two clicks in all)",
              (osc("/activate", AXIS, 1, label="activate1"), wait(1.0),
               osc("/move", AXIS, 200 * USTEP, label="move+"), idle(), wait(0.5),
               osc("/move", AXIS, -200 * USTEP, label="move-"), idle(), wait(0.5),
               osc("/activate", AXIS, 0, label="activate0"), wait(1.0)),
              "brake released about 100 ms after /activate 1, engaged at /activate 0, HiZ 100 ms later"),
    MotorStep("m1-12", "M1-1 / M1-2: /activate 1 then /activate 0, 3 times (stopped)",
              "rod tip: does it drop (mm) at each release and each engage; brake clicks",
              _activate_cycle(3),
              "release about 100 ms after /activate 1, engage at /activate 0 and HiZ 100 ms later; no drop"),
    MotorStep("m1-3", "M1-3: /activate 1 then /softHiZ, 3 times (stopped)",
              "rod tip: does it drop (mm) at each /softHiZ; brake clicks",
              _activate_cycle(3, "/softHiZ"), "as M1-2; no drop"),
    MotorStep("m1-4", "M1-4: /softHiZ while turning at 20 step/s through the horizontal position, 3 times",
              "rod: decelerates and stops, then does it drop (mm) from where it stopped",
              _moving_stop(3, "/softHiZ"), "decelerate, engage at the stop, HiZ 100 ms later; no drop"),
    MotorStep("m1-5", "M1-5: /hardHiZ while turning at 20 step/s through the horizontal position, 3 times",
              "rod: stops at once, then does it drop (mm) from where it stopped",
              _moving_stop(3, "/hardHiZ"), "hard stop (energized), engage, HiZ 100 ms later; no drop"),
    MotorStep("m1-6", "M1-6: /activate 0 then /activate 1 30 ms later (during the engage wait), 3 times",
              "rod tip: any drop (mm); brake clicks (engage then release again, or nothing)",
              sum(((osc("/activate", AXIS, 1), wait(1.0), osc("/activate", AXIS, 0, label=f"activate0_{k + 1}"),
                    wait(0.03), osc("/activate", AXIS, 1, label=f"activate1_{k + 1}"), wait(1.0),
                    query("/getBrakeStatus", "/brakeStatus", AXIS, label=f"state_{k + 1}"),
                    osc("/activate", AXIS, 0), wait(1.0)) for k in range(3)), ()),
              "the engage is cancelled, energized and released (state 2); no drop"),
    MotorStep("m1-7", "M1-7: /free, turn by hand, /activate 1",
              "after /free (operator prompt): turn the rod by hand, does it turn freely; after /activate 1: "
              "held, brake released",
              # KVAL_HOLD 0 first, so that the raise to KVAL_RUN / 2 is seen. The rod falls at /free;
              # the operator leaves it hanging down, and it is turned back to horizontal (+50 full
              # steps, the direction found in m0) with KVAL_HOLD back at its setting.
              (osc("/activate", AXIS, 1), wait(1.0), osc("/setKval", AXIS, 0, 16, 16, 16), wait(0.2),
               query("/getKval", "/kval", AXIS, label="kval_before"),
               osc("/free", AXIS, label="free"), wait(0.5),
               query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_free"),
               operator("turn the rod by hand now (15 s), then leave it hanging down", label="hand"),
               wait(15.0), osc("/resetPos", AXIS),
               osc("/activate", AXIS, 1, label="activate1"), wait(1.0),
               query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_activate"),
               query("/getKval", "/kval", AXIS, label="kval_after"), wait(0.5),
               osc("/setKval", AXIS, 16, 16, 16, 16), wait(0.2), osc("/move", AXIS, 50 * USTEP), idle(20.0),
               wait(0.5), osc("/activate", AXIS, 0), wait(0.5), osc("/resetPos", AXIS)),
              "/free: released + HiZ, turns by hand; /activate 1: KVAL_HOLD raised to 8, energized, released"),
    MotorStep("m1-8", "M1-8 (record): brakeTransitionDuration 0, /activate 1 then /activate 0, once",
              "rod tip: does it drop (mm) at the release and at the engage",
              (osc("/setBrakeTransitionDuration", AXIS, 0), wait(0.2)) + _activate_cycle(1)
              + (osc("/setBrakeTransitionDuration", AXIS, 100), wait(0.2)),
              "record: the brake and the excitation switch together, may drop up to about 12 mm"),
    MotorStep("m2-R", "M2-1: USB serial R, 3 times", "rod tip: does it drop (mm) at each restart",
              _restart("R"), "engage about 100 ms before the reset; no drop", restart=True),
    MotorStep("m2-resetDevice", "M2-2: /resetDevice, 3 times", "rod tip: does it drop (mm) at each restart",
              _restart("resetDevice"), "as M2-1", restart=True),
    MotorStep("m2-W", "M2-3: watchdog (USB serial W), 3 times", "rod tip: does it drop (mm) at each restart",
              _restart("W"), "engage at the early warning (WATCHDOG_TIMEOUT_MS), reset about 100 ms later; no drop",
              restart=True),
    MotorStep("m2-button", "M2-4 (record): reset button, 3 times (operator)",
              "rod tip: does it drop (mm) at each press", _restart("button"),
              "record: J7 engaged at once, no support until the brake acts (about 50 ms)", restart=True),
    MotorStep("m2-power", "M2-5: power cycle (USB and 24 V off, then on), 3 times (operator)",
              "rod tip: does it drop (mm) at the power off, at the power on, and at the release of the next "
              "/activate 1 after the restart (EL_POS reset, see m2-button)", _restart("power"),
              "brake engaged while off (power-off brake); J7 engaged after the power-on; no drop at off / on",
              restart=True),
    MotorStep("usb-open", "USB unplugged while the host keeps the serial port open (24 V on)",
              "rod: any drop; at the unplug and about 4 s later (restart)",
              (osc("/activate", AXIS, 1), wait(1.0),
               operator("unplug the USB cable of the board now; plug it back in after about 10 s", label="unplug"),
               wait_osc("/booted", 60.0, label="booted"), wait(1.0),
               query("/getBrakeStatus", "/brakeStatus", AXIS, label="brake_after"), wait(10.0)),
              "hypothesis: P0 stops at the unplug, J7 released until a watchdog reset about 4 s later "
              "(/booted resetCause 4)", restart=True),
    MotorStep("usb-closed", "USB unplugged with the serial port closed on the host (24 V on)",
              "rod: any drop; at the unplug and about 4 s later (restart)",
              (osc("/activate", AXIS, 1), wait(1.0), close_serial(label="close"), wait(1.0),
               operator("unplug the USB cable of the board now; plug it back in after about 10 s", label="unplug"),
               wait_osc("/booted", 30.0, label="booted"), wait(1.0),
               query("/getBrakeStatus", "/brakeStatus", AXIS, label="brake_after"), wait(10.0)),
              "as usb-open, or no hang", restart=True),
    MotorStep("m3-1", "M3-1: real overcurrent, holding (released, stopped): OCD_TH lowered 14 -> 0",
              "rod: does it drop (mm) when the driver shuts down; brake click",
              HORIZONTAL + (osc("/activate", AXIS, 1, label="activate1"), wait(1.0), ocd_sweep(),
                            wait(3.0), query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_trip"),
                            osc("/setOverCurrentThreshold", AXIS, 15, label="ocd_restore"), wait(0.5),
                            osc("/activate", AXIS, 1, label="recover"), wait(1.5),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_recover"),
                            query("/getHiZ", "/HiZ", AXIS, label="hiz_recover"),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "/overCurrent at once, /HiZ, BrakeEngagedOnHiZ, J7 engaged within 5 ms of the HiZ; recovery "
              "with /activate 1 without /getStatus"),
    MotorStep("m3-1s", "M3-1 servo: real overcurrent in the servo mode: OCD_TH lowered 14 -> 0",
              "rod: does it drop (mm) when the driver shuts down; brake click",
              HORIZONTAL + (osc("/activate", AXIS, 1), wait(1.0), osc("/enableServoMode", AXIS, 1, label="servo"),
                            wait(1.0), ocd_sweep(), wait(3.0),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_trip"),
                            osc("/setOverCurrentThreshold", AXIS, 15, label="ocd_restore"), wait(0.5),
                            osc("/activate", AXIS, 1, label="recover"), wait(1.5),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_recover"),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "as M3-1 plus ServoModeCancelledOnHiZ"),
    MotorStep("m3-2a", "M3-2a: both 24 V inlets (STEP400 and Brake Board) off and on, USB connected (operator)",
              "rod: drop (mm) at the power off; at the power on (must not fall); at the release after /activate 1",
              HORIZONTAL + (osc("/activate", AXIS, 1, label="activate1"), wait(1.0),
                            operator("switch off both 24 V inlets (STEP400 and Brake Board) (USB stays connected)", label="off"),
                            wait_reply("/getUvlo", "/uvlo", 1, 120.0, label="uvlo"), wait(2.0),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_off"),
                            operator("switch both 24 V inlets (STEP400 and Brake Board) on again", label="on"),
                            wait_reply("/getUvlo", "/uvlo", 0, 120.0, label="uvlo_cleared"), wait(2.0),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_on"),
                            osc("/activate", AXIS, 1, label="recover"), wait(1.5),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_recover"),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "/uvlo, /HiZ, BrakeEngagedOnHiZ, state 0; J7 stays engaged after the power on; /activate 1 "
              "releases (state 2), no BrakeReleaseFailed"),
    MotorStep("m3-2b", "M3-2b: the STEP400 24 V only (Brake Board stays on) off and on, USB connected (operator)",
              "rod: drop (mm) at the power off; at the power on (must not fall); at the release after /activate 1",
              HORIZONTAL + (osc("/activate", AXIS, 1, label="activate1"), wait(1.0),
                            operator("switch off the STEP400 24 V only (Brake Board stays on) (USB stays connected)", label="off"),
                            wait_reply("/getUvlo", "/uvlo", 1, 120.0, label="uvlo"), wait(2.0),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_off"),
                            operator("switch the STEP400 24 V only (Brake Board stays on) on again", label="on"),
                            wait_reply("/getUvlo", "/uvlo", 0, 120.0, label="uvlo_cleared"), wait(2.0),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_on"),
                            osc("/activate", AXIS, 1, label="recover"), wait(1.5),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_recover"),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "/uvlo, /HiZ, BrakeEngagedOnHiZ, state 0; J7 stays engaged after the power on; /activate 1 "
              "releases (state 2), no BrakeReleaseFailed"),
    MotorStep("m4-a", "M4: /run 20 step/s for 25 s against gravity (the rod lifts, slips and falls back), STALL_TH default (31)",
              "rod: when it slips and falls back (count, times)",
              (osc("/activate", AXIS, 1), wait(1.0), query("/getStallThreshold", "/stallThreshold", AXIS, label="stall_th"),
               osc("/run", AXIS, 20.0, label="run"), wait(25.0), osc("/softStop", AXIS, label="stop"), idle(5.0),
               query("/getStatus", "/status", AXIS, label="status"), osc("/activate", AXIS, 0), wait(0.5)),
              "record: /stall (first at once, then at most every 200 ms) and the STALL bits of /getStatus"),
    MotorStep("m4-b2", "M4: /run 20 step/s for 25 s against gravity (the rod lifts, slips and falls back), STALL_TH raw 2 (937.5 mA)",
              "rod: when it slips and falls back (count, times)",
              (osc("/activate", AXIS, 1), wait(1.0), osc("/setStallThreshold", AXIS, 2), wait(0.3), query("/getStallThreshold", "/stallThreshold", AXIS, label="stall_th"),
               osc("/run", AXIS, 20.0, label="run"), wait(25.0), osc("/softStop", AXIS, label="stop"), idle(5.0),
               query("/getStatus", "/status", AXIS, label="status"), osc("/setStallThreshold", AXIS, 31), osc("/activate", AXIS, 0), wait(0.5)),
              "record: /stall (first at once, then at most every 200 ms) and the STALL bits of /getStatus"),
    MotorStep("m4-b1", "M4: /run 20 step/s for 25 s against gravity (the rod lifts, slips and falls back), STALL_TH raw 1 (625 mA)",
              "rod: when it slips and falls back (count, times)",
              (osc("/activate", AXIS, 1), wait(1.0), osc("/setStallThreshold", AXIS, 1), wait(0.3), query("/getStallThreshold", "/stallThreshold", AXIS, label="stall_th"),
               osc("/run", AXIS, 20.0, label="run"), wait(25.0), osc("/softStop", AXIS, label="stop"), idle(5.0),
               query("/getStatus", "/status", AXIS, label="status"), osc("/setStallThreshold", AXIS, 31), osc("/activate", AXIS, 0), wait(0.5)),
              "record: /stall (first at once, then at most every 200 ms) and the STALL bits of /getStatus"),
    MotorStep("m5-1", "M5-1: servo mode, target steps 0, +-400, +-1600, +-3200 microsteps (2 s each)",
              "rod: follows each target and stops there; any step loss, noise, vibration, oscillation",
              HORIZONTAL + (osc("/activate", AXIS, 1), wait(1.0),
                            query("/getServoParam", "/servoParam", AXIS, label="servo_param"),
                            osc("/enableServoMode", AXIS, 1, label="servo_on"), wait(1.0))
              + sum(((osc("/setTargetPosition", AXIS, t, label=f"target_{t}"), wait(2.0),
                      query("/getPosition", "/position", AXIS, label=f"pos_{t}"))
                     for t in (400, 0, -400, 0, 1600, 0, -1600, 0, 3200, 0, -3200, 0)), ())
              + (osc("/enableServoMode", AXIS, 0), wait(0.5), osc("/activate", AXIS, 0), wait(0.5)),
              "position settles at each target; no step loss"),
    MotorStep("m5-2", "M5-2: servo mode, /setTargetPositionList at 100 Hz, sine +-800 microsteps, period 2 s, 10 s",
              "rod: follows the sine smoothly; any step loss, noise",
              HORIZONTAL + (osc("/activate", AXIS, 1), wait(1.0), osc("/enableServoMode", AXIS, 1, label="servo_on"),
                            wait(1.0), sine(800, 2.0, 10.0), wait(1.0),
                            query("/getPosition", "/position", AXIS, label="pos_end"),
                            osc("/enableServoMode", AXIS, 0), wait(0.5), osc("/activate", AXIS, 0), wait(0.5)),
              "follows with a delay, no step loss"),
    MotorStep("m5-3", "M5-3: servo mode with the limit switch pressed (prohibitMotionOnLimitSw 1, homingDirection REV)",
              "rod: with the switch pressed, does not move to +1600, moves to -1600, does not move back to 0; "
              "after the release, moves to 0",
              HORIZONTAL + (osc("/setProhibitMotionOnLimitSw", AXIS, 1), osc("/activate", AXIS, 1), wait(1.0),
                            osc("/enableServoMode", AXIS, 1), wait(1.0), press("limit", label="limit_on"), wait(0.5),
                            osc("/setTargetPosition", AXIS, 1600, label="t+1600"), wait(2.0),
                            query("/getPosition", "/position", AXIS, label="pos_plus_blocked"),
                            osc("/setTargetPosition", AXIS, -1600, label="t-1600"), wait(2.0),
                            query("/getPosition", "/position", AXIS, label="pos_minus"),
                            osc("/setTargetPosition", AXIS, 0, label="t0"), wait(2.0),
                            query("/getPosition", "/position", AXIS, label="pos_back_blocked"),
                            release("limit", label="limit_off"), wait(2.0),
                            query("/getPosition", "/position", AXIS, label="pos_released"),
                            osc("/enableServoMode", AXIS, 0), osc("/setProhibitMotionOnLimitSw", AXIS, 0), wait(0.5),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "+ (away from homingDirection) blocked while pressed, - moves; 0 reached after the release"),
    MotorStep("m5-4", "M5-4: /goUntil and /releaseSw in the servo mode",
              "rod: does not move",
              HORIZONTAL + (osc("/activate", AXIS, 1), wait(1.0), osc("/enableServoMode", AXIS, 1), wait(1.0),
                            osc("/goUntil", AXIS, 0, 20.0, label="goUntil"), wait(1.0),
                            osc("/releaseSw", AXIS, 0, 1, label="releaseSw"), wait(1.0),
                            query("/getPosition", "/position", AXIS, label="pos"),
                            osc("/enableServoMode", AXIS, 0), wait(0.5), osc("/activate", AXIS, 0), wait(0.5)),
              "/error/command InServoMode twice, position unchanged"),
    MotorStep("m6", "M6: current mode (TVAL 0/16/16/16, 1/16 step): hold raised at /activate 1 and after /free",
              "rod: held at horizontal after /activate 1 (3 s); +/-8 full steps; drop or jump at each release; "
              "falls at /free",
              HORIZONTAL + (osc("/setCurrentMode", AXIS, label="current_mode"), wait(0.3),
                            query("/getMicrostepMode", "/microstepMode", AXIS, label="step_mode"),
                            query("/getTval", "/tval", AXIS, label="tval_before"),
                            osc("/activate", AXIS, 1, label="activate1"), wait(1.0),
                            query("/getTval", "/tval", AXIS, label="tval_activate"), wait(3.0),
                            osc("/move", AXIS, 8 * 16, label="move+"), idle(), wait(0.5),
                            osc("/move", AXIS, -8 * 16, label="move-"), idle(), wait(0.5),
                            osc("/activate", AXIS, 0), wait(0.5),
                            osc("/setTval", AXIS, 0, 16, 16, 16), wait(0.2),
                            query("/getTval", "/tval", AXIS, label="tval_reset"),
                            osc("/activate", AXIS, 1), wait(1.0), osc("/free", AXIS, label="free"), wait(2.0),
                            osc("/move", AXIS, 16, label="move_after_free"), idle(), wait(0.5),
                            query("/getTval", "/tval", AXIS, label="tval_after_free"),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_after_free"),
                            osc("/activate", AXIS, 0), wait(0.5), osc("/setVoltageMode", AXIS), wait(0.3),
                            serial("R", label="R"), wait_osc("/booted", 20.0, label="booted"), reconnect(),
                            query("/getMicrostepMode", "/microstepMode", AXIS, label="step_mode_after_R")),
              "TVAL_HOLD 0 -> 8 at /activate 1 and at the motion command after /free; held; voltage mode and "
              "1/128 after R", restart=True),
    MotorStep("m7-home", "M7-2: startup homing with the brake (config.txt on the SD card), home switch by DIO 15",
              "rod: at the startup release (drop / slip), the homing motion (REV, 20 step/s), stop after the "
              "switch; drop at any point",
              HORIZONTAL + (serial("R", label="R"), wait_osc("/booted", 20.0, label="booted"), reconnect(),
                            query("/getConfigName", "/configName", label="config"),
                            wait_reply("/getHomingStatus", "/homingStatus", 1, 10.0, label="goUntil"), wait(1.0),
                            press("home", label="home_on"),
                            wait_reply("/getHomingStatus", "/homingStatus", 2, 5.0, label="releaseSw"),
                            release("home", label="home_off"),
                            wait_reply("/getHomingStatus", "/homingStatus", 3, 15.0, label="completed"), wait(0.5),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_after"),
                            query("/getHiZ", "/HiZ", AXIS, label="hiz_after"),
                            query("/getPosition", "/position", AXIS, label="pos_after"), wait(1.0),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "brake released (as /activate 1) before the homing starts; GoUntil -> ReleaseSW -> completed (3); "
              "brake RELEASED (2) and energized after it", restart=True),
    MotorStep("m7-timeout", "M7-3: startup homing with the brake, no switch: goUntil timeout (config.txt)",
              "rod: at the startup release, the homing motion, at the timeout (stops, held)",
              HORIZONTAL + (serial("R", label="R"), wait_osc("/booted", 20.0, label="booted"), reconnect(),
                            query("/getConfigName", "/configName", label="config"),
                            wait_reply("/getHomingStatus", "/homingStatus", 4, 15.0, label="timeout"), wait(0.5),
                            query("/getBrakeStatus", "/brakeStatus", AXIS, label="state_after"),
                            query("/getHiZ", "/HiZ", AXIS, label="hiz_after"),
                            query("/getPosition", "/position", AXIS, label="pos_after"), wait(1.0),
                            osc("/activate", AXIS, 0), wait(0.5)),
              "GoUntilTimeout (homingStatus 4): HardStop, energized, brake RELEASED (2)", restart=True),
]}


# --------------------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------------------


def _resolve(args: tuple, axis: int) -> tuple:
    return tuple(axis if a == AXIS else a for a in args)


def setup_actions(cfg: Config, brake: bool) -> list[tuple]:
    """OSC messages of the setup, applied before the recording and after each restart."""
    m = cfg.raw["motor"]
    a = cfg.test_axis
    out: list[tuple] = []
    bemf = m.get("bemf_param")
    if bemf:
        out.append(("/setBemfParam", a, *[int(v) for v in bemf]))
    kval = m.get("kval")
    if kval:
        out.append(("/setKval", a, *[int(v) for v in kval]))
    profile = m.get("speed_profile")
    if profile:
        out.append(("/setSpeedProfile", a, *[float(v) for v in profile]))
    for report in ("/enableHizReport", "/enableOverCurrentReport", "/enableStallReport", "/enableUvloReport"):
        out.append((report, a, 1))
    out.append(("/reportError", 1))
    out.append(("/setPositionReportInterval", a, int(m.get("position_report_ms", 20))))
    if brake:
        out.append(("/enableElectromagnetBrake", a, 1))
        out.append(("/setBrakeTransitionDuration", a, int(m.get("brake_transition_ms", 100))))
    return out


class Runner:
    def __init__(self, cfg: Config, ad2, diag: SerialDiag, link: OscLink, brake: bool):
        self.cfg, self.ad2, self.diag, self.link, self.brake = cfg, ad2, diag, link, brake
        self.events: list[dict] = []  # host-timed actions (serial, switches, operator, waits for messages)
        self.queries: list[dict] = []
        self.errors: list[str] = []

    def setup(self) -> None:
        for msg in setup_actions(self.cfg, self.brake):
            self.link.send(*msg)
            time.sleep(0.02)

    def _event(self, kind: str, detail: str, label: str = "", t_ns: int | None = None) -> None:
        self.events.append({"t_ns": t_ns or time.monotonic_ns(), "kind": kind, "detail": detail, "label": label})

    def run(self, actions) -> None:
        axis = self.cfg.test_axis
        for act in actions:
            if act.kind == "osc":
                self.link.send(act.address, *_resolve(act.args, axis))
            elif act.kind == "serial":
                self._event("serial", act.address, act.label)
                try:
                    self.diag.send(act.address)
                except Exception as ex:  # the port can be stale after a restart
                    self.errors.append(f"serial {act.address}: {type(ex).__name__}: {ex}")
            elif act.kind == "wait":
                time.sleep(act.seconds)
            elif act.kind == "idle":
                deadline = time.monotonic() + act.seconds
                while time.monotonic() < deadline:
                    time.sleep(0.05)
                    try:
                        e = self.link.request("/getBusy", axis, reply="/busy", timeout=0.3,
                                              predicate=lambda a: a[0] == axis)
                    except OscError:
                        continue
                    if not e.args[1]:
                        break
                else:
                    self.errors.append("idle: still busy after the timeout")
            elif act.kind == "wait_osc":
                t0 = time.monotonic_ns()
                e = self.link.wait_for(act.address, act.seconds, t0)
                self._event("wait_osc", f"{act.address} {'received' if e else 'NOT received'}", act.label)
                if e is None:
                    self.errors.append(f"no {act.address} within {act.seconds} s ({act.label})")
            elif act.kind in ("press", "release"):
                self._event(act.kind, act.address, act.label)
                (self.ad2.press if act.kind == "press" else self.ad2.release)(act.address)
            elif act.kind == "operator":
                self._event("operator", act.address, act.label)
                print(f"OPERATOR: {act.address}", flush=True)
                if sys.stdin.isatty():
                    input("press Enter when done ")
            elif act.kind == "reconnect":
                self._reconnect(act.seconds)
            elif act.kind == "query":
                reply, *args = act.args
                try:
                    e = self.link.request(act.address, *_resolve(tuple(args), axis), reply=reply, timeout=0.5)
                    self.queries.append({"label": act.label, "address": reply, "args": list(e.args)})
                except OscError as ex:
                    self.queries.append({"label": act.label, "address": reply, "error": str(ex)})
            elif act.kind == "ocd_sweep":
                start, stop = act.args
                t0 = time.monotonic_ns()
                hit = None
                for raw in range(start, stop - 1, -1):
                    self.link.send("/setOverCurrentThreshold", axis, raw)
                    self._event("ocd", f"OCD_TH raw {raw}", f"ocd_{raw}")
                    deadline = time.monotonic() + act.seconds
                    while hit is None and time.monotonic() < deadline:
                        hit = (self.link.wait_for("/overCurrent", 0.05, t0, lambda a: a[0] == axis)
                               or self.link.wait_for("/HiZ", 0.0, t0, lambda a: a[0] == axis and a[1] == 1))
                    if hit is not None:
                        self.queries.append({"label": "ocd_trip", "address": hit.address, "raw": raw})
                        break
                else:
                    self.queries.append({"label": "ocd_trip", "address": None, "raw": None})
            elif act.kind == "wait_reply":
                reply, value = act.args
                deadline = time.monotonic() + act.seconds
                ok = False
                while time.monotonic() < deadline:
                    try:
                        e = self.link.request(act.address, axis, reply=reply, timeout=0.3, attempts=1,
                                              predicate=lambda a: a[0] == axis)
                        if int(e.args[1]) == value:
                            ok = True
                            break
                    except OscError:
                        pass
                    time.sleep(0.3)
                self._event("wait_reply", f"{reply} {value} {'seen' if ok else 'NOT seen'}", act.label)
                if not ok:
                    self.errors.append(f"{reply} {value} not seen within {act.seconds} s ({act.label})")
            elif act.kind == "close_serial":
                self._event("serial", "USB serial port closed on the host", act.label)
                self.diag.close()
            elif act.kind == "sine":
                self._sine(*act.args)
            else:
                raise ValueError(act.kind)

    def _reconnect(self, timeout: float) -> None:
        try:
            self.diag.reconnect(timeout)
        except (DiagError, OSError) as ex:
            self.errors.append(f"serial reconnect: {ex}")
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.link.set_dest_ip()
                break
            except (OscError, OSError):
                if time.monotonic() >= deadline:
                    self.errors.append("no /destIp after the restart")
                    return
                time.sleep(0.5)
        # The brake setting after the restart, before the setup enables it again (default: -1).
        try:
            e = self.link.request("/getBrakeStatus", self.cfg.test_axis, reply="/brakeStatus", timeout=0.5)
            self.queries.append({"label": "brake_before_setup", "address": "/brakeStatus", "args": list(e.args)})
        except OscError as ex:
            self.queries.append({"label": "brake_before_setup", "address": "/brakeStatus", "error": str(ex)})
        self.setup()
        self._event("setup", "setup applied again")

    def _sine(self, amplitude: int, period: float, duration: float, rate: float) -> None:
        axis = self.cfg.test_axis
        n = self.cfg.num_motors
        pos = []
        for m in range(1, n + 1):
            pos.append(int(self.link.request("/getPosition", m, reply="/position", timeout=0.3,
                                             predicate=lambda a, m=m: a[0] == m).args[1]))
        center = pos[axis - 1]
        t0 = time.monotonic()
        k = 0
        while True:
            t = k / rate
            if t > duration:
                break
            delay = t0 + t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            pos[axis - 1] = center + int(round(amplitude * math.sin(2 * math.pi * t / period)))
            self.link.send("/setTargetPositionList", *pos)
            k += 1


# --------------------------------------------------------------------------------------
# Timeline
# --------------------------------------------------------------------------------------


def clock_fit(tx_host: np.ndarray, p1_rise: np.ndarray) -> tuple[analyze.ClockFit | None, np.ndarray, str]:
    """Host time -> AD2 time from the sent messages and the P1 rises. Returns the fit,
    the AD2 time of each sent message (nan when not matched) and the method.

    With as many P1 rises as messages, the n-th message is the n-th rise. Otherwise
    (messages sent to a board that was restarting, packets from elsewhere) each message is
    paired with the first P1 rise 0-20 ms after it, with the offset estimated first."""
    tx = np.asarray(tx_host, float)
    p1 = np.asarray(p1_rise, float)
    if tx.size == 0:
        return None, np.zeros(0), "none"
    if tx.size == p1.size:
        fit = analyze.match_commands(tx, p1)
        if fit.residual_max < 0.02:
            return fit, p1.copy(), "count"
    if p1.size == 0:
        return None, np.full(tx.size, np.nan), "none"
    d, n = analyze.estimate_offset(p1, tx, tolerance=0.005, max_offset=0.1)
    t_ad2 = np.full(tx.size, np.nan)
    for i, t in enumerate(tx + d):
        k = np.searchsorted(p1, t - 0.005)
        if k < p1.size and p1[k] - t < 0.02:
            t_ad2[i] = p1[k]
    ok = ~np.isnan(t_ad2)
    if ok.sum() >= 2:
        slope, offset = np.polyfit(tx[ok] - tx[0], t_ad2[ok], 1)
        res = t_ad2[ok] - (slope * (tx[ok] - tx[0]) + offset)
        fit = analyze.ClockFit(float(slope), float(offset), float(tx[0]), float(np.max(np.abs(res))),
                               float(np.std(res)))
    else:
        fit = analyze.ClockFit(1.0, float(d), 0.0, 0.0, 0.0)
    return fit, t_ad2, "nearest"


def p0_gaps(p0: analyze.Edges, gap: float = P0_GAP_S) -> list[tuple[float, float]]:
    """(last edge before, first edge after) of each gap in the P0 toggle longer than gap."""
    t = p0.all
    if t.size < 2:
        return []
    d = np.diff(t)
    return [(float(t[i]), float(t[i + 1])) for i in np.flatnonzero(d > gap)]


def build_timeline(cfg: Config, cap, tx: list, rx: list, events: list[dict]) -> tuple[list[dict], dict]:
    """Events sorted by AD2 time (ms). tx / rx: LogEntry lists in the capture window."""
    host = lambda t_ns: (t_ns - cap.host_trigger_ns) / 1e9  # noqa: E731
    probe = cfg.probe_dio
    p1 = analyze.bit_edges(cap.digital, probe[1]).rise if cap.digital is not None else np.zeros(0)
    fit, t_tx, method = clock_fit(np.array([host(e.t_ns) for e in tx]), p1)
    to_ad2 = (lambda t: fit.to_ad2(t)) if fit else (lambda t: t)
    rows: list[dict] = []

    def add(t_s, source, kind, detail):
        rows.append({"t_ms": None if t_s is None or np.isnan(t_s) else round(float(t_s) * 1e3, 3),
                     "source": source, "kind": kind, "detail": detail})

    for e, t in zip(tx, t_tx):
        if e.address in ("/getBusy",):
            continue
        add(t if not np.isnan(t) else to_ad2(host(e.t_ns)), "ad2" if not np.isnan(t) else "host",
            "osc_tx", f"{e.address} {' '.join(map(str, e.args))}".strip())
    last: dict = {}
    for e in rx:
        if e.address in TIMELINE_SKIP_RX:
            continue
        if e.address in TIMELINE_CHANGES_RX:  # polled replies: only the changes
            key = (e.address, e.args[0] if e.args else None)
            if last.get(key) == e.args:
                continue
            last[key] = e.args
        add(to_ad2(host(e.t_ns)), "host", "osc_rx", f"{e.address} {' '.join(map(str, e.args))}".strip())
    for ev in events:
        add(to_ad2(host(ev["t_ns"])), "host", ev["kind"], (ev["detail"] + (f" [{ev['label']}]" if ev["label"] else "")))
    if cap.digital is not None:
        for m, d in enumerate(cfg.ad2["dio"]["brake"], start=1):
            e = analyze.bit_edges(cap.digital, int(d))
            for t in e.rise:
                add(t, "ad2", "brake", f"motor {m} released (J7 high)")
            for t in e.fall:
                add(t, "ad2", "brake", f"motor {m} engaged (J7 low)")
        for name, d in cfg.switch_dios.items():
            e = analyze.bit_edges(cap.digital, int(d))
            for t in e.fall:
                add(t, "ad2", "switch", f"{name} low (pressed)")
            for t in e.rise:
                add(t, "ad2", "switch", f"{name} high (released)")
        if cfg.v33_dio is not None:
            e = analyze.bit_edges(cap.digital, cfg.v33_dio)
            for t in e.fall:
                add(t, "ad2", "power", "3.3 V low")
            for t in e.rise:
                add(t, "ad2", "power", "3.3 V high")
        for a, b in p0_gaps(analyze.bit_edges(cap.digital, probe[0])):
            add(a, "ad2", "mcu", f"P0 stops (last edge; gap {1e3 * (b - a):.1f} ms)")
            add(b, "ad2", "mcu", "P0 resumes")
    rows.sort(key=lambda r: (r["t_ms"] is None, r["t_ms"] if r["t_ms"] is not None else 0.0))
    info = {"method": method, "messages": len(tx), "p1_rises": int(p1.size),
            "matched": int(np.sum(~np.isnan(t_tx))) if len(t_tx) else 0,
            "residual_max_ms": fit.residual_max * 1e3 if fit else None,
            "note": "osc_rx and host-timed rows: host receive / action time mapped to the AD2 time "
                    "(network and host latency included)"}
    return rows, info


def brake_delays(rows: list[dict], axis: int) -> list[dict]:
    """For each brake output change of the test axis: the last OSC command or serial command
    before it and the delay."""
    out = []
    last = None
    for r in rows:
        if r["kind"] in ("osc_tx", "serial") and r["t_ms"] is not None:
            last = r
        elif r["kind"] == "brake" and r["detail"].startswith(f"motor {axis} ") and r["t_ms"] is not None:
            out.append({"t_ms": r["t_ms"], "change": r["detail"].split(" ")[2],
                        "after": last["detail"] if last else None,
                        "delay_ms": round(r["t_ms"] - last["t_ms"], 3) if last else None})
    return out


# --------------------------------------------------------------------------------------
# Command
# --------------------------------------------------------------------------------------


def _write_rows(path: Path, rows: list[dict]) -> None:
    from .commands import write_csv
    write_csv(path, ["t_ms", "source", "kind", "detail"],
              [[r["t_ms"], r["source"], r["kind"], r["detail"]] for r in rows])


def add_note(result_dir: str, notes: list[str]) -> int:
    """Stores operator observations (key=value) in summary.json of a motor result."""
    path = Path(result_dir) / "summary.json"
    data = json.loads(path.read_text())
    obs = data.setdefault("observations", {})
    for n in notes:
        key, _, value = n.partition("=")
        obs[key.strip()] = value.strip()
    obs["_entered"] = time.strftime("%Y-%m-%d %H:%M:%S")
    path.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(obs, indent=2, ensure_ascii=False))
    return 0


def cmd_motor(cfg: Config, args) -> int:
    from . import commands

    if args.note:
        return add_note(args.note[0], args.note[1:])
    if not args.step:
        print("error: --step or --note is needed")
        return 2
    if not bool(cfg.raw["motor"].get("motor_connected", False)):
        print("error: motor.motor_connected is false; the motor steps are for a connected motor")
        return 2
    step = STEPS[args.step]
    out = commands.new_result_dir(f"motor_{step.id}" + (f"_{args.label}" if args.label else ""))
    print(f"{step.id}: {step.title}\nOBSERVE: {step.observe}\nexpect: {step.expect}", flush=True)
    probe = cfg.probe_dio
    brake_dios = [int(d) for d in cfg.ad2["dio"]["brake"]]
    mask = commands.mask_of(brake_dios + probe + list(cfg.switch_dios.values())
                            + ([cfg.v33_dio] if cfg.v33_dio is not None else []))
    rate = args.rate or (1e4 if step.restart else 1e5)
    with commands.open_ad2(cfg) as ad2, SerialDiag(cfg.serial_port) as diag, commands.open_link(cfg) as link:
        if args.reboot:
            diag.reboot()
        commands.connect_board(link)
        runner = Runner(cfg, ad2, diag, link, step.brake)
        runner.setup()
        time.sleep(0.3)
        before = {"brake": _get(link, "/getBrakeStatus", "/brakeStatus", cfg.test_axis),
                  "hiz": _get(link, "/getHiZ", "/HiZ", cfg.test_axis),
                  "position": _get(link, "/getPosition", "/position", cfg.test_axis)}
        rec = ad2.start_recording(None, digital_rate=rate, sensible_mask=mask)
        time.sleep(0.3)
        try:
            runner.run(step.actions)
            time.sleep(0.5)
        finally:
            cap = rec.stop()
        cap.save(out / "capture.npz")
        after = {"brake": _get(link, "/getBrakeStatus", "/brakeStatus", cfg.test_axis),
                 "hiz": _get(link, "/getHiZ", "/HiZ", cfg.test_axis),
                 "position": _get(link, "/getPosition", "/position", cfg.test_axis)}
    link.save_log(out / "osc_log.jsonl")
    tx = [e for e in link.entries("tx", cap.host_trigger_ns, cap.host_stop_ns)]
    rx = [e for e in link.entries("rx", cap.host_trigger_ns, cap.host_stop_ns)]
    rows, info = build_timeline(cfg, cap, tx, rx, runner.events)
    _write_rows(out / "timeline.csv", rows)
    positions = [[round((e.t_ns - cap.host_trigger_ns) / 1e6, 1), e.args[1]] for e in rx
                 if e.address == "/position" and e.args and e.args[0] == cfg.test_axis]
    commands.write_csv(out / "positions.csv", ["host_ms", "position"], positions)
    delays = brake_delays(rows, cfg.test_axis)
    notifications: dict[str, int] = {}
    for e in rx:
        if e.address not in TIMELINE_SKIP_RX and e.address not in ("/destIp",):
            key = e.address + (f" {e.args[0]}" if e.address == "/error/command" and e.args else "")
            notifications[key] = notifications.get(key, 0) + 1
    summary = {"step": step.id, "title": step.title, "observe": step.observe, "expect": step.expect,
               "axis": cfg.test_axis, "digital_rate": rate, "capture_ok": cap.ok, "problems": cap.problems(),
               "setup": [list(m) for m in setup_actions(cfg, step.brake)], "before": before, "after": after,
               "clock": info, "brake_delays": delays, "notifications": notifications,
               "queries": runner.queries, "errors": runner.errors,
               "position_range": [min(p[1] for p in positions), max(p[1] for p in positions)] if positions else None,
               "observations": {}}
    commands.write_json(out / "summary.json", summary)
    for r in rows:
        if r["kind"] != "osc_tx" or not r["detail"].startswith(("/getPosition", "/getBrakeStatus", "/getKval")):
            t = "-" if r["t_ms"] is None else f"{r['t_ms']:10.1f}"
            print(f"{t:>10}  {r['source']:4}  {r['kind']:8}  {r['detail']}")
    print(f"brake delays: {delays}")
    print(f"queries: {runner.queries}")
    if runner.errors:
        print(f"errors: {runner.errors}")
    print(f"before {before} after {after}")
    print(f"results: {out}")
    print(f"enter the observations with: hil.py --config <toml> motor --note {out} key=value ...")
    return 0 if cap.ok and not runner.errors else 1


def _get(link: OscLink, address: str, reply: str, axis: int):
    try:
        e = link.request(address, axis, reply=reply, timeout=0.3, predicate=lambda a: a[0] == axis)
        return list(e.args[1:])
    except OscError:
        return None
