# Changelog

## [Unreleased]

### Behavior changes

These changes are visible to OSC clients.

- Electromagnetic brake (STEP400, STEP800): the brake of each motor follows a new state machine with six states, readable with `/getBrakeStatus`. Changes for a motor with the brake enabled:
  - `/activate 1` while the brake is released does nothing. It sent a hard stop, which stopped a turning motor abruptly.
  - `/activate 1` releases the brake only if a status read after the command shows the driver energized. If the driver stays HiZ (e.g. no motor power), the brake stays engaged and `/error/command BrakeReleaseFailed motorID` is sent. Before, the brake was released after `brakeTransitionDuration` in any case.
  - `/activate 1` during the deceleration of `/softHiZ` (or `/activate 0`) cancels the engaging and keeps the brake released; the deceleration goes on. Before, it stopped the motor with a hard stop, and the brake still engaged when the motor stopped.
  - `/activate 0` on a turning motor decelerates it and engages the brake when it has stopped, like `/softHiZ`. It engaged the brake without stopping the motor.
  - `/softHiZ` and `/activate 0` on a stopped motor engage the brake at the next status read (up to 5 ms later). The stop is now judged on a status read after the command.
  - After `/free`, a stop of the motor no longer engages the brake (when `/free` was sent during the deceleration of `/softHiZ`).
  - After `/free`, a motion command (`/run`, `/move`, `/goTo`, ...) is executed and the motor returns to the released (energized) state. `/activate 1` does the same. `/activate 0`, `/softHiZ` and `/hardHiZ` after `/free` engage the brake at once (the motor is already HiZ).
  - A motion command during the deceleration of `/softHiZ` or `/activate 0` cancels the engaging; the brake stays released. Before, the brake engaged when the motor stopped later.
  - `/softStop`, `/hardStop` and `/enableServoMode 0` while the brake is engaged or after `/free` (motor HiZ) do nothing. They energized the motor, and the brake state was left engaged, so `/activate 0` and `/hardHiZ` could not make the motor HiZ again.
  - `/enableServoMode 1` and `/homing` while the brake is not released reply `/error/command BrakeEngaged motorID` and do nothing. `/enableServoMode 1` energized and moved the motor with the brake engaged; `/homing` set the homing state although it did not start.
  - When the driver goes HiZ by itself while the brake is released (overcurrent shutdown, thermal shutdown, UVLO), the brake engages at the next status read (up to 5 ms) and `/error/command BrakeEngagedOnHiZ motorID` is sent. The brake stayed released. The servo mode is also cancelled (`ServoModeCancelledOnHiZ`, below).
  - `/resetMotorDriver` is accepted only while the brake is engaged or after `/free` (motor HiZ); otherwise it replies `CommandIgnored`. It made the driver HiZ with the brake released.
  - `/enableElectromagnetBrake` changes the setting only while the driver is HiZ, and disables the brake only while it is engaged or after `/free`; otherwise it replies `CommandIgnored`. Sending the current value does nothing. After a change the brake output is engaged and the state is engaged. Before, disabling a released brake engaged it on a turning motor, and enabling it on an energized motor left a state where `/activate 0` and `/hardHiZ` did nothing. The same rule applies to a config loaded over USB serial (a warning is printed for a motor that is not changed).
  - `/setBrakeTransitionDuration` with a value outside 0-65535 replies `CommandIgnored` and keeps the setting. The value wrapped around.
  - `/resetDevice` and the USB serial command `R` (used by the config tool to reboot) first stop the turning motors with a hard stop, engage their brakes and wait for the longest `brakeTransitionDuration` of them (at most 1 s), then restart. `R` replies `{"result":"ok","message":"Rebooting..."}` before this wait. Before, the board restarted at once. The MCU reset puts the drivers in standby (HiZ) at once (their reset input floats and is pulled down inside), while the STEP800 brake outputs kept their released state until `setup()` (about 0.5 s; STEP400: the pins float and the Brake Board pull-downs engage the brakes, without the transition time).
  - When the watchdog detects a hang (no reset for 2 s, as before), every brake output is engaged first, then the firmware waits for the longest `brakeTransitionDuration` of the motors with the brake enabled and not engaged (at most 1 s) before the watchdog resets the board. The reset therefore comes up to 1 s later than before; the reset cause in `/booted` is still `4`. No hard stop is sent at this point. Before, the brakes stayed released after the watchdog reset as above.
  - KVAL_HOLD (TVAL_HOLD in current mode) is raised to at least half of KVAL_RUN (TVAL_RUN) whenever a motor with the brake enabled is energized again: `/activate 1` from engaged or engaging (as before), and now also after `/free` (`/activate 1` or a motion command). After `/free` the motor was energized with the configured holding value.
  - Homing at startup (`homingAtStartup`) with the brake enabled first releases the brake as `/activate 1` does, and starts the homing once the brake is released. It failed because the brake was engaged, and the goUntil timeout later sent a hard stop to the HiZ motor.
  - `/activate 1` no longer toggles the LED (debug code).
- USB serial diagnostic command `b` (brake output test) is available only in test builds (`HIL_PROBE`). It switched the brake outputs regardless of the brake state.
- Stop and HiZ commands cancel pending motion, **also for motors with the brake disabled**: `/softStop`, `/hardStop`, `/softHiZ`, `/hardHiZ`, `/activate 0`, `/free`, `/enableServoMode` (both), `/resetMotorDriver`, and the automatic brake engage cancel
  - a `/goTo`, `/goToDir`, `/goHome` or `/goMark` waiting for the BUSY flag to clear. It was sent when the motor became not busy, so a stopped motor started again.
  - the homing, also the state left by a `/goUntil` or `/releaseSw` sent directly: its timeout sent a hard stop, which energized a motor made HiZ in the meantime. `/homingStatus` is reported as before (only for `/homing`).
  - the servo mode.
  An accepted motion command also cancels a `/goTo` (etc.) waiting for BUSY, so the waiting command no longer overrides it later.
- The automatic hard stops are not sent to a motor whose driver is HiZ, **also with the brake disabled**: home switch with `/setProhibitMotionOnHomeSw`, limit switch (STEP400, `SW_HARD_STOP`), switch event in the servo mode, and the goUntil / releaseSw timeouts (the timeout error is still sent). A hard stop energizes a HiZ driver.
- A motion command (e.g. `/run`) that clears the homing state now also clears the state of a `/goUntil` sent directly.
- Boolean values that were sent with the OSC type tags `T` / `F` are now sent as integers `0` / `1` (type tag `i`), as intended for all STEP series messages. Clients that expect `T` / `F` for these arguments must be updated:
  - `/uvlo motorID state`: the report sent when the UVLO state changes (enabled by `/enableUvloReport`). The reply to `/getUvlo` already used `i`.
  - `/lowSpeedOptimizeThreshold motorID threshold enabled`: the third argument (`enabled`), reply to `/getLowSpeedOptimizeThreshold`.
- `/booted` has a second argument, `resetCause` (`i`): `/booted myId resetCause`. Clients that check the exact number of arguments are affected. The numbers are part of the specification; new causes will only be added as 6, 7, ...

  | Value | Reset cause |
  |---|---|
  | 0 | Unknown |
  | 1 | Power-on |
  | 2 | Brown-out (supply voltage drop) |
  | 3 | Reset pin |
  | 4 | Watchdog |
  | 5 | Software reset (e.g. after firmware upload over USB) |

  If several causes are flagged, the first of brown-out, power-on, watchdog, reset pin, software is reported.
- Servo mode: the update period is 5 ms instead of 10 ms (STEP400, STEP800). This replaces the 10 ms +/- 0.7 ms period given under Fixed. The integral and derivative terms are scaled by the period, so the same `kP`, `kI` and `kD` were designed to give about the same response; with the shorter period the response can still differ, so retune the gains if needed. Measured on STEP800 with all motors in the servo mode, `/setTargetPositionList` at 100 Hz and position reports every 0, 10 or 20 ms (or `/setPositionListReportInterval` 20 ms): 5 ms +/- 0.7 ms (4.31 to 5.70 ms). An OSC message that sends several replies (e.g. `/getPosition 255`, about 2.7 ms) delays the next update by up to its length (measured up to 7.75 ms between two updates).
- The status of the drivers is read every 5 ms instead of every 10 ms, and the brake, the limit switches (STEP400), the homing timeouts (`goUntilTimeout`, `releaseSwTimeout`) and the servo update run with it. Status reports (`/busy`, `/HiZ`, `/homeSw`, `/dir`, `/motorStatus`, `/swEvent`, `/uvlo`, `/thermalStatus`, `/limitSw`) arrive up to 5 ms earlier; they are still sent only on a change. (`/overCurrent` and `/stall`: see below.) The brake times are in 5 ms steps: the release after `/activate 1` comes 0-5 ms after `brakeTransitionDuration` (measured 99.8-105.4 ms for 100 ms; before 99.8-107.6 ms), and the engage after a stop or a HiZ is judged at the next status read. Position reports, the DIP switch read, the Tx/Rx LEDs, the boot message and the watchdog still run every 10 ms, and the position report intervals are as accurate as before (measured on STEP800: 19.3 to 20.7 ms for 20 ms, 9.3 to 10.7 ms for 10 ms).
- `/overCurrent` and `/stall` are sent at once when the condition starts and then, while it lasts, at most every 200 ms per motor and kind. Before, they were sent at every status read while the condition lasted (every 10 ms; every 5 ms with the 5 ms status read). Detections within the 200 ms are not lost: they give one message at the end of the interval. Detections while the report is disabled (`/enableOverCurrentReport`, `/enableStallReport`) are not sent later. The messages are unchanged.
- When a driver goes HiZ by itself (overcurrent shutdown, thermal shutdown, UVLO) in the servo mode, the servo mode is cancelled and `/error/command ServoModeCancelledOnHiZ motorID` is sent (follows `/reportError`), with or without the electromagnetic brake. Nothing is sent to the driver. With the brake enabled, `BrakeEngagedOnHiZ` is also sent. The commands that make the driver HiZ (`/softHiZ`, `/hardHiZ`, `/activate 0`, `/free`, `/resetMotorDriver`, also for all motors and `/combine/...`) cancel the servo mode first and do not send this error. Enable the servo mode again with `/enableServoMode 1` (the current position becomes the target).
- Enabling the servo mode (`/enableServoMode`) resets the integral and the previous error of the PID controller. The previous values were kept from the last time the servo mode was used.
- The servo mode position error is wrapped correctly around the ends of the ABS_POS range (the wrap was off by one step).
- `/goUntilRaw 255 ...` (all motors) interprets the speed as a raw register value, like `/goUntilRaw` for a single motor. It was treated as steps/s.
- `/getDir 255` (all motors) returns the direction. It returned the HiZ state.
- `/getMinSpeed` returns `0.0` for a motor with low speed optimization enabled. While it is enabled, the MIN_SPEED register of the driver holds the optimization threshold (get it with `/getLowSpeedOptimizeThreshold`) and the actual minimum speed is 0. For a motor with low speed optimization disabled, it returns the MIN_SPEED register value of the driver in steps/s. A single motor and all motors (`255`) now behave the same:
  - Single motor: returned the MIN_SPEED register value (the threshold) while low speed optimization was enabled.
  - All motors (`255`): returned the stored minimum speed setting.
- `/setMinSpeed 255` (all motors) checks the low speed optimization setting of each motor. It read an invalid array element.
- `/hardHiZ` for a single motor no longer sends a soft stop before the hard HiZ; single motor, all motors (`255`) and `/combine/hardHiZ` behave the same. For a motor with the electromagnetic brake enabled and released (`/activate` done), the motor is stopped with a hard stop (still energized) before the brake is engaged, so it does not keep running during the brake transition time. While the brake is engaged or engaging, the command does nothing as before.
- `/goUntil` and `/goUntilRaw` report an activated home switch as `/error/command HomeSwActivating motorID`, the text in the error list. It was sent as `HomeSwActivated`. It now follows `/reportError` like the other command errors, so it is not sent while error reports are disabled.
- Received address patterns: in some corner cases where the CNMAT implementation read past the end of the pattern (e.g. characters following a `{...}` list), the match result can differ from before.
- The W5500 retransmission time is limited to 20 ms x 2 retries (60 ms in total). Sending to a destination that does not answer ARP no longer blocks for about 1.8 s.
- Inherited from driver libraries v1.2.0:
  - The acceleration and deceleration reported by `/getSpeedProfile` are about 4.9% smaller; they now match the datasheet.
  - `/setAcc`, `/setDec` and `/setSpeedProfile` round to the nearest register value, so the written value can differ by 1 LSB.
  - STEP400: the default slew rate setting is now the designed value, so the switching is faster.
  - STEP400: a maximum speed of 0 is written as 1 (the register value 0 is reserved) by `/setMaxSpeed`, `/setSpeedProfile` and the `maxSpeed` setting of the config file. `/setMaxSpeedRaw 0` still writes 0. STEP800 is unchanged.
  - STEP400: `/setFullstepSpeed` and `/setFullstepSpeedRaw` keep the BOOST_MODE bit of the FS_SPD register. They cleared it before. The firmware does not set BOOST_MODE, so this is not visible unless the bit is set by other means.
  - `/goUntilRaw` clamps the speed to 20 bits (`0xFFFFF`), like `/runRaw`. Larger values were sent as is, and the driver ignored the upper bits.

### Changed

- STEP800: the brake outputs change as soon as the brake state changes (0.2-0.3 ms after the command on the HIL bench). They were sent with the next DIP switch read and appeared at the read after it, 10-20 ms after the command, so the brake and motor overlapped for less than `brakeTransitionDuration` (about 89 ms instead of 100 ms).
- The brake outputs are engaged first in `setup()`, before the drivers are reset, for a restart that did not engage them (reset button, brown-out). STEP400: the pins are driven low. STEP800: the cleared shift register is latched into the 74HC595, whose outputs otherwise keep their state through an MCU reset. At power-up the STEP800 outputs are still released until then (about 250 ms, mostly the bootloader).
- The watchdog is driven by the firmware (`src/watchdog.cpp`) instead of the Adafruit SleepyDog library, whose interrupt handler cannot be replaced: same clock, early warning interrupt 2 s after the last reset, watchdog period 4 s.
- In every 5 ms slot of the main loop, the status of the drivers is read first; the brake, the limit switches (STEP400), the homing timeouts and the servo update use this status. The slots are scheduled on `micros()`.
- OSC messages are received with a single read into a static buffer and dispatched by a table lookup instead of about 140 sequential address comparisons. Patterns in received addresses (e.g. `/enable*Report`, `/set{Acc,Dec}`) and addresses followed by `/...` keep working as before.
- OSC messages are encoded into a static buffer and sent with a single write, without heap allocation.
- Status and position reads of all drivers (servo mode, status polling, position reports, `/getPositionList`, `/getStatusList`, `/getPosition 255`, `/getStatus 255`) use one packed daisy-chain transfer instead of one transfer per driver. Requires driver libraries v1.2.0.
- STEP400: the limit switch inputs, read with the ADC of the drivers, are polled with one packed daisy-chain transfer instead of one transfer per driver.
- Received packets larger than 512 bytes, and messages with more than 16 arguments, are answered with `/error/osc OscSyntaxError`.
- The CNMAT OSC library is no longer used.
- USB serial: the status JSON (`T`) includes `mcu`, `resetCause` and `resetCauseRaw` in the `board` object.

### Added

- `/getBrakeStatus motorID` replies `/brakeStatus motorID state` (`i`, `i`); `255` for all motors. `state` is `-1` for a motor with the brake disabled, otherwise:

  | Value | State | Brake output | Driver |
  |---|---|---|---|
  | 0 | Engaged | engaged | HiZ |
  | 1 | Releasing (`/activate 1`, waiting `brakeTransitionDuration`) | engaged | energized |
  | 2 | Released | released | energized |
  | 3 | Stopping (`/softHiZ`, `/activate 0`: decelerating, engages when stopped) | released | energized |
  | 4 | Engaging (waiting `brakeTransitionDuration`, then HiZ) | engaged | energized |
  | 5 | Free (`/free`) | released | HiZ |

  The numbers are part of the specification; new states will only be added as 6, 7, ...
- Command errors `BrakeReleaseFailed`, `BrakeEngagedOnHiZ` and `ServoModeCancelledOnHiZ` (`/error/command`, follow `/reportError`).
- USB serial: the brake states are shown with their numbers and names (`s`). HIL_PROBE builds (for the HIL bench): `Z` sends HardHiZ to all drivers, bypassing the brake; `D` prints the driver registers read at boot before the reset; `Y` floats the driver reset line for 300 ms; `W` hangs for the watchdog.
- USB serial command `p`: prints timing statistics (loop, OSC receive, servo update, status check, servo interval, OSC messages dispatched by pattern matching or not matched, slots started a whole period or more late, position report clock realigned to `millis()`) and resets them.
- STEP800: timing probes on spare pins, enabled with the `HIL_PROBE` build flag (no effect on normal builds).
- `tools/hil/`: a hardware-in-the-loop test bench using Digilent Analog Discovery 2.

### Fixed

- Servo mode: the update period alternated between about 7 and 13 ms (mean 10 ms) when position reports were enabled, because the servo update ran after the status poll, which is about 2.5 ms longer when it sends position reports (every other poll with `/setPositionReportInterval` 20 ms). The integral and derivative terms of the PID assume 10 ms, so the effective gains changed by about 25 % up and down from one update to the next. The servo update now runs at a fixed point of the status poll (after the status read, the brake, the limit switches and the homing timeout, before the position reports). Measured on STEP800 with all motors in the servo mode, `/setTargetPositionList` at 100 Hz and position reports every 0, 10 or 20 ms: the period is 10 ms +/- 0.7 ms (9.31 to 10.68 ms; before: 6.84 to 13.16 ms with reports every 20 ms). With the same gains, the response may change slightly because the period is now stable. OSC messages and arguments are unchanged. (The period is now 5 ms; see Behavior changes.)
- Servo mode with the electromagnetic brake disabled: when the driver went HiZ by itself (overcurrent shutdown, thermal shutdown, UVLO), the next servo update sent Run, which energized the driver again: an overcurrent shutdown could repeat (the status read clears the flag), and the motor could start moving to the target when the temperature or the motor supply came back. The servo mode is now cancelled (see `ServoModeCancelledOnHiZ` under Behavior changes).
- `/resetDevice` caused a HardFault, and the device was actually restarted by the watchdog timeout. It now resets with `NVIC_SystemReset()`, and `/booted` reports the reset cause `5` (software).
- USB serial diagnosis: `EN_TQREG` of the CONFIG register was always shown as 0.
- USB serial diagnosis: the microstep of EL_POS and the fields of the CONFIG register (`OSC_SEL`, `EXT_CLK`, `SW_MODE`, `EN_VSCOMP`, `OC_SD`, `UVLOVAL`, `VCCVAL`, `POW_SR`, `TSW`, `PRED_EN`, `F_PWM_DEC`, `F_PWM_INT`) were extracted with `%` instead of a bit mask and were shown wrong.
- The command error texts were constructed on the heap in every source file that included `globals.h`.
