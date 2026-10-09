# Changelog

## [Unreleased]

### Behavior changes

These changes are visible to OSC clients.

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

- OSC messages are received with a single read into a static buffer and dispatched by a table lookup instead of about 140 sequential address comparisons. Patterns in received addresses (e.g. `/enable*Report`, `/set{Acc,Dec}`) and addresses followed by `/...` keep working as before.
- OSC messages are encoded into a static buffer and sent with a single write, without heap allocation.
- Status and position reads of all drivers (servo mode, status polling, position reports, `/getPositionList`, `/getStatusList`, `/getPosition 255`, `/getStatus 255`) use one packed daisy-chain transfer instead of one transfer per driver. Requires driver libraries v1.2.0.
- STEP400: the limit switch inputs, read with the ADC of the drivers, are polled with one packed daisy-chain transfer instead of one transfer per driver.
- Received packets larger than 512 bytes, and messages with more than 16 arguments, are answered with `/error/osc OscSyntaxError`.
- The CNMAT OSC library is no longer used.
- USB serial: the status JSON (`T`) includes `mcu`, `resetCause` and `resetCauseRaw` in the `board` object.

### Added

- USB serial command `p`: prints timing statistics (loop, OSC receive, servo update, status check, servo interval, OSC messages dispatched by pattern matching or not matched) and resets them.

### Fixed

- `/resetDevice` caused a HardFault, and the device was actually restarted by the watchdog timeout. It now resets with `NVIC_SystemReset()`, and `/booted` reports the reset cause `5` (software).
- USB serial diagnosis: `EN_TQREG` of the CONFIG register was always shown as 0.
- USB serial diagnosis: the microstep of EL_POS and the fields of the CONFIG register (`OSC_SEL`, `EXT_CLK`, `SW_MODE`, `EN_VSCOMP`, `OC_SD`, `UVLOVAL`, `VCCVAL`, `POW_SR`, `TSW`, `PRED_EN`, `F_PWM_DEC`, `F_PWM_INT`) were extracted with `%` instead of a bit mask and were shown wrong.
- The command error texts were constructed on the heap in every source file that included `globals.h`.
