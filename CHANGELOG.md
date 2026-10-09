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
- `/getMinSpeed 255` (all motors) returns the value read from the driver, like `/getMinSpeed` for a single motor. It returned the stored minimum speed setting, which differs from the driver value while low speed optimization is enabled.
- `/setMinSpeed 255` (all motors) checks the low speed optimization setting of each motor. It read an invalid array element.
- The W5500 retransmission time is limited to 20 ms x 2 retries (60 ms in total). Sending to a destination that does not answer ARP no longer blocks for about 1.8 s.
- Inherited from driver libraries v1.2.0:
  - The acceleration and deceleration reported by `/getSpeedProfile` are about 4.9% smaller; they now match the datasheet.
  - `/setAcc`, `/setDec` and `/setSpeedProfile` round to the nearest register value, so the written value can differ by 1 LSB.
  - STEP400: the default slew rate setting is now the designed value, so the switching is faster.

### Changed

- OSC messages are received with a single read into a static buffer and dispatched by a table lookup instead of about 140 sequential address comparisons. Patterns in received addresses (e.g. `/enable*Report`, `/set{Acc,Dec}`) and addresses followed by `/...` keep working as before.
- OSC messages are encoded into a static buffer and sent with a single write, without heap allocation.
- Status and position reads of all drivers (servo mode, status polling, position reports, `/getPositionList`, `/getStatusList`, `/getPosition 255`, `/getStatus 255`) use one packed daisy-chain transfer instead of one transfer per driver. Requires driver libraries v1.2.0.
- Received packets larger than 512 bytes, and messages with more than 16 arguments, are answered with `/error/osc OscSyntaxError`.
- The CNMAT OSC library is no longer used.
- USB serial: the status JSON (`T`) includes `mcu`, `resetCause` and `resetCauseRaw` in the `board` object.

### Added

- USB serial command `p`: prints timing statistics (loop, OSC receive, servo update, status check, servo interval, OSC messages dispatched by pattern matching or not matched) and resets them.

### Fixed

- USB serial diagnosis: `EN_TQREG` of the CONFIG register was always shown as 0.
- The command error texts were constructed on the heap in every source file that included `globals.h`.
