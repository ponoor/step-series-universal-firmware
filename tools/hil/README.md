# HIL test bench (STEP800 + Analog Discovery 2)

Hardware-in-the-loop tests of the STEP800 firmware with a Digilent Analog Discovery 2
(AD2) connected to a Mac:

- **Timing**: firmware built with `-D HIL_PROBE` drives timing probes on spare pins
  (`src/hilProbe.h`). The AD2 logic analyzer records them and the bench computes the
  loop period and the duration of each handler.
- **Brake scenarios**: the brake outputs (J2) and the excitation of the test motor
  (OUT1A on the scope) are recorded while OSC commands are sent; the brake state
  transitions are judged from the physical signals.

## Setup

1. Install [WaveForms](https://digilent.com/reference/software/waveforms/waveforms-3/start)
   (tested with 3.25.1). Recent versions keep the runtime inside the app bundle
   (`/Applications/WaveForms.app/Contents/Frameworks/dwf.framework`). Loaded from there
   by Python, the runtime loads but finds no device: it looks for the FTDI library and its
   data directory relative to the executable or under `/Library/Frameworks/dwf.framework`.
   Link the framework there once:

   ```bash
   sudo ln -s /Applications/WaveForms.app/Contents/Frameworks/dwf.framework /Library/Frameworks/dwf.framework
   ```

   Close the WaveForms app while the bench runs (the AD2 can be opened by one program only).
2. Python 3.12 or later (the pinned numpy needs it; tested with 3.14.4):

   The repository is in a Dropbox folder, and Dropbox breaks a venv inside it (files
   rewritten during sync, and native modules marked with `com.apple.quarantine`, which
   macOS then refuses to load). Keep the venv outside Dropbox and link it as `.venv`:

   ```bash
   cd tools/hil
   python3 -m venv ~/.venvs/step-hil
   ln -s ~/.venvs/step-hil .venv
   .venv/bin/python -m pip install -r requirements.txt
   ```
3. Configuration: `cp bench.example.toml bench.toml` and edit `board.id` (DIP switch),
   `motor.test_axis` and, after Task 3, the measured values (`ad2.dio.brake`,
   `excitation`, `sync`). Keys left out of `bench.toml` take the example's values.
4. Network: with the firmware defaults the board is at `10.0.0.100 + id`, receives on
   port 50000 and sends to port `50100 + id`. The Mac needs an address in `10.0.0.x/24`.
   The bench sends `/setDestIp` at start and checks that the reply names the Mac.
   Allow incoming UDP for Python in the macOS firewall if no reply arrives.

   Standard network (since the `arp` investigation, 2026-10-14): the board and the Mac's
   USB-LAN adapter (NIC A, `en9`, 10.0.0.10) at an unmanaged switch without uplink, as in
   the field (topology T1 of `hil.py arp`). No router on 10.0.0.x; the Mac reaches the
   internet over Wi-Fi through another router. A second adapter (NIC B, `en7`) is kept for
   `hil.py arp`, its cable unplugged at the switch. With a direct cable instead, the Mac's
   link also goes down at every restart of the board.
5. Firmware: build and upload `step800_devlib_probe` (defined in `platformio_local.ini`,
   which is not committed):

   ```ini
   [env:step800_devlib_probe]
   extends = env:step800_devlib
   build_flags =
   	${samd_base.build_flags}
   	-D STEP800_R1
   	-D HIL_PROBE
   ```

## Wiring

STEP800_R1 (schematic: `ponoor/STEP800`, `hardware/step800_r1_0_0_schematics.pdf`):

| Signal | Board | Arduino | SAMD21 | AD2 | Direction |
|---|---|---|---|---|---|
| Brake outputs | J2-1 to J2-8 (74HC595 QA-QH, 3.3 V logic) | - | - | DIO 0-7 | input |
| P0 loop toggle | test point | D1 | PA10 | DIO 8 | input |
| P1 OSC packet processing | test point | A1 | PB08 | DIO 9 | input |
| P2 servo update | test point | D8 | PA06 | DIO 10 | input |
| P3 checkStatus | test point | D9 | PA07 | DIO 11 | input |
| P4 getMyId (shift registers) | J3-5 | D38 | PA13 | DIO 12 | input |
| P5 UDP send | J3-4 | SCL | PA23 | DIO 13 | input |
| P6 brakeStatus write | J3-3 | SDA | PA22 | DIO 14 | input |
| Home switch SW of the test axis | J5-J12 pin 2 | - | - | DIO 15 | open-drain output |
| OUT1A of the test axis / GND | motor connector | - | - | Scope 1+ / 1- | +/-25 V range |
| P0 (D1, in parallel with DIO 8) / GND | test point | D1 | PA10 | Scope 2+ / 2- | time reference of the Scope |
| 3.3 V (J3-2) / GND | J3 | - | - | Scope 2+ / 2- | power-up test only (move Scope 2+ here) |
| GND | J3-1 / J2-10 | - | - | GND | |

- **No motor and no brake are connected** (`motor.motor_connected = false`): only OUT1A
  of the test axis goes to Scope 1. The excitation detection was set up for this; a
  turning motor adds back-EMF to the floating OUT1A and could read as excited. If you
  connect a motor, review `[excitation]` (and the HiZ pre-check range) first.
- Brake output polarity: HIGH = released, LOW = engaged.
- Home switch connector: 1 = GND, 2 = SW, 3 = +5 V. **Leave pin 3 unconnected.**
- J2 pin n is the brake output of motor n (measured with `selftest`).

### Output safety

The bench drives **only DIO15**, and only as an open-drain output: `press()` sets the
output value 0 and then enables the output; `release()` disables it (high impedance,
the L6470 pulls the input up). DIO15 is never driven high. The home switch is released
when the bench opens the AD2 and before it closes it, also after an exception. The
waveform generators (W1/W2), the power supplies (V+/V-) and the pattern generator are
not used. `bench.toml` cannot assign the home switch to another pin.

## Usage

```bash
cd tools/hil
.venv/bin/python hil.py selftest            # wiring, probes, brake map, home switch, excitation
.venv/bin/python hil.py selftest --sync     # also the Scope / Digital In start offset
.venv/bin/python hil.py timing --label head --duration 60
.venv/bin/python hil.py timing --label head-r10 --position-report-ms 10
.venv/bin/python hil.py timing --label head-l20 --position-list-report-ms 20
.venv/bin/python hil.py timing --label head-burst --position-report-ms 20 --burst-hz 10
.venv/bin/python hil.py compare results/<baseline dir> results/<head dir>
.venv/bin/python hil.py brake               # all scenarios, or --scenario B3
.venv/bin/python hil.py powerup --repeat 5  # interactive: power cycles the board
.venv/bin/python hil.py reboot --how R      # brake and driver state across a restart
.venv/bin/python hil.py check --all-auto    # remaining checks C1-C7, C12 (see "Remaining checks")
.venv/bin/python hil.py check --test C9 --cases b   # one check (C6e, C8-C11 need a human)
.venv/bin/python hil.py check --test C6e --label prefix   # --label: suffix of the result directory
.venv/bin/python hil.py selftest --id 85    # board ID (DIP switch) other than board.id
.venv/bin/python hil.py arp --topology T1 --step e1   # client replaced, same IP (see "Client replacement")
```

`--id N` (before or after the subcommand) replaces `board.id` of `bench.toml`: the board is
then reached at `10.0.0.100 + N` and replies to port `50100 + N`; beyond `10.0.0.254`
(N 155 and up) the firmware does not add the ID (since `c1787dc`) and the bench uses
`10.0.0.100` (not with an explicit
`board.ip` / `board.out_port`).

Each run writes `results/<YYYYmmdd-HHMMSS>_<git short sha>_<label>/` with `summary.json`,
CSV tables, PNG plots, the raw captures (`*.npz`) and the OSC log (`osc_log.jsonl`).
The console shows a summary table and the result directory only.

| Command | What it does |
|---|---|
| `selftest` | Reboots the board. `probes`: records DIO 8-14 while the `P` serial command pulses each probe ch + 1 times, and checks the probe-to-DIO map and the firmware's pin map. `brake_map`: sends `b` 8 times and records which DIO 0-7 rises for each motor, and its delay against P4. `home_sw`: presses / releases DIO15 and checks `/homeSw` reports and `/getHomeSw`. `excitation`: records OUT1A energized at EL_POS 0 (`/hardStop`), energized 64 microsteps further (`/move`) and HiZ (`/hardHiZ`), KVAL from `[brake] kval`, and checks the classification. `sync` (with `--sync`): 5 recordings of P0 on Scope 2 and DIO 8, checks the per-recording alignment. |
| `timing` | Reboots the board, applies the load (servo mode on all motors, `/setTargetPositionList` at `[timing] target_rate_hz` or `--target-rate-hz`, position reports, optional bursts; see below), records the probes at 1 MHz for the duration, reads `p` before and after. Reports min / p50 / p99 / p99.9 / max / mean of the loop period, each handler, the slot, servo, poll (getMyId) and position report intervals and `servoDelay`, the CPU usage, the `slotLate` counter of `p`, and compares them with `perfStats`. Then records `servo_settle` (no pass / fail): stops the targets, sets the test axis target back to its start position, waits 1 s and reads the position. |
| `compare` | Table of min / p50 / p99 / p99.9 / max / mean of two `timing` results (every metric, and the CPU usage) and overlaid histograms. The statistics are computed again from both captures, so results recorded before a metric was added (`slotInterval`) or changed (the `servoDelay` window) are compared with the current definitions. |
| `brake` | For each scenario of `hil/scenarios.py`: reboot, common setup, record (digital 1 MHz, OUT1A and P0 at 200 kS/s) while running the steps and polling `/getPosition` every 20 ms, then judge INV-1 to INV-3 and the scenario's checks. Saves a timing diagram per scenario. |
| `reboot` | `--how R` (USB serial reboot), `resetDevice` (OSC) or `standby` (`Y`: the driver reset line floats for 300 ms while the MCU runs). With the test axis released and stopped: records the brake outputs, probes and OUT1A (10 kS/s across a restart), reports the OUT1A level segments from the driver standby entry and the driver registers read at boot (`D`). |
| `check` | The remaining checks C1-C11, see below. |
| `arp` | Client replaced by another MAC address with the same IP address, see below. |
| `powerup` | Needs Scope 2+ moved to 3.3 V and the STEP800 USB cable unplugged: USB VBUS is connected to the board's 5 V net without a diode (schematic sheet 1), so USB alone powers the 3.3 V rail and the 74HC595. Reports, per run, the initial level and every edge of each brake output until the first P4 rise. The 74HC595 output register is not cleared at power-up (SHIFTOUT_ENABLE / R3 clear only the shift register, G is tied to GND), so the outputs keep their power-up state until the first RCK edge. Afterwards, move Scope 2+ back to P0. Arms a recording triggered by the 3.3 V rise on Scope 2, asks for a power cycle, and reports any brake output that is HIGH (released) before the first P4 rise. |

### Timing load

| Option | Default | Load |
|---|---|---|
| `--position-report-ms <ms>` | `[timing] position_report_ms` | `/setPositionReportInterval 255 <ms>`: one `/position` per motor (8 UDP sends) every `<ms>`. `0`: no reports |
| `--position-list-report-ms <ms>` | off | `/setPositionListReportInterval <ms>`: one `/positionList` of all motors every `<ms>`, per-motor reports off. Not with `--position-report-ms` |
| `--burst-hz <Hz>` | 0 (none) | `/getPosition 255` at this rate besides the targets (8 `/position` replies per message, not used): the worst case of a handler that sends several messages just before a slot |
| `--target-rate-hz <Hz>` | `[timing] target_rate_hz` | rate of `/setTargetPositionList` |

The values used are stored in `summary.json` (`load`).

### Timing metrics

| Metric | Probe | Meaning |
|---|---|---|
| `loop` | P0 | time between two P0 edges (one `loop()` pass) |
| `oscReceive`, `updateServo`, `checkStatus`, `getMyId`, `udpSend` | P1-P5 | pulse width of the handler |
| `servoInterval` | P2 | time between two P2 rises (servo update period) |
| `slotInterval` | P3 | time between two P3 rises: the period of the status read (checkStatus), which starts every slot (5 ms; 10 ms with the status poll of earlier firmware) |
| `pollInterval` | P4 | time between two P4 rises (getMyId: every 10 ms, every other slot) |
| `reportInterval` | P2, P4, P5 | time between the starts of two position report bursts. In a 10 ms (housekeeping) slot the reports are sent after the servo update and before getMyId(), so the P5 rises (UDP sends) between a P2 fall and the next P4 rise of the same slot (less than 5 ms apart) are one burst: 8 sends of `/position`, or 1 of `/positionList`. Replies to OSC messages are sent while P1 is high and never fall in that window, so `--burst-hz` runs are included |
| `servoDelay` | P3, P2 | time from the last P3 rise (checkStatus, start of the slot) at or before a P2 rise to that P2 rise: how late the servo update starts in the slot. P2 rises with no P3 rise within 4 ms before them are left out (below the 5 ms slot, so that a P2 rise is never paired with the P3 rise of the slot before; it was 15 ms with the 10 ms poll) |

**CPU usage** (`summary.json` `cpu`, also in `compare`): every pass of `loop()` toggles
P0, so the P0 edge intervals are the loop passes. The shortest pass (taken as the p1 of
the intervals, `base_loop_us`, about 52 us: OSC and serial polls with nothing to do) is
the idle loop. `busy_fraction` = sum over all passes of (pass - base) for the passes
longer than base, divided by the sum of all passes (the recorded time). With all passes
at the base it is 0. It equals (mean - base) / mean of the loop periods, the stage 1
estimate (about 28 % with the 10 ms status poll).

`summary.json` also has `report_interval`: the report intervals that are off the configured
interval (`load`) by 2 ms or more (`off`, with up to 20 of the values; also a row of
`compare`), and `report_clock_realign`: the `reportClockRealign` counter of `p` (the clock
of the position reports realigned to `millis()`; `null` with firmware without it).
`summary.json` also has `load` (above), `slot_late` (the `slotLate` counter of `p` after
the recording: slots that started a whole period or more late; `null` with firmware
without it) and `servo_settle` (`target`, `position`, `error` = position - target).

## Remaining checks (`hil.py check`)

Checks of the phase 1 / brake fix / 5 ms slot changes that the other commands do not
cover. Every check reboots the board first (`R`) and records the USB serial output for
its whole duration (`serial_log.txt`, each line with the time since the start); every
line containing `Warning:` is listed in `summary.json` (`warnings`). Each item is PASS /
FAIL, or RECORD (recorded only). `--all-auto` runs C1-C7 and C12; `--test` runs the given
checks.

Checks that need a human print a line starting with `OPERATOR:` once the recording runs.
On a terminal they wait for Enter; without a terminal they wait for the effect of the
operation (a `/booted` message, the new ID in `T`, ...), up to 10 minutes.

| ID | What is done | Judged |
|---|---|---|
| C1 | `p` (counters reset), `/enable*Report 255 0`, `G`, `/enable*Report 255 1`, `G`, `T`, `p`; then other patterns one by one with `p` before and after: `/getPositio? 1`, `/get[PD]osition 1`, `/{getPosition,getDir} 1`, `/getPosition/extra 1`, `/noSuchCommand 1` | C1-2: the ten report flags of `G` false for all motors. C1-3: all true, nothing else in `G` changed, the servo mode (`T`) unchanged. C1-4: `oscSlowPath` 2, `oscNotMatched` 0. C1-5 (RECORD): replies and counters against `PATTERN_CASES` (CNMAT semantics, expected `/position`; `/position`; `/position`, `/dir`; `/position`; `/error/osc`) |
| C2 | `/setMinSpeed` on the test axis (10.0) and the next motor (5.0), then 255 (20.0); `/enableLowSpeedOptimize axis 1`, then 0; `/getLowSpeedOptimizeThreshold`, `/getUvlo 255` | C2-1, C2-2: `/getMinSpeed m` equals the value for m in `/getMinSpeed 255`, within one MIN_SPEED LSB (0.238 steps/s) of the setting. C2-3: `0.0` (single and 255) while enabled, other motors unchanged; C2-3r (RECORD): the value after disabling. C2-4: type tag `i` for the third argument of `/lowSpeedOptimizeThreshold` (1 / 0) and the second of `/uvlo` (python-osc decodes `T` / `F` as bool) |
| C3 | DIO15 pressed; `/goUntil axis 0 +-100.0`, `/goUntilRaw axis 0 +-6711`, with `/reportError 1` and 0; released: `/goUntil axis 0 -100.0`, pressed after 0.1 s; pressed: `/goUntil 255 0 100.0`. OUT1A recorded at 200 kS/s, KVAL 16 | C3-1: `/getHomeSw` 1. C3-2: `/error/command HomeSwActivating axis` within 300 ms, position unchanged, OUT1A not excited 50-450 ms after the command. C3-3: as C3-2 without any error. C3-4: the position changes while released (excited), stays constant after the press, `/homingStatus` 3. C3-5: one error for the test axis only, all other motors moved, the test axis did not |
| C4 | Homing: the settings (`/getHomingDirection` etc.), `/homing`, press when the position changes, release at `/homingStatus` 2; `/setGoUntilTimeout axis 1000` and `/homing` without a press. Positions: `/setPosition` 2^21-200 then 8 x `/move 50` (and from -2^21+200, `-50`), `/getPosition` and `/getPositionList` at every stop. Servo: position 2^21-52, `/enableServoMode`, targets -2^21+52 and back | C4-1: no `Warning:` line in the serial logs of all `check` runs with the same git sha (other than those caused on purpose: `sends paused`, C6 / C6e; `IP address out of range`, C10) (judged at the end of a run that includes C4). C4-2: `/homingStatus` 1, 2, 3, position 0 at the end (ReleaseSW with ACT 0), no error. C4-2t: `GoUntilTimeout` 1000-1300 ms after `/homing` (host time) and `/homingStatus` 4. C4-3: both readings equal the 22-bit wrapped expectation at all 18 stops. C4-4: every position step in the direction of `servo_error()` (the firmware's wrap, the short way) and a final error of at most 2; the servo mode is switched off if the motor moves 500 microsteps away |
| C5 | `s` and `c` saved; `/setHomeSwMode axis` toggled, a `/move`, `s` again | Each CONFIG field of `s` against the raw value with the datasheet positions (masks of `Ponoor_L6470Constants.h`, OSC_SEL / EXT_CLK split as in the datasheet), each STATUS text against the raw STATUS, EL_POS, ALARM_EN and the DIP switch BIN / DEC; SW_MODE follows the change |
| C6 | SD card removed (stops if `/configName` reports `sdInitializeSucceeded` 1) and `--dead-ip` unused (ping, ARP). Loads (`--cases`, default `abcd`) a: `/getPosition axis` at 2 Hz, b: `/setPositionListReportInterval 20`, c: `/setPositionReportInterval 255 20`, d: one `/getPosition 255`, w: one `/get* 255` (276 replies; to the dead destination only with the send pause, see below), each recorded at 1 MHz for `--duration` (10 s), first to this Mac, then after `{"network":{"destIp":[dead]}}` over `{`; `/setDestIp` back at the end, then `/setPositionListReportInterval 20` for 1 s | C6-cfg: only `network.destIp` changed in `G`. C6-p5: P5 width at most 45 ms (measured 40.8 ms: RTR 20 ms x RCR 2). C6-resume: 40 or more `/positionList` in the second after `/setDestIp`. C6-reset: no P2 pulse over 1 ms (early warning), no `/booted`, `resetCause` unchanged. Firmware with the send pause (`p` prints `udpSendFailed`): C6-pause: the blocked sends (P5 of 5 ms or more) at least 1 s apart, `udpSendFailed` equal to them (up to 2 more, `p` also covers the time around the recording), longest P0 interval at most 45 ms (not w); C6-warn: one `sends paused` line for the dead address, no `sends resumed`; C6-dropped (RECORD): sent + `udpSendDropped` against the sends to this Mac; C6-wild: longest P0 interval of w not longer than to this Mac (the handler takes about 35 ms besides its sends), no early warning, no `/booted`. Firmware without it: C6-loop-ab (at most 70 ms) and C6-loop-cd (RECORD) |
| C6e | Human: a destination that leaves the network after it was resolved. `--alias-ip` (10.0.0.251) must be unused and not on the interface to the board (`route get`). Per load (`--cases`, default `bc`: b `/setPositionListReportInterval 20`, c `/setPositionReportInterval 255 20`): the operator adds the alias (`sudo ifconfig <if> alias <ip> 255.255.255.255`), the bench sends `/setDestIp` and enables the load from that address (the destination is the alias), records P0-P6 at 1 MHz and asks for the removal (`sudo ifconfig <if> -alias <ip>`), then records 30 s segments until a send blocks (P5 of 5 ms or more) and `--duration` (30 s) has passed, or `--max-duration` (300 s). The operator adds the alias again, the bench sends `/getVersion` from it (the client is back). At the end `/setDestIp` from this Mac, the alias is removed, restart | C6e-x-dest: the `/destIp` reply arrives on the alias, reports arrive before the removal. RECORD: per segment P5 widths after the removal, blocked sends, longest P0 interval, `slotLate`, `udpSendFailed` / `udpSendDropped`; the time of the first blocked send after the removal (host time mapped to the AD2, about 0.2 s: the removal is polled); after the return the reply and first report delay, P5 widths and `sends resumed` lines. Position reports are enabled from the alias before it goes; a command from another address would not change the destination either |
| C7 | `R` with the test axis brake enabled and released, `brakeTransitionDuration` a 100, b 1000, c 5000 ms; d brake disabled (motor energized by `/hardStop`); e all motors, 100-1000 ms. Brake outputs, probes and OUT1A at 10 kS/s (`--cases`, `--repeat`) | Reply `Rebooting...` within 100 ms of writing `R` (host time); brake output fall to driver standby (first 1 ms OUT1A window above 3 V) within -1/+6 ms of 100 / 1000 / 1000 / - / 1000 ms (d: no brake edge, last loop P0 edge 49-56 ms before the standby: `R` waits at least 50 ms for the reply, since `50a5f86`; up to two P0 edges within 1 ms before the standby are the reset marker and the floating pin); `/booted` with resetCause 5 |
| C8 | a: `R`, then `/setDestIp` every 50 ms until `/destIp`; b: `R`, `/setDestIp` on `/booted` (again after 300 ms); c (human): power cycle, `/setDestIp` every 50 ms, counted from the first reply after 1 s without replies (the board keeps running on 12 V while the USB cable is out). 10 kS/s | One `/booted` per run with the cause `T` reports (no second restart). RECORD: time from the reset (standby entry; c: the first P0 edge after power-on) to the first P1, P5, `/destIp` and `/booted` (host times mapped with the trigger time, about +-5 ms), sends until the reply, P1 pulses until the reply, P5 max, longest P0 interval. C9a: resetCause 1 after the power cycles of C8c |
| C9 | b (human): reset button, with the test axis brake released (brake outputs and OUT1A at 10 kS/s); c: `pio run -e step800_devlib_probe -t upload` (serial port closed); d: `/resetDevice`. a is judged from C8c | `/booted` cause and `T` resetCause: b 3, d 5; c RECORD. C9b-brake (RECORD): the brake output edges from the driver standby entry |
| C10 | Human: DIP switch 1, 2, 4, ..., 128 (`--dip-values`) while running; brake outputs at 1 MHz | `T` myId equals the value, `W5500 RTR:200 RCR:2` and `New IP: 10.0.0.<100 + id>` printed (beyond .254: `10.0.0.100` and `Warning: IP address out of range with ID <id>, ID not added`), `T` myIp the same, `/setDestIp` answered from that address to port 50100 + id, no brake output edge, all outputs low. Beyond .254 the address stays `10.0.0.100` but the MAC changes with the ID: delete the Mac's ARP entry (`sudo arp -d 10.0.0.100`) after setting the switch, and again after setting it back, or the requests go to the old MAC |
| C10boot | DIP switch already at `--dip` (85); run with `--id 85`: `R`, brake outputs at 10 kS/s | `T` myId, `/setDestIp` answered; the boot output is printed before the port opens (recorded if seen). Then `hil.py selftest --id 85` |
| C11 | Test SD card in (stops unless `sdInitializeSucceeded` 1). `F/hil_save_test.json`; 5 x `G` sent back with `{`, 5 x `/saveConfig` (1 MHz); a save, then `W` (10 kS/s) | C11-2 / C11-3: `{"result":"ok",...}` / `/saveConfig Saved...`, no `/booted`, no early warning, `resetCause` unchanged; the longest P0 interval (the save, for `{` with the serial transfer) is recorded. C11-4: P2 early warning 1.95-2.06 s after the last P4 (watchdog reset), `/booted` and `T` resetCause 4 |
| C12 | SD card removed (stops otherwise). `{` with `network.myIp` (last byte 123) and `outPort` (50123); `{` with `bootedMsgEnable` only; `{` with the values before; `s` and `c` saved; restart | C12-1: `G` shows the sent values (only these two keys change), `T` myIp unchanged, replies still to the bench port. C12-2: keys left out do not change `G`. C12-c: no title printed twice in `c`, no `reportLimitSwStatus` (STEP800). C12-s: the thermal status texts are L6470 names (since `0d90d03`), the `s` fields match the raw values as in C5 |

## Client replacement (`hil.py arp`)

Checks whether the board keeps sending to the MAC address it resolved first when the
client is replaced by another device with the same IP address (another PC, another USB-LAN
adapter). Two USB-LAN adapters of the Mac, NIC A and NIC B (`[arp] nic_a`, `nic_b`), are
both set to `[arp] client_ip` / 255.255.255.0 without router and DNS (System Settings >
Network > the adapter > Details > TCP/IP, Manually). Only one is connected at a time (the
same address twice on the network would conflict). Topologies (`--topology`):

- **T1**: the board, NIC A and NIC B at an unmanaged switch without uplink. The swap
  unplugs the cable of one adapter at the switch and plugs in the other: the board's link
  stays up, as in the field.
- **T2**: the board cabled directly to NIC A; the swap moves the cable to NIC B: the
  board's link goes down once.

While a step runs the bench

- captures both adapters with `tcpdump -i <if> -e -n -U --immediate-mode -w <pcap>
  'ether host <board MAC> or arp'` (promiscuous, the tcpdump default). No `sudo`: the BPF
  devices must be readable (Wireshark's "Install ChmodBPF"; open a new terminal or log in
  again afterwards). The board MAC is read from `s` (`Current MAC address`; `c` prints the
  configured MAC before the ID is added). In T2 the adapter sees every frame of the board,
  also those to the other MAC address. In T1 a frame to the old MAC address reaches NIC B
  only if the switch floods it; there the board's ARP requests (broadcast) and the frames
  to NIC B decide.
- polls the link status of both adapters (`ifconfig`, every 0.1 s). The swap and the
  alias address are detected from it, so the bench also runs without a terminal: it prints
  `OPERATOR:` and goes on when the adapters change.
- records P0-P6 at 1 MHz in consecutive 30 s AD2 recordings (`ad2_<k>.npz`), stopped
  across a restart of the board: the USB re-enumeration dropped 0.53 s of a 10.5 s 1 MHz
  recording without a lost-sample report. A recording whose data are shorter than its host
  time by more than 5 ms is reported and not used. P1 is checked against the OSC packets
  sent (`ad2_segments` in `summary.json`).
- sends and receives OSC through sockets scoped to one adapter (`IP_BOUND_IF`) and bound
  to the client address, one log per socket (`osc_log_<k>_<NIC>_<address>.jsonl`). Only
  one socket per address is open; the previous one is closed when the other adapter takes
  over. `IP_BOUND_IF` does not filter what a receive socket gets: a socket of NIC B still
  open just after NIC A came back received two messages through NIC A. Judge from the
  capture, not from these logs.
- consecutive AD2 recordings leave a gap of about 0.3 s between them; `c1` / `c2` start a
  new recording before the command.

| Step | What is done |
|---|---|
| `check` | NIC A only. E1 1 (below), then `/getVersion` |
| `e1` | 1: NIC A (asked for if not the only active adapter), `R`, `/setDestIp` from NIC A, `/setPositionListReportInterval` `[arp] report_ms` (100). 2: swap to NIC B (human). 3: wait `--wait` s (300). 4: `/getVersion` from NIC B (2 s). 5: `/setDestIp` from NIC B, observe `--observe` s (30). 6: swap back to NIC A (human), 10 s without a command, `/getVersion` from NIC A. `p` before the swap, after 3 and after 5 |
| `e2a` | e1 1-5 (`--wait` 30), then the operator adds `[arp] alias_ip` to NIC B, `/setDestIp` from it (10 s), `/setDestIp` from the client address (`--observe`), the alias is removed, e1 6 |
| `e2b` | e1 1-5, then `R` and `/setDestIp` from NIC B, the reports again, `--observe` s, e1 6 |
| `c1`, `c2` | HIL_PROBE builds with the candidate fixes (`N`: reopen the UDP socket; `U`: 1 byte to 255.255.255.255:9). e1 1-5, then a new AD2 recording and the serial command while `/getVersion` is sent from NIC B 20 times every 2 ms, `--observe` s; e1 6 and the command again (back on NIC A the board keeps the MAC address of NIC B) |
| `n1`, `n2` | The command of `c1` / `c2` three times while the client is reachable (e1 1, reports every 20 ms, 3 s apart), no swap |
| `fix` | The ARP refresh with the client swapped (T1, human). `--heartbeat` (F1): `/setDestIp` every 1.0 s from the connected adapter (paused during a swap, resumed from the new adapter once it has the address), 30 s on NIC A, swap to NIC B, 30 s, swap back, 30 s, the destination MAC (`s`, `T`) after each 30 s. Without (F2): one `/setDestIp`; after each swap 10 s without a packet, the destination MAC, `/getVersion` from the new adapter, 10 s, the destination MAC |
| `gap` | The ARP refresh of the product firmware (`src/arpRefresh.h`), NIC A only, no human. `R`, `G` (initial `destIp`, `canSendMsgBeforeDestIp`), 0: `/setDestIp`, reports `[arp] report_ms`; a: `/setDestIp` every 1.0 s x 30 (heartbeat); b: `/getVersion` every 2.0 s x 10; c: 3 s silence, `/getVersion` every 10 ms x 10; d: 3 s silence, one `/getVersion` (the part "another source first" is left out: the Mac has no other address that reaches the board). `p` is read at the end of each pattern |

`summary.json` has, per phase (each step above starts one): the board's ARP requests
(time from the phase start, the address asked for), its frames by destination MAC
(`NIC A`, `NIC B`, `broadcast`) and OSC address, the ARP to the board (requests for its
address, replies, gratuitous ARP), what NIC B received, P1 / P5 counts and P5 widths, the
P5 pulses paired with the captured frames of the board (within 5 ms after the clock
offset, only within the AD2 recordings), and the OSC messages received per socket. `arp
H1` merges the phases 3-5: H1 is observed when the board sent no ARP request for the
client address and no frame to NIC B. All items are RECORD, except for `gap`:

| Item | Judged |
|---|---|
| `arp gap 0`, `a` - `d` | the board's dummy broadcasts (UDP to 255.255.255.255:9) in the capture per pattern equal the count the firmware rule gives for the packets actually sent (from the capture, so a shifted interval is taken into account; nominal 0, 0, 10, 1, 1), and the `arpRefresh` counter of `p` equals them |
| `arp gap-order` | every refresh: the packet to the board, the dummy, the board's ARP request for the client, the ARP reply, the board's reply, in this order, no other board frame between the dummy and the ARP request |
| `arp gap-p5` (RECORD) | P5 width of the dummy and of the next send (with the ARP) |
| `arp fix-heartbeat` | F1: no dummy and `arpRefresh` 0 while the heartbeat runs on NIC A |
| `arp F1/F2 to NIC A/B` | after each swap (from the new adapter having the address to the next swap request or the last event): one dummy and `arpRefresh` 1, the board's reply (`/destIp`, F1; `/version`, F2) and `/positionList` sent to the new MAC address, no `udpSendFailed`; recorded: the times from the first packet of the new adapter to the dummy, the ARP request and the first frame to the new MAC address |
| `arp F1/F2 on ...` | the destination MAC address of `s` and `T`: the old adapter before the first packet (F2), the new one afterwards |

A heartbeat socket of the unplugged adapter sent through the other adapter while it was
still probing its address (no ARP reply yet): the refresh then fails (41 ms, the send
pause) and the board reaches the client 1-2 s later, with one more refresh. So the
heartbeat sends only from the connected adapter. Each run saves the wall clock samples
(`clock_wall_minus_mono.npy`) used to move the pcap times to `time.monotonic()`.

## How it works

- **Recording**: Digital In in record mode with sample compression
  (`FDwfDigitalInSampleSensibleSet`): the device stores a sample only when a selected line
  changes, and `FDwfDigitalInStatusCompressed` returns (value, span - 1) pairs. This keeps
  long recordings of sparse signals small. Every recording checks the lost / corrupted
  sample counts and fails if they are not 0.
- **Scope + Digital In together**: both wait for the PC trigger and are started by one
  `FDwfDeviceTriggerPC`, but in record mode their data still start a different amount
  apart each time (measured -395 to +26 us; it also stays so with Digital In triggered by
  the Scope). So Scope 2 is connected to P0 in parallel with DIO 8, and each recording is
  aligned from the P0 edges seen by both instruments (`ad2.scope.p0_ref`; all edges
  matched over 10 s recordings, so there is no drift within a recording).
  `selftest --sync` checks this alignment.
- **Host time vs AD2 time**: the n-th OSC message sent during a recording is paired with
  the n-th P1 rise (the board starts processing the packet). A linear fit maps host times
  (e.g. received `/position` replies) to AD2 time. If the counts differ (another packet
  reached the board, or one was lost), the measurement fails.
- **Excitation**: measured on OUT1A at KVAL 16 / 12 V. While energized, OUT1A switches
  0-12 V (about 23.4 kHz, pulses under 3 us) only in the half of the electrical cycle
  where phase A is positive; in the other half, and at EL_POS 0 after a reset, it is held
  at 0 V while the B bridge switches. HiZ floats at about 1.8 V. A 1 ms window counts as
  excited if it has at least `min_edges` comparator transitions or its median is below
  `clamp_v`. A HiZ motor that is still turning can swing below `clamp_v` (back-EMF) and
  read as excited. The brake tests record the Scope at 200 kS/s: at 500 kS/s (two
  channels, Digital In at 1 MHz) it lost samples in 4 of 10 recordings, 2 of them as a
  silent drop of a whole buffer (8192 samples) that only the P0 alignment detects. A
  recording with a reported or detected loss is run again (up to 3 attempts). The firmware's default KVAL_HOLD is 0; the bench sends `/setKval`
  with `[brake] kval`.
- dwfpy has no wrapper for the compressed record or the PC-triggered start of both
  instruments, so `hil/ad2.py` calls those WaveForms functions through `dwfpy.bindings`
  (dwfpy is used to load the runtime and open the device).

### Brake invariants

| ID | Condition | Allowance |
|---|---|---|
| INV-1 | The brake is never released while the motor is not excited | none (resolution: one excitation window) |
| INV-2 | Brake engaged + motor excited only around a brake transition | `transition_ms` + `latency_allowance_ms` per transition |
| INV-3 | The position does not change while the brake is engaged | none |

Before the steps of each scenario (brake engaged, motor HiZ after the common setup), the
median of OUT1A must be in `[excitation] hiz_median_v` (the floating level, about 1.8 V).
Otherwise the scenario is reported as ABORTED (wiring problem) and not judged.

Missing `/position` replies are recorded in `summary.json` (`reply_loss`): `missing`
(requests minus replies), `board_no_reply` (requests with no P5 pulse between their P1
rise and the next one: the board did not answer; listed with their times) and
`lost_in_transit` (sent by the board, not received). Positions are time-stamped at the
P1 rise of their request, and the home switch steps at the DIO15 edges, so the network
delay does not enter the judgement (over Wi-Fi it varied from 3 to 86 ms).

Scenarios can exempt an invariant over a range (e.g. INV-1 after `/free`, which releases
the brake with the motor HiZ on purpose; for INV-3, the pairs of readings that overlap the
range). A range starts one excitation window (1 ms) before its step, since an excitation
edge is known to one window only. Scenarios also add checks:

| Check | Judged on | Meaning |
|---|---|---|
| `Check` | AD2 | the brake output or the excitation keeps a value over a window |
| `Motion` | `/position` readings, timed at the P1 rise of their request | the position changes between every two readings (moving), or never changes |
| `EngageAfterStop` | AD2 + `/position` | the brake engages only after the last reading that is followed by a change |
| `Reply` | host time | an OSC message from the board (e.g. `/error/command BrakeEngaged <axis>`) arrives, or not, within 300 ms after the step. The window is wide because it includes the network delay |

Steps can also be USB serial commands (`serial("Z")`). They have no probe pulse, so their
AD2 time is the first excitation-off edge on OUT1A after the previous step. `Z` (HIL_PROBE
builds only) sends HardHiZ to every driver, bypassing the brake state machine, to
simulate a driver that goes HiZ by itself (overcurrent, thermal shutdown, UVLO).

**Driver standby.** The driver reset line (L6470 STBY/RST) is not reachable on the board.
An MCU reset leaves it floating, and the drivers' internal pull-down puts them in standby
at once: OUT1A of an energized driver jumps to about 6 V and decays to about 0.13 V within
80 ms. The excitation detection reads 0.13 V as excited (below `clamp_v`), so judge a
restart with `hil.py reboot` (the standby entry is the first 1 ms window above 3 V) and
with `D`: CONFIG 0x2E88 (power-on default) at boot means the drivers were reset.
HIL_PROBE builds also set P0 high just before a firmware reset; the bootloader drives
P0 (D1, UART TX) about 30 ms after the reset.

**Servo mode and driver HiZ (S1-S3).** S1 and S2 run the servo mode to a far target and
send `Z` while the motor moves: the firmware must cancel the servo mode from the HiZ
status (no excitation and no position change after `Z` up to the next command, and
`/error/command ServoModeCancelledOnHiZ` within 300 ms), and a second
`/enableServoMode 1` must work again (excited at once, follows a new target 0).

| ID | Setup | Steps | Checks |
|---|---|---|---|
| S1 | brake disabled | `/enableServoMode 1`, target 12800, `Z` 0.2 s later, 1.0 s, `/enableServoMode 1`, target 0, `/enableServoMode 0` | not excited and position unchanged after `Z`; `ServoModeCancelledOnHiZ`; excited after the second `/enableServoMode 1`, moving after target 0 |
| S2 | brake enabled, `/activate 1` | as S1, with `/activate 1` before the second `/enableServoMode 1` | as S1, and `BrakeEngagedOnHiZ`, brake engaged after `Z` (next status read), released after `/activate 1` |
| S3 | brake disabled, then enabled | servo mode, then `/hardHiZ`; servo mode with a far target, `/softHiZ` while running; `/enableElectromagnetBrake 1` (driver HiZ); `/activate 1`, servo mode with a far target, `/activate 0` while running; `/activate 1`, servo mode with a far target, `/free` | no `ServoModeCancelledOnHiZ` after any of the four commands (the windows cover the deceleration and the brake transition); HiZ after each command (and the deceleration); brake engaged after `/activate 0`, released after `/free`; the motor runs before `/softHiZ`, `/activate 0` and `/free` |

In S3 the invariants INV-2 and INV-3 are not checked while the brake is disabled (the
output stays low while the motor moves), and INV-1 not after `/free`. Before the fix of
the servo mode on HiZ, S1 fails on all three `Z` checks and S2 only on the reply.

B18-B20 restart the board: only the first P1 pulses (one per message sent)
are matched, because the probe pins float during the restart, and the position is not
polled. They run last (`hil.py brake` does). B19 and B20 time their serial step by the
brake engage edge and check that it comes before the driver standby entry.

## Tests

```bash
cd tools/hil
.venv/bin/python -m pytest
```

The tests use synthetic data and run without the hardware.

## Files

| File | Contents |
|---|---|
| `hil.py` | CLI |
| `hil/ad2.py` | AD2 open/close, recording, DIO15 open-drain control |
| `hil/osc.py` | OSC send/receive with timestamps, `/setDestIp` check |
| `hil/diag.py` | USB serial commands (`p`, `P`, `b`, `s`, `R`; HIL_PROBE builds: `Z`, `D`, `Y`, `W`) |
| `hil/restart.py` | `reboot`: restart and driver standby analysis |
| `hil/analyze.py` | edges, durations, statistics, command matching, excitation, invariants |
| `hil/scenarios.py` | brake scenarios as data |
| `hil/commands.py` | subcommands |
| `hil/config.py` | `bench.toml` loading and validation |
| `hil/plots.py` | PNG output |
| `hil/arp.py` | `arp`: pcap parsing, capture, link status, AD2 segments, client replacement steps |
| `hil/checks.py` | `check`: C1-C11, the logged USB serial (`SerialLog`) and their judgement helpers |
