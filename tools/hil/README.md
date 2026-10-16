# HIL test bench (STEP800 / STEP400 + Analog Discovery 2)

Hardware-in-the-loop tests of the STEP800 firmware with a Digilent Analog Discovery 2
(AD2) connected to a Mac. The STEP400_R1 is supported with its own configuration file,
see [STEP400_R1](#step400_r1); everything else in this file describes the STEP800 bench.

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

The bench drives **only DIO15** on the STEP800 (DIO14 and DIO15 on the STEP400), and only
as an open-drain output: `press()` sets the output value 0 and then enables the output;
`release()` disables it (high impedance, the L6470 pulls the input up). No pin is ever
driven high. The switches are released when the bench opens the AD2 and before it closes
it, also after an exception. The waveform generators (W1/W2), the power supplies (V+/V-)
and the pattern generator are not used. `bench.toml` cannot assign the home switch to
another pin than 15 (nor the STEP400 limit switch to another than 14), and on the STEP800
it cannot drive DIO14 (P6).

Before every subcommand except `compare` (and the STEP800 `powerup`, whose USB cable is
unplugged), the bench reads `s` and `P` over the USB serial and stops unless the firmware
name and the HIL probe map are those of `board.model`: the STEP800 probe pins are brake
outputs and a DIP switch on the STEP400.

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
| `gap` | The ARP refresh of the product firmware (`src/arpRefresh.h`), NIC A only (captured alone; NIC B need not be attached), no human. `R`, `G` (initial `destIp`, `canSendMsgBeforeDestIp`), 0: `/setDestIp`, reports `[arp] report_ms`; a: `/setDestIp` every 1.0 s x 30 (heartbeat); b: `/getVersion` every 2.0 s x 10; c: 3 s silence, `/getVersion` every 10 ms x 10; d: 3 s silence, one `/getVersion` (the part "another source first" is left out: the Mac has no other address that reaches the board). `p` is read at the end of each pattern |

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

## STEP400_R1

Same bench, AD2 and Mac; the STEP400 configuration is a separate file:

```bash
cd tools/hil
cp bench.step400.example.toml bench.step400.toml   # gitignored; edit board.id, motor.test_axis
.venv/bin/python hil.py --config bench.step400.toml selftest
```

Keys left out of `bench.step400.toml` take the values of `bench.step400.example.toml`,
then of `bench.example.toml`. Firmware: `step400_devlib_probe` (`platformio_local.ini`, not
committed):

```ini
[env:step400_devlib_probe]
extends = env:step400_devlib
build_flags =
	${samd_base.build_flags}
	-D STEP400_R1
	-D HIL_PROBE
```

### Wiring

STEP400_R1 (schematic `step400_r1_schematics.pdf`). No motor and no physical brake; the
STEP400 Brake Board is connected to J7.

| Signal | Board | Arduino | SAMD21 | AD2 | Direction |
|---|---|---|---|---|---|
| Brake outputs EM_1-EM_4 | J7-1 to J7-4 (in parallel with the Brake Board) | A1, D8, D5, D1 | PB08, PA06, PA15, PA10 | DIO 0-3 | input |
| 3.3 V | J8-2 | - | - | DIO 4 | input |
| P0 loop toggle | J8-5 | D38 | PA13 | DIO 8 + Scope 2+ | input |
| P1 OSC packet processing | J8-4 | SCL | PA23 | DIO 9 | input |
| P2 servo update, watchdog early warning, `Y` | J8-3 | SDA | PA22 | DIO 10 | input |
| Limit switch of the test axis | CN1-CN4 pin 2 (ADCIN through 100 ohm) | - | - | DIO 14 | open-drain output |
| Home switch of the test axis | CN5-CN8 pin 2 (SW through 100 ohm) | - | - | DIO 15 | open-drain output |
| OUT1A of the test axis / GND | motor connector | - | - | Scope 1+ / 1- | +/-25 V range |
| GND | J8-1 / J7-6 / CN pin 1 | - | - | GND | |

- Brake output polarity: HIGH = released, LOW = engaged. The pins are driven by the MCU
  directly (`digitalWrite()`), with no pull-up or pull-down on the board; while the MCU is
  in reset or in the bootloader they float and the Brake Board's gate pull-down engages.
  EM_4 (D1) is also the UART TX pin of SERCOM0.
- Leave pin 3 (5 V) of the switch connectors unconnected. DIO 5-7 and 11-13 are not used.
- The STEP400_R1 has no pin for P3-P6 (D9 is a DIP switch, A1 / D8 / D1 are brake
  outputs): `ad2.dio.probes` is `[8, 9, 10, -1, -1, -1, -1]` and the metrics of P3-P6
  (`checkStatus`, `getMyId`, `udpSend`, `slotInterval`, `pollInterval`, `servoDelay`,
  `reportInterval`, the brake-write pulses P6, the reply loss on the board side) are left
  out. The position report intervals come from the host receive times.
- The 3.3 V is on DIO 4 (not on the Scope): every recording starts only if it reads high
  (wiring), and Scope 2 stays on P0 also for `powerup`. The DIO threshold is about 1.5 V,
  so power-up times are from that crossing, not from the Scope 1.65 V crossing of the
  STEP800 runs.
- Network: as the STEP800 (`10.0.0.100 + id`); take the STEP800 off the switch.

### Configuration (`bench.step400.example.toml`)

| Key | STEP400 | Meaning |
|---|---|---|
| `board.model` | `"step400"` | firmware name `STEP400_R1_UNIVERSAL`, 4 motors, probes P0-P2, CONFIG power-on default 0x2C88 (PowerSTEP01) |
| `ad2.dio.brake` | `[0, 1, 2, 3]` | DIO of the brake output of motor 1-4 |
| `ad2.dio.probes` | `[8, 9, 10, -1, -1, -1, -1]` | DIO of P0-P6, -1: no pin |
| `ad2.dio.v33` | `4` | 3.3 V |
| `ad2.dio.limit_sw` / `home_sw` | `14` / `15` | the only pins the bench drives |
| `ad2.scope.v33` | `0` | none (the 3.3 V is on the DIO) |
| `[excitation]` | `hiz_median_v = [11.5, 13.5]` | the PowerSTEP01 output floats at the motor supply when HiZ (12.6 V at 12 V; the L6470 at 1.8 V); the rest as the STEP800 |

### Commands on the STEP400

| Command | STEP400 difference |
|---|---|
| `selftest` | with `motor.motor_connected = true` the `excitation` item is left out, and `brake_map` releases each brake while the motor is HiZ (a loaded axis can turn); `--only` also takes `v33` and `limit_sw`. Adds `v33` (DIO 4 high) first and `limit_sw` (as `home_sw`, with `/enableLimitSwReport`, `/getLimitSw`, `/limitSw`, DIO 14); `probes` checks P0-P2 on DIO 8-10 and no edge on DIO 11-13; `brake_map` sends `b` 4 times (no P4: the delay is from the host time of `b`, USB latency included) |
| `timing` | the P0-P2 metrics only (`loop`, `oscReceive`, `updateServo`, `servoInterval`), `p` as before |
| `brake` | the same scenarios (they use the test axis and 255 only); the brake output delay of a command is from its P1 rise (`delays[].brake_change_ms`); no P4 / P6 rows |
| `reboot` | as the STEP800, plus `--how W` (watchdog), `--how button` (reset button: prints `OPERATOR:` and records until `/booted`, up to 10 minutes) and `--all-motors`; judged items below. **The judged items need `--all-motors --move 64`** (e.g. `reboot --how R --all-motors --move 64`): every brake released, and OUT1A moved off EL_POS 0, where it stays at 0 V also while energized, so that the driver standby shows as the end of the PWM |
| `powerup` | 10 kS/s, both USB and 12 V are switched. No trigger: with the board off the AD2 inputs float and toggle, so the rail counts as off when 25 % of the reads over 1 s are low, the recording starts before the power-on and runs to 8 s after `/booted`, and the 3.3 V rise is the start of the last high span of DIO 4. The operator is asked which came on first; judged below |
| `check` | C1, C2, C4, C7, C9 (`--all-auto`: C1, C2, C4, C7). The others use P3-P6, the L6470 bit layout or the STEP800 brake outputs and are refused |
| `arp` | refused (needs P5) |
| `motor` | tests with a motor, brake and load, record only; see [Motor tests](#motor-tests-hilpy-motor) |
| `usb` | USB cable unplugged / plugged in, record only; see [USB cable unplugged](#usb-cable-unplugged-hilpy-usb) |

Scenario steps can press / release either switch: `press(label, switch="limit")`
(default `"home"`); the step's AD2 time is the edge on that switch's DIO.

### Judged items (`reboot`, `powerup`)

`reboot` (also on the STEP800, in `summary.json` `judged`; the earlier fields are kept):

| Item | PASS when |
|---|---|
| `engage` | every brake output is engaged at the driver standby entry (first OUT1A window above 3 V) and stays engaged; for `R` / `resetDevice` it engaged `transition_ms` -1/+6 ms before the standby (safeSystemReset() waits the longest transition). `W`: before the standby (the early warning engages them about 2 s earlier) |
| `boot` | no brake output is high from the MCU reset (last P0 edge before the longest gap) to `loop()` (the first P0 edge followed by 10 edges within 5 ms; single edges of a floating pin or a bootloader are not). High spans are listed per motor as [ms from the reference, length ms] (`brakes_high_during_boot`) |
| `drivers` | CONFIG read by `D` at boot is the power-on default of every driver (0x2C88 PowerSTEP01, 0x2E88 L6470): the MCU reset put them in standby |
| `cause` | `/booted <id> <resetCause>`: 5 for `R` / `resetDevice`, 4 for `W`, 3 for `button` |

On the STEP400 the drivers do not go to standby at the MCU reset: they stay energized for
about 517 ms after it (measured with `R`, `/resetDevice` and `W`, and `Y` floats the reset
line for 300 ms without a standby), then lose the excitation; CONFIG is the power-on
default at `setup()`. The OUT1A jump above 3 V (HiZ is 12.6 V there) is that loss of
excitation, so `engage` of `R` / `resetDevice` reads about 620 ms instead of
`transition_ms`. The standby is searched from the brake engage on (before it the motor can
be HiZ, above the jump level). For `W` the MCU reset is the last P0 edge before the gap
that `loop()` resumes after (the 2 s hang before the early warning is a longer gap).

`timing` also records the position report intervals from the host receive times
(`report_interval_host`: per motor, or of `/positionList`), the only ones on the STEP400.

`powerup` (STEP400): at the power-on every AD2 input (brakes, probes, switches) was seen
high at once for 2-5 ms while P0 on the Scope stayed low (an AD2-side artifact, e.g. a ground
shift); brake spans that start in such a span are listed as `brake_high_in_common` and not
judged (PASS*). PASS per run when no other brake output span is high after the 3.3 V rise
(brake disabled by default, so low before and after `setup()`); a high span is listed as
motor, [ms from the rise, length ms]. RECORD: the first P0 edge (`hilProbeInit()`, start of
`setup()`), the first OUT1A segment that stays in `[excitation] hiz_median_v` for 100 ms
(drivers set up, HiZ), the OUT1A level segments, `/booted` and its host time after arming
(no AD2 time: the trigger time is not known on the host).

### Motor tests (`hil.py motor`)

With a motor (and its brake) connected to the test axis, OUT1A is not on the Scope (Scope 1
disconnected: at 24 V the HiZ output floats near the +/-25 V input range, and a turning motor
makes the excitation detection meaningless). Set `motor.motor_connected = true`: `brake`,
`reboot` and `powerup` then refuse to run (they judge the excitation on OUT1A), and the motor
tests record only.

```bash
.venv/bin/python hil.py --config bench.step400.toml motor --step m1-12
.venv/bin/python hil.py --config bench.step400.toml motor --note results/<dir> drop_mm=0 "clicks=two per cycle"
```

`[motor]` keys used by `motor` (in `bench.step400.toml`), applied before the recording and
again after every restart (`/setDestIp` included); nothing is saved on the board:

| Key | Message | Use |
|---|---|---|
| `bemf_param = [int_spd, st_slp, fn_slp_acc, fn_slp_dec]` | `/setBemfParam` | match the back-EMF compensation to the motor and supply (the defaults over-compensate a 42 mm motor at 24 V several times) |
| `kval = [hold, run, acc, dec]` | `/setKval` | |
| `speed_profile = [acc, dec, max_speed]` | `/setSpeedProfile` | a slow profile for a load the motor can just lift |
| `brake_transition_ms` (100) | `/enableElectromagnetBrake`, `/setBrakeTransitionDuration` | the brake of the test axis is enabled for every step |
| `position_report_ms` (20) | `/setPositionReportInterval` | positions in `positions.csv` |

The HiZ, overcurrent, stall and UVLO reports of the test axis and `/reportError` are enabled.

Each step (`hil/motor.py`, `STEPS`) is a list of actions: OSC messages, USB serial commands,
waits, `/getBusy` polling, waiting for a message (`/booted`) or for a polled value
(`/getUvlo`, `/getHomingStatus`), switch press / release on DIO 14 / 15, `/getXxx` queries
stored in `summary.json`, operator requests, an OCD_TH sweep and a `/setTargetPositionList`
sine. Before the run the console shows `OBSERVE:` (what the operator watches) and `expect:`.
Operator requests print `OPERATOR: ...`; without a terminal the step goes on and the next
action waits for the effect (e.g. `/booted` for at most 3 minutes; the motor stays energized
meanwhile). The AD2 records the brake outputs, P0-P2, both switches and the 3.3 V (Digital
In only) at 100 kS/s, 10 kS/s for steps with a restart.

| Step | What |
|---|---|
| `m0` | brake on, `/activate 1`, +/-200 full steps, `/activate 0` |
| `m1-12`, `m1-3`, `m1-4`, `m1-5`, `m1-6`, `m1-7`, `m1-8` | brake hand-over: `/activate 1` / `0`, `/softHiZ`, `/softHiZ` and `/hardHiZ` while turning, `/activate 1` during the engage wait, `/free` then `/activate 1`, transition time 0 |
| `m2-R`, `m2-resetDevice`, `m2-W`, `m2-button`, `m2-power` | restarts, 3 each (button and power by the operator); the brake state before the setup is read after each restart |
| `usb-open`, `usb-closed` | USB unplugged by the operator, with the serial port open / closed on the host |
| `m3-1`, `m3-1s` | real overcurrent: OCD_TH lowered one step at a time until `/overCurrent` or `/HiZ`, then restored and `/activate 1` (also in the servo mode) |
| `m3-2a`, `m3-2b` | 24 V off and on by the operator (both inlets / STEP400 only), USB connected |
| `m4-a`, `m4-b2`, `m4-b1` | `/run` with STALL_TH default, raw 2, raw 1 |
| `m5-1` to `m5-4` | servo mode: target steps, 100 Hz sine, limit switch blocking, `/goUntil` and `/releaseSw` refused |
| `m6` | current mode: the hold value raised at `/activate 1` and after `/free`, back to voltage mode by `R` |
| `m7-home`, `m7-timeout` | startup homing with the brake from `config.txt` on the SD card (the file is written beforehand with `F` and `{`) |

Most steps start from the horizontal rod (`HORIZONTAL`: `/free`, the rod falls to hanging
down, +50 full steps, brake engaged, position 0); the direction is that of the bench.

Output in `results/<time>_<sha>_motor_<step>[_label]/`: `summary.json` (setup, state before /
after, queries, notifications counted, brake output delays, errors, `observations`),
`timeline.csv`, `positions.csv`, `capture.npz`, `osc_log.jsonl`. The timeline puts on the AD2
time base: OSC commands at their P1 rise (the n-th message to the n-th rise, or, when the
counts differ because of a restart, each to the first rise 0-20 ms after it), brake output,
switch and 3.3 V edges, MCU stops (P0 gaps over 50 ms) and the restart of `loop()`. OSC
messages received and the host-side actions (serial commands, switch operations, operator
requests) have host times mapped with the command fit (`source` `host`: network and host
latency included). `/position`, `/busy` and `/brakeStatus` are left out, `/uvlo` is listed
only when it changes. `brake_delays`: each brake output change of the test axis with the
last command before it.

### USB cable unplugged (`hil.py usb`)

The firmware stopped for about 4 s when the board's USB cable was unplugged while it ran on
24 V (TASK_usb_unplug). `usb` records such an unplug (or plug-in) for one condition; like
`motor` it records only, also with `motor.motor_connected = true`.

```bash
.venv/bin/python hil.py --config bench.step400.toml usb --step u1
.venv/bin/python hil.py --config bench.step400.toml usb --note results/<dir> drop_mm=0
```

| Step | Condition |
|---|---|
| `u1` | brake enabled, released (`/activate 1`), stopped, serial port closed on the host |
| `u2` | as `u1`, the port stays open |
| `u3` | brake enabled, engaged (`/activate 0`) |
| `u4` | test axis turning at 5 step/s (`/run`; take the weight off) |
| `u5` | as `u1` with the position report every 20 ms (the others: no report) |
| `u6` | unplug, wait for the restart, then plug in again: the plug-in is recorded |

The `motor` setup is applied (except the report interval of the step) before the recording
and again after a `/booted`. The AD2 records J7, P0-P2, the 3.3 V and, with `usb.vbus_dio`
set, the USB VBUS (through a divider; not wired by default) at 10 kS/s. During the recording
the host sends `/getVersion` every 100 ms. After `OPERATOR: unplug ...` (Enter with a
terminal) it waits up to 60 s for the replies to stop and then up to 10 s for `/booted`;
neither is an error (no stop is a result). After the recording the operator plugs the cable
back in, the USB serial is reopened and the fault record of the HIL_PROBE build (`H`) is read
into `summary.json` (`fault_record`, null with a firmware without it).

The fault record (`src/hilFault.h`, HIL_PROBE builds) survives a watchdog or software reset
in RAM after `.bss` that neither the startup code nor the bootloader writes. It holds the
exception frame of a HardFault (the handler then toggles P2 every 0.5 ms and waits for the
watchdog period) and the frame of the code the early warning interrupted, with the USB
registers and interrupt counts. The USB interrupt keeps the core's priority 0, as in the
product builds (the TASK_usb_unplug builds lowered it to 1). USB serial
`H` prints and clears the record, `X` causes a HardFault (unaligned read). Turn the PC / LR
into functions with `arm-none-eabi-addr2line -f -e .pio/build/step400_devlib_probe/firmware.elf
0x<pc>`.

`summary.json` `stops`: each P0 gap of 0.2 s or more, times in ms from its last P0 edge (the
stop): `early_warning_ms` (a P2 pulse of 5 ms or more: the early warning holds P2 high for
the brake wait; the servo update pulses are microseconds), `brake_changes_ms` (J7 changes in
the gap: the early warning, or the pins floating at the MCU reset, which the Brake Board pulls
to engaged), `loop_resumes_ms`, `booted_ms` and `resetCause` (host times mapped to the AD2
time), `heartbeat_last_ms` / `heartbeat_next_ms`, and with VBUS the unplug before the stop.
Without VBUS the unplug time is not known; the stop is the last P0 edge.

`--no-ad2` (TASK_usb_unplug_followup): the measuring instruments disconnected from the board.
No AD2, no SWD: each gap of the `/getVersion` replies with the `/booted` after it
(`gaps`: `gap_s`, `resetCause`, `kind`: a gap of 3 s or more with resetCause 4 is a stop
until the watchdog period, HardFault or hang; `open` when no reply came back). `--conditions
key=value ...` stores the wiring / power / firmware of the run. The fault record is read
after the plug-in, as a step of its own: `usb --read-record results/<dir>` (`fault_record`,
`fault`: flags and the PC / LR / xPSR of the HardFault).

```bash
.venv/bin/python hil.py --config bench.step400.toml usb --step u1 --no-ad2 --conditions ad2=removed ice=removed power=24V
.venv/bin/python hil.py --config bench.step400.toml usb --read-record results/<dir>
```

`--no-probe`, for a firmware without the HIL probes (e.g. `main`, TASK_usb_unplug Task 4):
no board check before the run, no P0-P2 and no fault record; `heartbeat_stops` lists each
gap of the `/getVersion` replies (0.3 s or more) with the J7 changes in it and the `/booted`
after it, in ms from the last reply (host times: judge with a window of about 300 ms).

### CPU state through SWD (`hil.py swd`)

With an Atmel-ICE (CMSIS-DAP, SAM port) on the board's SWD connection, `swd` runs OpenOCD
(`interface/cmsis-dap.cfg`, `target/at91samdXX.cfg`) as a server and drives it over its Tcl
port. It never sends `reset` or a flash command; `at91samdXX.cfg` has no
`connect_assert_srst`, so connecting does not reset the board.

```bash
.venv/bin/python hil.py --config bench.step400.toml swd --info --elf results/elf/<firmware>.elf
.venv/bin/python hil.py --config bench.step400.toml swd --halt --elf results/elf/<firmware>.elf
.venv/bin/python hil.py --config bench.step400.toml swd --on-hang --elf results/elf/<firmware>.elf --gdb
```

- `--info`: no halt. DSU DID, PM RCAUSE, DHCSR (halted / lockup), and the flash compared word
  for word with the `.bin` next to `--elf` (or `--bin`): which firmware is on the board.
- `--halt`: halts the CPU, reads the state, resumes it (`--stay-halted`: not). While halted
  nothing runs: the brake outputs and the drivers keep their state.
- `--on-hang`: `/getVersion` every 100 ms; the operator unplugs the USB cable, and the CPU is
  halted as soon as no reply came for 0.3 s.

`summary.json` `dump`: the registers, the exception being executed (IPSR: 0 thread,
3 HardFault, 16 + n IRQ n: USB 23, WDT 18), ICSR, NVIC enabled / pending / priorities, WDT,
USB device and endpoint registers, 128 stack words and the exception frames on the stack
(after an EXC_RETURN word), with a HIL_PROBE ELF also the fault record and the USB interrupt
counters; code addresses with addr2line. `--gdb` adds a gdb backtrace.

Line numbers need an ELF with debug information. `-g` does not change the code: a build
with `PLATFORMIO_BUILD_FLAGS=-g` gives the same `.bin` byte for byte (checked on
`step400_devlib_probe`). Build the firmware on the board that way, with
`SOURCE_DATE_EPOCH` set to its build time (UTC of the time `/getVersion` reports), and keep
the ELF and `.bin` under `results/elf/`.

## Tests

```bash
cd tools/hil
.venv/bin/python -m pytest
```

The tests use synthetic data and run without the hardware.

## Files

| File | Contents |
|---|---|
| `hil.py` | CLI, board check before a subcommand |
| `bench.example.toml` | defaults (STEP800) |
| `bench.step400.example.toml` | STEP400 defaults, over `bench.example.toml` |
| `hil/ad2.py` | AD2 open/close, recording, DIO14 / DIO15 open-drain control |
| `hil/osc.py` | OSC send/receive with timestamps, `/setDestIp` check |
| `hil/diag.py` | USB serial commands (`p`, `P`, `b`, `s`, `R`; HIL_PROBE builds: `Z`, `D`, `Y`, `W`) |
| `hil/restart.py` | `reboot`: restart and driver standby analysis |
| `hil/analyze.py` | edges, durations, statistics, command matching, excitation, invariants |
| `hil/scenarios.py` | brake scenarios as data |
| `hil/commands.py` | subcommands |
| `hil/config.py` | `bench.toml` loading and validation, board models |
| `hil/plots.py` | PNG output |
| `hil/arp.py` | `arp`: pcap parsing, capture, link status, AD2 segments, client replacement steps |
| `hil/checks.py` | `check`: C1-C11, the logged USB serial (`SerialLog`) and their judgement helpers |
| `hil/motor.py` | `motor`: tests with a motor, brake and load (steps as data, timeline, observations) |
| `hil/swd.py` | `swd`: OpenOCD (Atmel-ICE) over its Tcl port: chip info, flash check, halt and dump, addr2line |
| `hil/usb.py` | `usb`: USB cable unplugged / plugged in (heartbeat, stops, early warning, fault record) |
