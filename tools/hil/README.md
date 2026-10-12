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
.venv/bin/python hil.py compare results/<baseline dir> results/<head dir>
.venv/bin/python hil.py brake               # all scenarios, or --scenario B3
.venv/bin/python hil.py powerup --repeat 5  # interactive: power cycles the board
```

Each run writes `results/<YYYYmmdd-HHMMSS>_<git short sha>_<label>/` with `summary.json`,
CSV tables, PNG plots, the raw captures (`*.npz`) and the OSC log (`osc_log.jsonl`).
The console shows a summary table and the result directory only.

| Command | What it does |
|---|---|
| `selftest` | Reboots the board. `probes`: records DIO 8-14 while the `P` serial command pulses each probe ch + 1 times, and checks the probe-to-DIO map and the firmware's pin map. `brake_map`: sends `b` 8 times and records which DIO 0-7 rises for each motor, and its delay against P4. `home_sw`: presses / releases DIO15 and checks `/homeSw` reports and `/getHomeSw`. `excitation`: records OUT1A energized at EL_POS 0 (`/hardStop`), energized 64 microsteps further (`/move`) and HiZ (`/hardHiZ`), KVAL from `[brake] kval`, and checks the classification. `sync` (with `--sync`): 5 recordings of P0 on Scope 2 and DIO 8, checks the per-recording alignment. |
| `timing` | Reboots the board, applies the load (servo mode on all motors, `/setTargetPositionList` at 100 Hz, position reports), records the probes at 1 MHz for the duration, reads `p` before and after. Reports min / p50 / p99 / p99.9 / max / mean of the loop period, each handler, the servo and status poll intervals, and compares them with `perfStats`. |
| `compare` | Table of p50 / p99 / p99.9 / max of two `timing` results and overlaid histograms. |
| `brake` | For each scenario of `hil/scenarios.py`: reboot, common setup, record (digital 1 MHz, OUT1A and P0 at 200 kS/s) while running the steps and polling `/getPosition` every 20 ms, then judge INV-1 to INV-3 and the scenario's checks. Saves a timing diagram per scenario. |
| `powerup` | Needs Scope 2+ moved to 3.3 V and the STEP800 USB cable unplugged: USB VBUS is connected to the board's 5 V net without a diode (schematic sheet 1), so USB alone powers the 3.3 V rail and the 74HC595. Reports, per run, the initial level and every edge of each brake output until the first P4 rise. The 74HC595 output register is not cleared at power-up (SHIFTOUT_ENABLE / R3 clear only the shift register, G is tied to GND), so the outputs keep their power-up state until the first RCK edge. Afterwards, move Scope 2+ back to P0. Arms a recording triggered by the 3.3 V rise on Scope 2, asks for a power cycle, and reports any brake output that is HIGH (released) before the first P4 rise. |

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
the brake with the motor HiZ on purpose) and add checks:

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

B18 (`/resetDevice`) restarts the board: only the first P1 pulses (one per message sent)
are matched, because the probe pins float during the restart, and the position is not
polled. Run it last (`hil.py brake` does).

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
| `hil/diag.py` | USB serial commands (`p`, `P`, `b`, `s`, `R`, `Z`) |
| `hil/analyze.py` | edges, durations, statistics, command matching, excitation, invariants |
| `hil/scenarios.py` | brake scenarios as data |
| `hil/commands.py` | subcommands |
| `hil/config.py` | `bench.toml` loading and validation |
| `hil/plots.py` | PNG output |
