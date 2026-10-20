# STEP-series Universal Firmware

<img src="https://ponoor.com/cms/wp-content/uploads/2022/02/IMG_4728.jpg" />

Firmware for the OSC (Open Sound Control) stepper motor drivers [STEP400](https://ponoor.com/en/products/step400/) and [STEP800](https://ponoor.com/en/products/step800/) from [Ponoor Experiments](https://ponoor.com/en/). Both products, and the X-NUCLEO based STEP100 / STEP200, are built from this single source tree. The firmware runs on the SAMD21 (Arduino Zero compatible) with a W5500 Ethernet controller and STMicroelectronics L6470 / PowerSTEP01 motor drivers.

- Latest release: see [Releases](https://github.com/ponoor/step-series-universal-firmware/releases). Changes and compatibility notes: [CHANGELOG.md](CHANGELOG.md).
- Documentation, OSC command reference: [STEP400/800 documents](https://ponoor.com/en/docs/step-series/)
- Questions and discussion: [Forum](https://github.com/ponoor/step-series-support/discussions)

## Supported boards

| Board | Driver | Motors | PlatformIO env | Status |
| --- | --- | --- | --- | --- |
| STEP400 (R1) | PowerSTEP01 | 4 | `step400` | Tested |
| STEP800 (R1) | L6470 | 8 | `step800` | Tested |
| STEP100 (Arduino Zero + [X-NUCLEO-IHM03A1](https://www.st.com/en/ecosystems/x-nucleo-ihm03a1.html) + Ethernet shield) | PowerSTEP01 | 1 | `step100` | Builds; not tested since v2.1.0 |
| STEP200 (Arduino Zero + [X-NUCLEO-IHM02A1](https://www.st.com/en/ecosystems/x-nucleo-ihm02a1.html) + Ethernet shield) | L6470 | 2 | `step200` | Builds; not tested since v2.1.0 |

Prototype boards (STEP400 proto R3/R4, STEP800 proto R1/R3) are still defined in `src/boardsDef.h` but are not maintained. STEP100/200 documents: [ponoor.com/en/docs/step100-200](https://ponoor.com/en/docs/step100-200/).

## Updating the firmware

STEP400 and STEP800 are updated through the USB bootloader: build and upload with PlatformIO or the Arduino IDE (below). See [Compiling the Arduino sketch](https://ponoor.com/en/docs/step-series/technical-information/compile-the-arduino-sketch/) in the documents.

**Before updating a running system, read the compatibility notes of the new version in [CHANGELOG.md](CHANGELOG.md).** Some versions change the behavior of OSC commands or add error replies.

## Configuration tool

`index.html` is the STEP Series Config Tool, a single HTML page that runs in Chrome or Edge (it uses the Web Serial API).

- Connect to a board over USB, read the current configuration, edit it, save it to the board's microSD card (the board applies it after a reboot), reboot the board, and watch its status.
- Load and save configuration files on your computer without a board (works in any browser).

Open the file locally in the browser. It replaces the earlier offline configuration tools for STEP400 and STEP800.

## Building

### PlatformIO (recommended)

The project is a [PlatformIO](https://platformio.org/) project with the Arduino framework. Dependencies are installed automatically from `platformio.ini`.

**Selecting the board (changed in 2.2.0).** Up to 2.1.0, the board was selected by uncommenting a line in `src/boardsDef.h`. With PlatformIO, the board is now selected by the environment in `platformio.ini`: `step400`, `step800`, `step100` or `step200`. Leave every board line in `boardsDef.h` commented out.

1. Open `platformio.ini` and set `default_envs` to your board:

   ```ini
   [platformio]
   default_envs = step800
   ```

2. Build and upload with the PlatformIO buttons in VS Code, or:

   ```sh
   pio run              # build
   pio run -t upload    # build and upload over USB
   ```

Without editing the file, you can also pick the environment each time: `pio run -e step800 -t upload`, or choose `env:step800` in the environment selector of the VS Code status bar.

- If you had uncommented a board in `boardsDef.h` for an earlier version, comment it out again. A board defined in both places does not compile.
- The environment `zeroUSB` of earlier versions is kept and builds STEP400.
- With more than one board connected, add the port to the environment, e.g. `upload_port = /dev/cu.usbmodem1101` under `[env:step800]`.

### Arduino IDE

Download `step-series-universal-firmware.zip` from the assets of the latest release. Install the **Arduino SAMD Boards** core and select **Arduino Zero (Native USB Port)**. Select the board in `boardsDef.h` by uncommenting one definition (STEP400_R1 is the default when none is defined):

```c
// #define STEP400_R1
// #define STEP800_R1
// #define STEP100_R1 // X-NUCLEO-IHM03A1, PowerSTEP01 x1
// #define STEP200_R1 // X-NUCLEO-IHM02A1, L6470 x2
```

Install these libraries with the Library Manager:

| Library | Version | Used for |
| --- | --- | --- |
| [Ponoor PowerSTEP01 Library](https://github.com/ponoor/Ponoor_PowerSTEP01_Library) | 1.2.0 or later | STEP400, STEP100 |
| [Ponoor L6470 Library](https://github.com/ponoor/Ponoor_L6470_Library) | 1.2.0 or later | STEP800, STEP200 |
| [Ethernet](https://github.com/arduino-libraries/Ethernet) | 2.0.2 or later | |
| [SdFat](https://github.com/greiman/SdFat) (Bill Greiman) | 2.2.3 or later | |
| [ArduinoJson](https://arduinojson.org/) | 7.4.1 or later (7.x) | |

The OSC parser and the watchdog are part of the firmware; the CNMAT OSC library and Adafruit SleepyDog are no longer needed.

### Firmware and driver library versions

| Firmware | Ponoor L6470 / PowerSTEP01 Library |
| --- | --- |
| 2.2.0 | 1.2.0 |
| 2.1.0 and earlier | 1.1.x |

## Development

```
src/          firmware sources
test/         host unit tests (pio test -e native)
tools/hil/    hardware-in-the-loop test bench (see tools/hil/README.md)
index.html    configuration tool
```

- Host unit tests: `pio test -e native` (OSC parsing and routing, argument checks, brake state machine, and other logic without hardware).
- Hardware tests: `tools/hil/` drives a board over OSC and measures signals with a Digilent Analog Discovery 2. Setup and wiring are in [tools/hil/README.md](tools/hil/README.md).
- Branches: development happens on `develop` with `feature/*` branches; releases are merged to `main` and tagged `vX.Y.Z`. Add user-visible changes to `[Unreleased]` in [CHANGELOG.md](CHANGELOG.md).

## License and credits

[MIT License](LICENSE), Ponoor Experiments Inc.

- The driver libraries are based on the [SparkFun AutoDriver library](https://github.com/sparkfun/L6470-AutoDriver) and, for PowerSTEP01, the Megunolink powerSTEP01 library.
- [python-step-series](https://github.com/ponoor/python-step-series) by @JulianOrteil.
