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

The configuration (`config.txt` or a `.json` file on the microSD card) is kept across updates. See [microSD card](#microsd-card) for the card format and which file is read.

## Configuration tool

`index.html` is the STEP Series Config Tool, a single HTML page that runs in Chrome or Edge (it uses the Web Serial API).

- Connect to a board over USB, read the current configuration, edit it, save it to the board's microSD card (the board applies it after a reboot), reboot the board, and watch its status.
- Load and save configuration files on your computer without a board (works in any browser).

Open the file locally in the browser. It replaces the earlier offline configuration tools for STEP400 and STEP800.

## microSD card

The board reads its configuration from a microSD card and saves the configuration to it (`/saveConfig`, or the configuration tool). The card is read once, at start-up: insert it before powering on, or reset the board after inserting it.

- **Format the card as FAT16 or FAT32.** exFAT is not supported. Cards of 64 GB or more (SDXC) are formatted as exFAT by default; format them as FAT32 (e.g. macOS Disk Utility, "MS-DOS (FAT)"). The board reports an exFAT card as `No SD card` and does not read or change anything on it. This is the same for the PlatformIO and the Arduino IDE builds.
- **Which file is read**, in this order:
  1. `config.txt` in the root directory.
  2. If `config.txt` is missing or cannot be parsed: the alphabetically first `*.json` file in the root directory (the extension is not case sensitive). If that file is not valid (not JSON, or without `information.configVersion`), the defaults are used; the next `.json` file is not tried.
  3. The defaults, when there is no card or no valid file.
- Files and directories whose name starts with `.` are ignored. This includes the `._*` files that macOS writes next to every file it copies to a FAT card, and `.Trashes`. Subdirectories are not searched.
- Long file names work (the path of the file can be up to 63 characters). A `.json` file can be named for the product or the setup, e.g. `B_long_descriptive_name_STEP800_24V.json`.
- **Saving** writes the configuration to the file that was read (`config.txt` when none was read). The USB serial command `F<path>` selects another file (the path starts with `/` and ends with `.json` or `.txt`). The configuration is written to a hidden temporary file `.<name>.tmp` in the same directory first, and replaces the saved file when it has been written. If the save fails (the card or the root directory is full, the directory does not exist, a write error), the existing file stays as it was and the reply is `SD open failed`; without a usable card the reply is `No SD card`. The save takes about 0.1 s (up to about 0.3 s over the USB serial `{` command, which includes the transfer); the watchdog is stopped for that time. The first save after a card was inserted can take longer (up to about 0.7 s was measured).
- If the card is removed while the board is running, saves fail with `SD open failed` and the board goes on with the configuration it read at start-up. Reset the board after inserting the card again.
- A FAT16 root directory holds at most 512 entries (a long file name takes several); keep it small, or use FAT32.

## Building

### PlatformIO (recommended)

The project is a [PlatformIO](https://platformio.org/) project with the Arduino framework. Dependencies are installed automatically from `platformio.ini`.

```sh
pio run -e step400               # build
pio run -e step400 -t upload     # build and upload over USB
```

Use `step800`, `step100` or `step200` for the other boards. `pio run` without `-e` builds STEP400 and STEP800.

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
| [SdFat](https://github.com/greiman/SdFat) (Bill Greiman) | 2.2.3 or later | FAT16 / FAT32 (the sketch sets `SDFAT_FILE_TYPE` 1 in `sdConfig.h`) |
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
