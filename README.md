# STEP-series Universal Firmware

<img src="https://ponoor.com/cms/wp-content/uploads/2022/02/IMG_4728.jpg" />

The Arduino sketch for the OSC (Open Sound Control) based stepper motor driver STEP series from [Ponoor Experiments](https://ponoor.com/en/). Basically this sketch will work on Arduino SAMD architectures with Ethernet module (W5100/W5500) and motor drivers L6470/PowerSTEP01 from STMicroelectronics.

## Background
The two hardware from Ponoor Experiments, [STEP400](https://ponoor.com/products/step400/) and [STEP800](https://ponoor.com/products/step800/), they have their own firmware but most of their codes are common or similar. This project aims to compile the each firmware from the same source codes. 

Also covers some variants and X-Nucleos with Arduino.


## Target boards
<!--This firmware covers STEP400 and STEP800 from Ponoor Experiments inc, and also X-Nucleos with Arduino. -->

- [STEP400](https://github.com/ponoor/STEP400/)
- [STEP800](https://github.com/ponoor/STEP800/)
- Arduino Zero + [X-NUCLEO-IHM03A1](https://www.st.com/en/ecosystems/x-nucleo-ihm03a1.html) + Ethernet shield (a.k.a. STEP100)
- Arduino Zero + [X-NUCLEO-IHM02A1](https://www.st.com/en/ecosystems/x-nucleo-ihm02a1.html) + Ethernet shield (a.k.a. STEP200)
<!--
<img src="https://ponoor.com/cms/wp-content/uploads/2021/03/IMG_4219-1600x1067.jpg" width="50%" /><img src="https://ponoor.com/cms/wp-content/uploads/2020/10/step800_r1_1-1600x1067.jpg" width="50%" />

<img src="https://www.st.com/bin/ecommerce/api/image.PF262214.en.feature-description-include-personalized-no-cpn-large.jpg" width="50%" /><img src="https://www.st.com/bin/ecommerce/api/image.PF261319.en.feature-description-include-personalized-no-cpn-large.jpg" width="50%" />
-->

## Compile
This project is based on PlatformIO with Arduino framework. Uncomment one of board definitions listed in `boardsDef.h`.

```
// Products
// #define STEP400_R1
// #define STEP800_R1

// X-Nucleos
// #define STEP100_R1 // X-NUCLEO-IHM03A1, PowerSTEP01 x1
// #define STEP200_R1 // X-NUCLEO-IHM02A1, L6470 x2 

// Prototypes
...
```

If you want to compile/upload with Arduino IDE, please download a `step-series-universal-firmware.zip` from Assets of the latest release. You need to install the SAMD core and following libraries to your Arduino IDE for compiling it. For more details, please see [Compiling the Arduino sketch](https://ponoor.com/en/docs/step-series/technical-information/compile-the-arduino-sketch/) in STEP400/800 documents.

### Library Dependency
- **[OSC Library](https://github.com/CNMAT/OSC)** from CNMAT
- **[ArduinoJSON Library](https://arduinojson.org/)**
- **[Adafruit SleepyDog Arduino Library](https://github.com/adafruit/Adafruit_SleepyDog)**
- **[Ponoor L6470 Library](https://github.com/ponoor/Ponoor_L6470_Library)** for L6470 based boards
- **[Ponoor PowerSTEP01 Library](https://github.com/ponoor/Ponoor_PowerSTEP01_Library)** for PowerSTEP01 based boards

## Documents
- [STEP100/200](https://ponoor.com/en/docs/step100-200/)
- [STEP400/800](https://ponoor.com/en/docs/step-series/)
- [Forum](https://github.com/ponoor/step-series-support/discussions)

## Progress

| Board | Basic test | Full test |
| --- | --- | --- |
| STEP400 R1 | Ok | Ok |
| STEP800 R1 | Ok | Ok |
| STEP400 proto R4 | Ok | Ok |
| STEP800 proto R3 | Ok | Ok |
| STEP800 proto R1 | Ok | Ok |
| X-NUCLEO-IHM02A1 | Ok | Ok |
| X-NUCLEO-IHM03A1 | Ok | Ok |

The full test was done with the test codes of [python-step-series](https://github.com/ponoor/python-step-series) library written by @JulianOrteil.