// hilProbe.cpp
// Timing probes for the hardware-in-the-loop test bench (see hilProbe.h).

#include "hilProbe.h"

#ifdef HIL_PROBE

#include "utils.h"
#include "wiring_private.h" // pinPeripheral()

uint8_t hilProbePort[HIL_PROBE_COUNT];
uint32_t hilProbeMask[HIL_PROBE_COUNT]; // 0 until hilProbeInit(), and for P3-P6 without a pin

struct HilProbePin
{
    uint8_t pin;
    const char *name;
    uint8_t expectedPort; // 0: PA, 1: PB
    uint8_t expectedBit;
};

#if defined(STEP800_R1)
static constexpr HilProbePin probePins[HIL_PROBE_PIN_COUNT] = {
    {1u, "D1", 0, 10},
    {A1, "A1", 1, 8},
    {8u, "D8", 0, 6},
    {9u, "D9", 0, 7},
    {38u, "D38", 0, 13},
    {SCL, "SCL", 0, 23},
    {SDA, "SDA", 0, 22},
};
#elif defined(STEP400_R1)
static constexpr HilProbePin probePins[HIL_PROBE_PIN_COUNT] = {
    {38u, "D38", 0, 13},
    {SCL, "SCL", 0, 23},
    {SDA, "SDA", 0, 22},
};
#endif

// Pins the firmware uses on this board. A probe must not share a pin with any of them:
// on STEP400_R1, D1 / A1 / D8 are brake outputs (J7) and D9 is a DIP switch to GND.
static constexpr uint8_t boardPins[] = {
    ledPin,
    PIN_DRIVER_MISO, PIN_DRIVER_MOSI, PIN_DRIVER_SCK, PIN_DRIVER_CS, PIN_DRIVER_RESET,
    PIN_W5500_RESET, PIN_W5500_CS,
    PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SCK, // W5500 and SD card (SPI)
#ifdef HAVE_SD
    PIN_SD_CS,
#endif
#ifdef PIN_SD_DETECT
    PIN_SD_DETECT,
#endif
#ifdef HAVE_DIP_SW_SPI
    PIN_DIPSW_MISO, PIN_DIPSW_MOSI, PIN_DIPSW_SCK, PIN_DIPSW_LATCH,
#endif
#ifdef PIN_BRAKE_SHIFTOUT_ENABLE
    PIN_BRAKE_SHIFTOUT_ENABLE,
#endif
};

static constexpr bool pinInList(uint8_t pin, const uint8_t *list, size_t n)
{
    return (n != 0) && ((list[0] == pin) || pinInList(pin, list + 1, n - 1));
}

// dipSwPin[] and brakePin[] (boardsDef.h) are const, not constexpr, so they are checked
// in probePinsAreSafe() only.
static constexpr bool probeSharesBoardPin(size_t ch)
{
    return pinInList(probePins[ch].pin, boardPins, sizeof(boardPins));
}

static constexpr bool probeSharesProbePin(size_t ch, size_t other)
{
    return (other < ch) && ((probePins[other].pin == probePins[ch].pin) || probeSharesProbePin(ch, other + 1));
}

static constexpr bool anyProbePinConflict(size_t ch)
{
    return (ch < HIL_PROBE_PIN_COUNT) &&
           (probeSharesBoardPin(ch) || probeSharesProbePin(ch, 0) || anyProbePinConflict(ch + 1));
}

static_assert(!anyProbePinConflict(0), "HIL_PROBE: a probe pin is used by the board or by another probe");

// Same check on the port and bit, the pins as the MCU sees them (two pin numbers can
// be the same port pin). Before any probe pin is made an output.
static bool samePortPin(uint8_t a, uint8_t b)
{
    return (g_APinDescription[a].ulPort == g_APinDescription[b].ulPort) &&
           (g_APinDescription[a].ulPin == g_APinDescription[b].ulPin);
}

static bool portPinInList(uint8_t pin, const uint8_t *list, size_t n, uint8_t *found)
{
    for (size_t i = 0; i < n; i++)
    {
        if (samePortPin(pin, list[i]))
        {
            *found = list[i];
            return true;
        }
    }
    return false;
}

static bool probePinsAreSafe()
{
    bool safe = true;
    for (uint8_t ch = 0; ch < HIL_PROBE_PIN_COUNT; ch++)
    {
        const uint8_t pin = probePins[ch].pin;
        uint8_t found = 0;
        bool conflict = portPinInList(pin, boardPins, sizeof(boardPins), &found);
#ifdef HAVE_DIP_SW
        conflict = conflict || portPinInList(pin, dipSwPin, sizeof(dipSwPin), &found);
#endif
#ifdef STEP400_R1
        conflict = conflict || portPinInList(pin, brakePin, sizeof(brakePin), &found);
#endif
        for (uint8_t other = 0; !conflict && (other < ch); other++)
        {
            if (samePortPin(pin, probePins[other].pin))
            {
                conflict = true;
                found = probePins[other].pin;
            }
        }
        if (conflict)
        {
            dbgPrint("Error: HIL probe P%d (%s) is the port pin of pin %d\n", ch, probePins[ch].name, found);
            safe = false;
        }
        const uint8_t port = (uint8_t)g_APinDescription[pin].ulPort;
        const uint8_t bit = (uint8_t)g_APinDescription[pin].ulPin;
        if ((port != probePins[ch].expectedPort) || (bit != probePins[ch].expectedBit))
        {
            dbgPrint("Error: HIL probe P%d (%s) is P%c%02d, expected P%c%02d\n", ch, probePins[ch].name,
                     'A' + port, bit, 'A' + probePins[ch].expectedPort, probePins[ch].expectedBit);
            safe = false;
        }
    }
    return safe;
}

void hilProbeInit()
{
    if (!probePinsAreSafe())
    {
        // The masks stay 0: PROBE_* write nothing and no pin is made an output.
        dbgPrint("Error: HIL probe pins not set up\n");
        return;
    }
    for (uint8_t ch = 0; ch < HIL_PROBE_PIN_COUNT; ch++)
    {
        const uint8_t pin = probePins[ch].pin;
        hilProbePort[ch] = (uint8_t)g_APinDescription[pin].ulPort;
        hilProbeMask[ch] = 1ul << g_APinDescription[pin].ulPin;
        pinMode(pin, OUTPUT);
        PROBE_LOW(ch);
    }
    hilProbePrintMap();
}

void hilProbePrintMap()
{
    bool mismatch = false;
    dbgPrint("HIL probe:");
    for (uint8_t ch = 0; ch < HIL_PROBE_PIN_COUNT; ch++)
    {
        const uint8_t pin = probePins[ch].pin;
        const uint8_t port = (uint8_t)g_APinDescription[pin].ulPort;
        const uint8_t bit = (uint8_t)g_APinDescription[pin].ulPin;
        dbgPrint(" P%d=%s(P%c%02d)", ch, probePins[ch].name, 'A' + port, bit);
        mismatch |= (port != probePins[ch].expectedPort) || (bit != probePins[ch].expectedBit);
    }
    dbgPrint("\n");
    if (mismatch)
    {
        dbgPrint("Warning: HIL probe pins differ from the expected map:");
        for (uint8_t ch = 0; ch < HIL_PROBE_PIN_COUNT; ch++)
        {
            dbgPrint(" P%d=P%c%02d", ch, 'A' + probePins[ch].expectedPort, probePins[ch].expectedBit);
        }
        dbgPrint("\n");
    }
    if (hilProbeMask[0] == 0)
        dbgPrint("Warning: HIL probe pins not set up\n");
}

void hilProbeSelfTest()
{
    hilProbePrintMap();
    // Start from low on every probe (P0 may be high from the loop toggle), then a quiet
    // 1 ms so the pulses below can be told apart from normal activity.
    for (uint8_t ch = 0; ch < HIL_PROBE_PIN_COUNT; ch++)
    {
        PROBE_LOW(ch);
    }
    delayMicroseconds(1000);
    for (uint8_t ch = 0; ch < HIL_PROBE_PIN_COUNT; ch++)
    {
        for (uint8_t n = 0; n <= ch; n++)
        {
            PROBE_HIGH(ch);
            delayMicroseconds(10);
            PROBE_LOW(ch);
            delayMicroseconds(10);
        }
        delayMicroseconds(1000);
    }
    dbgPrint("HIL probe self-test done\n");
}

static uint16_t bootConfig[NUM_OF_MOTOR];
static uint16_t bootStatus[NUM_OF_MOTOR];
static bool bootCaptured = false;

void hilCaptureDriversAtBoot(SPIClassSAMD &spi)
{
    // High before output: an output with the reset value of OUT (low) would reset them.
    digitalWrite(PIN_DRIVER_RESET, HIGH);
    pinMode(PIN_DRIVER_RESET, OUTPUT);
    digitalWrite(PIN_DRIVER_CS, HIGH);
    pinMode(PIN_DRIVER_CS, OUTPUT);
    spi.begin();
    pinPeripheral(PIN_DRIVER_MISO, EPIO_DRIVER_MISO);
    pinPeripheral(PIN_DRIVER_MOSI, EPIO_DRIVER_MOSI);
    pinPeripheral(PIN_DRIVER_SCK, EPIO_DRIVER_SCK);
    spi.setDataMode(SPI_MODE3);
    delay(1); // a driver leaving standby needs its internal oscillator
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        stepper[i].SPIPortConnect(&spi);
        bootConfig[i] = stepper[i].getParam(CONFIG);
        bootStatus[i] = stepper[i].getParam(REG_STATUS); // GetParam: the flags are kept
    }
    bootCaptured = true;
}

void hilPrintDriversAtBoot()
{
    if (!bootCaptured)
    {
        dbgPrint("HIL boot drivers: not captured\n");
        return;
    }
    // CONFIG at its power-on default: the driver was reset (standby) with the MCU.
#if defined(DRIVER_L6470)
    dbgPrint("HIL boot drivers: CONFIG power-on default 0x2E88 (L6470)\n");
#elif defined(DRIVER_POWERSTEP01)
    dbgPrint("HIL boot drivers: CONFIG power-on default 0x2C88 (PowerSTEP01)\n");
#endif
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        dbgPrint("HIL boot driver %d: CONFIG 0x%04X STATUS 0x%04X now CONFIG 0x%04X\n", i + MOTOR_ID_FIRST,
                 bootConfig[i], bootStatus[i], (uint16_t)stepper[i].getParam(CONFIG));
    }
    dbgPrint("HIL boot drivers done\n");
}

void hilDriverStandbyTest()
{
    PROBE_HIGH(2);
    pinMode(PIN_DRIVER_RESET, INPUT); // floating: the internal pull-down of the drivers
    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < 300)
        Watchdog.reset();
    digitalWrite(PIN_DRIVER_RESET, HIGH);
    pinMode(PIN_DRIVER_RESET, OUTPUT);
    PROBE_LOW(2);
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        resetMotorDriver(i + MOTOR_ID_FIRST);
    dbgPrint("HIL standby test done\n");
}

void hilReopenUdp()
{
    const uint32_t start = micros();
    Udp.stop();
    Udp.begin(inPort);
    dbgPrint("HIL: UDP socket reopened in %lu us\n", (unsigned long)(micros() - start));
}

void hilSendOtherDest()
{
    const uint32_t start = micros();
    const bool ok = sendArpRefreshDummy();
    dbgPrint("HIL: broadcast to port 9 (ok %d) in %lu us\n", ok, (unsigned long)(micros() - start));
}

#endif // HIL_PROBE
