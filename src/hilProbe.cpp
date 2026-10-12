// hilProbe.cpp
// Timing probes for the hardware-in-the-loop test bench (see hilProbe.h).

#include "hilProbe.h"

#ifdef HIL_PROBE

#include "utils.h"
#include "wiring_private.h" // pinPeripheral()

uint8_t hilProbePort[HIL_PROBE_COUNT];
uint32_t hilProbeMask[HIL_PROBE_COUNT];

struct HilProbePin
{
    uint8_t pin;
    const char *name;
    uint8_t expectedPort; // 0: PA, 1: PB
    uint8_t expectedBit;
};

static const HilProbePin probePins[HIL_PROBE_COUNT] = {
    {1u, "D1", 0, 10},
    {A1, "A1", 1, 8},
    {8u, "D8", 0, 6},
    {9u, "D9", 0, 7},
    {38u, "D38", 0, 13},
    {SCL, "SCL", 0, 23},
    {SDA, "SDA", 0, 22},
};

void hilProbeInit()
{
    for (uint8_t ch = 0; ch < HIL_PROBE_COUNT; ch++)
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
    for (uint8_t ch = 0; ch < HIL_PROBE_COUNT; ch++)
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
        for (uint8_t ch = 0; ch < HIL_PROBE_COUNT; ch++)
        {
            dbgPrint(" P%d=P%c%02d", ch, 'A' + probePins[ch].expectedPort, probePins[ch].expectedBit);
        }
        dbgPrint("\n");
    }
}

void hilProbeSelfTest()
{
    hilProbePrintMap();
    // Start from low on every probe (P0 may be high from the loop toggle), then a quiet
    // 1 ms so the pulses below can be told apart from normal activity.
    for (uint8_t ch = 0; ch < HIL_PROBE_COUNT; ch++)
    {
        PROBE_LOW(ch);
    }
    delayMicroseconds(1000);
    for (uint8_t ch = 0; ch < HIL_PROBE_COUNT; ch++)
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
    // L6470 CONFIG power-on default: 0x2E88
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

#endif // HIL_PROBE
