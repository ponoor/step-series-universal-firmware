#include "boardsSpecific.h"
#include "packedTransfer.h"
#include "hilProbe.h"
#include "wiring_private.h" // pinPeripheral() function

#if defined(STEP400_R1) || defined(STEP400_PROTO_R4) || defined(STEP400_PROTO_R3)
powerSTEP stepper[] = {
    powerSTEP(3, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    powerSTEP(2, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    powerSTEP(1, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    powerSTEP(0, PIN_DRIVER_CS, PIN_DRIVER_RESET)};
#elif defined(STEP800_R1) || defined(STEP800_PROTO_R3)
SPIClassSAMD shiftRegisterSPI(&DIPSW_SERCOM, PIN_DIPSW_MISO, PIN_DIPSW_SCK, PIN_DIPSW_MOSI, PAD_DIPSW_SPI, PAD_DIPSW_RX);
AutoDriver stepper[] = {
    AutoDriver(7, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(6, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(5, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(4, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(3, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(2, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(1, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(0, PIN_DRIVER_CS, PIN_DRIVER_RESET)};
#elif defined(STEP800_PROTO_R1)
AutoDriver stepper[] = {
    AutoDriver(7, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(6, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(5, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(4, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(3, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(2, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(1, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(0, PIN_DRIVER_CS, PIN_DRIVER_RESET)};
#elif defined(STEP100_R1)
powerSTEP stepper[] = {
    powerSTEP(0, PIN_DRIVER_CS, PIN_DRIVER_RESET)};
#elif defined(STEP200_R1)
AutoDriver stepper[] = {
    AutoDriver(1, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    AutoDriver(0, PIN_DRIVER_CS, PIN_DRIVER_RESET)};
#endif

void initDipSw()
{
#if defined(STEP400_R1) || defined(STEP400_PROTO_R4) || defined(STEP400_PROTO_R3)
    for (uint8_t i = 0; i < DIP_SW_DIGITS; i++)
    {
        pinMode(dipSwPin[i], INPUT_PULLUP);
    }

#elif defined(STEP800_R1) || defined(STEP800_PROTO_R3)
    shiftRegisterSPI.begin();
    pinPeripheral(PIN_DIPSW_MISO, EPIO_DIPSW_MISO);
    pinPeripheral(PIN_DIPSW_MOSI, EPIO_DIPSW_MOSI);
    pinPeripheral(PIN_DIPSW_SCK, EPIO_DRIVER_SCK);
    shiftRegisterSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    pinMode(PIN_DIPSW_LATCH, OUTPUT);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
#elif defined(STEP800_PROTO_R1)
    pinMode(PIN_DIPSW_MISO, INPUT);
    pinMode(PIN_DIPSW_SCK, OUTPUT);
    pinMode(PIN_DIPSW_LATCH, OUTPUT);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
#elif defined(STEP100_R1) || defined(STEP200_R1)
    #ifdef HAVE_DIP_SW
    for (uint8_t i = 0; i < DIP_SW_DIGITS; i++)
    {
        pinMode(dipSwPin[i], INPUT_PULLUP);
    }
    #endif
#endif

#if defined(STEP800_PROTO_R3)
    pinMode(SETUP_SW_PIN, INPUT_PULLUP);
#endif
}

#if defined(STEP800_R1)
// Shift register chain: MOSI -> 74HC165 (DIP switch) -> 74HC595 (brake outputs) -> MISO.
// The latch (PIN_DIPSW_LATCH) is the SH/LD of the 74HC165 and the RCK of the 74HC595.
// The first latch loads the DIP switch; after the two bytes, brakeOut is in the shift
// register of the 74HC595 and the second latch puts it on the outputs.
static uint8_t exchangeShiftRegisters()
{
    PROBE_HIGH(4);
    digitalWrite(PIN_DIPSW_LATCH, LOW);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
    shiftRegisterSPI.transfer(brakeOut);
    const uint8_t dipSw = shiftRegisterSPI.transfer(0);
    digitalWrite(PIN_DIPSW_LATCH, LOW);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
    PROBE_LOW(4);
    return dipSw;
}
#endif

#ifdef HAVE_BRAKE
// First thing in setup(): engages every brake output before the drivers are reset.
void initBrakeOutputsEngaged()
{
#ifdef STEP400_R1
    // The pins float (Brake Board pull-down: engaged) until here.
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        digitalWrite(brakePin[i], LOW);
        pinMode(brakePin[i], OUTPUT);
    }
#endif
    // STEP800: latching the cleared shift register here did not shorten the released
    // outputs at power-up (HIL bench: 236-262 ms vs 251-262 ms), so it was not kept.
    // The outputs go low at the first latch, in initDipSw().
}

void initBrake()
{
#if defined(STEP800_R1)
    pinMode(PIN_BRAKE_SHIFTOUT_ENABLE, OUTPUT);
    digitalWrite(PIN_BRAKE_SHIFTOUT_ENABLE, HIGH);
#endif
}

// HIGH: released, LOW: engaged. The output changes at once.
void setBrake(uint8_t motorId, bool state)
{
#ifdef STEP400_R1
    digitalWrite(brakePin[motorId], state);
#elif defined(STEP800_R1)
    if (state)
    {
        bitSet(brakeOut, motorId);
    }
    else
    {
        bitClear(brakeOut, motorId);
    }
    exchangeShiftRegisters();
#endif
}

#endif

uint8_t getMyId()
{
    uint8_t _id = 0;
#if defined(STEP400_R1) || defined(STEP400_PROTO_R4) || defined(STEP400_PROTO_R3)
    for (auto i = 0; i < DIP_SW_DIGITS; ++i)
    {
        _id |= (!digitalRead(dipSwPin[i])) << i;
    }
#elif defined(STEP800_R1)
    _id = ~exchangeShiftRegisters();
#elif defined(STEP800_PROTO_R3)
    byte shiftInByte[3];
    digitalWrite(PIN_DIPSW_LATCH, LOW);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
    for (uint8_t i = 0; i < 3; i++)
    {
        shiftInByte[i] = shiftRegisterSPI.transfer(0);
    }
    _id = shiftInByte[2];
#elif defined(STEP800_PROTO_R1)
    uint32_t shiftInByte;
    digitalWrite(PIN_DIPSW_LATCH, LOW);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
    shiftInByte = !digitalRead(PIN_DIPSW_MISO);
    for (uint8_t i = 1; i < 24; i++)
    {
        digitalWrite(PIN_DIPSW_SCK, HIGH);
        shiftInByte |= (!digitalRead(PIN_DIPSW_MISO)) << i;
        digitalWrite(PIN_DIPSW_SCK, LOW);
    }
    _id = shiftInByte >> 16;
#elif defined(STEP100_R1) || defined(STEP200_R1)
    #ifndef HAVE_DIP_SW
    _id = ID;
    #else
    for (auto i = 0; i < DIP_SW_DIGITS; ++i)
    {
        _id |= (!digitalRead(dipSwPin[i])) << i;
    }
    #endif
#endif
    return _id;
}

#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)

#ifdef HAVE_LIMIT_GPIO
void initLimitSw()
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        pinMode(limitSwPin[i], INPUT_PULLUP);
    }
}
#endif
void checkLimitSw()
{
#ifdef HAVE_LIMIT_ADC
    uint32_t adcOut[NUM_OF_MOTOR];
    readAllParams(ADC_OUT, adcOut);
#endif
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        bool t;
#ifdef HAVE_LIMIT_ADC
        t = (adcOut[i] < 15U);
#elif defined(HAVE_LIMIT_GPIO)
        t = !digitalRead(limitSwPin[i]);
#endif
        if (limitSwState[i] != t)
        {
            limitSwState[i] = !limitSwState[i];
            // Not to a HiZ motor: HardStop would energize it.
            if (limitSwState[i] && (limitSwMode[i] == SW_HARD_STOP) && !HiZ[i])
            {
                stepper[i].hardStop();
            }
            if (reportLimitSwStatus[i])
            {
                sendThreeInt("/limitSw", i + MOTOR_ID_FIRST, limitSwState[i], dir[i]);
            }
        }
    }
}
#endif