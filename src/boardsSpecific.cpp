#include "boardsSpecific.h"
#include "wiring_private.h" // pinPeripheral() function

#if defined(STEP400_R1) || defined(STEP400_PROTO_R4)
powerSTEP stepper[] = {
    powerSTEP(3, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    powerSTEP(2, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    powerSTEP(1, PIN_DRIVER_CS, PIN_DRIVER_RESET),
    powerSTEP(0, PIN_DRIVER_CS, PIN_DRIVER_RESET)};
#elif defined(STEP800_R1) || defined(STEP800_PROTO_R3)
SPIClass shiftRegisterSPI(&DIPSW_SERCOM, PIN_DIPSW_MISO, PIN_DIPSW_SCK, PIN_DIPSW_MOSI, PAD_DIPSW_SPI, PAD_DIPSW_RX);
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
#if defined(STEP400_R1) || defined(STEP400_PROTO_R4)
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

#ifdef HAVE_BRAKE
void initBrake()
{
#ifdef STEP400_R1
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (electromagnetBrakeEnable[i])
        {
            pinMode(brakePin[i], OUTPUT);
        }
    }
#elif defined(STEP400_PROTO_R4)
#elif defined(STEP800_R1)
    pinMode(PIN_BRAKE_SHIFTOUT_ENABLE, OUTPUT);
    digitalWrite(PIN_BRAKE_SHIFTOUT_ENABLE, HIGH);
#endif
}

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
#elif defined(STEP400_R1)
    digitalWrite(brakePin[motorId], state);
#endif
}

#endif

uint8_t getMyId()
{
    uint8_t _id = 0;
#if defined(STEP400_R1) || defined(STEP400_PROTO_R4)
    for (auto i = 0; i < DIP_SW_DIGITS; ++i)
    {
        _id |= (!digitalRead(dipSwPin[i])) << i;
    }
#elif defined(STEP800_R1)
    digitalWrite(PIN_DIPSW_LATCH, LOW);
    digitalWrite(PIN_DIPSW_LATCH, HIGH);
    shiftRegisterSPI.transfer(brakeOut);
    _id = shiftRegisterSPI.transfer(0);
    _id = ~_id;
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
        shiftInByte |= (!digitalRead(PIN_DIPSW_SCK)) << i;
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
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        bool t;
#ifdef HAVE_LIMIT_ADC
        t = (stepper[i].getParam(ADC_OUT) < 15U);
#elif defined(HAVE_LIMIT_GPIO)
        t = !digitalRead(limitSwPin[i]);
#endif
        if (limitSwState[i] != t)
        {
            limitSwState[i] = !limitSwState[i];
            if (limitSwState[i] && (limitSwMode[i] == SW_HARD_STOP))
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