#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <OSCMessage.h>         // https://github.com/CNMAT/OSC
#include <ArduinoJson.h>        //https://arduinojson.org/
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog

#include "boardsDef.h"
#ifdef DRIVER_POWERSTEP01
#include <Ponoor_PowerSTEP01Library.h>
#elif defined(DRIVER_L6470)
#include <Ponoor_L6470Library.h>
#endif

#include "globals.h"
#include "boardsSpecific.h"
#include "utils.h"
#include "loadConfig.h"
#include "oscListeners.h"
#include "diagnosis.h"

const String productName = PRODUCT_NAME;
const char *firmwareName = FIRMWARE_NAME;
SPIClass driverSPI(&DRIVER_SERCOM, PIN_DRIVER_MISO, PIN_DRIVER_SCK, PIN_DRIVER_MOSI, PAD_DRIVER_SPI, PAD_DRIVER_RX);

// Servo mode
constexpr auto position_tolerance = 0; // steps

// for PlatformIO
void checkStatus();
void updateServo(uint32_t currentTimeMicros);
void checkHomingTimeout(uint32_t _currentTimeMillis);
void updatePositionReport(uint32_t _currentTimeMillis);
void updatePositionReportList(uint32_t _currentTimeMillis);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void checkLimitSw();
#endif

void setup()
{
    SerialUSB.begin(9600);
    // while (!SerialUSB);

    pinMode(ledPin, OUTPUT);

#ifdef HAVE_SD
    pinMode(SD_CS_PIN, OUTPUT);
#ifdef SD_DETECT_PIN
    pinMode(SD_DETECT_PIN, INPUT_PULLUP);
#endif
#endif
    pinMode(PIN_DRIVER_CS, OUTPUT);
    pinMode(PIN_DRIVER_MOSI, OUTPUT);
    pinMode(PIN_DRIVER_MISO, INPUT);
    pinMode(PIN_DRIVER_SCK, OUTPUT);
    pinMode(PIN_DRIVER_RESET, OUTPUT);

    digitalWrite(PIN_DRIVER_RESET, DRIVER_RESET_PIN_OPERATION_STATE);
    digitalWrite(PIN_DRIVER_RESET, !DRIVER_RESET_PIN_OPERATION_STATE);
    delay(10);
    digitalWrite(PIN_DRIVER_RESET, DRIVER_RESET_PIN_OPERATION_STATE);
    digitalWrite(PIN_DRIVER_CS, HIGH);
    driverSPI.begin();
    pinPeripheral(PIN_DRIVER_MISO, EPIO_DRIVER_MISO);
    pinPeripheral(PIN_DRIVER_MOSI, EPIO_DRIVER_MOSI);
    pinPeripheral(PIN_DRIVER_SCK, EPIO_DRIVER_SCK);
    driverSPI.setDataMode(SPI_MODE3);

    initDipSw();
#ifdef HAVE_BRAKE
    initBrake();
#endif
#ifdef HAVE_LIMIT_GPIO
    initLimitSw();
#endif
    loadConfig();
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        stepper[i].SPIPortConnect(&driverSPI);
        resetMotorDriver(i + MOTOR_ID_FIRST);
        digitalWrite(ledPin, HIGH);
        delay(5);
        digitalWrite(ledPin, LOW);
        delay(5);
    }

    myId = getMyId();
    p("myId:%ld\n", myId);
    delay(1);
    initEthernet();
    p("eth ok\n");
    isWaitingSendBootMsg = bootedMsgEnable;

    Watchdog.enable(100);

    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (bHomingAtStartup[i])
        {
            homing(i);
        }
    }
}

void checkStatus()
{
    uint32_t t;
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        const auto status = stepper[i].getStatus();
        // HiZ, high for HiZ
        t = (status & STATUS_HIZ) > 0;
        if (HiZ[i] != t)
        {
            HiZ[i] = t;
            if (reportHiZ[i])
                sendTwoData("/HiZ", i + MOTOR_ID_FIRST, (int32_t)t);
        }
        // BUSY, low for busy
        t = (status & STATUS_BUSY) == 0;
        if (busy[i] != t)
        {
            busy[i] = t;
            if (reportBUSY[i])
                sendTwoData("/busy", i + MOTOR_ID_FIRST, (int32_t)t);
            
            if (!busy[i])
            {
                if (busyClearWaitStatus[i] != NONE)
                {
                    switch (busyClearWaitStatus[i])
                    {
                    case WAIT_FOR_GOTO:
                        stepper[i].goTo(busyClearWaitGoToPosition[i]);
                        busyClearWaitStatus[i] = NONE;
                        break;
                    case WAIT_FOR_GOTO_DIR:
                        stepper[i].goToDir(busyClearWaitGoToDir[i], busyClearWaitGoToPosition[i]);
                        busyClearWaitStatus[i] = NONE;
                        break;
                    case WAIT_FOR_GOHOME:
                        stepper[i].goHome();
                        busyClearWaitStatus[i] = NONE;
                        break;
                    case WAIT_FOR_GOMARK:
                        stepper[i].goMark();
                        busyClearWaitStatus[i] = NONE;
                        break;
                    default:
                        break;
                    }
                }
                else if (homingStatus[i] == HOMING_RELEASESW)
                {
                    homingStatus[i] = HOMING_COMPLETED;
                    if (bHoming[i])
                    {
                        sendTwoData("/homingStatus", i + MOTOR_ID_FIRST, homingStatus[i]);
                        bHoming[i] = false;
                    }
                }
            }
        }
        // DIR
        t = (status & STATUS_DIR) > 0;
        if (dir[i] != t)
        {
            dir[i] = t;
            if (reportDir[i])
                sendTwoData("/dir", i + MOTOR_ID_FIRST, (int32_t)t);
        }
        // SW_F, low for open, high for close
        t = (status & STATUS_SW_F) > 0;
        if (homeSwState[i] != t)
        {
            homeSwState[i] = t;
            if (reportHomeSwStatus[i])
                getHomeSw(i);
        }
        // SW_EVN, active high, latched
        t = (status & STATUS_SW_EVN) > 0;
        if (t)
        {
            if (homingStatus[i] == HOMING_GOUNTIL)
            {
                if (bHoming[i])
                {
                    releaseSw(i, 0, !homingDirection[i]);
                    homingStatus[i] = HOMING_RELEASESW;
                    sendTwoData("/homingStatus", i + MOTOR_ID_FIRST, homingStatus[i]);
                }
                else
                {
                    homingStatus[i] = HOMING_COMPLETED;
                }
            }
            if (reportSwEvn[i])
                sendOneDatum("/swEvent", i + MOTOR_ID_FIRST);
        }

        // MOT_STATUS
        t = (status & STATUS_MOT_STATUS) >> 5;
        if (motorStatus[i] != t)
        {
            motorStatus[i] = t;
            if (reportMotorStatus[i])
                sendTwoData("/motorStatus", i + MOTOR_ID_FIRST, motorStatus[i]);
#ifdef HAVE_BRAKE
            if (bBrakeDecWaiting[i] && (motorStatus[i] == 0)) // motor stopped
            {
                activate(i, false);
                bBrakeDecWaiting[i] = false;
            }
#endif
        }
        // CMD_ERROR, active high, latched
#ifdef DRIVER_L6470
        t = (status & (STATUS_WRONG_CMD | STATUS_NOTPERF_CMD)) > 0;
#elif defined(DRIVER_POWERSTEP01)
        t = (status & STATUS_CMD_ERROR) > 0;
#endif
        if (t)
            sendCommandError(i + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
        // UVLO, active low
        t = (status & STATUS_UVLO) == 0;
        if (t != uvloStatus[i])
        {
            uvloStatus[i] = !uvloStatus[i];
            if (reportUVLO[i])
                sendTwoData("/uvlo", i + MOTOR_ID_FIRST, uvloStatus[i]);
        }
        // TH_STATUS
#ifdef DRIVER_L6470
        t = (status & (STATUS_TH_WRN | STATUS_TH_SD)) >> 10;
        t = (~t) & 0x0003U;
#elif defined(DRIVER_POWERSTEP01)
        t = (status & STATUS_TH_STATUS) >> 11;
#endif
        if (thermalStatus[i] != t)
        {
            thermalStatus[i] = t;
            if (reportThermalStatus[i])
                sendTwoData("/thermalStatus", i + MOTOR_ID_FIRST, thermalStatus[i]);
        }
        // OCD, active low, latched
        t = (status & STATUS_OCD) == 0;
        if (t && reportOCD[i])
            sendOneDatum("/overCurrent", i + 1);

            // STALL A&B, active low, latched
#ifdef DRIVER_L6470
        t = (status & (STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B)) >> 13;
#elif defined(DRIVER_POWERSTEP01)
        t = (status & (STATUS_STALL_A | STATUS_STALL_B)) >> 14;
#endif
        if ((t != 3) && reportStall[i])
            sendOneDatum("/stall", i + MOTOR_ID_FIRST);
    }
}

void checkHomingTimeout(uint32_t _currentTimeMillis)
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if ((homingStatus[i] == HOMING_GOUNTIL) && (goUntilTimeout[i] > 0))
        {
            if ((uint32_t)(_currentTimeMillis - homingStartTime[i]) >= goUntilTimeout[i])
            {
                stepper[i].hardStop();
                sendCommandError(i + MOTOR_ID_FIRST, ERROR_GOUNTIL_TIMEOUT);
                homingStatus[i] = HOMING_TIMEOUT;
                if (bHoming[i])
                {
                    sendTwoData("/homingStatus", i + MOTOR_ID_FIRST, homingStatus[i]);
                    bHoming[i] = false;
                }
            }
        }
        else if ((homingStatus[i] == HOMING_RELEASESW) && (releaseSwTimeout[i] > 0))
        {
            if ((uint32_t)(_currentTimeMillis - homingStartTime[i]) >= releaseSwTimeout[i])
            {
                stepper[i].hardStop();
                sendCommandError(i + MOTOR_ID_FIRST, ERROR_RELEASESW_TIMEOUT);
                homingStatus[i] = HOMING_TIMEOUT;
                if (bHoming[i])
                {
                    sendTwoData("/homingStatus", i + MOTOR_ID_FIRST, homingStatus[i]);
                    bHoming[i] = false;
                }
            }
        }
    }
}

void updatePositionReport(uint32_t _currentTimeMillis)
{
    static uint32_t lastPollTime[NUM_OF_MOTOR] = {0};
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (reportPosition[i])
        {
            if ((uint32_t)(_currentTimeMillis - lastPollTime[i]) >= reportPositionInterval[i])
            {
                sendTwoData("/position", i + MOTOR_ID_FIRST, stepper[i].getPos());
                lastPollTime[i] = _currentTimeMillis;
            }
        }
    }
}

void updatePositionReportList(uint32_t _currentTimeMillis)
{
    static uint32_t lastPollTime = 0;
    if ((uint32_t)(_currentTimeMillis - lastPollTime) >= reportPositionListInterval)
    {
        getPositionList();
        lastPollTime = _currentTimeMillis;
    }
}

void updateServo(uint32_t currentTimeMicros)
{
    static uint32_t lastServoUpdateTime = 0;
    static float eZ1[NUM_OF_MOTOR] = {0.0},
                 eZ2[NUM_OF_MOTOR] = {0.0},
                 integral[NUM_OF_MOTOR] = {0.0};
    float spd = 0.0;
    if ((uint32_t)(currentTimeMicros - lastServoUpdateTime) >= 100)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (isServoMode[i])
            {
                int32_t error = targetPosition[i] - stepper[i].getPos();
                integral[i] += ((error + eZ1[i]) / 2.0f);
                if (integral[i] > 1500.0f)
                    integral[i] = 1500.0f;
                else if (integral[i] < -1500.0f)
                    integral[i] = -1500.0f;
                if (fabsf(error) > position_tolerance)
                {
                    double diff = error - eZ1[i];

                    spd = error * kP[i] + integral[i] * kI[i] + diff * kD[i];
                }
                eZ2[i] = eZ1[i];
                eZ1[i] = error;
                float absSpd = fabsf(spd);
                // if (absSpd < 1.0f) {
                //     spd = 0.0;
                // }
                stepper[i].run((spd > 0.0f), absSpd);
            }
        }
        lastServoUpdateTime = currentTimeMicros;
    }
}

void loop()
{
    uint32_t
        currentTimeMillis = millis(),
        currentTimeMicros = micros();
    static uint32_t lastPollTime = 0;

    if ((uint32_t)(currentTimeMillis - lastPollTime) >= STATUS_POLL_PERIOD)
    {
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
        checkLimitSw();
#endif
#ifdef HAVE_BRAKE
        updateBrake(currentTimeMillis);
#endif
        checkHomingTimeout(currentTimeMillis);
        updatePositionReport(currentTimeMillis);
        if (reportPositionList)
            updatePositionReportList(currentTimeMillis);
        checkStatus();
        updateRxTxLed(currentTimeMillis);
        uint8_t t = getMyId();
        if (myId != t)
        {
            myId = t;
            initEthernet();
        }
        Watchdog.reset();
        lastPollTime = currentTimeMillis;
    }

    if (SerialUSB.available() > 0)
    {
        diagnosisCommand(SerialUSB.read());
    }
    if (isWaitingSendBootMsg)
    {
        sendBootMsg(currentTimeMillis);
    }
    OSCMsgReceive();
    updateServo(currentTimeMicros);
}