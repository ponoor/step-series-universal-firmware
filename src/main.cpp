#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>        //https://arduinojson.org/
#include "watchdog.h"

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
#include "perfStats.h"
#include "packedTransfer.h"
#include "resetCause.h"
#include "homingTimeout.h"
#include "servoSwitch.h"
#include "hilProbe.h"
#include "hilFault.h"

const String productName = PRODUCT_NAME;
const char *firmwareName = FIRMWARE_NAME;
SPIClassSAMD driverSPI(&DRIVER_SERCOM, PIN_DRIVER_MISO, PIN_DRIVER_SCK, PIN_DRIVER_MOSI, PAD_DRIVER_SPI, PAD_DRIVER_RX);

// Servo mode
constexpr auto position_tolerance = 0; // steps

// for PlatformIO
void checkStatus();
void updateServo();
void checkHomingTimeout(uint32_t _currentTimeMillis);
void updatePositionReport(uint32_t _currentTimeMillis);
void updatePositionReportList(uint32_t _currentTimeMillis);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void checkLimitSw();
#endif

void setup()
{
    resetCause = readResetCause(&resetCauseRaw); // before any other initialization
    SerialUSB.begin(9600);
    HIL_PROBE_INIT();
    HIL_FAULT_INIT();
#ifdef HIL_PROBE
    hilCaptureDriversAtBoot(driverSPI);
#endif
#ifdef HAVE_BRAKE
    initBrakeOutputsEngaged(); // before the drivers are reset
#endif
    // while (!SerialUSB); // DEBUG

    pinMode(ledPin, OUTPUT);

#ifdef HAVE_SD
    pinMode(PIN_SD_CS, OUTPUT);
#ifdef PIN_SD_DETECT
    pinMode(PIN_SD_DETECT, INPUT_PULLUP);
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
    dbgPrint("myId:%ld\n", myId);
    delay(1);
    initOscRouter();
    initEthernet();
    dbgPrint("eth ok\n");
    isWaitingSendBootMsg = bootedMsgEnable;

    Watchdog.enable(WATCHDOG_TIMEOUT_MS);

    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (bHomingAtStartup[i])
        {
#ifdef HAVE_BRAKE
            // Release the brake as /activate 1 does; updateBrake() starts the homing
            // once the brake is released.
            if (electromagnetBrakeEnable[i])
            {
                activate(i, true);
                homingAtStartupPending[i] = true;
                continue;
            }
#endif
            homing(i);
        }
    }
}

void checkStatus()
{
    uint32_t t;
    uint16_t statusList[NUM_OF_MOTOR];
    readAllStatus(statusList);
    const uint32_t nowMs = millis();
#ifdef HAVE_BRAKE
    // Read after every brake action so far; an action below makes it stale again.
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        brakeStatusFresh[i] = true;
#endif
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        const uint16_t status = statusList[i];
        // HiZ, high for HiZ
        t = (status & STATUS_HIZ) > 0;
        if (HiZ[i] != t)
        {
            HiZ[i] = t;
            if (reportHiZ[i])
                sendTwoData("/HiZ", i + MOTOR_ID_FIRST, (int32_t)t);
            // The driver went HiZ by itself (overcurrent, thermal shutdown, UVLO): the
            // commands that make it HiZ cancel the servo mode first. Cancel it here, before
            // the servo update of this poll, whose Run would energize the driver again.
            // Nothing is sent to the driver. With the brake enabled, updateBrake() also
            // engages the brake and sends BrakeEngagedOnHiZ.
            if (t && isServoMode[i])
            {
                isServoMode[i] = false;
                sendCommandError(i + MOTOR_ID_FIRST, ERROR_SERVO_MODE_CANCELLED_ON_HIZ);
            }
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
                // Defensive: a stop or HiZ command cancels the pending command.
                if ((busyClearWaitStatus[i] != NONE) && (HiZ[i] || !brakeReleasedOrDisabled(i)))
                {
                    busyClearWaitStatus[i] = NONE;
                }
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
            // Not to a HiZ motor: HardStop would energize it.
            if (bProhibitMotionOnHomeSw[i] && homeSwState[i] && (dir[i] == homingDirection[i]) && !HiZ[i])
            {
                stepper[i].hardStop();
            }
        }
        // SW_EVN, active high, latched
        t = (status & STATUS_SW_EVN) > 0;
        if (t)
        {
            // This action is also done in the driver chip but required for the servo mode.
            if (isServoMode[i] && (!homeSwMode[i]) && !HiZ[i])
                stepper[i].hardStop();
            // Trigger the next sequence of the homing.
            if (homingStatus[i] == HOMING_GOUNTIL)
            {
                if (bHoming[i])
                {
                    if (releaseSw(i, 0, !homingDirection[i]))
                        sendTwoData("/homingStatus", i + MOTOR_ID_FIRST, homingStatus[i]);
                    else
                        clearHomingStatus(i);
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
        // OCD, active low, latched. The latched flags are set again at every read while
        // the condition lasts: reported at once, then at most every
        // LATCHED_REPORT_INTERVAL_MS per motor and kind (latchedReport.h).
        t = (status & STATUS_OCD) == 0;
        if (latchedReportPoll(overCurrentReport[i], t, reportOCD[i], nowMs))
            sendOneDatum("/overCurrent", i + MOTOR_ID_FIRST);

        // STALL A&B, active low, latched
#ifdef DRIVER_L6470
        t = (status & (STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B)) >> 13;
#elif defined(DRIVER_POWERSTEP01)
        t = (status & (STATUS_STALL_A | STATUS_STALL_B)) >> 14;
#endif
        if (latchedReportPoll(stallReport[i], t != 3, reportStall[i], nowMs))
            sendOneDatum("/stall", i + MOTOR_ID_FIRST);
    }
}

void checkHomingTimeout(uint32_t _currentTimeMillis)
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        // homingStartTime can be later than the slot time (step started in this slot).
        if (homingStatus[i] == HOMING_GOUNTIL)
        {
            if (homingTimeoutElapsed(homingStartTime[i], _currentTimeMillis, goUntilTimeout[i]))
            {
                if (!HiZ[i]) // HardStop would energize a HiZ motor
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
        else if (homingStatus[i] == HOMING_RELEASESW)
        {
            if (homingTimeoutElapsed(homingStartTime[i], _currentTimeMillis, releaseSwTimeout[i]))
            {
                if (!HiZ[i]) // HardStop would energize a HiZ motor
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
    bool due[NUM_OF_MOTOR];
    bool anyDue = false;
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        due[i] = reportPosition[i] &&
                 ((uint32_t)(_currentTimeMillis - lastPollTime[i]) >= reportPositionInterval[i]);
        anyDue |= due[i];
    }
    if (!anyDue)
        return;
    // Reading every position with one packed transfer costs the same as reading one.
    int32_t pos[NUM_OF_MOTOR];
    readAllPositions(pos);
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (due[i])
        {
            sendTwoData("/position", i + MOTOR_ID_FIRST, pos[i]);
            lastPollTime[i] = _currentTimeMillis;
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
#define INTEGRAL_LIMIT 1500.0f
// Gains are tuned for a 10 ms period. Scaling the I and D terms by the period keeps
// the same kP/kI/kD usable when servoPollPeriod changes (identity at 10 ms). At the 5 ms
// slot it is 0.5: each update adds half as much to the integral, and the difference of
// the error is divided by 0.5, so the continuous-time response stays about the same.
constexpr float servoDtRatio = servoPollPeriod / 10.0f;

// PID state of the servo mode, reset by resetServoState() when the servo mode is enabled.
static float servoIntegral[NUM_OF_MOTOR] = {0.0f};
static float servoPrevError[NUM_OF_MOTOR] = {0.0f};

void resetServoState(uint8_t motorId)
{
    servoIntegral[motorId] = 0.0f;
    servoPrevError[motorId] = 0.0f;
}

// Computes the run command of a motor in the servo mode from its current position.
static void computeServoSpeed(uint8_t i, int32_t currentPos, bool &dir, float &absSpd)
{
    float spd = 0.0f;
    int32_t error = targetPosition[i] - currentPos;
    // Wrap into the ABS_POS range (22 bit, -2^21 to 2^21-1).
    if (error > 0x1FFFFFL)
        error -= 0x400000L;
    else if (error < -0x200000L)
        error += 0x400000L;
    if (labs(error) > position_tolerance)
    {
        servoIntegral[i] += (((float)error + servoPrevError[i]) * 0.5f) * servoDtRatio;
        if (servoIntegral[i] > INTEGRAL_LIMIT)
            servoIntegral[i] = INTEGRAL_LIMIT;
        else if (servoIntegral[i] < -INTEGRAL_LIMIT)
            servoIntegral[i] = -INTEGRAL_LIMIT;
        float diff = ((float)error - servoPrevError[i]) / servoDtRatio;

        spd = (float)error * kP[i] + servoIntegral[i] * kI[i] + diff * kD[i];
    }
    servoPrevError[i] = error;
    absSpd = fabsf(spd);
    dir = (spd > 0.0f);
    // The home switch and the limit switch are judged separately (servoSwitch.h).
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
    if (servoSwitchBlocks(homeSwState[i], limitSwState[i], bProhibitMotionOnHomeSw[i],
                          bProhibitMotionOnLimitSw[i], homeSwMode[i], limitSwMode[i],
                          homingDirection[i], dir))
#else
    if (servoSwitchBlocks(homeSwState[i], false, bProhibitMotionOnHomeSw[i], false,
                          homeSwMode[i], true, homingDirection[i], dir))
#endif
        absSpd = 0.0f;
}

// The servo update runs once per slot (see loop()), so its period is the slot period.
static_assert(servoPollPeriod * 1000 == SLOT_PERIOD_US,
              "updateServo() runs in every slot: a servo period other than SLOT_PERIOD_US "
              "needs a servo schedule separate from the slots");

// One servo update. Called once per slot; the P2 pulse and the timing statistics are
// produced on every call, also when no motor is in the servo mode.
void updateServo()
{
    static uint32_t lastServoUpdateMicros = 0;
    PROBE_HIGH(2);

    const uint32_t nowMicros = micros();
    if (lastServoUpdateMicros != 0)
        perfAdd(perfStats.servoInterval, nowMicros - lastServoUpdateMicros);
    lastServoUpdateMicros = nowMicros;

    bool servoActive[NUM_OF_MOTOR];
    bool anyServo = false;
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        // Defensive: the servo mode is cancelled when the brake engages.
        servoActive[i] = isServoMode[i] && brakeReleasedOrDisabled(i);
        anyServo |= servoActive[i];
    }
    if (anyServo)
    {
        int32_t pos[NUM_OF_MOTOR];
        bool runDir[NUM_OF_MOTOR];
        float absSpd[NUM_OF_MOTOR];
        readAllPositions(pos);
        // Motors not in the servo mode stay unprepared and receive NOP.
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (servoActive[i])
            {
                computeServoSpeed(i, pos[i], runDir[i], absSpd[i]);
                stepper[i].prepareRun(runDir[i], absSpd[i]);
            }
        }
        if (!performPreparedTransfer())
        {
            for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
            {
                if (servoActive[i])
                    stepper[i].run(runDir[i], absSpd[i]);
            }
        }
    }
    PROBE_LOW(2);
}

void loop()
{
    PROBE_TOGGLE(0);
    HIL_FAULT_LOOP_MARK();
    static uint32_t lastLoopStartMicros = 0;
    const uint32_t loopStartMicros = micros();
    if (lastLoopStartMicros != 0)
        perfAdd(perfStats.loop, loopStartMicros - lastLoopStartMicros);
    lastLoopStartMicros = loopStartMicros;

    // Slots every SLOT_PERIOD_US on micros() (the 1 ms steps of millis() would be 20 % of
    // a slot). The first slot runs in the first pass. The next slot is due one period
    // after the previous due time, so a slot that starts late is followed by a shorter
    // interval; if it starts a whole period or more late, the schedule restarts from now
    // (counted in perfStats). The differences are taken as int32_t, so the wrap of
    // micros() (about 71 min) does not matter: the due time stays within one period of now.
    static bool slotScheduled = false;
    static uint32_t nextSlotMicros;
    static uint8_t housekeepingSlot = 0;
    // Clock of the position reports [ms]: advances by exactly HOUSEKEEPING_PERIOD_MS at
    // every housekeeping run. millis() at the slot start would not do: a slot delayed by an
    // OSC packet can start past a millisecond boundary, and the report interval measured
    // from it to the next, punctual slot is then 1 ms short, so a report waited one more
    // housekeeping run (30 ms instead of 20 ms). The other functions keep millis(): they
    // compare it with times taken with millis() while OSC messages are processed.
    static bool reportClockAligned = false;
    static uint32_t reportClockMillis;
    if (!slotScheduled)
    {
        nextSlotMicros = loopStartMicros;
        slotScheduled = true;
    }
    if ((int32_t)(loopStartMicros - nextSlotMicros) >= 0)
    {
        nextSlotMicros += SLOT_PERIOD_US;
        if ((int32_t)(loopStartMicros - nextSlotMicros) >= 0)
        {
            nextSlotMicros = loopStartMicros + SLOT_PERIOD_US;
            perfStats.slotLateCount++;
            reportClockAligned = false; // the slots no longer follow the old schedule
        }
        // The functions that time in milliseconds get the time of the slot start.
        const uint32_t currentTimeMillis = millis();

        // checkStatus() first: the brake poll, the limit switches, the homing timeout and
        // the servo update use the HiZ flags and the switch states read in this slot.
        uint32_t t0 = micros();
        PROBE_HIGH(3);
        checkStatus();
        PROBE_LOW(3);
        perfAdd(perfStats.checkStatus, micros() - t0);
#ifdef HAVE_BRAKE
        updateBrake(currentTimeMillis);
#endif
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
        checkLimitSw();
#endif
        checkHomingTimeout(currentTimeMillis);
        // The servo update runs here, at a fixed point after the start of the slot: its
        // start does not depend on the length of the position reports below or of the OSC
        // packets processed in the loop, so its period stays at SLOT_PERIOD_US. It uses
        // the status, the switch states and the brake state of this slot.
        t0 = micros();
        updateServo();
        perfAdd(perfStats.updateServo, micros() - t0);

        // Every HOUSEKEEPING_SLOTS slots (10 ms), after the servo update.
        if (housekeepingSlot == 0)
        {
            if (!reportClockAligned)
            {
                reportClockMillis = currentTimeMillis;
                reportClockAligned = true;
            }
            else
            {
                reportClockMillis += HOUSEKEEPING_PERIOD_MS; // uint32_t: wraps with millis()
                const int32_t offset = (int32_t)(currentTimeMillis - reportClockMillis);
                if ((offset > 20) || (offset < -20)) // not expected: the slots follow micros()
                {
                    reportClockMillis = currentTimeMillis;
                    perfStats.reportClockRealignCount++;
                }
            }
            updatePositionReport(reportClockMillis);
            if (reportPositionList)
                updatePositionReportList(reportClockMillis);
            updateRxTxLed(currentTimeMillis);
            uint8_t t = getMyId();
            if (myId != t)
            {
                myId = t;
                initEthernet();
            }
            if (isWaitingSendBootMsg)
            {
                sendBootMsg(currentTimeMillis);
            }
            // Not more often: reset() waits for the previous clear to synchronize (2-3 ms).
            Watchdog.reset();
        }
        if (++housekeepingSlot >= HOUSEKEEPING_SLOTS)
            housekeepingSlot = 0;
    }

    if (SerialUSB.available() > 0)
    {
        diagnosisCommand(SerialUSB.read());
    }
    uint32_t t0 = micros();
    if (OSCMsgReceive())
        perfAdd(perfStats.oscReceive, micros() - t0);
}