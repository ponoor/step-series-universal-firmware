//
//
//

#include "oscListeners.h"
#include "utils.h"
#include "diagnosis.h"
#include "packedTransfer.h"
#include "oscRouter.h"
#include "oscArgs.h"
#include "perfStats.h"
#include "resetCause.h"
#include "hilProbe.h"

// Receive buffer with guard bytes for oscRouteMatches(), see oscRouter.h
static uint8_t oscRxStorage[OSC_RX_GUARD_BEFORE + OSC_RX_BUFFER_SIZE + OSC_RX_GUARD_AFTER];
static uint8_t *const oscRxBuffer = oscRxStorage + OSC_RX_GUARD_BEFORE;
static OscMessage msgIN;

#define OSC_ROUTE(address, handler, args) {address, handler, args},
static const OscRoute oscRoutes[] = {
#include "oscRoutes.h"
};
#undef OSC_ROUTE
constexpr size_t OSC_ROUTE_COUNT = sizeof(oscRoutes) / sizeof(oscRoutes[0]);
static_assert(OSC_ROUTE_COUNT <= 255, "the route index is uint8_t");
static uint8_t oscRouteIndex[OSC_ROUTE_COUNT];
static OscRouter oscRouter;

// Arguments missing or of the wrong type, or a motor ID that does not fit in a byte: the
// handler is not called and nothing is applied (the arguments of each route: oscRoutes.h).
static bool validateOscArgs(OscMessage &msg, const char *args)
{
    int32_t badMotorId = 0;
    switch (oscCheckArgs(msg, args, NUM_OF_MOTOR, &badMotorId))
    {
    case OSC_ARGS_OK:
        return true;
    case OSC_ARGS_MISSING:
        if (reportErrors)
            sendOneDatum("/error/osc", "MissingArgument");
        break;
    case OSC_ARGS_WRONG_TYPE:
        if (reportErrors)
            sendOneDatum("/error/osc", "WrongDataType");
        break;
    case OSC_ARGS_BAD_MOTOR_ID:
        if (reportErrors)
            sendTwoData("/error/command", commandErrorText[ERROR_MOTORID_NOTMATCH], badMotorId);
        break;
    }
    return false;
}

void initOscRouter()
{
    oscRouter.begin(oscRoutes, OSC_ROUTE_COUNT, oscRouteIndex, validateOscArgs);
}

bool OSCMsgReceive()
{
    bool bMsgHasError = false;
    int size;
    if ((size = Udp.parsePacket()) > 0)
    {
        PROBE_HIGH(1);
        sendBackoffOnReceive(Udp.remoteIP()); // before any reply
        if (size > OSC_RX_BUFFER_SIZE)
        {
            // Too large: the rest of the packet is discarded by the next parsePacket().
            bMsgHasError = true;
        }
        else
        {
            int len = Udp.read(oscRxBuffer, size);
            if (len < 0)
                len = 0;
            oscSetRxGuards(oscRxBuffer, len);
            msgIN.decode(oscRxBuffer, len);
            bMsgHasError = msgIN.hasError();
        }
        if (!bMsgHasError)
        {
            const OscDispatchResult result = oscRouter.dispatch(msgIN);
            turnOnRXL();
            if (result == OSC_DISPATCH_SLOW)
            {
                perfStats.oscSlowPathCount++;
            }
            else if (result == OSC_DISPATCH_NOT_MATCHED)
            {
                perfStats.oscNotMatchedCount++;
                if (reportErrors)
                    sendOneDatum("/error/osc", "MessageNotMatch");
            }
        }
        else if (reportErrors)
        {
            sendOneDatum("/error/osc", "OscSyntaxError");
        }
        arpRxOnReceive(Udp.remoteIP()); // after the handler: destIp as set by /setDestIp
        PROBE_LOW(1);
        return true;
    }
    return false;
}

bool isCorrectMotorId(uint8_t motorID)
{
    bool bCorrectId = (MOTOR_ID_FIRST <= motorID) && (motorID <= MOTOR_ID_LAST);
    if ((!bCorrectId) && (motorID != MOTOR_ID_ALL))
    {
        sendCommandError(motorID, ERROR_MOTORID_NOTMATCH);
    }
    return bCorrectId;
}

// For a command with a value to check (the arguments are present, see validateOscArgs()):
// true if it may be applied. A motor ID that is neither a motor nor 255 gets
// MotorIdNotMatch, a value out of range CommandIgnored (as /setBrakeTransitionDuration).
static bool acceptTarget(uint8_t motorID, bool valueValid)
{
    if (!isCorrectMotorId(motorID) && motorID != MOTOR_ID_ALL)
        return false;
    if (!valueValid)
    {
        sendCommandError(motorID, ERROR_COMMAND_IGNORED);
        return false;
    }
    return true;
}

bool checkGoToDirection(uint8_t motorId, int32_t targetPos)
{
    int32_t diff = stepper[motorId].getPos() - targetPos;
    bool dir = diff > 0;
    bool bLongPath = ((uint32_t)abs(diff)) >> 11 > 0;
    if (bLongPath)
    {
        dir = !dir;
    }
    return dir;
}

#pragma region config_commands_osc_listener
// The reply /destIp is sent once the MAC address of destIp is resolved, when this
// /setDestIp starts a resolution (another address, or after a gap of 1.5 s in the packets
// from destIp: a new device may have the address, arpResolve.h); otherwise at once.
void setDestIp(OscMessage &msg)
{
    bool bIpUpdated = false;
    for (auto i = 0; i < 4; i++)
    {
        bIpUpdated |= (destIp[i] != Udp.remoteIP()[i]);
    }
    setDestIpReceived(Udp.remoteIP(), isDestIpSet, bIpUpdated);
}

void getVersion(OscMessage &msg)
{
    String version = String(FIRMWARE_NAME) + String(" ") + String(firmwareVersion[0]) + String(".") + String(firmwareVersion[1]) + String(".") + String(firmwareVersion[2]) + String(" ") + String(COMPILE_DATE) + String(" ") + String(COMPILE_TIME);
    sendOneDatum("/version", version.c_str());
}

void getConfigName(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/configName");
    newMes.add(configName.c_str()).add((int32_t)sdInitializeSucceeded).add((int32_t)configFileOpenSucceeded).add((int32_t)configFileParseSucceeded).add(configFilename);
    sendOscMessage(newMes, destIp, outPort);
}

void setConfigName(OscMessage &msg)
{
    if (msg.isString(0))
    {
        int len = msg.getDataLength(0);
        char buf[len + 1];
        msg.getString(0, buf, len + 1);
        configName = buf;
    }
}

void getConfigRegister(uint8_t deviceId)
{
    sendTwoData("/configRegister", deviceId + MOTOR_ID_FIRST, stepper[deviceId].getParam(CONFIG));
}
void getConfigRegister(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getConfigRegister(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getConfigRegister(i);
        }
    }
}

// reset the motor driver chip and setup it
// Not while the brake is released or in transition: the driver would go HiZ.
static void resetMotorDriverChecked(uint8_t motorId)
{
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId] && !brakeCommand(motorId, BRAKE_CMD_RESET_DRIVER).execute)
        return; // CommandIgnored
#endif
    cancelPendingMotion(motorId);
    resetMotorDriver(motorId + MOTOR_ID_FIRST);
}
void resetMotorDriver(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        resetMotorDriverChecked(motorID - MOTOR_ID_FIRST);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            resetMotorDriverChecked(i);
        }
    }
}

void getAdcVal(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/adcVal", motorID, stepper[motorID - MOTOR_ID_FIRST].getParam(ADC_OUT));
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/adcVal", i + MOTOR_ID_FIRST, stepper[i].getParam(ADC_OUT));
        }
    }
}

// // simply send reset command to the driverchip via SPI
// void resetDev(OscMessage& msg) {
//     uint8_t motorID = getInt(msg, 0);
//     if(isCorrectMotorId(motorID)) {
//         motorID -= MOTOR_ID_FIRST;
//         stepper[motorID].hardHiZ();
//         stepper[motorID].resetDev();
//     }
//     else if (motorID == MOTOR_ID_ALL) {
//         for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
//             stepper[i].hardHiZ();
//             stepper[i].resetDev();
//         }
//     }
// }

void resetDevice(OscMessage &msg)
{
    safeSystemReset();
}

void saveConfig(OscMessage &msg)
{
    saveCurrentConfigToSd();
}

void reportError(OscMessage &msg)
{
    reportErrors = getBool(msg, 0);
}

void enableBusyReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportBUSY[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportBUSY[i] = bEnable;
        }
    }
}

void enableHizReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportHiZ[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportHiZ[i] = bEnable;
        }
    }
}
void enableHomeSwReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportHomeSwStatus[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportHomeSwStatus[i] = bEnable;
        }
    }
}
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void enableLimitSwReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportLimitSwStatus[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportLimitSwStatus[i] = bEnable;
        }
    }
}
#endif

void enableDirReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportDir[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportDir[i] = bEnable;
        }
    }
}
void enableMotorStatusReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportMotorStatus[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportMotorStatus[i] = bEnable;
        }
    }
}
void enableSwEventReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportSwEvn[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportSwEvn[i] = bEnable;
        }
    }
}
void enableUvloReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportUVLO[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportUVLO[i] = bEnable;
        }
    }
}
void enableThermalStatusReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportThermalStatus[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportThermalStatus[i] = bEnable;
        }
    }
}
void enableOverCurrentReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    // A detection pending while the report is disabled is not sent later.
    if (isCorrectMotorId(motorID))
    {
        reportOCD[motorID - MOTOR_ID_FIRST] = bEnable;
        if (!bEnable)
            latchedReportCancel(overCurrentReport[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportOCD[i] = bEnable;
            if (!bEnable)
                latchedReportCancel(overCurrentReport[i]);
        }
    }
}
void enableStallReport(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    // A detection pending while the report is disabled is not sent later.
    if (isCorrectMotorId(motorID))
    {
        reportStall[motorID - MOTOR_ID_FIRST] = bEnable;
        if (!bEnable)
            latchedReportCancel(stallReport[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportStall[i] = bEnable;
            if (!bEnable)
                latchedReportCancel(stallReport[i]);
        }
    }
}

void setPositionReportInterval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t interval = getInt(msg, 1);
    if (interval < 0)
        interval = 0; // 0 disables the report
    if (!acceptTarget(motorID, interval <= 0xFFFF))
        return;
    bool bEnable = interval > 0;
    if (isCorrectMotorId(motorID))
    {
        reportPosition[motorID - MOTOR_ID_FIRST] = bEnable;
        reportPositionInterval[motorID - MOTOR_ID_FIRST] = interval;
        if (bEnable)
            reportPositionList = false;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportPosition[i] = bEnable;
            reportPositionInterval[i] = interval;
        }
        if (bEnable)
        {
            reportPositionList = false;
            reportPositionListInterval = 0;
        }
    }
}
void setPositionListReportInterval(OscMessage &msg)
{
    int32_t interval = getInt(msg, 0);
    if (interval < 0)
        interval = 0; // 0 disables the report
    if (interval > 0xFFFF)
    {
        sendCommandError(MOTOR_ID_ALL, ERROR_COMMAND_IGNORED);
        return;
    }
    bool bEnable = interval > 0;
    reportPositionList = bEnable;
    reportPositionListInterval = interval;
    if (bEnable)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportPosition[i] = false;
            reportPositionInterval[i] = 0;
        }
    }
}

void getHomeSw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        getHomeSw(motorId);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getHomeSw(i);
        }
    }
}
void getHomeSw(uint8_t motorId)
{
    sendThreeInt("/homeSw", motorId + MOTOR_ID_FIRST, (int32_t)homeSwState[motorId], (int32_t)dir[motorId]);
}

#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void getLimitSw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        getLimitSw(motorId);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getLimitSw(i);
        }
    }
}
void getLimitSw(uint8_t motorId)
{
    sendThreeInt("/limitSw", motorId + MOTOR_ID_FIRST, (int32_t)limitSwState[motorId], (int32_t)dir[motorId]);
}
#endif

void getBusy(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/busy", motorID, (int32_t)busy[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/busy", i + MOTOR_ID_FIRST, (int32_t)busy[i]);
        }
    }
}
void getDir(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/dir", motorID, (int32_t)dir[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/dir", i + MOTOR_ID_FIRST, (int32_t)dir[i]);
        }
    }
}
void getHiZ(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/HiZ", motorID, (int32_t)HiZ[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/HiZ", i + MOTOR_ID_FIRST, (int32_t)HiZ[i]);
        }
    }
}
void getUvlo(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/uvlo", motorID, (int32_t)uvloStatus[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/uvlo", i + MOTOR_ID_FIRST, (int32_t)uvloStatus[i]);
        }
    }
}

void getMotorStatus(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/motorStatus", motorID, motorStatus[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/motorStatus", i + MOTOR_ID_FIRST, motorStatus[i]);
        }
    }
}

void getThermalStatus(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/thermalStatus", motorID, thermalStatus[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/thermalStatus", i + MOTOR_ID_FIRST, thermalStatus[i]);
        }
    }
}

void getStatus(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        // GetParam: GetStatus would clear the latched flags before checkStatus() reports them.
        sendTwoData("/status", motorID, (int)stepper[motorID - MOTOR_ID_FIRST].getParam(REG_STATUS));
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        uint32_t status[NUM_OF_MOTOR];
        readAllParams(REG_STATUS, status);
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/status", i + MOTOR_ID_FIRST, (int)status[i]);
        }
    }
}

void getStatusList(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint32_t status[NUM_OF_MOTOR];
    readAllParams(REG_STATUS, status); // GetParam: does not clear the latched flags
    OscWriter newMes("/statusList");
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        newMes.add((int32_t)status[i]);
    }

    sendOscMessage(newMes, destIp, outPort);
}

void setMicrostepMode(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t value = getInt(msg, 1);
    value = constrain(value, STEP_FS, STEP_FS_128); // 0-7
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].configStepMode(value);
        microStepMode[motorID - MOTOR_ID_FIRST] = value;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].configStepMode(value);
            microStepMode[i] = value;
        }
    }
}

void getMicrostepMode(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/microstepMode", motorID, stepper[motorID - MOTOR_ID_FIRST].getStepMode());
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/microstepMode", i + MOTOR_ID_FIRST, stepper[i].getStepMode());
        }
    }
}

void getHomeSwMode(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/homeSwMode", motorID, (int32_t)(stepper[motorID - MOTOR_ID_FIRST].getSwitchMode() > 0));
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/homeSwMode", i + MOTOR_ID_FIRST, (int32_t)(stepper[i].getSwitchMode() > 0));
        }
    }
}

void setHomeSwMode(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t value = (getBool(msg, 1)) ? SW_USER : SW_HARD_STOP;
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setSwitchMode(value);
        homeSwMode[motorID - MOTOR_ID_FIRST] = (value > 0);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setSwitchMode(value);
            homeSwMode[i] = (value > 0);
        }
    }
}
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void getLimitSwMode(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/limitSwMode", motorID, (int32_t)limitSwMode[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/limitSwMode", i + MOTOR_ID_FIRST, (int32_t)limitSwMode[i]);
        }
    }
}

void setLimitSwMode(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t switchMode = (getBool(msg, 1)) ? SW_USER : SW_HARD_STOP;
    if (isCorrectMotorId(motorID))
    {
        limitSwMode[motorID - MOTOR_ID_FIRST] = switchMode > 0;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            limitSwMode[i] = switchMode > 0;
        }
    }
}
#endif

void getHomingStatus(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        sendTwoData("/homingStatus", motorID + MOTOR_ID_FIRST, homingStatus[motorID]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/homingStatus", i + MOTOR_ID_FIRST, homingStatus[i]);
        }
    }
}

// STALL_TH register is 5bit in PowerSTEP01, 7bit in L6470
void setStallThreshold(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
#ifdef DRIVER_L6470
    uint8_t threshold = getInt(msg, 1) & 0x7F; // 7bit
#else
    uint8_t threshold = getInt(msg, 1) & 0x1F; // 5bit
#endif
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setParam(STALL_TH, threshold);
        getStallThreshold(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setParam(STALL_TH, threshold);
            getStallThreshold(i);
        }
    }
}
void getStallThreshold(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
#ifdef DRIVER_L6470
    uint8_t stall_th_raw = stepper[motorId].getParam(STALL_TH) & 0x7F;
    float threshold = (stall_th_raw + 1) * 31.25f;
#else
    uint8_t stall_th_raw = stepper[motorId].getParam(STALL_TH) & 0x1F;
    float threshold = (stall_th_raw + 1) * 312.5f;
#endif
    sendTwoData("/stallThreshold", motorId + MOTOR_ID_FIRST, threshold);
}
void getStallThreshold(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getStallThreshold(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getStallThreshold(i);
        }
    }
}
// OCD_TH register is 5bit in PowerSTEP01, 4bit in L6470
void setOverCurrentThreshold(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
#ifdef DRIVER_L6470
    uint8_t threshold = getInt(msg, 1) & 0xF; // 4bit
#else
    uint8_t threshold = getInt(msg, 1) & 0x1F; // 5bit
#endif
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setParam(OCD_TH, threshold);
        getOverCurrentThreshold(motorID);
        overCurrentThreshold[motorID - MOTOR_ID_FIRST] = threshold;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setParam(OCD_TH, threshold);
            overCurrentThreshold[i] = threshold;
            getOverCurrentThreshold(i + MOTOR_ID_FIRST);
        }
    }
}
void getOverCurrentThreshold(uint8_t motorID)
{
    if (!isDestIpSet)
    {
        return;
    }
#ifdef DRIVER_L6470
    uint8_t ocd_th_raw = stepper[motorID - MOTOR_ID_FIRST].getParam(OCD_TH) & 0xF;
    float threshold = (ocd_th_raw + 1) * 375.0f;
#else
    uint8_t ocd_th_raw = stepper[motorID - MOTOR_ID_FIRST].getParam(OCD_TH) & 0x1F;
    float threshold = (ocd_th_raw + 1) * 312.5f;
#endif
    sendTwoData("/overCurrentThreshold", motorID, threshold);
}
void getOverCurrentThreshold(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        getOverCurrentThreshold(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getOverCurrentThreshold(i + MOTOR_ID_FIRST);
        }
    }
}

void setLowSpeedOptimizeThreshold(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _lowSpdOptThreshold = getFloat(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        lowSpeedOptimizeThreshold[motorID] = _lowSpdOptThreshold;
        if (lowSpeedOptimizeEnable[motorID])
            stepper[motorID].setMinSpeed(_lowSpdOptThreshold);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            lowSpeedOptimizeThreshold[i] = _lowSpdOptThreshold;
            if (lowSpeedOptimizeEnable[i])
                stepper[i].setMinSpeed(_lowSpdOptThreshold);
        }
    }
}
void getLowSpeedOptimizeThreshold(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getLowSpeedOptimizeThreshold(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getLowSpeedOptimizeThreshold(i);
        }
    }
}
void getLowSpeedOptimizeThreshold(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
    bool optimizationEnabled = (stepper[motorId].getParam(MIN_SPEED) & (1 << 12)) > 0;
    OscWriter newMes("/lowSpeedOptimizeThreshold");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add(lowSpeedOptimizeThreshold[motorId]);
    newMes.add(optimizationEnabled);
    sendOscMessage(newMes, destIp, outPort);
}

void enableLowSpeedOptimize(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool state = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setLoSpdOpt(state);
        lowSpeedOptimizeEnable[motorID] = state;
        if (state)
        {
            stepper[motorID].setMinSpeed(lowSpeedOptimizeThreshold[motorID]);
        }
        else
        {
            stepper[motorID].setMinSpeed(minSpeed[motorID]);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setLoSpdOpt(state);
            lowSpeedOptimizeEnable[i] = state;
            if (state)
            {
                stepper[i].setMinSpeed(lowSpeedOptimizeThreshold[i]);
            }
            else
            {
                stepper[i].setMinSpeed(minSpeed[i]);
            }
        }
    }
}

void setBemfParam(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t intSpeed = constrain(getInt(msg, 1), 0, 0x3FFF);
    uint8_t
        stSlp = constrain(getInt(msg, 2), 0, 255),
        fnSlpAcc = constrain(getInt(msg, 3), 0, 255),
        fnSlpDec = constrain(getInt(msg, 4), 0, 255);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        intersectSpeed[motorID] = intSpeed;
        startSlope[motorID] = stSlp;
        accFinalSlope[motorID] = fnSlpAcc;
        decFinalSlope[motorID] = fnSlpDec;
        stepper[motorID].setParam(INT_SPD, intersectSpeed[motorID]);
        stepper[motorID].setParam(ST_SLP, startSlope[motorID]);
        stepper[motorID].setParam(FN_SLP_ACC, accFinalSlope[motorID]);
        stepper[motorID].setParam(FN_SLP_DEC, decFinalSlope[motorID]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            intersectSpeed[i] = intSpeed;
            startSlope[i] = stSlp;
            accFinalSlope[i] = fnSlpAcc;
            decFinalSlope[i] = fnSlpDec;
            stepper[i].setParam(INT_SPD, intersectSpeed[i]);
            stepper[i].setParam(ST_SLP, startSlope[i]);
            stepper[i].setParam(FN_SLP_ACC, accFinalSlope[i]);
            stepper[i].setParam(FN_SLP_DEC, decFinalSlope[i]);
        }
    }
}
void getBemfParam(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getBemfParam(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getBemfParam(i);
        }
    }
}
void getBemfParam(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/bemfParam");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((int32_t)stepper[motorId].getParam(INT_SPD));
    newMes.add((int32_t)stepper[motorId].getParam(ST_SLP));
    newMes.add((int32_t)stepper[motorId].getParam(FN_SLP_ACC));
    newMes.add((int32_t)stepper[motorId].getParam(FN_SLP_DEC));
    sendOscMessage(newMes, destIp, outPort);
}

#ifdef DRIVER_POWERSTEP01
void setDecayModeParam(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t
        tFast = constrain(getInt(msg, 1), 0, 255),
        tOnMin = constrain(getInt(msg, 2), 0, 255),
        tOffMin = constrain(getInt(msg, 3), 0, 255);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        fastDecaySetting[motorID] = tFast;
        minOnTime[motorID] = tOnMin;
        minOffTime[motorID] = tOffMin;
        stepper[motorID].setParam(T_FAST, fastDecaySetting[motorID]);
        stepper[motorID].setParam(TON_MIN, minOnTime[motorID]);
        stepper[motorID].setParam(TOFF_MIN, minOffTime[motorID]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            fastDecaySetting[i] = tFast;
            minOnTime[i] = tOnMin;
            minOffTime[i] = tOffMin;
            stepper[i].setParam(T_FAST, fastDecaySetting[i]);
            stepper[i].setParam(TON_MIN, minOnTime[i]);
            stepper[i].setParam(TOFF_MIN, minOffTime[i]);
        }
    }
}
void getDecayModeParam(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getDecayModeParam(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getDecayModeParam(i);
        }
    }
}
void getDecayModeParam(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/decayModeParam");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((int32_t)stepper[motorId].getParam(T_FAST));
    newMes.add((int32_t)stepper[motorId].getParam(TON_MIN));
    newMes.add((int32_t)stepper[motorId].getParam(TOFF_MIN));
    sendOscMessage(newMes, destIp, outPort);
}
#endif

#ifdef HAVE_BRAKE
// Changes only while the driver is HiZ, and disables only while the brake is engaged or
// free. Either way the brake output is engaged and the state is BRAKE_ENGAGED.
bool enableElectromagnetBrake(uint8_t motorId, bool bEnable)
{
    if (electromagnetBrakeEnable[motorId] == bEnable)
        return true;
    // GetParam does not clear the latched flags (GetStatus does).
    if (!(stepper[motorId].getParam(REG_STATUS) & STATUS_HIZ))
        return false;
    if (!bEnable && (brakeStatus[motorId] != BRAKE_ENGAGED) && (brakeStatus[motorId] != BRAKE_FREE))
        return false;
    setBrake(motorId, LOW);
    brakeStatus[motorId] = BRAKE_ENGAGED;
    PROBE_PULSE(6);
    brakeStatusFresh[motorId] = false;
    electromagnetBrakeEnable[motorId] = bEnable;
    updateFaultBrakeWait();
    return true;
}
void enableElectromagnetBrake(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (!enableElectromagnetBrake(motorID, bEnable))
            sendCommandError(motorID + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (!enableElectromagnetBrake(i, bEnable))
                sendCommandError(i + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
        }
    }
}

void setBrakeTransitionDuration(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t duration = getInt(msg, 1);
    const bool valid = (0 <= duration) && (duration <= 0xFFFF);
    if (isCorrectMotorId(motorID))
    {
        if (!valid)
        {
            sendCommandError(motorID, ERROR_COMMAND_IGNORED);
            return;
        }
        motorID -= MOTOR_ID_FIRST;
        brakeTransitionDuration[motorID] = duration;
        updateFaultBrakeWait();
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        if (!valid)
        {
            sendCommandError(motorID, ERROR_COMMAND_IGNORED);
            return;
        }
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            brakeTransitionDuration[i] = duration;
        }
        updateFaultBrakeWait();
    }
}

void getBrakeTransitionDuration(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        sendTwoData("/brakeTransitionDuration", motorID + MOTOR_ID_FIRST, brakeTransitionDuration[motorID]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/brakeTransitionDuration", i + MOTOR_ID_FIRST, brakeTransitionDuration[i]);
        }
    }
}

// State number of brakeControl.h, or -1 with the brake disabled.
static void sendBrakeStatus(uint8_t motorId)
{
    const int32_t state = electromagnetBrakeEnable[motorId] ? (int32_t)brakeStatus[motorId] : -1;
    sendTwoData("/brakeStatus", (int32_t)(motorId + MOTOR_ID_FIRST), state);
}
void getBrakeStatus(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendBrakeStatus(motorID - MOTOR_ID_FIRST);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendBrakeStatus(i);
        }
    }
}
#endif

void setGoUntilTimeout(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t timeout = getInt(msg, 1);
    if (!acceptTarget(motorID, timeout >= 0))
        return;
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        goUntilTimeout[motorID] = timeout;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            goUntilTimeout[i] = timeout;
        }
    }
}
void getGoUntilTimeout(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/goUntilTimeout", motorID, (int32_t)goUntilTimeout[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/goUntilTimeout", i + MOTOR_ID_FIRST, (int32_t)goUntilTimeout[i]);
        }
    }
}
void setReleaseSwTimeout(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t timeout = getInt(msg, 1);
    if (!acceptTarget(motorID, timeout >= 0))
        return;
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        releaseSwTimeout[motorID] = timeout;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            releaseSwTimeout[i] = timeout;
        }
    }
}
void getReleaseSwTimeout(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/releaseSwTimeout", motorID, (int32_t)releaseSwTimeout[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/releaseSwTimeout", i + MOTOR_ID_FIRST, (int32_t)releaseSwTimeout[i]);
        }
    }
}

void setHomingDirection(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool dir = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        homingDirection[motorID] = dir;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            homingDirection[i] = dir;
        }
    }
}
void getHomingDirection(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/homingDirection", motorID, (int32_t)homingDirection[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/homingDirection", i + MOTOR_ID_FIRST, (int32_t)homingDirection[i]);
        }
    }
}

void setHomingSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float spd = getFloat(msg, 1);
    spd = constrain(spd, 0.0, 15610.0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        homingSpeed[motorID] = spd;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            homingSpeed[i] = spd;
        }
    }
}
void getHomingSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/homingSpeed", motorID, homingSpeed[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/homingSpeed", i + MOTOR_ID_FIRST, homingSpeed[i]);
        }
    }
}

void setProhibitMotionOnHomeSw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool state = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        bProhibitMotionOnHomeSw[motorID] = state;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            bProhibitMotionOnHomeSw[i] = state;
        }
    }
}
void getProhibitMotionOnHomeSw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/prohibitMotionOnHomeSw", motorID, (int32_t)bProhibitMotionOnHomeSw[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/prohibitMotionOnHomeSw", i + MOTOR_ID_FIRST, (int32_t)bProhibitMotionOnHomeSw[i]);
        }
    }
}

#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void setProhibitMotionOnLimitSw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool state = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        bProhibitMotionOnLimitSw[motorID] = state;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            bProhibitMotionOnLimitSw[i] = state;
        }
    }
}
void getProhibitMotionOnLimitSw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/prohibitMotionOnLimitSw", motorID, (int32_t)bProhibitMotionOnLimitSw[motorID - MOTOR_ID_FIRST]);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/prohibitMotionOnLimitSw", i + MOTOR_ID_FIRST, (int32_t)bProhibitMotionOnLimitSw[i]);
        }
    }
}
#endif

#pragma endregion config_commands_osc_listener

#pragma region kval_commands_osc_listener

void setKval(uint8_t motorId, uint8_t hold, uint8_t run, uint8_t acc, uint8_t dec)
{
#ifdef DRIVER_POWERSTEP01
    if (!isCurrentMode[motorId])
#endif
    {
        stepper[motorId].setHoldKVAL(hold);
        stepper[motorId].setRunKVAL(run);
        stepper[motorId].setAccKVAL(acc);
        stepper[motorId].setDecKVAL(dec);
    }
    kvalHold[motorId] = hold;
    kvalRun[motorId] = run;
    kvalAcc[motorId] = acc;
    kvalDec[motorId] = dec;
}

void setKval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int hold = getInt(msg, 1);
    hold = constrain(hold, 0, 255);
    int run = getInt(msg, 2);
    run = constrain(run, 0, 255);
    int acc = getInt(msg, 3);
    acc = constrain(acc, 0, 255);
    int dec = getInt(msg, 4);
    dec = constrain(dec, 0, 255);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        setKval(motorID, hold, run, acc, dec);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            setKval(i, hold, run, acc, dec);
        }
    }
}

void setHoldKval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
#ifdef DRIVER_POWERSTEP01
        if (!isCurrentMode[motorId])
#endif
        {
            stepper[motorId].setHoldKVAL(kvalInput);
        }
        kvalHold[motorId] = kvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
#ifdef DRIVER_POWERSTEP01
            if (!isCurrentMode[i])
#endif
            {
                stepper[i].setHoldKVAL(kvalInput);
            }
            kvalHold[i] = kvalInput;
        }
    }
}
void setRunKval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);

    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
#ifdef DRIVER_POWERSTEP01
        if (!isCurrentMode[motorId])
#endif
        {
            stepper[motorId].setRunKVAL(kvalInput);
        }
        kvalRun[motorId] = kvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
#ifdef DRIVER_POWERSTEP01
            if (!isCurrentMode[i])
#endif
            {
                stepper[i].setRunKVAL(kvalInput);
            }
            kvalRun[i] = kvalInput;
        }
    }
}
void setAccKval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
#ifdef DRIVER_POWERSTEP01
        if (!isCurrentMode[motorId])
#endif
        {
            stepper[motorId].setAccKVAL(kvalInput);
        }
        kvalAcc[motorId] = kvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
#ifdef DRIVER_POWERSTEP01
            if (!isCurrentMode[i])
#endif
            {
                stepper[i].setAccKVAL(kvalInput);
            }
            kvalAcc[i] = kvalInput;
        }
    }
}
void setDecKval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
#ifdef DRIVER_POWERSTEP01
        if (!isCurrentMode[motorId])
#endif
        {
            stepper[motorId].setDecKVAL(kvalInput);
        }
        kvalDec[motorId] = kvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
#ifdef DRIVER_POWERSTEP01
            if (!isCurrentMode[i])
#endif
            {
                stepper[i].setDecKVAL(kvalInput);
            }
            kvalDec[i] = kvalInput;
        }
    }
}

void getKval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        getKval(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getKval(i + MOTOR_ID_FIRST);
        }
    }
}
void getKval(uint8_t motorID)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/kval");
    newMes.add((int32_t)motorID);
    uint8_t motorId = motorID - MOTOR_ID_FIRST;
    newMes.add((int32_t)stepper[motorId].getHoldKVAL());
    newMes.add((int32_t)stepper[motorId].getRunKVAL());
    newMes.add((int32_t)stepper[motorId].getAccKVAL());
    newMes.add((int32_t)stepper[motorId].getDecKVAL());
    sendOscMessage(newMes, destIp, outPort);
}
#pragma endregion kval_commands_osc_listener

#ifdef DRIVER_POWERSTEP01
#pragma region tval_commands_osc_listener
void setTval(uint8_t motorId, uint8_t hold, uint8_t run, uint8_t acc, uint8_t dec)
{
    if (isCurrentMode[motorId])
    {
        stepper[motorId].setHoldTVAL(hold);
        stepper[motorId].setRunTVAL(run);
        stepper[motorId].setAccTVAL(acc);
        stepper[motorId].setDecTVAL(dec);
    }
    tvalHold[motorId] = hold;
    tvalRun[motorId] = run;
    tvalAcc[motorId] = acc;
    tvalDec[motorId] = dec;
}
void setTval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int hold = constrain(getInt(msg, 1), 0, TVAL_LIMIT_VAL);
    int run = constrain(getInt(msg, 2), 0, TVAL_LIMIT_VAL);
    int acc = constrain(getInt(msg, 3), 0, TVAL_LIMIT_VAL);
    int dec = constrain(getInt(msg, 4), 0, TVAL_LIMIT_VAL);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        setTval(motorID, hold, run, acc, dec);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            setTval(i, hold, run, acc, dec);
        }
    }
}

void setHoldTval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t tvalInput = constrain(getInt(msg, 1), 0, TVAL_LIMIT_VAL);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (isCurrentMode[motorID])
        {
            stepper[motorID].setHoldTVAL(tvalInput);
        }
        tvalHold[motorID] = tvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (isCurrentMode[i])
            {
                stepper[i].setHoldTVAL(tvalInput);
            }
            tvalHold[i] = tvalInput;
        }
    }
}
void setRunTval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t tvalInput = constrain(getInt(msg, 1), 0, TVAL_LIMIT_VAL);

    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (isCurrentMode[motorID])
        {
            stepper[motorID].setRunTVAL(tvalInput);
        }
        tvalRun[motorID] = tvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (isCurrentMode[i])
            {
                stepper[i].setRunTVAL(tvalInput);
            }
            tvalRun[i] = tvalInput;
        }
    }
}
void setAccTval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t tvalInput = constrain(getInt(msg, 1), 0, TVAL_LIMIT_VAL);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (isCurrentMode[motorID])
        {
            stepper[motorID].setAccTVAL(tvalInput);
        }
        tvalAcc[motorID] = tvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (isCurrentMode[i])
            {
                stepper[i].setAccTVAL(tvalInput);
            }
            tvalAcc[i] = tvalInput;
        }
    }
}
void setDecTval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t tvalInput = constrain(getInt(msg, 1), 0, TVAL_LIMIT_VAL);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (isCurrentMode[motorID])
        {
            stepper[motorID].setDecTVAL(tvalInput);
        }
        tvalDec[motorID] = tvalInput;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (isCurrentMode[i])
            {
                stepper[i].setDecTVAL(tvalInput);
            }
            tvalDec[i] = tvalInput;
        }
    }
}

void getTval(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        getTval(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getTval(i + MOTOR_ID_FIRST);
        }
    }
}
void getTval(uint8_t motorID)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/tval");
    newMes.add((int32_t)motorID);
    uint8_t motorId = motorID - MOTOR_ID_FIRST;
    newMes.add((int32_t)stepper[motorId].getHoldTVAL());
    newMes.add((int32_t)stepper[motorId].getRunTVAL());
    newMes.add((int32_t)stepper[motorId].getAccTVAL());
    newMes.add((int32_t)stepper[motorId].getDecTVAL());
    sendOscMessage(newMes, destIp, outPort);
}
float TvalToCurrent(uint8_t tval)
{
    return (tval + 1) * 78.125f;
}
void getTval_mA(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/tval_mA");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add(TvalToCurrent(stepper[motorId].getHoldTVAL()));
    newMes.add(TvalToCurrent(stepper[motorId].getRunTVAL()));
    newMes.add(TvalToCurrent(stepper[motorId].getAccTVAL()));
    newMes.add(TvalToCurrent(stepper[motorId].getDecTVAL()));
    sendOscMessage(newMes, destIp, outPort);
}
void getTval_mA(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getTval_mA(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getTval_mA(i);
        }
    }
}
#pragma endregion tval_commands_osc_listener
#endif

#pragma region speed_commands_osc_listener

void setSpeedProfile(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _acc = getFloat(msg, 1);
    float _dec = getFloat(msg, 2);
    float _maxSpeed = getFloat(msg, 3);

    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setAcc(_acc);
        stepper[motorID].setDec(_dec);
        stepper[motorID].setMaxSpeed(_maxSpeed);
        acc[motorID] = _acc;
        dec[motorID] = _dec;
        maxSpeed[motorID] = _maxSpeed;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setAcc(_acc);
            stepper[i].setDec(_dec);
            stepper[i].setMaxSpeed(_maxSpeed);
            acc[i] = _acc;
            dec[i] = _dec;
            maxSpeed[i] = _maxSpeed;
        }
    }
}

void setMaxSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _maxSpeed = getFloat(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setMaxSpeed(_maxSpeed);
        maxSpeed[motorID] = _maxSpeed;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setMaxSpeed(_maxSpeed);
            maxSpeed[i] = _maxSpeed;
        }
    }
}

void setMinSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _minSpeed = getFloat(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        minSpeed[motorID] = _minSpeed;
        if (!lowSpeedOptimizeEnable[motorID])
        {
            stepper[motorID].setMinSpeed(_minSpeed);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            minSpeed[i] = _minSpeed;
            if (!lowSpeedOptimizeEnable[i])
            {
                stepper[i].setMinSpeed(_minSpeed);
            }
        }
    }
}

void getMinSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendMinSpeed(motorID - MOTOR_ID_FIRST);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendMinSpeed(i);
        }
    }
}
// While low speed optimization is enabled, the MIN_SPEED register holds the
// optimization threshold and the actual minimum speed is 0.
void sendMinSpeed(uint8_t motorId)
{
    float value = lowSpeedOptimizeEnable[motorId] ? 0.0f : stepper[motorId].getMinSpeed();
    sendTwoData("/minSpeed", (int32_t)motorId + MOTOR_ID_FIRST, value);
}

void setFullstepSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _fullStepSpeed = getFloat(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setFullSpeed(_fullStepSpeed);
        fullStepSpeed[motorID] = _fullStepSpeed;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setFullSpeed(_fullStepSpeed);
            fullStepSpeed[i] = _fullStepSpeed;
        }
    }
}
void getFullstepSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _fullStepSpeed;
    if (isCorrectMotorId(motorID))
    {
        _fullStepSpeed = stepper[motorID - MOTOR_ID_FIRST].getFullSpeed();
        sendTwoData("/fullstepSpeed", motorID, _fullStepSpeed);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            _fullStepSpeed = stepper[i].getFullSpeed();
            sendTwoData("/fullstepSpeed", i + MOTOR_ID_FIRST, _fullStepSpeed);
        }
    }
}
void setAcc(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _acc = getFloat(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setAcc(_acc);
        acc[motorID] = _acc;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setAcc(_acc);
            acc[i] = _acc;
        }
    }
}
void setDec(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float _dec = getFloat(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        stepper[motorID].setDec(_dec);
        dec[motorID] = _dec;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setDec(_dec);
            dec[i] = _dec;
        }
    }
}

void setSpeedProfileRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t _accRaw = getInt(msg, 1);
    uint16_t _decRaw = getInt(msg, 2);
    uint16_t _maxSpeedRaw = getInt(msg, 3);

    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setAccRaw(_accRaw);
        stepper[motorID - MOTOR_ID_FIRST].setDecRaw(_decRaw);
        stepper[motorID - MOTOR_ID_FIRST].setMaxSpeedRaw(_maxSpeedRaw);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setAccRaw(_accRaw);
            stepper[i].setDecRaw(_decRaw);
            stepper[i].setMaxSpeedRaw(_maxSpeedRaw);
        }
    }
}

void setMaxSpeedRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t t = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setMaxSpeedRaw(t);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setMaxSpeedRaw(t);
        }
    }
}
void setMinSpeedRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t t = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setMinSpeedRaw(t);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setMinSpeedRaw(t);
        }
    }
}

void setFullstepSpeedRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t t = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setFullSpeedRaw(t);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setFullSpeedRaw(t);
        }
    }
}

void setAccRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t t = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setAccRaw(t);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setAccRaw(t);
        }
    }
}
void setDecRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t t = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setDecRaw(t);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setDecRaw(t);
        }
    }
}

void getSpeed(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float s;
    if (isCorrectMotorId(motorID))
    {
        s = stepper[motorID - MOTOR_ID_FIRST].getSpeed();
        if (dir[motorID - MOTOR_ID_FIRST] == REV)
        {
            s *= -1.0f;
        }
        sendTwoData("/speed", motorID, s);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            s = stepper[i].getSpeed();
            if (dir[i] == REV)
            {
                s *= -1.0f;
            }
            sendTwoData("/speed", i + MOTOR_ID_FIRST, s);
        }
    }
}

void getSpeedProfile(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getSpeedProfile(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getSpeedProfile(i);
        }
    }
}
void getSpeedProfile(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/speedProfile");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((float)stepper[motorId].getAcc());
    newMes.add((float)stepper[motorId].getDec());
    newMes.add((float)stepper[motorId].getMaxSpeed());
    sendOscMessage(newMes, destIp, outPort);
}

void getSpeedProfileRaw(OscMessage &msg)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getSpeedProfileRaw(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getSpeedProfileRaw(i);
        }
    }
}
void getSpeedProfileRaw(uint8_t motorId)
{
    OscWriter newMes("/speedProfileRaw");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((int32_t)stepper[motorId].getAccRaw());
    newMes.add((int32_t)stepper[motorId].getDecRaw());
    newMes.add((int32_t)stepper[motorId].getMaxSpeedRaw());
    sendOscMessage(newMes, destIp, outPort);
}
#pragma endregion speed_commands_osc_listener

#pragma region operational_commands_osc_listener

void getPosition(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/position", motorID, stepper[motorID - MOTOR_ID_FIRST].getPos());
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        int32_t pos[NUM_OF_MOTOR];
        readAllPositions(pos);
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/position", i + MOTOR_ID_FIRST, pos[i]);
        }
    }
}

void getPositionList()
{
    int32_t pos[NUM_OF_MOTOR];
    readAllPositions(pos);
    sendAllData("/positionList", pos);
}
void getPositionList(OscMessage &msg)
{
    getPositionList();
}

void getMark(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/mark", motorID, stepper[motorID - MOTOR_ID_FIRST].getMark());
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/mark", i + MOTOR_ID_FIRST, stepper[i].getMark());
        }
    }
}

void run(uint8_t motorID, float absSpeed, bool dir)
{
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (checkMotionStartConditions(motorID, dir))
        {
            commitMotionStart(motorID);
            stepper[motorID].run(dir, absSpeed);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, dir))
            {
                commitMotionStart(i);
                stepper[i].run(dir, absSpeed);
            }
        }
    }
}
void run(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float stepsPerSec = getFloat(msg, 1);
    float absSpeed = fabsf(stepsPerSec);
    boolean dir = stepsPerSec > 0.0f;
    run(motorID, absSpeed, dir);
}

void combinedRun(OscMessage &msg)
{
    uint8_t motorID[2];
    motorID[0] = getInt(msg, 0);
    motorID[1] = getInt(msg, 1);
    float stepsPerSec = getFloat(msg, 2);
    float absSpeed = fabsf(stepsPerSec);
    boolean dir = stepsPerSec > 0.0f;
    run(motorID[0], absSpeed, dir);
    run(motorID[1], absSpeed, dir);
}
void runRaw(uint8_t motorID, bool dir, int32_t speed)
{
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (checkMotionStartConditions(motorID, dir))
        {
            commitMotionStart(motorID);
            stepper[motorID].runRaw(dir, speed);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, dir))
            {
                commitMotionStart(i);
                stepper[i].runRaw(dir, speed);
            }
        }
    }
}

void runRaw(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t speed = getInt(msg, 1);
    bool dir = speed > 0;
    speed = abs(speed);

    runRaw(motorID, dir, speed);
}
void combinedRunRaw(OscMessage &msg)
{
    uint8_t motorID[2];
    motorID[0] = getInt(msg, 0);
    motorID[1] = getInt(msg, 1);
    int32_t speed = getInt(msg, 2);
    bool dir = speed > 0;
    speed = abs(speed);
    runRaw(motorID[0], dir, speed);
    runRaw(motorID[1], dir, speed);
}

void move(uint8_t motorID, bool newDir, int32_t steps)
{
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        if (checkMotionStartConditions(motorID, newDir, false))
        {
            commitMotionStart(motorID);
            stepper[motorID].move(newDir, steps);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, newDir, false))
            {
                commitMotionStart(i);
                stepper[i].move(newDir, steps);
            }
        }
    }
}
void move(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t steps = getInt(msg, 1);
    bool newDir = steps > 0;
    steps = abs(steps);
    move(motorID, newDir, steps);
}

void combinedMove(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    int32_t steps = getInt(msg, 2);
    bool newDir = steps > 0;
    steps = abs(steps);
    move(motorID[0], newDir, steps);
    move(motorID[1], newDir, steps);

}
// Try to clear BUSY flag and return TRUE if succeeded.
// GOTO and GOTO_DIR is only executable when not in BUSY state
// So clear the BUSY with RUN command first.
bool clearBusyForGoTo(uint8_t motorId)
{
    if (busy[motorId])
    {
        int32_t rawSpeed = stepper[motorId].getParam(SPEED);
        stepper[motorId].runRaw(dir[motorId], rawSpeed);
        // GetParam: GetStatus would clear the latched flags before checkStatus() reports them.
        bool isReadyForGoTo = (stepper[motorId].getParam(REG_STATUS) & STATUS_BUSY) > 0;
        return isReadyForGoTo;
    }
    else
    {
        return true;
    }
}
void goTo(uint8_t motorID, int32_t pos)
{
    bool newDir = 0;
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        newDir = checkGoToDirection(motorId, pos);
        if (checkMotionStartConditions(motorId, newDir))
        {
            commitMotionStart(motorId);
            if (clearBusyForGoTo(motorId))
            {
                stepper[motorId].goTo(pos);
            } else {
                busyClearWaitStatus[motorId] = WAIT_FOR_GOTO;
                busyClearWaitGoToPosition[motorId] = pos;
            }
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            newDir = checkGoToDirection(i, pos);
            if (checkMotionStartConditions(i, newDir))
            {
                commitMotionStart(i);
                if (clearBusyForGoTo(i))
                {
                    stepper[i].goTo(pos);
                } else {
                    busyClearWaitStatus[i] = WAIT_FOR_GOTO;
                    busyClearWaitGoToPosition[i] = pos;
                }
            }
        }
    }
}
void goTo(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t pos = getInt(msg, 1);
    goTo(motorID, pos);
}
void combinedGoTo(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    int32_t pos = getInt(msg, 2);
    goTo(motorID[0], pos);
    goTo(motorID[1], pos);
}

void goToDir(uint8_t motorID, bool dir, int32_t pos)
{
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        if (checkMotionStartConditions(motorId, dir, false))
        {
            commitMotionStart(motorId);
            if (clearBusyForGoTo(motorId))
            {
                stepper[motorId].goToDir(dir, pos);
            } else {
                busyClearWaitStatus[motorId] = WAIT_FOR_GOTO_DIR;
                busyClearWaitGoToPosition[motorId] = pos;
                busyClearWaitGoToDir[motorId] = dir;
            }
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, dir, false))
            {
                commitMotionStart(i);
                if (clearBusyForGoTo(i))
                {
                    stepper[i].goToDir(dir, pos);
                } else {
                    busyClearWaitStatus[i] = WAIT_FOR_GOTO_DIR;
                    busyClearWaitGoToPosition[i] = pos;
                    busyClearWaitGoToDir[i] = dir;
                }
            }
        }
    }
}
void goToDir(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    boolean dir = getBool(msg, 1);
    int32_t pos = getInt(msg, 2);
    goToDir(motorID, dir, pos);
}
void combinedGoToDir(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    boolean dir = getBool(msg, 2);
    int32_t pos = getInt(msg, 3);
    goToDir(motorID[0], dir, pos);
    goToDir(motorID[1], dir, pos);
}
void homing(uint8_t motorId)
{
    if (bHoming[motorId])
    {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
    }
    else if (isServoMode[motorId])
    {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_IN_SERVO_MODE);
    }
    else
    {
        // goUntil() / releaseSw() set homingStatus when the command is sent.
        bool started;
        if (homeSwState[motorId])
            started = releaseSw(motorId, 0, !homingDirection[motorId]);
        else
            started = goUntil(motorId, 0, homingDirection[motorId], homingSpeed[motorId]);
        if (started)
        {
            bHoming[motorId] = true;
            sendTwoData("/homingStatus", motorId + MOTOR_ID_FIRST, homingStatus[motorId]);
        }
    }
}
void homing(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        homing(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            homing(i);
        }
    }
}

// Start conditions of /goUntil and /goUntilRaw. Commits the start when they are met.
static bool goUntilStartConditions(uint8_t motorId) {
    if (!brakeAllowsMotion(motorId))
        return false;
    if (isServoMode[motorId]) {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_IN_SERVO_MODE);
        return false;
    }
    if (homeSwState[motorId]) {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_HOMESW_ACTIVATING);
        return false;
    }
    commitMotionStart(motorId);
    return true;
}
bool goUntil(uint8_t motorId, bool action, bool dir, float stepsPerSec) {
    if (!goUntilStartConditions(motorId))
        return false;
    stepper[motorId].goUntil(action, dir, stepsPerSec);
    homingStatus[motorId] = HOMING_GOUNTIL;
    homingStartTime[motorId] = millis();
    return true;
}
void goUntil(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool action = getBool(msg, 1);
    float stepsPerSec = getFloat(msg, 2);
    bool dir = stepsPerSec > 0.0f;
    stepsPerSec = fabsf(stepsPerSec);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        goUntil(motorID, action, dir, stepsPerSec);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            goUntil(i, action, dir, stepsPerSec);
        }
    }
}
void goUntilRaw(OscMessage& msg) {
    uint8_t motorID = getInt(msg, 0);
    bool action = getBool(msg, 1);
    int32_t speed = getInt(msg, 2);
    bool dir = speed > 0;
    speed = abs(speed);
    if(isCorrectMotorId(motorID)) {
        motorID -= MOTOR_ID_FIRST;
        if (goUntilStartConditions(motorID)) {
            stepper[motorID].goUntilRaw(action, dir, speed);
            homingStatus[motorID] = HOMING_GOUNTIL;
            homingStartTime[motorID] = millis();
        }
    }
    else if (motorID == MOTOR_ID_ALL) {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
            if (goUntilStartConditions(i)) {
                stepper[i].goUntilRaw(action, dir, speed);
                homingStatus[i] = HOMING_GOUNTIL;
                homingStartTime[i] = millis();
            }
        }
    }
}

bool releaseSw(uint8_t motorId, bool action, bool dir) {
    if (!brakeAllowsMotion(motorId))
        return false;
    // The homing (homing(), and checkStatus() after the goUntil) never gets here in the
    // servo mode: homing() refuses it, and /enableServoMode 1 clears the homing state.
    if (isServoMode[motorId]) {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_IN_SERVO_MODE);
        return false;
    }
    commitMotionStart(motorId);
    stepper[motorId].releaseSw(action, dir);
    homingStatus[motorId] = HOMING_RELEASESW;
    homingStartTime[motorId] = millis();
    return true;
}
void releaseSw(OscMessage& msg) {
    uint8_t motorID = getInt(msg, 0);
    bool action = getBool(msg, 1);
    bool dir = getBool(msg, 2);
    if(isCorrectMotorId(motorID)) {
        motorID -= MOTOR_ID_FIRST;
        releaseSw(motorID, action, dir);
    }
    else if (motorID == MOTOR_ID_ALL) {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
            releaseSw(i, action, dir);
        }
    }
}

void goHome(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool dir;
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        dir = checkGoToDirection(motorId, 0);
        if (checkMotionStartConditions(motorId, dir, false))
        {
            commitMotionStart(motorId);
            if (clearBusyForGoTo(motorId))
            {
                stepper[motorId].goHome();
            } else {
                busyClearWaitStatus[motorId] = WAIT_FOR_GOHOME;
            }
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            dir = checkGoToDirection(i, 0);
            if (checkMotionStartConditions(i, dir, false))
            {
                commitMotionStart(i);
                if (clearBusyForGoTo(i))
                {
                    stepper[i].goHome();
                } else {
                    busyClearWaitStatus[i] = WAIT_FOR_GOHOME;
                }
            }
        }
    }
}
void goMark(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool dir;
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        dir = checkGoToDirection(motorId, stepper[motorId].getMark());
        if (checkMotionStartConditions(motorId, dir, false))
        {
            commitMotionStart(motorId);
            if (clearBusyForGoTo(motorId))
            {
                stepper[motorId].goMark();
            } else {
                busyClearWaitStatus[motorId] = WAIT_FOR_GOMARK;
            }
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            dir = checkGoToDirection(i, stepper[i].getMark());
            if (checkMotionStartConditions(i, dir, false))
            {
                commitMotionStart(i);
                if (clearBusyForGoTo(i))
                {
                    stepper[i].goMark();
                } else {
                    busyClearWaitStatus[i] = WAIT_FOR_GOMARK;
                }
            }
        }
    }
}
void setMark(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t newMark = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setMark(newMark);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setMark(newMark);
        }
    }
}
void setPosition(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t newPos = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setPos(newPos);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setPos(newPos);
        }
    }
}
void resetPos(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].resetPos();
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].resetPos();
        }
    }
}
void executeSoftStop(uint8_t motorId)
{
    cancelPendingMotion(motorId);
    if (brakeAllowsStop(motorId))
        stepper[motorId].softStop();
}
void softStop(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        executeSoftStop(motorID - MOTOR_ID_FIRST);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            executeSoftStop(i);
        }
    }
}
void softStop(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    softStop(motorID);
}
void combinedSoftStop(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    softStop(motorID[0]);
    softStop(motorID[1]);
}
void executeHardStop(uint8_t motorId)
{
    cancelPendingMotion(motorId);
    if (brakeAllowsStop(motorId))
        stepper[motorId].hardStop();
}
void hardStop(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        executeHardStop(motorID - MOTOR_ID_FIRST);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            executeHardStop(i);
        }
    }
}
void hardStop(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    hardStop(motorID);
}
void combinedHardStop(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    hardStop(motorID[0]);
    hardStop(motorID[1]);

}
void executeSoftHiZ(uint8_t motorId)
{
    cancelPendingMotion(motorId);
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId])
    {
        // Decelerates (SoftStop) and engages the brake once the poll reads the stop.
        brakeCommand(motorId, BRAKE_CMD_SOFT_HIZ);
    }
    else
#endif
    {
        stepper[motorId].softHiZ();
    }
}
void softHiZ(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        executeSoftHiZ(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            executeSoftHiZ(i);
        }
    }
}
void softHiZ(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    softHiZ(motorID);
}
void combinedSoftHiZ(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    softHiZ(motorID[0]);
    softHiZ(motorID[1]);

}
void executeHardHiZ(uint8_t motorId)
{
    cancelPendingMotion(motorId);
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId])
    {
        // Stops the motor (keeping it energized) before engaging the brake, so it does not
        // keep running during brakeTransitionDuration. Never sent to a HiZ motor.
        brakeCommand(motorId, BRAKE_CMD_HARD_HIZ);
    }
    else
#endif
    {
        stepper[motorId].hardHiZ();
    }
}
void hardHiZ(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        executeHardHiZ(motorID - MOTOR_ID_FIRST);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            executeHardHiZ(i);
        }
    }
}
void hardHiZ(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    hardHiZ(motorID);
}
void combinedHardHiZ(OscMessage &msg)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    hardHiZ(motorID[0]);
    hardHiZ(motorID[1]);
}

#ifdef HAVE_BRAKE
void activate(uint8_t motorId, bool state)
{
    if (electromagnetBrakeEnable[motorId])
    {
        if (state)
        {
            // applyBrakeAction() raises the holding current when the motor is energized.
            brakeCommand(motorId, BRAKE_CMD_ACTIVATE);
        }
        else
        {
            cancelPendingMotion(motorId);
            brakeCommand(motorId, BRAKE_CMD_DEACTIVATE);
        }
    }
    else
    {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
    }
}
void activate(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool state = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        activate(motorID, state);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            activate(i, state);
        }
    }
}
void free(uint8_t motorId)
{
    if (electromagnetBrakeEnable[motorId])
    {
        cancelPendingMotion(motorId);
        brakeCommand(motorId, BRAKE_CMD_FREE);
    }
    else
    {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
    }
}
void free(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        free(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            free(i);
        }
    }
}
#endif

#pragma endregion operational_commands_osc_listener

#pragma region servo_commands_osc_listener

void setTargetPosition(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t position = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        targetPosition[motorID - MOTOR_ID_FIRST] = position;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            targetPosition[i] = position;
        }
    }
}

void setTargetPositionList(OscMessage &msg)
{
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        targetPosition[i] = getInt(msg, i);
    }
}

void enableServoMode(uint8_t motorId, bool enable)
{
    if (enable)
    {
        if (!brakeAllowsMotion(motorId))
            return;
        cancelPendingMotion(motorId);
        commitMotionStart(motorId);
        targetPosition[motorId] = stepper[motorId].getPos();
        reportBUSY[motorId] = false;
        reportMotorStatus[motorId] = false;
        reportDir[motorId] = false;
        stepper[motorId].hardStop();
        resetServoState(motorId);
        isServoMode[motorId] = true;
    }
    else
    {
        cancelPendingMotion(motorId);
        if (brakeAllowsStop(motorId))
            stepper[motorId].softStop();
    }
}
void enableServoMode(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        enableServoMode(motorID, bEnable);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            enableServoMode(i, bEnable);
        }
    }
}

void setServoParam(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    float
        _kp = getFloat(msg, 1),
        _ki = getFloat(msg, 2),
        _kd = getFloat(msg, 3);
    if (!acceptTarget(motorID, (_kp >= 0.0f) && (_ki >= 0.0f) && (_kd >= 0.0f)))
        return;
    if (isCorrectMotorId(motorID))
    {
        kP[motorID - MOTOR_ID_FIRST] = _kp;
        kI[motorID - MOTOR_ID_FIRST] = _ki;
        kD[motorID - MOTOR_ID_FIRST] = _kd;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            kP[i] = _kp;
            kI[i] = _ki;
            kD[i] = _kd;
        }
    }
}

void getServoParam(uint8_t motorId)
{
    if (!isDestIpSet)
    {
        return;
    }
    OscWriter newMes("/servoParam");
    newMes.add(motorId + MOTOR_ID_FIRST);
    newMes.add(kP[motorId]).add(kI[motorId]).add(kD[motorId]);
    sendOscMessage(newMes, destIp, outPort);
}

void getServoParam(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        getServoParam(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            getServoParam(i);
        }
    }
}

#pragma endregion servo_commands_osc_listener

#pragma region PowerSTEP01_config_osc_listener
#ifdef DRIVER_POWERSTEP01
void setVoltageMode(uint8_t motorId)
{
    // stepper[motorId].hardHiZ();
    stepper[motorId].setPWMFreq(PWM_DIV_1, PWM_MUL_0_75);
    stepper[motorId].setHoldKVAL(kvalHold[motorId]);
    stepper[motorId].setRunKVAL(kvalRun[motorId]);
    stepper[motorId].setAccKVAL(kvalAcc[motorId]);
    stepper[motorId].setDecKVAL(kvalDec[motorId]);
    stepper[motorId].setParam(ST_SLP, startSlope[motorId]);
    stepper[motorId].setParam(FN_SLP_ACC, accFinalSlope[motorId]);
    stepper[motorId].setParam(FN_SLP_DEC, decFinalSlope[motorId]);
    stepper[motorId].setVoltageMode();
    isCurrentMode[motorId] = false;
}
void setVoltageMode(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        setVoltageMode(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            setVoltageMode(i);
        }
    }
}

void setCurrentMode(uint8_t motorId)
{
    // stepper[motorId].hardHiZ();
    stepper[motorId].setPredictiveControl(CONFIG_PRED_ENABLE);
    stepper[motorId].setSwitchingPeriod(5);
    if (stepper[motorId].getStepMode() > STEP_SEL_1_16)
    {
        stepper[motorId].configStepMode(STEP_SEL_1_16);
        microStepMode[motorId] = STEP_SEL_1_16;
    }
    stepper[motorId].setHoldTVAL(tvalHold[motorId]);
    stepper[motorId].setRunTVAL(tvalRun[motorId]);
    stepper[motorId].setAccTVAL(tvalAcc[motorId]);
    stepper[motorId].setDecTVAL(tvalDec[motorId]);
    stepper[motorId].setParam(T_FAST, fastDecaySetting[motorId]);
    stepper[motorId].setParam(TON_MIN, minOnTime[motorId]);
    stepper[motorId].setParam(TOFF_MIN, minOffTime[motorId]);
    stepper[motorId].setCurrentMode();
    isCurrentMode[motorId] = true;
}
void setCurrentMode(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        setCurrentMode(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            setCurrentMode(i);
        }
    }
}
#endif

void setElPos(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t newFullStep = getInt(msg, 1);
    int32_t newMicroStep = getInt(msg, 2);
    if (!acceptTarget(motorID, (0 <= newFullStep && newFullStep <= 3) && (0 <= newMicroStep && newMicroStep <= 127)))
        return;
    uint16_t newElPos = (newFullStep << 7) | newMicroStep;
    if (isCorrectMotorId(motorID))
    {
        stepper[motorID - MOTOR_ID_FIRST].setElPos(newElPos);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            stepper[i].setElPos(newElPos);
        }
    }
}
void getElPos(OscMessage &msg)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t elPos;
    uint8_t microStepPos, step;
    if (isCorrectMotorId(motorID))
    {
        elPos = stepper[motorID - MOTOR_ID_FIRST].getElPos();
        microStepPos = elPos & 0x7F;
        step = elPos >> 7;
        sendThreeInt("/elPos", motorID, step, microStepPos);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            elPos = stepper[i].getElPos();
            microStepPos = elPos & 0x7F;
            step = elPos >> 7;
            sendThreeInt("/elPos", i + MOTOR_ID_FIRST, step, microStepPos);
        }
    }
}
// #pragma endregion PowerSTEP01_config_osc_listener
