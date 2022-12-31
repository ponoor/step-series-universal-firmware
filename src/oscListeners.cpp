//
//
//

#include "oscListeners.h"
#include "utils.h"

void OSCMsgReceive()
{
    bool bMsgHasError = false, bMsgRouted = false;
    OSCMessage msgIN;
    int size;
    if ((size = Udp.parsePacket()) > 0)
    {
        while (size--)
            msgIN.fill(Udp.read());

        bMsgHasError = msgIN.hasError();
        if (!bMsgHasError)
        {
            // some possible frequent messages
            bMsgRouted |= msgIN.route("/setTargetPosition", setTargetPosition);
            bMsgRouted |= msgIN.route("/setTargetPositionList", setTargetPositionList);
            bMsgRouted |= msgIN.route("/getPosition", getPosition);
            bMsgRouted |= msgIN.route("/getPositionList", getPositionList);

            bMsgRouted |= msgIN.route("/getSpeed", getSpeed);
            bMsgRouted |= msgIN.route("/run", run);
            bMsgRouted |= msgIN.route("/runRaw", runRaw);

            // combined
            bMsgRouted |= msgIN.route("/combine/move", combinedMove);
            bMsgRouted |= msgIN.route("/combine/goTo", combinedGoTo);
            bMsgRouted |= msgIN.route("/combine/goToDir", combinedGoToDir);
            bMsgRouted |= msgIN.route("/combine/run", combinedRun);
            bMsgRouted |= msgIN.route("/combine/runRaw", combinedRunRaw);
            bMsgRouted |= msgIN.route("/combine/hardHiZ", combinedHardHiZ);
            bMsgRouted |= msgIN.route("/combine/hardStop", combinedHardStop);
            bMsgRouted |= msgIN.route("/combine/softHiZ", combinedSoftHiZ);
            bMsgRouted |= msgIN.route("/combine/softStop", combinedSoftStop);

            // motion
            bMsgRouted |= msgIN.route("/move", move);
            bMsgRouted |= msgIN.route("/goTo", goTo);
            bMsgRouted |= msgIN.route("/goToDir", goToDir);
            bMsgRouted |= msgIN.route("/goUntil", goUntil);
            bMsgRouted |= msgIN.route("/goUntilRaw", goUntilRaw);
            bMsgRouted |= msgIN.route("/releaseSw", releaseSw);
            bMsgRouted |= msgIN.route("/goHome", goHome);
            bMsgRouted |= msgIN.route("/goMark", goMark);
            bMsgRouted |= msgIN.route("/setMark", setMark);
            bMsgRouted |= msgIN.route("/getMark", getMark);
            bMsgRouted |= msgIN.route("/setPosition", setPosition);
            bMsgRouted |= msgIN.route("/resetPos", resetPos);
            bMsgRouted |= msgIN.route("/softStop", softStop);
            bMsgRouted |= msgIN.route("/hardStop", hardStop);
            bMsgRouted |= msgIN.route("/softHiZ", softHiZ);
            bMsgRouted |= msgIN.route("/hardHiZ", hardHiZ);
#ifdef HAVE_BRAKE
            bMsgRouted |= msgIN.route("/activate", activate);
            bMsgRouted |= msgIN.route("/free", free);
#endif
            bMsgRouted |= msgIN.route("/homing", homing);
            // servo mode
            bMsgRouted |= msgIN.route("/enableServoMode", enableServoMode);
            bMsgRouted |= msgIN.route("/setServoParam", setServoParam);
            bMsgRouted |= msgIN.route("/getServoParam", getServoParam);

            // speed
            bMsgRouted |= msgIN.route("/setSpeedProfile", setSpeedProfile);
            bMsgRouted |= msgIN.route("/setMaxSpeed", setMaxSpeed);
            bMsgRouted |= msgIN.route("/setMinSpeed", setMinSpeed);
            bMsgRouted |= msgIN.route("/getMinSpeed", getMinSpeed);
            bMsgRouted |= msgIN.route("/setFullstepSpeed", setFullstepSpeed);
            bMsgRouted |= msgIN.route("/getFullstepSpeed", getFullstepSpeed);
            bMsgRouted |= msgIN.route("/setAcc", setAcc);
            bMsgRouted |= msgIN.route("/setDec", setDec);
            bMsgRouted |= msgIN.route("/getSpeedProfile", getSpeedProfile);

            // Kval
            bMsgRouted |= msgIN.route("/setKval", setKval);
            bMsgRouted |= msgIN.route("/setAccKval", setAccKval);
            bMsgRouted |= msgIN.route("/setDecKval", setDecKval);
            bMsgRouted |= msgIN.route("/setRunKval", setRunKval);
            bMsgRouted |= msgIN.route("/setHoldKval", setHoldKval);
            bMsgRouted |= msgIN.route("/getKval", getKval);

            // TVAL
#ifdef DRIVER_POWERSTEP01
            bMsgRouted |= msgIN.route("/setTval", setTval);
            bMsgRouted |= msgIN.route("/setAccTval", setAccTval);
            bMsgRouted |= msgIN.route("/setDecTval", setDecTval);
            bMsgRouted |= msgIN.route("/setRunTval", setRunTval);
            bMsgRouted |= msgIN.route("/setHoldTval", setHoldTval);
            bMsgRouted |= msgIN.route("/getTval", getTval);
            bMsgRouted |= msgIN.route("/getTval_mA", getTval_mA);
#endif
            // config
            bMsgRouted |= msgIN.route("/setDestIp", setDestIp);
            bMsgRouted |= msgIN.route("/getVersion", getVersion);
            bMsgRouted |= msgIN.route("/getConfigName", getConfigName);
            bMsgRouted |= msgIN.route("/getConfigRegister", getConfigRegister);
            bMsgRouted |= msgIN.route("/getStatus", getStatus);
            bMsgRouted |= msgIN.route("/getStatusList", getStatusList);
            bMsgRouted |= msgIN.route("/reportError", reportError);
            bMsgRouted |= msgIN.route("/getHomeSw", getHomeSw);
            bMsgRouted |= msgIN.route("/getBusy", getBusy);
            bMsgRouted |= msgIN.route("/getDir", getDir);
            bMsgRouted |= msgIN.route("/getHiZ", getHiZ);
            bMsgRouted |= msgIN.route("/getUvlo", getUvlo);
            bMsgRouted |= msgIN.route("/getMotorStatus", getMotorStatus);
            bMsgRouted |= msgIN.route("/getThermalStatus", getThermalStatus);
            bMsgRouted |= msgIN.route("/resetMotorDriver", resetMotorDriver);
            bMsgRouted |= msgIN.route("/enableBusyReport", enableBusyReport);
            bMsgRouted |= msgIN.route("/enableHizReport", enableHizReport);
            bMsgRouted |= msgIN.route("/enableHomeSwReport", enableHomeSwReport);
            bMsgRouted |= msgIN.route("/enableDirReport", enableDirReport);
            bMsgRouted |= msgIN.route("/enableMotorStatusReport", enableMotorStatusReport);
            bMsgRouted |= msgIN.route("/enableSwEventReport", enableSwEventReport);
            bMsgRouted |= msgIN.route("/enableUvloReport", enableUvloReport);
            bMsgRouted |= msgIN.route("/enableThermalStatusReport", enableThermalStatusReport);
            bMsgRouted |= msgIN.route("/enableOverCurrentReport", enableOverCurrentReport);
            bMsgRouted |= msgIN.route("/enableStallReport", enableStallReport);
            bMsgRouted |= msgIN.route("/setPositionReportInterval", setPositionReportInterval);
            bMsgRouted |= msgIN.route("/setPositionListReportInterval", setPositionListReportInterval);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
            bMsgRouted |= msgIN.route("/getLimitSw", getLimitSw);
            bMsgRouted |= msgIN.route("/getLimitSwMode", getLimitSwMode);
            bMsgRouted |= msgIN.route("/setLimitSwMode", setLimitSwMode);
            bMsgRouted |= msgIN.route("/enableLimitSwReport", enableLimitSwReport);
#endif
#ifdef HAVE_BRAKE
            bMsgRouted |= msgIN.route("/enableElectromagnetBrake", enableElectromagnetBrake);
            bMsgRouted |= msgIN.route("/setBrakeTransitionDuration", setBrakeTransitionDuration);
            bMsgRouted |= msgIN.route("/getBrakeTransitionDuration", getBrakeTransitionDuration);
#endif
            bMsgRouted |= msgIN.route("/setMicrostepMode", setMicrostepMode);
            bMsgRouted |= msgIN.route("/getMicrostepMode", getMicrostepMode);
            bMsgRouted |= msgIN.route("/getHomeSwMode", getHomeSwMode);
            bMsgRouted |= msgIN.route("/setHomeSwMode", setHomeSwMode);
            bMsgRouted |= msgIN.route("/setStallThreshold", setStallThreshold);
            bMsgRouted |= msgIN.route("/getStallThreshold", getStallThreshold);
            bMsgRouted |= msgIN.route("/setOverCurrentThreshold", setOverCurrentThreshold);
            bMsgRouted |= msgIN.route("/getOverCurrentThreshold", getOverCurrentThreshold);
            bMsgRouted |= msgIN.route("/enableLowSpeedOptimize", enableLowSpeedOptimize);
            bMsgRouted |= msgIN.route("/setLowSpeedOptimizeThreshold", setLowSpeedOptimizeThreshold);
            bMsgRouted |= msgIN.route("/getLowSpeedOptimizeThreshold", getLowSpeedOptimizeThreshold);

            bMsgRouted |= msgIN.route("/setSpeedProfileRaw", setSpeedProfileRaw);
            bMsgRouted |= msgIN.route("/setMaxSpeedRaw", setMaxSpeedRaw);
            bMsgRouted |= msgIN.route("/setMinSpeedRaw", setMinSpeedRaw);
            bMsgRouted |= msgIN.route("/setFullstepSpeedRaw", setFullstepSpeedRaw);
            bMsgRouted |= msgIN.route("/setAccRaw", setAccRaw);
            bMsgRouted |= msgIN.route("/setDecRaw", setDecRaw);
            bMsgRouted |= msgIN.route("/getSpeedProfileRaw", getSpeedProfileRaw);

            bMsgRouted |= msgIN.route("/setBemfParam", setBemfParam);
            bMsgRouted |= msgIN.route("/getBemfParam", getBemfParam);
#ifdef DRIVER_POWERSTEP01
            bMsgRouted |= msgIN.route("/setVoltageMode", setVoltageMode);
            bMsgRouted |= msgIN.route("/setCurrentMode", setCurrentMode);
            bMsgRouted |= msgIN.route("/setDecayModeParam", setDecayModeParam);
            bMsgRouted |= msgIN.route("/getDecayModeParam", getDecayModeParam);
#endif
            bMsgRouted |= msgIN.route("/getAdcVal", getAdcVal);
            bMsgRouted |= msgIN.route("/setGoUntilTimeout", setGoUntilTimeout);
            bMsgRouted |= msgIN.route("/getGoUntilTimeout", getGoUntilTimeout);
            bMsgRouted |= msgIN.route("/setReleaseSwTimeout", setReleaseSwTimeout);
            bMsgRouted |= msgIN.route("/getReleaseSwTimeout", getReleaseSwTimeout);
            bMsgRouted |= msgIN.route("/setHomingDirection", setHomingDirection);
            bMsgRouted |= msgIN.route("/getHomingDirection", getHomingDirection);
            bMsgRouted |= msgIN.route("/setHomingSpeed", setHomingSpeed);
            bMsgRouted |= msgIN.route("/getHomingSpeed", getHomingSpeed);
            bMsgRouted |= msgIN.route("/getHomingStatus", getHomingStatus);
            bMsgRouted |= msgIN.route("/setProhibitMotionOnHomeSw", setProhibitMotionOnHomeSw);
            bMsgRouted |= msgIN.route("/getProhibitMotionOnHomeSw", getProhibitMotionOnHomeSw);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
            bMsgRouted |= msgIN.route("/setProhibitMotionOnLimitSw", setProhibitMotionOnLimitSw);
            bMsgRouted |= msgIN.route("/getProhibitMotionOnLimitSw", getProhibitMotionOnLimitSw);
#endif
            bMsgRouted |= msgIN.route("/getElPos", getElPos);
            bMsgRouted |= msgIN.route("/setElPos", setElPos);
            bMsgRouted |= msgIN.route("/resetDevice", resetDevice);
            turnOnRXL();
            if ((!bMsgRouted) && reportErrors)
            {
                sendOneDatum("/error/osc", "MessageNotMatch");
            }
        }
        else if (reportErrors)
        {
            sendOneDatum("/error/osc", "OscSyntaxError");
        }
    }
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
void setDestIp(OSCMessage &msg, int addrOffset)
{
    bool bIpUpdated = false;
    OSCMessage newMes("/destIp");
    for (auto i = 0; i < 4; i++)
    {
        bIpUpdated |= (destIp[i] != Udp.remoteIP()[i]);
        newMes.add(Udp.remoteIP()[i]);
    }
    destIp = Udp.remoteIP();
    newMes.add((int32_t)bIpUpdated);
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    isDestIpSet = true;
    turnOnTXL();
}

void getVersion(OSCMessage &msg, int addrOffset)
{
    String version = String(FIRMWARE_NAME) + String(" ") + String(firmwareVersion[0]) + String(".") + String(firmwareVersion[1]) + String(".") + String(firmwareVersion[2]) + String(" ") + String(COMPILE_DATE) + String(" ") + String(COMPILE_TIME);
    sendOneDatum("/version", version.c_str());
}

void getConfigName(OSCMessage &msg, int addrOffset)
{
    if (!isDestIpSet)
    {
        return;
    }
    OSCMessage newMes("/configName");
    newMes.add(configName.c_str()).add((int32_t)sdInitializeSucceeded).add((int32_t)configFileOpenSucceeded).add((int32_t)configFileParseSucceeded);
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

void getConfigRegister(uint8_t deviceId)
{
    sendTwoData("/configRegister", deviceId + MOTOR_ID_FIRST, stepper[deviceId].getParam(CONFIG));
}
void getConfigRegister(OSCMessage &msg, int addrOffset)
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
void resetMotorDriver(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        resetMotorDriver(motorID);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            resetMotorDriver(i + MOTOR_ID_FIRST);
        }
    }
}

void getAdcVal(OSCMessage &msg, int addrOffset)
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
// void resetDev(OSCMessage& msg, int addrOffset) {
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

void resetDevice(OSCMessage &msg, int addrOffset)
{
    void (*resetFunc)(void) = 0;
    resetFunc();
}

void reportError(OSCMessage &msg, int addrOffset)
{
    reportErrors = getBool(msg, 0);
}

void enableBusyReport(OSCMessage &msg, int addrOffset)
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

void enableHizReport(OSCMessage &msg, int addrOffset)
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
void enableHomeSwReport(OSCMessage &msg, int addrOffset)
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
void enableLimitSwReport(OSCMessage &msg, int addrOffset)
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

void enableDirReport(OSCMessage &msg, int addrOffset)
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
void enableMotorStatusReport(OSCMessage &msg, int addrOffset)
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
void enableSwEventReport(OSCMessage &msg, int addrOffset)
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
void enableUvloReport(OSCMessage &msg, int addrOffset)
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
void enableThermalStatusReport(OSCMessage &msg, int addrOffset)
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
void enableOverCurrentReport(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportOCD[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportOCD[i] = bEnable;
        }
    }
}
void enableStallReport(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        reportStall[motorID - MOTOR_ID_FIRST] = bEnable;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            reportStall[i] = bEnable;
        }
    }
}

void setPositionReportInterval(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    int16_t interval = getInt(msg, 1);
    if (interval < 0)
        interval = 0;
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
void setPositionListReportInterval(OSCMessage &msg, int addrOffset)
{
    int16_t interval = getInt(msg, 0);
    if (interval < 0)
        interval = 0;
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

void getHomeSw(OSCMessage &msg, int addrOffset)
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
void getLimitSw(OSCMessage &msg, int addrOffset)
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

void getBusy(OSCMessage &msg, int addrOffset)
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
void getDir(OSCMessage &msg, int addrOffset)
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
            sendTwoData("/dir", i + MOTOR_ID_FIRST, (int32_t)HiZ[i]);
        }
    }
}
void getHiZ(OSCMessage &msg, int addrOffset)
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
void getUvlo(OSCMessage &msg, int addrOffset)
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
            sendTwoData("/uvlo", i + 1, (int32_t)uvloStatus[i]);
        }
    }
}

void getMotorStatus(OSCMessage &msg, int addrOffset)
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
            sendTwoData("/motorStatus", i + 1, motorStatus[i]);
        }
    }
}

void getThermalStatus(OSCMessage &msg, int addrOffset)
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
            sendTwoData("/thermalStatus", i + 1, thermalStatus[i]);
        }
    }
}

void getStatus(OSCMessage &msg, int addrOffset)
{
    if (!isDestIpSet)
    {
        return;
    }
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/status", motorID, stepper[motorID - MOTOR_ID_FIRST].getStatus());
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/status", i + MOTOR_ID_FIRST, stepper[i].getStatus());
        }
    }
}

void getStatusList(OSCMessage &msg, int addrOffset)
{
    if (!isDestIpSet)
    {
        return;
    }
    OSCMessage newMes("/statusList");
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        newMes.add((int32_t)stepper[i].getStatus());
    }

    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

void setMicrostepMode(OSCMessage &msg, int addrOffset)
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

void getMicrostepMode(OSCMessage &msg, int addrOffset)
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

void getHomeSwMode(OSCMessage &msg, int addrOffset)
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

void setHomeSwMode(OSCMessage &msg, int addrOffset)
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
void getLimitSwMode(OSCMessage &msg, int addrOffset)
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

void setLimitSwMode(OSCMessage &msg, int addrOffset)
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

void getHomingStatus(OSCMessage &msg, int addrOffset)
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
void setStallThreshold(OSCMessage &msg, int addrOffset)
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
void getStallThreshold(OSCMessage &msg, int addrOffset)
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
void setOverCurrentThreshold(OSCMessage &msg, int addrOffset)
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
void getOverCurrentThreshold(OSCMessage &msg, int addrOffset)
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

void setLowSpeedOptimizeThreshold(OSCMessage &msg, int addrOffset)
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
void getLowSpeedOptimizeThreshold(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/lowSpeedOptimizeThreshold");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add(lowSpeedOptimizeThreshold[motorId]);
    newMes.add(optimizationEnabled);
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

void enableLowSpeedOptimize(OSCMessage &msg, int addrOffset)
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

void setBemfParam(OSCMessage &msg, int addrOffset)
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
void getBemfParam(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/bemfParam");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((int32_t)stepper[motorId].getParam(INT_SPD));
    newMes.add((int32_t)stepper[motorId].getParam(ST_SLP));
    newMes.add((int32_t)stepper[motorId].getParam(FN_SLP_ACC));
    newMes.add((int32_t)stepper[motorId].getParam(FN_SLP_DEC));
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

#ifdef DRIVER_POWERSTEP01
void setDecayModeParam(OSCMessage &msg, int addrOffset)
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
void getDecayModeParam(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/decayModeParam");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((int32_t)stepper[motorId].getParam(T_FAST));
    newMes.add((int32_t)stepper[motorId].getParam(TON_MIN));
    newMes.add((int32_t)stepper[motorId].getParam(TOFF_MIN));
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}
#endif

#ifdef HAVE_BRAKE
void enableElectromagnetBrake(uint8_t motorId, bool bEnable)
{
    electromagnetBrakeEnable[motorId] = bEnable;
    if (bEnable)
    {
        // pinMode(brakePin[motorId], OUTPUT);
    }
    else
    {
        setBrake(motorId, false);
    }
}
void enableElectromagnetBrake(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    bool bEnable = getBool(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        enableElectromagnetBrake(motorID, bEnable);
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            enableElectromagnetBrake(i, bEnable);
        }
    }
}

void setBrakeTransitionDuration(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    uint16_t duration = getInt(msg, 1);
    if (isCorrectMotorId(motorID))
    {
        motorID -= MOTOR_ID_FIRST;
        brakeTransitionDuration[motorID] = duration;
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            brakeTransitionDuration[i] = duration;
        }
    }
}

void getBrakeTransitionDuration(OSCMessage &msg, int addrOffset)
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
#endif

void setGoUntilTimeout(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    uint32_t timeout = getInt(msg, 1);
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
void getGoUntilTimeout(OSCMessage &msg, int addrOffset)
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
void setReleaseSwTimeout(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    uint32_t timeout = getInt(msg, 1);
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
void getReleaseSwTimeout(OSCMessage &msg, int addrOffset)
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

void setHomingDirection(OSCMessage &msg, int addrOffset)
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
void getHomingDirection(OSCMessage &msg, int addrOffset)
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

void setHomingSpeed(OSCMessage &msg, int addrOffset)
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
void getHomingSpeed(OSCMessage &msg, int addrOffset)
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

void setProhibitMotionOnHomeSw(OSCMessage &msg, int addrOffset)
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
void getProhibitMotionOnHomeSw(OSCMessage &msg, int addrOffset)
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
void setProhibitMotionOnLimitSw(OSCMessage &msg, int addrOffset)
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
void getProhibitMotionOnLimitSw(OSCMessage &msg, int addrOffset)
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

void setKval(OSCMessage &msg, int addrOffset)
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

void setHoldKval(OSCMessage &msg, int addrOffset)
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
void setRunKval(OSCMessage &msg, int addrOffset)
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
void setAccKval(OSCMessage &msg, int addrOffset)
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
void setDecKval(OSCMessage &msg, int addrOffset)
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

void getKval(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/kval");
    newMes.add((int32_t)motorID);
    uint8_t motorId = motorID - MOTOR_ID_FIRST;
    newMes.add((int32_t)stepper[motorId].getHoldKVAL());
    newMes.add((int32_t)stepper[motorId].getRunKVAL());
    newMes.add((int32_t)stepper[motorId].getAccKVAL());
    newMes.add((int32_t)stepper[motorId].getDecKVAL());
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
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
void setTval(OSCMessage &msg, int addrOffset)
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

void setHoldTval(OSCMessage &msg, int addrOffset)
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
void setRunTval(OSCMessage &msg, int addrOffset)
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
void setAccTval(OSCMessage &msg, int addrOffset)
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
void setDecTval(OSCMessage &msg, int addrOffset)
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

void getTval(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/tval");
    newMes.add((int32_t)motorID);
    uint8_t motorId = motorID - MOTOR_ID_FIRST;
    newMes.add((int32_t)stepper[motorId].getHoldTVAL());
    newMes.add((int32_t)stepper[motorId].getRunTVAL());
    newMes.add((int32_t)stepper[motorId].getAccTVAL());
    newMes.add((int32_t)stepper[motorId].getDecTVAL());
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
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
    OSCMessage newMes("/tval_mA");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add(TvalToCurrent(stepper[motorId].getHoldTVAL()));
    newMes.add(TvalToCurrent(stepper[motorId].getRunTVAL()));
    newMes.add(TvalToCurrent(stepper[motorId].getAccTVAL()));
    newMes.add(TvalToCurrent(stepper[motorId].getDecTVAL()));
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}
void getTval_mA(OSCMessage &msg, int addrOffset)
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

void setSpeedProfile(OSCMessage &msg, int addrOffset)
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

void setMaxSpeed(OSCMessage &msg, int addrOffset)
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

void setMinSpeed(OSCMessage &msg, int addrOffset)
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
            if (!lowSpeedOptimizeEnable[motorID])
            {
                stepper[i].setMinSpeed(_minSpeed);
            }
        }
    }
}

void getMinSpeed(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/minSpeed", motorID, stepper[motorID - MOTOR_ID_FIRST].getMinSpeed());
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/minSpeed", i + 1, minSpeed[i]);
        }
    }
}

void setFullstepSpeed(OSCMessage &msg, int addrOffset)
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
void getFullstepSpeed(OSCMessage &msg, int addrOffset)
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
            sendTwoData("/fullstepSpeed", i + 1, _fullStepSpeed);
        }
    }
}
void setAcc(OSCMessage &msg, int addrOffset)
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
void setDec(OSCMessage &msg, int addrOffset)
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

void setSpeedProfileRaw(OSCMessage &msg, int addrOffset)
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

void setMaxSpeedRaw(OSCMessage &msg, int addrOffset)
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
void setMinSpeedRaw(OSCMessage &msg, int addrOffset)
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

void setFullstepSpeedRaw(OSCMessage &msg, int addrOffset)
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

void setAccRaw(OSCMessage &msg, int addrOffset)
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
void setDecRaw(OSCMessage &msg, int addrOffset)
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

void getSpeed(OSCMessage &msg, int addrOffset)
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

void getSpeedProfile(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/speedProfile");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((float)stepper[motorId].getAcc());
    newMes.add((float)stepper[motorId].getDec());
    newMes.add((float)stepper[motorId].getMaxSpeed());
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

void getSpeedProfileRaw(OSCMessage &msg, int addrOffset)
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
    OSCMessage newMes("/speedProfileRaw");
    newMes.add((int32_t)motorId + MOTOR_ID_FIRST);
    newMes.add((int32_t)stepper[motorId].getAccRaw());
    newMes.add((int32_t)stepper[motorId].getDecRaw());
    newMes.add((int32_t)stepper[motorId].getMaxSpeedRaw());
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}
#pragma endregion speed_commands_osc_listener

#pragma region operational_commands_osc_listener

void getPosition(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    if (isCorrectMotorId(motorID))
    {
        sendTwoData("/position", motorID, stepper[motorID - MOTOR_ID_FIRST].getPos());
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            sendTwoData("/position", i + MOTOR_ID_FIRST, stepper[i].getPos());
        }
    }
}

void getPositionList()
{
    int32_t pos[NUM_OF_MOTOR];
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        pos[i] = stepper[i].getPos();
    }
    sendAllData("/positionList", pos);
}
void getPositionList(OSCMessage &msg, int addrOffset)
{
    getPositionList();
}

void getMark(OSCMessage &msg, int addrOffset)
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
            stepper[motorID].run(dir, absSpeed);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, dir))
            {
                stepper[i].run(dir, absSpeed);
            }
        }
    }
}
void run(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    float stepsPerSec = getFloat(msg, 1);
    float absSpeed = fabsf(stepsPerSec);
    boolean dir = stepsPerSec > 0.0f;
    run(motorID, absSpeed, dir);
}

void combinedRun(OSCMessage &msg, int addrOffset)
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
            stepper[motorID].runRaw(dir, speed);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, dir))
            {
                stepper[i].runRaw(dir, speed);
            }
        }
    }
}

void runRaw(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t speed = getInt(msg, 1);
    bool dir = speed > 0;
    speed = abs(speed);

    runRaw(motorID, dir, speed);
}
void combinedRunRaw(OSCMessage &msg, int addrOffset)
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
            stepper[motorID].move(newDir, steps);
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            if (checkMotionStartConditions(i, newDir, false))
            {
                stepper[i].move(newDir, steps);
            }
        }
    }
}
void move(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t steps = getInt(msg, 1);
    bool newDir = steps > 0;
    steps = abs(steps);
    move(motorID, newDir, steps);
}

void combinedMove(OSCMessage &msg, int addrOffset)
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
        bool isReadyForGoTo = (stepper[motorId].getStatus() & STATUS_BUSY) > 0;
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
void goTo(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    int32_t pos = getInt(msg, 1);
    goTo(motorID, pos);
}
void combinedGoTo(OSCMessage &msg, int addrOffset)
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
void goToDir(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    boolean dir = getBool(msg, 1);
    int32_t pos = getInt(msg, 2);
    goToDir(motorID, dir, pos);
}
void combinedGoToDir(OSCMessage &msg, int addrOffset)
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
        bHoming[motorId] = true;
        if (homeSwState[motorId])
        {
            releaseSw(motorId, 0, !homingDirection[motorId]);
            homingStatus[motorId] = HOMING_RELEASESW;
        }
        else
        {
            goUntil(motorId, 0, homingDirection[motorId], homingSpeed[motorId]);
            homingStatus[motorId] = HOMING_GOUNTIL;
        }
        sendTwoData("/homingStatus", motorId + MOTOR_ID_FIRST, homingStatus[motorId]);
    }
}
void homing(OSCMessage &msg, int addrOffset)
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

void goUntil(uint8_t motorId, bool action, bool dir, float stepsPerSec) {
    if (isBrakeDisEngaged(motorId)) {
        if (homeSwState[motorId]) {
            sendTwoData("/error/command", "HomeSwActivated", motorId+MOTOR_ID_FIRST);
        } else {
            stepper[motorId].goUntil(action, dir, stepsPerSec);
            homingStatus[motorId] = HOMING_GOUNTIL;
            homingStartTime[motorId] = millis();
        }
    }
}
void goUntil(OSCMessage &msg, int addrOffset)
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
void goUntilRaw(OSCMessage& msg, int addrOffset) {
    uint8_t motorID = getInt(msg, 0);
    bool action = getBool(msg, 1);
    int32_t speed = getInt(msg, 2);
    bool dir = speed > 0;
    speed = abs(speed);
    if(isCorrectMotorId(motorID)) {
        motorID -= MOTOR_ID_FIRST;
        if (isBrakeDisEngaged(motorID)) {
            if (homeSwState[motorID]) {
                sendTwoData("/error/command", "HomeSwActivated", motorID+MOTOR_ID_FIRST);
            } else {
                stepper[motorID].goUntilRaw(action, dir, speed);
                homingStatus[motorID] = HOMING_GOUNTIL;
                homingStartTime[motorID] = millis();
            }
        }
    }
    else if (motorID == MOTOR_ID_ALL) {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
            if (isBrakeDisEngaged(i)) {
                if (homeSwState[i]) {
                    sendTwoData("/error/command", "HomeSwActivated", i+MOTOR_ID_FIRST);
                } else {
                    stepper[i].goUntil(action, dir, speed);
                    homingStatus[i] = HOMING_GOUNTIL;
                    homingStartTime[i] = millis();
                }
            }
        }
    }
}

void releaseSw(uint8_t motorId, bool action, bool dir) {
    if (isBrakeDisEngaged(motorId)) {
        stepper[motorId].releaseSw(action, dir);
        homingStatus[motorId] = HOMING_RELEASESW;
        homingStartTime[motorId] = millis();
    }
}
void releaseSw(OSCMessage& msg, int addrOffset) {
    uint8_t motorID = getInt(msg, 0);
    uint8_t action = getInt(msg, 1);
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

void goHome(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    bool dir;
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        dir = checkGoToDirection(motorId, 0);
        if (checkMotionStartConditions(motorId, dir, false))
        {
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
void goMark(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    bool dir;
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        dir = checkGoToDirection(motorId, stepper[motorId].getMark());
        if (checkMotionStartConditions(motorId, dir, false))
        {
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
void setMark(OSCMessage &msg, int addrOffset)
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
void setPosition(OSCMessage &msg, int addrOffset)
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
void resetPos(OSCMessage &msg, int addrOffset)
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
void softStop(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        isServoMode[motorId] = false;
        clearHomingStatus(motorId);
        stepper[motorId].softStop();
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            isServoMode[i] = false;
            clearHomingStatus(i);
            stepper[i].softStop();
        }
    }
}
void softStop(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    softStop(motorID);
}
void combinedSoftStop(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    softStop(motorID[0]);
    softStop(motorID[1]);
}
void hardStop(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        isServoMode[motorId] = false;
        clearHomingStatus(motorId);
        stepper[motorId].hardStop();
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            isServoMode[i] = false;
            clearHomingStatus(i);
            stepper[i].hardStop();
        }
    }
}
void hardStop(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    hardStop(motorID);
}
void combinedHardStop(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    hardStop(motorID[0]);
    hardStop(motorID[1]);

}
void executeSoftHiZ(uint8_t motorId)
{
    isServoMode[motorId] = false;
    clearHomingStatus(motorId);
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId])
    {
        if (motorStatus[motorId] == 0)
        { // motor stopped
            activate(motorId, false);
        }
        else
        {
            stepper[motorId].softStop();
            bBrakeDecWaiting[motorId] = true;
        }
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
void softHiZ(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    softHiZ(motorID);
}
void combinedSoftHiZ(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID[2] = { getInt(msg, 0), getInt(msg, 1) };
    softHiZ(motorID[0]);
    softHiZ(motorID[1]);

}
void hardHiZ(uint8_t motorID)
{
    if (isCorrectMotorId(motorID))
    {
        uint8_t motorId = motorID - MOTOR_ID_FIRST;
        isServoMode[motorId] = false;
        clearHomingStatus(motorId);
        stepper[motorId].softStop();
#ifdef HAVE_BRAKE
        if (electromagnetBrakeEnable[motorId])
        {
            activate(motorId, false);
        }
        else
#endif
        {
            stepper[motorId].hardHiZ();
        }
    }
    else if (motorID == MOTOR_ID_ALL)
    {
        for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
        {
            isServoMode[i] = false;
            clearHomingStatus(i);
#ifdef HAVE_BRAKE
            if (electromagnetBrakeEnable[i])
            {
                activate(i, false);
            }
            else
#endif
            {
                stepper[i].hardHiZ();
            }
        }
    }
}
void hardHiZ(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    hardHiZ(motorID);
}
void combinedHardHiZ(OSCMessage &msg, int addrOffset)
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
        if (((brakeStatus[motorId] == BRAKE_ENGAGED) || (brakeStatus[motorId] == BRAKE_MOTORHIZ_WAITING)) && state)
        {
#ifdef DRIVER_POWERSTEP01
            if (!isCurrentMode[motorId])
#endif
            {
                digitalWrite(ledPin, !digitalRead(ledPin));
                uint8_t t = kvalRun[motorId] >> 1;
                if (kvalHold[motorId] < t)
                {
                    stepper[motorId].setHoldKVAL(t);
                    kvalHold[motorId] = t;
                }
            }
#ifdef DRIVER_POWERSTEP01
            else
            {
                uint8_t t = tvalRun[motorId] >> 1;
                if (tvalHold[motorId] < t)
                {
                    stepper[motorId].setHoldTVAL(t);
                    tvalHold[motorId] = t;
                }
            }
#endif
            stepper[motorId].hardStop();
            brakeStatus[motorId] = BRAKE_DISENGAGE_WAITING;
            brakeTransitionTrigTime[motorId] = millis();
        }
        else if ((brakeStatus[motorId] == BRAKE_DISENGAGED) || (brakeStatus[motorId] == BRAKE_DISENGAGE_WAITING))
        {
            if (state)
            {                                // from /free state
                stepper[motorId].hardStop(); // to activate the motor current
            }
            else
            {
                setBrake(motorId, LOW);
                brakeStatus[motorId] = BRAKE_MOTORHIZ_WAITING;
                isServoMode[motorId] = false;
                clearHomingStatus(motorId);
                brakeTransitionTrigTime[motorId] = millis();
            }
        }
    }
    else
    {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
    }
}
void activate(OSCMessage &msg, int addrOffset)
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
        setBrake(motorId, HIGH);
        stepper[motorId].hardHiZ();
        brakeStatus[motorId] = BRAKE_DISENGAGED;
        isServoMode[motorId] = false;
        clearHomingStatus(motorId);
    }
    else
    {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
    }
}
void free(OSCMessage &msg, int addrOffset)
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

void setTargetPosition(OSCMessage &msg, int addrOffset)
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

void setTargetPositionList(OSCMessage &msg, int addrOffset)
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
        targetPosition[motorId] = stepper[motorId].getPos();
        reportBUSY[motorId] = false;
        reportMotorStatus[motorId] = false;
        reportDir[motorId] = false;
        stepper[motorId].hardStop();
        clearHomingStatus(motorId);
    }
    else
    {
        stepper[motorId].softStop();
    }
    isServoMode[motorId] = enable;
}
void enableServoMode(OSCMessage &msg, int addrOffset)
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

void setServoParam(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    float
        _kp = getFloat(msg, 1),
        _ki = getFloat(msg, 2),
        _kd = getFloat(msg, 3);
    if (_kp <= 0.0f)
        _kp = 0.0f;
    if (_ki <= 0.0f)
        _ki = 0.0f;
    if (_kd <= 0.0f)
        _kd = 0.0f;
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
    OSCMessage newMes("/servoParam");
    newMes.add(motorId + MOTOR_ID_FIRST);
    newMes.add(kP[motorId]).add(kI[motorId]).add(kD[motorId]);
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

void getServoParam(OSCMessage &msg, int addrOffset)
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
void setVoltageMode(OSCMessage &msg, int addrOffset)
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
void setCurrentMode(OSCMessage &msg, int addrOffset)
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

void setElPos(OSCMessage &msg, int addrOffset)
{
    uint8_t motorID = getInt(msg, 0);
    uint8_t newFullStep = getInt(msg, 1);
    uint8_t newMicroStep = getInt(msg, 2);
    newFullStep = constrain(newFullStep, 0, 3);
    newMicroStep = constrain(newMicroStep, 0, 127);
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
void getElPos(OSCMessage &msg, int addrOffset)
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
