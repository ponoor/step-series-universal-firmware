// 
// 
// 

#include "utils.h"
#include "oscListeners.h"
#include <stdarg.h>

char* p_(const __FlashStringHelper* fmt, ...)
{
	char buf[128]; // resulting string limited to 128 chars
	va_list args;
	va_start(args, fmt);
#ifdef __AVR__
	vsnprintf_P(buf, sizeof(buf), (const char*)fmt, args); // progmem for AVR
#else
	vsnprintf(buf, sizeof(buf), (const char*)fmt, args); // for the rest of the world
#endif
	va_end(args);
	SerialUSB.print(buf);
	return buf;
}

void turnOnRXL() {
    digitalWrite(PIN_LED_RXL, LOW); // turn on
    RXL_blinkStartTime = millis();
    rxLedEnabled = true;
}

void turnOnTXL() {
    digitalWrite(PIN_LED_TXL, LOW); // turn on
    TXL_blinkStartTime = millis();
    txLedEnabled = true;
}

void updateRxTxLed(uint32_t _currentTimeMillis) {
    if (rxLedEnabled)
    {
        if ((uint32_t)(_currentTimeMillis - RXL_blinkStartTime) >= RXL_TXL_BLINK_DURATION)
        {
            rxLedEnabled = false;
            digitalWrite(PIN_LED_RXL, HIGH); // turn off
        }
    }
    if (txLedEnabled)
    {
        if ((uint32_t)(_currentTimeMillis - TXL_blinkStartTime) >= RXL_TXL_BLINK_DURATION)
        {
            txLedEnabled = false;
            digitalWrite(PIN_LED_TXL, HIGH); // turn off
        }
    }
}

void sendWrongDataTypeError() {
    if (reportErrors)
        sendOneDatum(F("/error/osc"),"WrongDataType");
}

int getInt(OSCMessage &msg, uint8_t offset) 
{
	int msgVal = 0;
	if (msg.isFloat(offset))
	{
		msgVal = (int) msg.getFloat(offset);
	}
	else if (msg.isInt(offset))
	{
		msgVal = msg.getInt(offset);
	}
    else {
        sendWrongDataTypeError();
    }
	return msgVal;
}

float getFloat(OSCMessage &msg, uint8_t offset)
{
	float msgVal = 0;
	if (msg.isFloat(offset))
	{
		msgVal = msg.getFloat(offset);
	}
	else if (msg.isInt(offset))
	{
		msgVal = msg.getInt(offset);
	}
    else {
        sendWrongDataTypeError();
    }
	return msgVal;
}

bool getBool(OSCMessage &msg, uint8_t offset)
{
    bool msgVal = 0;
	if (msg.isFloat(offset))
	{
		msgVal = msg.getFloat(offset) >= 1.0f;
	}
	else if (msg.isInt(offset))
	{
		msgVal = msg.getInt(offset) > 0;
	}
    else if (msg.isBoolean(offset)) 
    {
        msgVal = msg.getBoolean(offset);
    }
    else {
        sendWrongDataTypeError();
    }
	return msgVal;
    
}

void sendCommandError(uint8_t motorId, uint8_t errorNum)
{
    if (reportErrors) {
        sendTwoData(F("/error/command"), commandErrorText[errorNum].c_str(), motorId);
        //if (SerialUSB)
            //p("/error/command %s %d\n", commandErrorText[errorNum].c_str(), motorId);
    }
}

void resetMotorDriver(uint8_t deviceID) {
    if (MOTOR_ID_FIRST <= deviceID && deviceID <= MOTOR_ID_LAST) {
        p("ID: %d CONFIG: 0x%02X", deviceID, stepper[deviceID-1].getParam(CONFIG));
        deviceID -= MOTOR_ID_FIRST;
        stepper[deviceID].resetDev();
        stepper[deviceID].hardHiZ(); // Required when HOME_SW is activated during reset L6470. PowerSTEP01 doesn't have this issue.
        stepper[deviceID].setSwitchMode(SW_USER);
        stepper[deviceID].configStepMode(microStepMode[deviceID]);
        stepper[deviceID].setMaxSpeed(maxSpeed[deviceID]);
        stepper[deviceID].setLoSpdOpt(lowSpeedOptimizeEnable[deviceID]);
        if (lowSpeedOptimizeEnable[deviceID]) {
            stepper[deviceID].setMinSpeed(lowSpeedOptimizeThreshold[deviceID]); // Low speed optimization threshold
        } else {
            stepper[deviceID].setMinSpeed(minSpeed[deviceID]);
        }
        stepper[deviceID].setFullSpeed(fullStepSpeed[deviceID]);
        stepper[deviceID].setAcc(acc[deviceID]);
        stepper[deviceID].setDec(dec[deviceID]);
        stepper[deviceID].setSlewRate(slewRate[deviceID]);
        stepper[deviceID].setPWMFreq(PWM_DIV_1, PWM_MUL_0_75);
        stepper[deviceID].setVoltageComp(VS_COMP_DISABLE);
        stepper[deviceID].setOCThreshold(overCurrentThreshold[deviceID]); // 5A for 0.1ohm shunt resistor
        stepper[deviceID].setOCShutdown(OC_SD_ENABLE);
        stepper[deviceID].setOscMode(DRIVER_OSC);
        
#ifdef DRIVER_POWERSTEP01
        if (isCurrentMode[deviceID]) {
            stepper[deviceID].setHoldTVAL(tvalHold[deviceID]);
            stepper[deviceID].setRunTVAL(tvalRun[deviceID]);
            stepper[deviceID].setAccTVAL(tvalAcc[deviceID]);
            stepper[deviceID].setDecTVAL(tvalDec[deviceID]);
            stepper[deviceID].setParam(T_FAST, fastDecaySetting[deviceID]);
            stepper[deviceID].setParam(TON_MIN, minOnTime[deviceID]);
            stepper[deviceID].setParam(TOFF_MIN, minOffTime[deviceID]);
            setCurrentMode(deviceID);
        } else 
#endif
        {
            stepper[deviceID].setRunKVAL(kvalRun[deviceID]);
            stepper[deviceID].setAccKVAL(kvalAcc[deviceID]);
            stepper[deviceID].setDecKVAL(kvalDec[deviceID]);
            stepper[deviceID].setHoldKVAL(kvalHold[deviceID]);
            stepper[deviceID].setParam(INT_SPD, intersectSpeed[deviceID]);
            stepper[deviceID].setParam(ST_SLP, startSlope[deviceID]);
            stepper[deviceID].setParam(FN_SLP_ACC, accFinalSlope[deviceID]);
            stepper[deviceID].setParam(FN_SLP_DEC, decFinalSlope[deviceID]);
        }
        stepper[deviceID].setParam(STALL_TH, stallThreshold[deviceID]);
        stepper[deviceID].setParam(ALARM_EN, 0xEF); // Enable alarms except ADC UVLO
        uint16_t swMode = homeSwMode[deviceID] ? SW_USER : SW_HARD_STOP;
        stepper[deviceID].setSwitchMode(swMode);//

        delay(1);
        stepper[deviceID].getStatus(); // clears error flags
        p(" -> 0x%02X\n", stepper[deviceID].getParam(CONFIG));
    }
}

void initEthernet() {
    Udp.stop();
#ifdef W5500_RESET_PIN
    pinMode(PIN_W5500_RESET, OUTPUT);
    digitalWrite(PIN_W5500_RESET, W5500_RESET_PIN_OPERATION_STATE);
    digitalWrite(PIN_W5500_RESET, !W5500_RESET_PIN_OPERATION_STATE);
    digitalWrite(ledPin, HIGH);
    delay(10); // This delay is necessary to refresh the network configration.
    digitalWrite(PIN_W5500_RESET, W5500_RESET_PIN_OPERATION_STATE);
    digitalWrite(ledPin, LOW);
    delay(1);
#endif
    if ( isMyIpAddId ) myIp[3] = myIp_from_config[3] + myId;
    if ( isMacAddId ) mac[5] = mac_from_config[5] + myId;
    if ( isOutPortAddId ) outPort = outPort_from_config + myId;
    Ethernet.init(PIN_W5500_CS);
    Ethernet.begin(mac, myIp, dns, gateway, subnet);
    Udp.begin(inPort);
    p("New IP: %d.%d.%d.%d\n", myIp[0], myIp[1], myIp[2], myIp[3]);
}

void sendBootMsg(uint32_t _currentTime) {
    static uint32_t linkedTime;
    if ( Ethernet.linkStatus() == LinkOFF ) {
        linkedTime = _currentTime;
        return; 
    }
    else {
        if ((uint32_t)(_currentTime - linkedTime) < BOOT_MSG_WAIT_TIME) return;
    }
    OSCMessage newMes("/booted");
    newMes.add(myId);
    Udp.beginPacket(broadcastIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
    isWaitingSendBootMsg = false;
}
bool isBrakeDisEngaged(uint8_t motorId) {
#ifdef HAVE_BRAKE
    bool state = electromagnetBrakeEnable[motorId] && (brakeStatus[motorId] != BRAKE_DISENGAGED);
    if (state) {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_BRAKE_ENGAGED);
    }
    return !state;
#else
    return true;
#endif
}
#ifdef HAVE_BRAKE
void updateBrake(uint32_t _currentTimeMillis) {
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (electromagnetBrakeEnable[i]) {
            if (brakeStatus[i] == BRAKE_DISENGAGE_WAITING) {
                if ((uint32_t)(_currentTimeMillis - brakeTransitionTrigTime[i]) >= brakeTransitionDuration[i]) {
                    setBrake(i, HIGH);
                    brakeStatus[i] = BRAKE_DISENGAGED;
                }
            } else if (brakeStatus[i] == BRAKE_MOTORHIZ_WAITING) {
                if ((uint32_t)(_currentTimeMillis - brakeTransitionTrigTime[i]) >= brakeTransitionDuration[i]) {
                    stepper[i].hardHiZ();
                    brakeStatus[i] = BRAKE_ENGAGED;
                }
            }
        }
    }
}
#endif

// When the stop commands are sent during a homing process, 
// clear the homing status as HOMING_UNDEFINED.
void clearHomingStatus(uint8_t motorId)
{
    if (bHoming[motorId])
    {
        homingStatus[motorId] = HOMING_UNDEFINED;
        bHoming[motorId] = false;
        sendTwoData("/homingStatus", motorId + MOTOR_ID_FIRST, homingStatus[motorId]);
    }
}

bool checkMotionStartConditions(uint8_t motorId, bool dir, bool checkHomingStatus) {
    if (checkHomingStatus)
        clearHomingStatus(motorId);
#ifdef HAVE_BRAKE
    if (!isBrakeDisEngaged(motorId)) {
        return false;
    }
    else 
#endif
    if ( isServoMode[motorId] ) {
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_IN_SERVO_MODE);
        return false;
    }
    else if (bProhibitMotionOnHomeSw[motorId] && (dir == homingDirection[motorId])) {
        if (homeSwState[motorId]) {
            sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_HOMESW_ACTIVATING);
            return false;
        }
    }
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
    else if (bProhibitMotionOnLimitSw[motorId] && (dir != homingDirection[motorId])) {
        if (limitSwState[motorId]) {
            sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_LIMITSW_ACTIVATING);
            return false;
        }
    }
#endif
    return true;
}

void sendThreeInt(String address, int32_t data1, int32_t data2, int32_t data3) {
    if (!isDestIpSet) { return; }
    OSCMessage newMes(address.c_str());
    newMes.add(data1).add(data2).add(data3);
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

void sendAllData(String address, int32_t *data) {
    if (!isDestIpSet) { return; }
    OSCMessage newMes(address.c_str());
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        newMes.add(data[i]);
    }
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}