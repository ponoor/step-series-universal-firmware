// 
// 
// 

#include "utils.h"
#include "oscListeners.h"
#include "hilProbe.h"
#include "resetCause.h"
#include "perfStats.h"
#include "sendBackoff.h"
#include "arpRefresh.h"
#include "idOffset.h"
#include <stdarg.h>
#include <utility/w5100.h>

char* dbgPrint_(const __FlashStringHelper* fmt, ...)
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
    fastDigitalWriteLOW(PIN_LED_RXL); // turn on
    RXL_blinkStartTime = millis();
    rxLedEnabled = true;
}

void turnOnTXL() {
    fastDigitalWriteLOW(PIN_LED_TXL); // turn on
    TXL_blinkStartTime = millis();
    txLedEnabled = true;
}

void updateRxTxLed(uint32_t _currentTimeMillis) {
    if (rxLedEnabled)
    {
        if ((uint32_t)(_currentTimeMillis - RXL_blinkStartTime) >= RXL_TXL_BLINK_DURATION)
        {
            rxLedEnabled = false;
            fastDigitalWriteHIGH(PIN_LED_RXL); // turn off
        }
    }
    if (txLedEnabled)
    {
        if ((uint32_t)(_currentTimeMillis - TXL_blinkStartTime) >= RXL_TXL_BLINK_DURATION)
        {
            txLedEnabled = false;
            fastDigitalWriteHIGH(PIN_LED_TXL); // turn off
        }
    }
}

void sendWrongDataTypeError() {
    if (reportErrors)
        sendOneDatum(F("/error/osc"),"WrongDataType");
}

int getInt(OscMessage &msg, uint8_t offset) 
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

float getFloat(OscMessage &msg, uint8_t offset)
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

bool getBool(OscMessage &msg, uint8_t offset)
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
        sendTwoData(F("/error/command"), commandErrorText[errorNum], motorId);
        //if (SerialUSB)
            //dbgPrint("/error/command %s %d\n", commandErrorText[errorNum], motorId);
    }
}

void resetMotorDriver(uint8_t deviceID) {
    if (MOTOR_ID_FIRST <= deviceID && deviceID <= MOTOR_ID_LAST) {
        dbgPrint("ID: %d CONFIG: 0x%02X", deviceID, stepper[deviceID-1].getParam(CONFIG));
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
        dbgPrint(" -> 0x%02X\n", stepper[deviceID].getParam(CONFIG));
    }
}

// Watchdog early warning (interrupt, watchdog.h): the MCU resets right after this, and
// the reset puts the drivers in standby. Engage the brakes and wait for them. No driver
// SPI here (the interrupted code may be using it), so no hard stop.
void onWatchdogEarlyWarning() {
#ifdef HAVE_BRAKE
    PROBE_HIGH(2); // HIL: early warning
    engageAllBrakeOutputsFromIsr();
    uint16_t waitMs = 0;
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (electromagnetBrakeEnable[i] && (brakeStatus[i] != BRAKE_ENGAGED))
        {
            const uint16_t t = min(brakeTransitionDuration[i], (uint16_t)1000);
            if (t > waitMs)
                waitMs = t;
        }
    }
    // The watchdog resets at twice the early warning time (4 s), well after this.
    for (uint16_t ms = 0; ms < waitMs; ms++)
        delayMicroseconds(1000);
#endif
}

void safeSystemReset(uint16_t minWaitMs) {
    uint16_t waitMs = minWaitMs;
#ifdef HAVE_BRAKE
    // The MCU reset puts the drivers in standby (their STBY/RST input floats and is
    // pulled down inside), so engage the brakes and wait for them first.
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (electromagnetBrakeEnable[i] && (brakeStatus[i] != BRAKE_ENGAGED))
        {
            // GetParam does not clear the latched flags (GetStatus does).
            const uint16_t status = stepper[i].getParam(REG_STATUS);
            if (!(status & STATUS_HIZ) && (status & STATUS_MOT_STATUS))
                stepper[i].hardStop();
            setBrake(i, LOW);
            const uint16_t t = min(brakeTransitionDuration[i], (uint16_t)1000);
            if (t > waitMs)
                waitMs = t;
        }
    }
#endif
    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < waitMs)
        Watchdog.reset();
    PROBE_HIGH(0); // HIL: the last P0 edge marks the MCU reset (the pin floats then)
    systemReset();
}

// The W5500 defaults (RTR 200 ms, RCR 8) can block Udp.endPacket() for 1.6 s or more
// (not measured) when the destination does not answer ARP. With RTR 20 ms and RCR 2 one send to such a
// destination blocked 40.8 ms on the HIL bench (STEP800), i.e. RTR x RCR (measured; the
// earlier estimate RTR x (RCR + 1) = 60 ms was not reached). sendOscMessage() then pauses
// the sends to that destination (sendBackoff.h).
// Ethernet 2.0.2 setRetransmissionTimeout()/setRetransmissionCount() write the W5100
// register addresses (0x0017/0x0019), which are SIR/SIMR and the upper byte of RTR on
// the W5500. On the W5500, write RTR (0x0019-0x001A, 100 us units) and RCR (0x001B) directly.
static void setEthernetRetransmission(uint16_t timeoutMs, uint8_t count) {
    if (W5100.getChip() != 55) {
        Ethernet.setRetransmissionTimeout(timeoutMs);
        Ethernet.setRetransmissionCount(count);
        return;
    }
    constexpr uint16_t W5500_RTR = 0x0019;
    constexpr uint16_t W5500_RCR = 0x001B;
    const uint16_t rtr = timeoutMs * 10;
    const uint8_t rtrBytes[2] = {(uint8_t)(rtr >> 8), (uint8_t)(rtr & 0xFF)};
    uint8_t readBack[3];
    SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
    W5100.write(W5500_RTR, rtrBytes, 2);
    W5100.write(W5500_RCR, count);
    W5100.read(W5500_RTR, readBack, 3);
    SPI.endTransaction();
    dbgPrint("W5500 RTR:%u RCR:%u\n", (readBack[0] << 8) | readBack[1], readBack[2]);
}

void initEthernet() {
    Udp.stop();
#ifdef PIN_W5500_RESET
    pinMode(PIN_W5500_RESET, OUTPUT);
    digitalWrite(PIN_W5500_RESET, W5500_RESET_PIN_OPERATION_STATE);
    digitalWrite(PIN_W5500_RESET, !W5500_RESET_PIN_OPERATION_STATE);
    digitalWrite(ledPin, HIGH);
    delay(10); // This delay is necessary to refresh the network configration.
    digitalWrite(PIN_W5500_RESET, W5500_RESET_PIN_OPERATION_STATE);
    digitalWrite(ledPin, LOW);
    delay(1);
#endif
    if ( isMyIpAddId ) {
        uint8_t last;
        if (!addIdToIpLastByte(myIp_from_config[3], myId, last))
            dbgPrint("Warning: IP address out of range with ID %d, ID not added\n", myId);
        myIp[3] = last;
    }
    if ( isMacAddId ) mac[5] = mac_from_config[5] + myId; // wraps, still unique per ID
    if ( isOutPortAddId ) {
        uint16_t port;
        if (!addIdToPort(outPort_from_config, myId, port))
            dbgPrint("Warning: port out of range with ID %d, ID not added\n", myId);
        outPort = port;
    }
    Ethernet.init(PIN_W5500_CS);
    Ethernet.begin(mac, myIp, dns, gateway, subnet);
    // Must be set after Ethernet.begin(), which resets the chip.
    setEthernetRetransmission(W5500_RETRANSMISSION_TIMEOUT_MS, W5500_RETRANSMISSION_COUNT);
    Udp.begin(inPort);
    dbgPrint("New IP: %d.%d.%d.%d\n", myIp[0], myIp[1], myIp[2], myIp[3]);
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
    OscWriter newMes("/booted");
    newMes.add(myId).add(resetCause);
    sendOscMessage(newMes, broadcastIp, outPort);
    isWaitingSendBootMsg = false;
}
#ifdef HAVE_BRAKE
// The holding current must be able to hold the load once the brake is released:
// KVAL_HOLD (TVAL_HOLD in current mode) at least half of the RUN value.
static void ensureHoldingCurrent(uint8_t motorId) {
#ifdef DRIVER_POWERSTEP01
    if (isCurrentMode[motorId])
    {
        uint8_t t = tvalRun[motorId] >> 1;
        if (tvalHold[motorId] < t)
        {
            stepper[motorId].setHoldTVAL(t);
            tvalHold[motorId] = t;
        }
        return;
    }
#endif
    uint8_t t = kvalRun[motorId] >> 1;
    if (kvalHold[motorId] < t)
    {
        stepper[motorId].setHoldKVAL(t);
        kvalHold[motorId] = t;
    }
}

void applyBrakeAction(uint8_t motorId, const BrakeAction &action) {
    const BrakeState current = brakeStatus[motorId];
    const bool wasReleased = brakeOutputReleased(current);
    const bool willRelease = brakeOutputReleased(action.next);
    // From HiZ (or engaging) back to energized, before the driver is energized: also
    // after /free, where the brake is released already.
    if (((current == BRAKE_ENGAGED) || (current == BRAKE_ENGAGING) || (current == BRAKE_FREE)) &&
        ((action.next == BRAKE_RELEASING) || (action.next == BRAKE_RELEASED)))
        ensureHoldingCurrent(motorId);
    // Release the brake before the driver goes HiZ (/free), and stop the motor before
    // the brake engages (/hardHiZ).
    if (willRelease && !wasReleased)
        setBrake(motorId, HIGH);
    switch (action.driver) {
    case BRAKE_DRIVER_HARD_STOP:
        stepper[motorId].hardStop();
        break;
    case BRAKE_DRIVER_SOFT_STOP:
        stepper[motorId].softStop();
        break;
    case BRAKE_DRIVER_HARD_HIZ:
        stepper[motorId].hardHiZ();
        break;
    default:
        break;
    }
    if (!willRelease && wasReleased)
        setBrake(motorId, LOW);
    if (action.startTimer)
        brakeTransitionTrigTime[motorId] = millis();
    if ((action.driver != BRAKE_DRIVER_NONE) || (action.next != current))
        brakeStatusFresh[motorId] = false;
    if (action.next != current) {
        brakeStatus[motorId] = action.next;
        PROBE_PULSE(6);
    }
    if (action.cancelPending)
        cancelPendingMotion(motorId);
    switch (action.error) {
    case BRAKE_ERROR_ENGAGED:
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_BRAKE_ENGAGED);
        break;
    case BRAKE_ERROR_IGNORED:
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_COMMAND_IGNORED);
        break;
    case BRAKE_ERROR_RELEASE_FAILED:
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_BRAKE_RELEASE_FAILED);
        break;
    case BRAKE_ERROR_ENGAGED_ON_HIZ:
        sendCommandError(motorId + MOTOR_ID_FIRST, ERROR_BRAKE_ENGAGED_ON_HIZ);
        break;
    default:
        break;
    }
}

BrakeAction brakeCommand(uint8_t motorId, BrakeCommand command) {
    const BrakeAction action = brakeOnCommand(brakeStatus[motorId], command);
    applyBrakeAction(motorId, action);
    return action;
}

void updateBrake(uint32_t _currentTimeMillis) {
    static bool warned[NUM_OF_MOTOR] = {false};
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        if (!electromagnetBrakeEnable[i])
            continue;
        BrakePollInput in;
        in.hiz = HiZ[i];
        in.stopped = (motorStatus[i] == 0);
        in.statusFresh = brakeStatusFresh[i];
        in.elapsed = (uint32_t)(_currentTimeMillis - brakeTransitionTrigTime[i]) >= brakeTransitionDuration[i];
        const BrakeAction action = brakeOnPoll(brakeStatus[i], in);
        if (action.warn && !warned[i]) {
            warned[i] = true;
            dbgPrint("Warning: motor %d is %s while the brake state is %d\n", i + MOTOR_ID_FIRST,
                     in.hiz ? "HiZ" : "energized", brakeStatus[i]);
        }
        applyBrakeAction(i, action);
        if (homingAtStartupPending[i]) {
            if (brakeStatus[i] == BRAKE_RELEASED) {
                homingAtStartupPending[i] = false;
                homing(i);
            } else if (brakeStatus[i] == BRAKE_ENGAGED) {
                // The release failed (BrakeReleaseFailed has been sent).
                homingAtStartupPending[i] = false;
            }
        }
    }
}
#endif

bool brakeReleasedOrDisabled(uint8_t motorId) {
#ifdef HAVE_BRAKE
    return !electromagnetBrakeEnable[motorId] || (brakeStatus[motorId] == BRAKE_RELEASED);
#else
    return true;
#endif
}

bool brakeAllowsMotion(uint8_t motorId) {
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId]) {
        const BrakeAction action = brakeOnCommand(brakeStatus[motorId], BRAKE_CMD_MOTION);
        if (!action.execute) {
            applyBrakeAction(motorId, action); // BrakeEngaged
            return false;
        }
    }
#endif
    return true;
}

void commitMotionStart(uint8_t motorId) {
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId])
        brakeCommand(motorId, BRAKE_CMD_MOTION);
#endif
    // A goTo waiting for BUSY must not overwrite this command later.
    busyClearWaitStatus[motorId] = NONE;
}

bool brakeAllowsStop(uint8_t motorId) {
#ifdef HAVE_BRAKE
    if (electromagnetBrakeEnable[motorId])
        return brakeCommand(motorId, BRAKE_CMD_STOP).execute;
#endif
    return true;
}

void cancelPendingMotion(uint8_t motorId) {
    busyClearWaitStatus[motorId] = NONE;
    clearHomingStatus(motorId);
    isServoMode[motorId] = false;
#ifdef HAVE_BRAKE
    homingAtStartupPending[motorId] = false;
#endif
}

// When the stop commands are sent during a homing process,
// clear the homing status as HOMING_UNDEFINED.
// A /goUntil or /releaseSw sent directly (not /homing) is cleared without a report.
void clearHomingStatus(uint8_t motorId)
{
    if (bHoming[motorId])
    {
        homingStatus[motorId] = HOMING_UNDEFINED;
        bHoming[motorId] = false;
        sendTwoData("/homingStatus", motorId + MOTOR_ID_FIRST, homingStatus[motorId]);
    }
    else if ((homingStatus[motorId] == HOMING_GOUNTIL) || (homingStatus[motorId] == HOMING_RELEASESW))
    {
        homingStatus[motorId] = HOMING_UNDEFINED;
    }
}

bool checkMotionStartConditions(uint8_t motorId, bool dir, bool checkHomingStatus) {
    if (checkHomingStatus)
        clearHomingStatus(motorId);
    if (!brakeAllowsMotion(motorId)) {
        return false;
    }
    else if ( isServoMode[motorId] ) {
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

static SendBackoff sendBackoff;

static void printSendBackoffEvent(SendBackoffEvent event, const IPAddress &ip) {
    if (event == SEND_BACKOFF_PAUSED)
        dbgPrint("Warning: destination %d.%d.%d.%d unreachable, sends paused\n", ip[0], ip[1], ip[2], ip[3]);
    else if (event == SEND_BACKOFF_RESUMED)
        dbgPrint("Destination %d.%d.%d.%d reachable, sends resumed\n", ip[0], ip[1], ip[2], ip[3]);
}

void sendOscMessage(const OscWriter &msg, const IPAddress &ip, uint16_t port) {
    static uint8_t oscTxBuffer[OSC_TX_BUFFER_SIZE];
    static bool warned = false;
    const uint16_t len = msg.encode(oscTxBuffer, sizeof(oscTxBuffer));
    if (len == 0) {
        if (!warned) {
            warned = true;
            dbgPrint("Warning: OSC message too long, not sent\n");
        }
        return;
    }
    const uint32_t ipRaw = (uint32_t)ip;
    if (!sendBackoffShouldSend(sendBackoff, ipRaw, millis())) {
        perfStats.udpSendDroppedCount++;
        return;
    }
    PROBE_HIGH(5);
    Udp.beginPacket(ip, port);
    Udp.write(oscTxBuffer, len);
    const bool ok = Udp.endPacket(); // 0: timeout (no ARP reply), see sendBackoff.h
    PROBE_LOW(5);
    if (!ok)
        perfStats.udpSendFailedCount++;
    printSendBackoffEvent(sendBackoffSent(sendBackoff, ipRaw, ok, millis()), ip);
    turnOnTXL();
}

void sendBackoffOnReceive(const IPAddress &ip) {
    printSendBackoffEvent(sendBackoffReceived(sendBackoff, (uint32_t)ip), ip);
}

bool sendArpRefreshDummy() {
    // 1 byte, not 0: a SEND of an empty datagram is not defined for the W5500 (WIZnet's
    // ioLibrary refuses it), and socketSendUDP() would wait for SEND_OK or TIMEOUT
    // forever. Not through sendBackoff: the broadcast needs no ARP.
    static const uint8_t dummy = 0;
    PROBE_HIGH(5);
    Udp.beginPacket(broadcastIp, 9);
    Udp.write(&dummy, 1);
    const bool ok = Udp.endPacket();
    PROBE_LOW(5);
    return ok;
}

static ArpRefresh arpRefresh;

void arpRefreshOnReceive(const IPAddress &ip) {
    if (!arpRefreshReceived(arpRefresh, (uint32_t)ip, (uint32_t)destIp, isDestIpSet, millis()))
        return;
    perfStats.arpRefreshCount++;
    if (!sendArpRefreshDummy())
        perfStats.arpRefreshFailedCount++;
}

void sendThreeInt(const char *address, int32_t data1, int32_t data2, int32_t data3) {
    if (!isDestIpSet) { return; }
    OscWriter newMes(address);
    newMes.add(data1).add(data2).add(data3);
    sendOscMessage(newMes, destIp, outPort);
}

void sendAllData(const char *address, int32_t *data) {
    if (!isDestIpSet) { return; }
    OscWriter newMes(address);
    for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
    {
        newMes.add(data[i]);
    }
    sendOscMessage(newMes, destIp, outPort);
}
