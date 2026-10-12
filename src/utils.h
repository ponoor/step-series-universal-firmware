// utils.h

#ifndef _UTILS_h
#define _UTILS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "oscParser.h"
#include "oscWriter.h"
#include <Ethernet.h>
#include <Adafruit_SleepyDog.h>
#include "globals.h"
#include "boardsSpecific.h"

char* dbgPrint_(const __FlashStringHelper* fmt, ...);
#define dbgPrint(fmt, ...)	 dbgPrint_(F(fmt), ##__VA_ARGS__)

inline __attribute__((always_inline)) void fastDigitalWriteHIGH(uint32_t ulPin)
{
	EPortType port = g_APinDescription[ulPin].ulPort;
	uint32_t pin = g_APinDescription[ulPin].ulPin;
	uint32_t pinMask = (1ul << pin);
	PORT->Group[port].OUTSET.reg = pinMask;
}

inline __attribute__((always_inline)) void fastDigitalWriteLOW(uint32_t ulPin)
{
	EPortType port = g_APinDescription[ulPin].ulPort;
	uint32_t pin = g_APinDescription[ulPin].ulPin;
	uint32_t pinMask = (1ul << pin);
	PORT->Group[port].OUTCLR.reg = pinMask;
}

void turnOnRXL();
void turnOnTXL();
void updateRxTxLed(uint32_t _currentTimeMillis);

int getInt(OscMessage &msg, uint8_t offset);
float getFloat(OscMessage &msg, uint8_t offset);
bool getBool(OscMessage &msg, uint8_t offset);

#ifdef HAVE_BRAKE
// Carries out a brake action: brake output, driver command, state, timer, error.
void applyBrakeAction(uint8_t motorId, const BrakeAction &action);
// brakeOnCommand() + applyBrakeAction(). Only for a motor with the brake enabled.
BrakeAction brakeCommand(uint8_t motorId, BrakeCommand command);
// Status poll of the brakes, after checkStatus(). Also starts the homing at startup.
void updateBrake(uint32_t _currentTimeMillis);
#endif
// true when the brake is disabled or released (the driver may be run).
bool brakeReleasedOrDisabled(uint8_t motorId);
// Motion commands: brakeAllowsMotion() checks (sends BrakeEngaged), then after all the
// other start conditions, commitMotionStart() just before the command is sent.
bool brakeAllowsMotion(uint8_t motorId);
void commitMotionStart(uint8_t motorId);
// Stop commands (/softStop, /hardStop, /enableServoMode 0): false if the stop command
// must not be sent (brake engaged or free: the driver is HiZ).
bool brakeAllowsStop(uint8_t motorId);
// Cancels the pending motion: goTo waiting for BUSY, homing, servo mode, homing at startup.
void cancelPendingMotion(uint8_t motorId);

void clearHomingStatus(uint8_t motorId);
// Start conditions of a motion command, including the brake. Does not change any state
// on success (except the homing status with checkHomingStatus); call commitMotionStart().
bool checkMotionStartConditions(uint8_t motorId, bool dir, bool checkHomingStatus = true);
void sendCommandError(uint8_t motorID, uint8_t errorNum);
void resetMotorDriver(uint8_t deviceID);
void initEthernet();
void sendBootMsg(uint32_t _currentTime);

// Largest outgoing message: /configName with a name set by /setConfigName (fits in
// the 512-byte receive buffer) and the config filename.
constexpr uint16_t OSC_TX_BUFFER_SIZE = 640;

// Encodes msg into the static send buffer, sends it and lights TXL. A message that does not
// fit is not sent (warning on USB serial once).
void sendOscMessage(const OscWriter &msg, const IPAddress &ip, uint16_t port);

template <class T>
void sendOneDatum(const char *address, T data) {
	if (!isDestIpSet) { return; }
    OscWriter newMes(address);
    newMes.add(data);
    sendOscMessage(newMes, destIp, outPort);
}
template <class T>
void sendOneDatum(const __FlashStringHelper *address, T data) {
    sendOneDatum(reinterpret_cast<const char *>(address), data);
}

template <class T, class U>
void sendTwoData(const char *address, T data1, U data2) {
	if (!isDestIpSet) { return; }
    OscWriter newMes(address);
    newMes.add(data1).add(data2);
    sendOscMessage(newMes, destIp, outPort);
}
template <class T, class U>
void sendTwoData(const __FlashStringHelper *address, T data1, U data2) {
    sendTwoData(reinterpret_cast<const char *>(address), data1, data2);
}
void sendThreeInt(const char *address, int32_t data1, int32_t data2, int32_t data3);
void sendAllData(const char *address, int32_t *data);
#endif