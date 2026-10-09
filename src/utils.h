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

bool isBrakeDisEngaged(uint8_t motorId);
#ifdef HAVE_BRAKE
void updateBrake(uint32_t _currentTimeMillis);
#endif

void clearHomingStatus(uint8_t motorId);
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