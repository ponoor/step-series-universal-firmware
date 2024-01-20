// utils.h

#ifndef _UTILS_h
#define _UTILS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <OSCMessage.h>
#include <Ethernet.h>
#include <Adafruit_SleepyDog.h>
#include "globals.h"
#include "boardsSpecific.h"
const uint8_t testPin[4] = { 9, 8, A1, 1};
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

char* p_(const __FlashStringHelper* fmt, ...);
#define p(fmt, ...)	 p_(F(fmt), ##__VA_ARGS__)

void turnOnRXL();
void turnOnTXL();
void updateRxTxLed(uint32_t _currentTimeMillis);

int getInt(OSCMessage &msg, uint8_t offset);
float getFloat(OSCMessage &msg, uint8_t offset);
bool getBool(OSCMessage &msg, uint8_t offset);

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

template <class T>
void sendOneDatum(String address, T data) {
	if (!isDestIpSet) { return; }
    OSCMessage newMes(address.c_str());
    newMes.add(data);;
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}

template <class T, class U>
void sendTwoData(String address, T data1, U data2) {
	if (!isDestIpSet) { return; }
    OSCMessage newMes(address.c_str());
    newMes.add(data1).add(data2);
    Udp.beginPacket(destIp, outPort);
    newMes.send(Udp);
    Udp.endPacket();
    newMes.empty();
    turnOnTXL();
}
void sendThreeInt(String address, int32_t data1, int32_t data2, int32_t data3);
void sendAllData(String address, int32_t *data);
#endif