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
#include "watchdog.h"
#include "globals.h"
#include "boardsSpecific.h"

void dbgPrint_(const __FlashStringHelper* fmt, ...);
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
// The longest brakeTransitionDuration (at most 1000 ms) of the motors with the brake
// enabled and not engaged: the wait before a reset.
uint16_t longestBrakeWaitMs();
// Sets faultBrakeWaitMs (faultReset.h) for the CPU fault handler. Call after every change
// of brakeStatus[], electromagnetBrakeEnable[] or brakeTransitionDuration[].
void updateFaultBrakeWait();
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
// Restart of the MCU (/resetDevice, USB serial 'R'): hard-stops the turning motors with
// the brake enabled, engages their brakes, waits for the longest brakeTransitionDuration
// of them (at most 1 s), then resets. The reset comes at least minWaitMs after the call;
// the brakes are engaged first in any case.
void safeSystemReset(uint16_t minWaitMs = 0);
void initEthernet();
void sendBootMsg(uint32_t _currentTime);

// Largest outgoing message: /configName with a name set by /setConfigName (fits in
// the 512-byte receive buffer) and the config filename.
constexpr uint16_t OSC_TX_BUFFER_SIZE = 640;

// Encodes msg into the static send buffer, sends it and lights TXL. A message that does not
// fit is not sent (warning on USB serial once). After a failed send (the destination does
// not answer ARP) the messages to that address are dropped for 1 s (sendBackoff.h).
void sendOscMessage(const OscWriter &msg, const IPAddress &ip, uint16_t port);
// A packet was received from ip: resumes the sends to it if they were paused.
void sendBackoffOnReceive(const IPAddress &ip);
// Sends one byte to 255.255.255.255:9 (discard), P5 high: the next send to destIp goes
// to another destination IP than the send before, so the W5500 resolves its MAC address
// again (arpRefresh.h). Returns the result of endPacket().
bool sendArpRefreshDummy();
// A packet was received from ip (before any reply): after a gap of 1.5 s or more in the
// packets from destIp, sends the dummy broadcast (arpRefresh.h).
void arpRefreshOnReceive(const IPAddress &ip);
// Reads Sn_DHAR of the UDP socket: the MAC address the last send went to. After a send to
// destIp it is the MAC address the W5500 resolved by ARP for destIp; after a broadcast
// (/booted, the ARP refresh dummy) and before the first send ff:ff:ff:ff:ff:ff (measured).
// Returns false and all zeros if the socket is not open. For the diagnosis only (s, T):
// one SPI transfer to the W5500, not to be called from loop().
bool readDestMac(uint8_t out[6]);

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