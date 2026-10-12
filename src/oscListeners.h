// oscListeners.h

#ifndef _OSCLISTENERS_h
#define _OSCLISTENERS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "globals.h"
#include "oscParser.h"
#include "boardsSpecific.h"
// 'motorID' : 1-4/1-8, 255
// 'motorId' : 0-3/0-7

void initOscRouter();
bool OSCMsgReceive(); // returns true if a packet was received
bool isCorrectMotorId(uint8_t motorID);

void activate(uint8_t motorId, bool state);
void free(uint8_t motorId);

// config_commands_osc_listener
void setDestIp(OscMessage& msg);
void getVersion(OscMessage& msg);
void getConfigName(OscMessage& msg);
void setConfigName(OscMessage& msg);
void getConfigRegister(uint8_t deviceId);
void getConfigRegister(OscMessage& msg);
void resetMotorDriver(OscMessage& msg);
void reportError(OscMessage& msg);
void getAdcVal(OscMessage& msg);
void resetDevice(OscMessage& msg);
void saveConfig(OscMessage& msg);
void enableBusyReport(OscMessage& msg);
void enableHizReport(OscMessage& msg);
void enableHomeSwReport(OscMessage& msg);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void enableLimitSwReport(OscMessage& msg);
#endif
void enableDirReport(OscMessage& msg);
void enableMotorStatusReport(OscMessage& msg);
void enableSwEventReport(OscMessage& msg);
void enableUvloReport(OscMessage& msg);
void enableThermalStatusReport(OscMessage& msg);
void enableOverCurrentReport(OscMessage& msg);
void enableStallReport(OscMessage& msg);
void setPositionReportInterval(OscMessage& msg);
void setPositionListReportInterval(OscMessage& msg);
void getHomeSw(OscMessage& msg);
void getHomeSw(uint8_t motorId);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void getLimitSw(OscMessage& msg);
void getLimitSw(uint8_t motorId);
#endif
void getBusy(OscMessage& msg);
void getDir(OscMessage& msg);
void getHiZ(OscMessage& msg);
void getUvlo(OscMessage& msg);
void getMotorStatus(OscMessage& msg);
void getThermalStatus(OscMessage& msg);
void getStatus(OscMessage& msg);
void getStatusList(OscMessage& msg);
void setMicrostepMode(OscMessage& msg);
void getMicrostepMode(OscMessage& msg);
void getHomeSwMode(OscMessage& msg);
void setHomeSwMode(OscMessage& msg);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void getLimitSwMode(OscMessage& msg);
void setLimitSwMode(OscMessage& msg);
#endif
void getHomingStatus(OscMessage& msg); 
// STALL_TH register is 5bit in PowerSTEP01, 7bit in L6470
void setStallThreshold(OscMessage& msg);
void getStallThreshold(uint8_t motorId);
void getStallThreshold(OscMessage& msg);
// OCD_TH register is 5bit in PowerSTEP01, 4bit in L6470
void setOverCurrentThreshold(OscMessage& msg);
void getOverCurrentThreshold(uint8_t motorId);
void getOverCurrentThreshold(OscMessage& msg);
void setLowSpeedOptimizeThreshold(OscMessage& msg);
void getLowSpeedOptimizeThreshold(OscMessage& msg);
void getLowSpeedOptimizeThreshold(uint8_t motorId);
void enableLowSpeedOptimize(OscMessage& msg);
void setBemfParam(OscMessage& msg);
void getBemfParam(OscMessage& msg);
void getBemfParam(uint8_t motorId);
#ifdef DRIVER_POWERSTEP01
void setDecayModeParam(OscMessage& msg);
void getDecayModeParam(OscMessage& msg);
void getDecayModeParam(uint8_t motorId);
#endif
#ifdef HAVE_BRAKE
void enableElectromagnetBrake(uint8_t motorId, bool bEnable);
void enableElectromagnetBrake(OscMessage& msg);
void setBrakeTransitionDuration(OscMessage& msg);
void getBrakeTransitionDuration(OscMessage& msg);
#endif
void setGoUntilTimeout(OscMessage& msg);
void getGoUntilTimeout(OscMessage& msg);
void setReleaseSwTimeout(OscMessage& msg);
void getReleaseSwTimeout(OscMessage& msg);
void setHomingDirection(OscMessage& msg);
void getHomingDirection(OscMessage& msg);
void setHomingSpeed(OscMessage& msg);
void getHomingSpeed(OscMessage& msg);
void setProhibitMotionOnHomeSw(OscMessage& msg);
void getProhibitMotionOnHomeSw(OscMessage& msg);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void setProhibitMotionOnLimitSw(OscMessage& msg);
void getProhibitMotionOnLimitSw(OscMessage& msg);
#endif
//  kval_commands_osc_listener
void setKval(OscMessage& msg);
void setHoldKval(OscMessage& msg);
void setRunKval(OscMessage& msg);
void setAccKval(OscMessage& msg);
void setDecKval(OscMessage& msg);
void getKval(OscMessage& msg);
void getKval(uint8_t motorId);

// tval_commands_osc_listener
#ifdef DRIVER_POWERSTEP01
void setTval(OscMessage& msg);
void setHoldTval(OscMessage& msg);
void setRunTval(OscMessage& msg);
void setAccTval(OscMessage& msg);
void setDecTval(OscMessage& msg);
void getTval(OscMessage& msg);
void getTval(uint8_t motorId);
float TvalToCurrent(uint8_t tval);
void getTval_mA(uint8_t motorId);
void getTval_mA(OscMessage& msg);
#endif

// speed_commands_osc_listener
void setSpeedProfile(OscMessage& msg);
void setMaxSpeed(OscMessage& msg);
void setMinSpeed(OscMessage& msg);
void getMinSpeed(OscMessage& msg);
void sendMinSpeed(uint8_t motorId);
void setFullstepSpeed(OscMessage& msg);
void getFullstepSpeed(OscMessage& msg);
void setAcc(OscMessage& msg);
void setDec(OscMessage& msg);
void setSpeedProfileRaw(OscMessage& msg);
void setMaxSpeedRaw(OscMessage& msg);
void setMinSpeedRaw(OscMessage& msg);
void setFullstepSpeedRaw(OscMessage& msg);
void setAccRaw(OscMessage& msg);
void setDecRaw(OscMessage& msg);
void getSpeed(OscMessage& msg);
void getSpeedProfile(OscMessage& msg);
void getSpeedProfile(uint8_t motorId);
void getSpeedProfileRaw(OscMessage& msg);
void getSpeedProfileRaw(uint8_t motorId);

// operational_commands_osc_listener
void getPosition(OscMessage& msg);
void getPositionList();
void getPositionList(OscMessage& msg);
void getMark(OscMessage& msg);
void run(OscMessage& msg);
void runRaw(OscMessage& msg);
void move(OscMessage& msg);
void goTo(OscMessage& msg);
void goToDir(OscMessage& msg);
void homing(uint8_t motorId);
void homing(OscMessage& msg);
bool goUntil(uint8_t motorId, bool action, bool dir, float stepsPerSec); // true if sent
void goUntil(OscMessage& msg);
void goUntilRaw(OscMessage& msg);
bool releaseSw(uint8_t motorId, bool action, bool dir); // true if sent
void releaseSw(OscMessage& msg);
void goHome(OscMessage& msg);
void goMark(OscMessage& msg);
void setMark(OscMessage& msg);
void setPosition(OscMessage& msg);
void resetPos(OscMessage& msg);
void softStop(OscMessage& msg);
void hardStop(OscMessage& msg);
void softHiZ(OscMessage& msg);
void hardHiZ(OscMessage& msg);
#ifdef HAVE_BRAKE
void activate(OscMessage& msg);
void free(OscMessage& msg);
#endif
void combinedRun(OscMessage& msg);
void combinedRunRaw(OscMessage& msg);
void combinedMove(OscMessage& msg);
void combinedGoTo(OscMessage& msg);
void combinedGoToDir(OscMessage& msg);
void combinedSoftStop(OscMessage& msg);
void combinedHardStop(OscMessage& msg);
void combinedSoftHiZ(OscMessage& msg);
void combinedHardHiZ(OscMessage& msg);

// servo_commands_osc_listener
void setTargetPosition(OscMessage& msg);
void setTargetPositionList(OscMessage& msg);
void enableServoMode(OscMessage& msg);
void setServoParam(OscMessage& msg);
void getServoParam(uint8_t motorId);
void getServoParam(OscMessage& msg);

// PowerSTEP01_config_osc_listener
#ifdef DRIVER_POWERSTEP01
void setVoltageMode(uint8_t motorId);
void setVoltageMode(OscMessage& msg);
void setCurrentMode(uint8_t motorId);
void setCurrentMode(OscMessage& msg);
#endif
void setElPos(OscMessage& msg);
void getElPos(OscMessage& msg);
#endif

