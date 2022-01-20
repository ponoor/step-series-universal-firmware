// globals.h

#ifndef _GLOBALS_h
#define _GLOBALS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include <SPI.h>
// #include <Ponoor_L6470Library.h>
#include <Ethernet.h>
#include "boardsDef.h"
#ifdef DRIVER_POWERSTEP01
#include <Ponoor_PowerSTEP01Library.h>
#else
#include <Ponoor_L6470Library.h>
#endif

// =============================================================================
// Constants and variables
// =============================================================================

// General
#define COMPILE_DATE __DATE__
#define COMPILE_TIME __TIME__
extern const uint8_t firmwareVersion[3]; // defined in boardsDef.h
extern int8_t loadedConfigVersion[2];
extern const uint8_t applicableConfigVersion[2]; // defined in boardsDef.h
#define STATUS_POLL_PERIOD   1 // [ms]


// Tx, Rx LED
extern bool rxLedEnabled, txLedEnabled;
extern uint32_t RXL_blinkStartTime, TXL_blinkStartTime;
#define RXL_TXL_BLINK_DURATION	30 // ms

// Json configuration file
constexpr char* filename = "/config.txt";
extern String configName;
extern String configTargetProduct;
extern bool sdInitializeSucceeded;
extern bool configFileOpenSucceeded;
extern bool configFileParseSucceeded;
extern bool configProductApplicable;

// Motor drivers
#ifdef DRIVER_POWERSTEP01
extern powerSTEP stepper[]; // boardsSpecific.cpp
#else
extern AutoDriver stepper[]; // boardsSpecific.cpp
#endif
#define MOTOR_ID_ALL    255
#define MOTOR_ID_FIRST  1
#define MOTOR_ID_LAST   NUM_OF_MOTOR

// Network
extern uint8_t mac[6], mac_from_config[6], myId;
extern IPAddress
    myIp,
    myIp_from_config,
    destIp,
    dns,
    gateway,
    subnet;
const IPAddress broadcastIp(255, 255, 255, 255);
extern unsigned int inPort, outPort, outPort_from_config;
extern EthernetUDP Udp;
extern bool
    isDestIpSet,
    isMyIpAddId,
    isMacAddId,
    isOutPortAddId,
    bootedMsgEnable,
    isWaitingSendBootMsg,
    reportErrors;
#define BOOT_MSG_WAIT_TIME  1000    //[ms], The waiting duration for sending a boot message after Ethernet has been linked

// Motor settings.
#ifdef DRIVER_POWERSTEP01
#define TVAL_LIMIT_VAL  64 // approx. 5A
#endif

// Brake
#ifdef HAVE_BRAKE
extern uint8_t brakeOut;
#endif
extern uint8_t brakeStatus[NUM_OF_MOTOR];
enum {
    BRAKE_ENGAGED = 0,
    BRAKE_DISENGAGE_WAITING, // motor excited
    BRAKE_DISENGAGED,
    BRAKE_MOTORHIZ_WAITING // brake closed
};
extern uint32_t brakeTransitionTrigTime[NUM_OF_MOTOR];
extern bool bBrakeDecWaiting[NUM_OF_MOTOR]; // Waiting deceleration for the brake engaging procedure

// Homing
extern uint32_t homingStartTime[NUM_OF_MOTOR];
extern uint8_t homingStatus[NUM_OF_MOTOR];
extern bool bHoming[NUM_OF_MOTOR];
extern float homingSpeed[NUM_OF_MOTOR];
enum {
    HOMING_UNDEFINED = 0,
    HOMING_GOUNTIL,
    HOMING_RELEASESW,
    HOMING_COMPLETED,
    HOMING_TIMEOUT
};

// Command errors
enum {
    ERROR_COMMAND_IGNORED = 0,
    ERROR_MOTORID_NOTMATCH,
    ERROR_BRAKE_ENGAGED,
    ERROR_HOMESW_ACTIVATING,
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
    ERROR_LIMITSW_ACTIVATING,
#endif
    ERROR_GOUNTIL_TIMEOUT,
    ERROR_RELEASESW_TIMEOUT,
    ERROR_IN_SERVO_MODE
};
const String commandErrorText[8] = {
    F("CommandIgnored"),
    F("MotorIdNotMatch"),
    F("BrakeEngaged"),
    F("HomeSwActivating"),
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
    F("LimitSwActivating"),
#endif
    F("GoUntilTimeout"),
    F("ReleaseSwTimeout"),
    F("InServoMode")
};
// These values will be initialized at loadConfig()
extern bool
    busy[NUM_OF_MOTOR],
    HiZ[NUM_OF_MOTOR],
    homeSwState[NUM_OF_MOTOR],
    dir[NUM_OF_MOTOR],
    uvloStatus[NUM_OF_MOTOR];
extern uint8_t
    motorStatus[NUM_OF_MOTOR],
    thermalStatus[NUM_OF_MOTOR];

extern bool
    reportBUSY[NUM_OF_MOTOR],
    reportHiZ[NUM_OF_MOTOR],
    reportHomeSwStatus[NUM_OF_MOTOR],
    reportDir[NUM_OF_MOTOR],
    reportMotorStatus[NUM_OF_MOTOR],
    reportSwEvn[NUM_OF_MOTOR],
    reportUVLO[NUM_OF_MOTOR],
    reportThermalStatus[NUM_OF_MOTOR],
    reportOCD[NUM_OF_MOTOR],
    reportStall[NUM_OF_MOTOR],
    reportPosition[NUM_OF_MOTOR],
    reportPositionList,
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
    limitSwState[NUM_OF_MOTOR],
    reportLimitSwStatus[NUM_OF_MOTOR],
    limitSwMode[NUM_OF_MOTOR],
#endif
    homingDirection[NUM_OF_MOTOR],
    bProhibitMotionOnHomeSw[NUM_OF_MOTOR],
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
    bProhibitMotionOnLimitSw[NUM_OF_MOTOR],
#endif
    bHomingAtStartup[NUM_OF_MOTOR];
extern uint16_t
    goUntilTimeout[NUM_OF_MOTOR],
    releaseSwTimeout[NUM_OF_MOTOR],
    reportPositionInterval[NUM_OF_MOTOR],
    reportPositionListInterval;
extern uint8_t kvalHold[NUM_OF_MOTOR], kvalRun[NUM_OF_MOTOR], kvalAcc[NUM_OF_MOTOR], kvalDec[NUM_OF_MOTOR];
#ifdef DRIVER_POWERSTEP01
extern uint8_t tvalHold[NUM_OF_MOTOR], tvalRun[NUM_OF_MOTOR], tvalAcc[NUM_OF_MOTOR], tvalDec[NUM_OF_MOTOR];
extern bool isCurrentMode[NUM_OF_MOTOR];
#endif
extern bool homeSwMode[NUM_OF_MOTOR];

extern uint16_t intersectSpeed[NUM_OF_MOTOR]; // INT_SPEED
extern uint8_t
    startSlope[NUM_OF_MOTOR], // ST_SLP
    accFinalSlope[NUM_OF_MOTOR], // FN_SLP_ACC
    decFinalSlope[NUM_OF_MOTOR], // FN_SLP_DEC
    stallThreshold[NUM_OF_MOTOR]; // STALL_TH
#ifdef DRIVER_POWERSTEP01
// extern uint8_t
    fastDecaySetting[NUM_OF_MOTOR], // T_FAST
    minOnTime[NUM_OF_MOTOR], // TON_MIN
    minOffTime[NUM_OF_MOTOR]; // TOFF_MIN
#endif
extern uint8_t
    overCurrentThreshold[NUM_OF_MOTOR], // OCD
    microStepMode[NUM_OF_MOTOR]; // STEP_MODE
extern uint16_t slewRate[NUM_OF_MOTOR]; // GATECFG1
extern uint8_t slewRateNum[NUM_OF_MOTOR]; // [0]114, [1]220, [2]400, [3]520, [4]790, [5]980.
extern float lowSpeedOptimizeThreshold[NUM_OF_MOTOR];
extern bool lowSpeedOptimizeEnable[NUM_OF_MOTOR];
extern bool electromagnetBrakeEnable[NUM_OF_MOTOR];
extern uint16_t brakeTransitionDuration[NUM_OF_MOTOR];
extern float
    acc[NUM_OF_MOTOR],
    dec[NUM_OF_MOTOR],
    maxSpeed[NUM_OF_MOTOR],
    minSpeed[NUM_OF_MOTOR],
    fullStepSpeed[NUM_OF_MOTOR];

// Servo mode
extern int32_t targetPosition[NUM_OF_MOTOR];
extern float kP[NUM_OF_MOTOR], kI[NUM_OF_MOTOR], kD[NUM_OF_MOTOR];
extern boolean isServoMode[NUM_OF_MOTOR];
#endif

