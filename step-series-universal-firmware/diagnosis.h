// diagnosis.h

#ifndef _DIAGNOSIS_h
#define _DIAGNOSIS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Adafruit_SleepyDog.h>
#include "boardsDef.h"
#include "globals.h"
#include "loadConfig.h"

void diagnosisCommand(uint8_t inByte);
void printMenu();
void testMotion();
void printCurrentState();
void printConfigurations();
void printTitle(String title);
void showBoolResult(String title, bool val);
template <class T>
T printAllData(String title, T* val);

void printAllBools(String title, bool* val);
void printIpAddress(String title, IPAddress ip);
void boldHeader(String header);
void printHeader(String header);
#ifdef HAVE_BRAKE
void testBrake();
#endif

#endif