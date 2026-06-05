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

// Web Serial API commands
void printConfigAsJson();
void printStatusAsJson();
void receiveConfigJson();
void receiveFilename();
void rebootBoard();

// Shared config serialization (writes all sections except "board" to any Print stream)
void serializeConfigSectionsTo(Print& out);

// SD save helpers
void saveRawConfigToSd(const char* json, size_t len); // diagnostic-only, kept for reference
void saveCurrentConfigAsJson();   // Web Serial: re-serialize globals → SD, respond with JSON
void saveCurrentConfigToSd();     // OSC /saveConfig: same write, respond with OSC

#endif
