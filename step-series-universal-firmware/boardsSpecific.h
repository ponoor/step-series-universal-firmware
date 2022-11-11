// Boards specific functions

#ifndef _BOARDSSPECIFIC_h
#define _BOARDSSPECIFIC_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include <SPI.h>
#include "boardsDef.h"
#include "globals.h"
#include "utils.h"

#ifdef STEP400_R1
#elif defined(STEP400_PROTO_R4)
#elif defined(STEP800_R1)
extern SPIClassSAMD shiftRegisterSPI;
#elif defined(STEP800_PROTO_R3)
extern SPIClassSAMD shiftRegisterSPI;
#elif defined(STEP800_PROTO_R1)
#elif defined(STEP100_R1)
#elif defined(STEP200_R1)
#endif

void initDipSw();
uint8_t getMyId();

#ifdef HAVE_BRAKE
void initBrake();
void setBrake(uint8_t motorId, bool state);
#endif

#ifdef HAVE_LIMIT_GPIO
void initLimitSw();
#endif

#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
void checkLimitSw();
#endif

#endif