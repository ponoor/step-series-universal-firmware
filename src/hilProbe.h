// hilProbe.h
// Timing probes on spare pins for the hardware-in-the-loop test bench (tools/hil/).
// Enabled with the HIL_PROBE build flag. Without it, every macro is empty and
// hilProbe.cpp compiles to nothing, so normal builds are not affected.
//
//   ch  pin  SAMD21  logic analyzer  signal
//   P0  D1   PA10    DIO8            toggled at the start of loop()
//   P1  A1   PB08    DIO9            high while a received OSC packet is processed
//   P2  D8   PA06    DIO10           high while updateServo() runs a servo update
//   P3  D9   PA07    DIO11           high during checkStatus()
//   P4  D38  PA13    DIO12           high during getMyId() (shift register transfer)
//   P5  SCL  PA23    DIO13           high from Udp.beginPacket() to Udp.endPacket()
//   P6  SDA  PA22    DIO14           pulse on every write of brakeStatus[]

#ifndef _HILPROBE_h
#define _HILPROBE_h

#ifdef HIL_PROBE

#include <Arduino.h>
#include "boardsDef.h"

#if !defined(STEP800_R1)
#error "HIL_PROBE: the probe pins are defined only for STEP800_R1"
#endif

#define HIL_PROBE_COUNT 7
// Width of PROBE_PULSE, long enough to be sampled by the logic analyzer at 100 kHz.
#define HIL_PROBE_PULSE_US 20

extern uint8_t hilProbePort[HIL_PROBE_COUNT];
extern uint32_t hilProbeMask[HIL_PROBE_COUNT];

void hilProbeInit();     // pins to output low, prints the pin map
void hilProbePrintMap();
void hilProbeSelfTest(); // 'P' diagnosis command: ch + 1 pulses on each probe

// Driver state across a restart (HIL tests of the brake). Called first in setup(),
// before the drivers are reset: releases the driver reset line without a reset pulse
// and reads CONFIG and STATUS of every driver with GetParam. 'D' prints them: CONFIG
// at its power-on default means the drivers were reset (standby) while the MCU was.
class SPIClassSAMD;
void hilCaptureDriversAtBoot(SPIClassSAMD &spi);
void hilPrintDriversAtBoot(); // 'D'
// 'Y': floats the driver reset line (as during an MCU reset) for 300 ms, P2 high, then
// drives it high again and sets the drivers up (resetMotorDriver()).
void hilDriverStandbyTest();

// OUTSET / OUTCLR / OUTTGL only (no read-modify-write), so the probes can be used
// from interrupt handlers.
#define PROBE_HIGH(ch) (PORT->Group[hilProbePort[ch]].OUTSET.reg = hilProbeMask[ch])
#define PROBE_LOW(ch) (PORT->Group[hilProbePort[ch]].OUTCLR.reg = hilProbeMask[ch])
#define PROBE_TOGGLE(ch) (PORT->Group[hilProbePort[ch]].OUTTGL.reg = hilProbeMask[ch])
#define PROBE_PULSE(ch)                         \
    do                                          \
    {                                           \
        PROBE_HIGH(ch);                         \
        delayMicroseconds(HIL_PROBE_PULSE_US);  \
        PROBE_LOW(ch);                          \
    } while (0)
#define HIL_PROBE_INIT() hilProbeInit()

#else

#define PROBE_HIGH(ch) do {} while (0)
#define PROBE_LOW(ch) do {} while (0)
#define PROBE_TOGGLE(ch) do {} while (0)
#define PROBE_PULSE(ch) do {} while (0)
#define HIL_PROBE_INIT() do {} while (0)

#endif // HIL_PROBE

#endif
