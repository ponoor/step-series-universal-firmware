// faultReset.h
// Reset after a CPU fault (HardFault on the SAMD21).
//
// HardFault_Handler (faultReset.cpp, overrides the core's weak handler, which loops until
// the watchdog period) runs from RAM and, in this order:
//   1. drives every brake output to engaged (all of them, whatever the brake settings),
//      with direct writes to the PORT registers,
//   2. writes the fault marker in RAM that the startup code does not clear (.noinit),
//   3. waits faultBrakeWaitMs, clearing the watchdog meanwhile,
//   4. resets the MCU (NVIC_SystemReset(): PM->RCAUSE SYST).
// It reads neither the stack nor EXC_RETURN before the brakes: a USB unplug on STEP400_R1
// has given HardFaults while the flash reads were still wrong (REPORT_usb_unplug.md).
// After the reset, readResetCause() reports RESET_CAUSE_CPU_FAULT if the marker is set.
// A fault in the handler (lockup) is reset by the watchdog period.

#ifndef _FAULTRESET_h
#define _FAULTRESET_h

#include <stdint.h>

// The longest brakeTransitionDuration (at most 1000 ms) of the motors with the brake
// enabled and not engaged: the wait of the handler after the brakes. Kept up to date by
// updateFaultBrakeWait() (utils.cpp) whenever a brake state or setting changes, so the
// handler does not scan the arrays.
extern "C" volatile uint32_t faultBrakeWaitMs;

// Reads the fault marker and clears it. Returns true if it was set. Call once at startup.
bool cpuFaultMarkerTake();

#endif
