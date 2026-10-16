// resetCause.h
// Cause of the last reset, as an MCU-independent number.
// These numbers are part of the OSC specification (/booted). Never change the meaning
// of an existing number; add new causes as 6, 7, ...

#ifndef _RESETCAUSE_h
#define _RESETCAUSE_h

#include <stdint.h>

enum ResetCause : uint8_t
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_POWER_ON = 1,
    RESET_CAUSE_BROWN_OUT = 2,
    RESET_CAUSE_EXTERNAL = 3, // reset pin
    RESET_CAUSE_WATCHDOG = 4,
    RESET_CAUSE_SOFTWARE = 5, // e.g. NVIC_SystemReset(), upload over USB
    RESET_CAUSE_CPU_FAULT = 6 // the reset after a CPU exception (HardFault on the SAMD21)
};

// MCU-specific: reads the reset cause register once. Stores the raw register value in
// *raw (0 on unsupported MCUs) and returns the ResetCause.
ResetCause readResetCause(uint8_t *raw);

// Converts the SAMD21 PM->RCAUSE value and the CPU fault marker (faultReset.h). If several
// causes are set, the priority is CPU fault > brown-out > power-on > watchdog > reset pin >
// software, except that the marker is ignored after a power-on or a brown-out: the RAM
// that holds it is not kept then. (The reset after a fault is a software reset, or a
// watchdog reset if the fault handler hangs, so RCAUSE alone cannot tell it.)
ResetCause resetCauseFromSamd21Rcause(uint8_t rcause, bool cpuFaultMarker);

// Name of the MCU for the diagnosis output.
const char *mcuName();

// MCU-specific: resets the MCU (reported as RESET_CAUSE_SOFTWARE after the restart).
// Does not return on supported MCUs; does nothing on unsupported MCUs.
void systemReset();

#endif
