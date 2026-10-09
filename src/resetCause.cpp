// resetCause.cpp

#include "resetCause.h"

#if defined(__SAMD21G18A__) || defined(__SAMD21__)
#define RESET_CAUSE_SAMD21
#include <Arduino.h> // PM register definitions
#define RCAUSE_POR PM_RCAUSE_POR
#define RCAUSE_BOD12 PM_RCAUSE_BOD12
#define RCAUSE_BOD33 PM_RCAUSE_BOD33
#define RCAUSE_EXT PM_RCAUSE_EXT
#define RCAUSE_WDT PM_RCAUSE_WDT
#define RCAUSE_SYST PM_RCAUSE_SYST
#else
// Host tests only: bit positions of PM.RCAUSE from the SAMD21 datasheet. The firmware
// build uses the definitions of the device header above.
#define RCAUSE_POR (1u << 0)
#define RCAUSE_BOD12 (1u << 1)
#define RCAUSE_BOD33 (1u << 2)
#define RCAUSE_EXT (1u << 4)
#define RCAUSE_WDT (1u << 5)
#define RCAUSE_SYST (1u << 6)
#endif

ResetCause resetCauseFromSamd21Rcause(uint8_t rcause)
{
    if (rcause & (RCAUSE_BOD12 | RCAUSE_BOD33))
        return RESET_CAUSE_BROWN_OUT;
    if (rcause & RCAUSE_POR)
        return RESET_CAUSE_POWER_ON;
    if (rcause & RCAUSE_WDT)
        return RESET_CAUSE_WATCHDOG;
    if (rcause & RCAUSE_EXT)
        return RESET_CAUSE_EXTERNAL;
    if (rcause & RCAUSE_SYST)
        return RESET_CAUSE_SOFTWARE;
    return RESET_CAUSE_UNKNOWN;
}

ResetCause readResetCause(uint8_t *raw)
{
#ifdef RESET_CAUSE_SAMD21
    *raw = PM->RCAUSE.reg;
    return resetCauseFromSamd21Rcause(*raw);
#else
    *raw = 0;
    return RESET_CAUSE_UNKNOWN;
#endif
}

const char *mcuName()
{
#ifdef RESET_CAUSE_SAMD21
    return "SAMD21";
#else
    return "unknown";
#endif
}

void systemReset()
{
#ifdef RESET_CAUSE_SAMD21
    NVIC_SystemReset();
#endif
}
