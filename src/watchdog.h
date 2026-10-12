// watchdog.h
// SAMD21 watchdog with an early warning interrupt. Replaces Adafruit SleepyDog, whose
// WDT_Handler cannot be replaced.
//
// The clock is the same as with SleepyDog: GCLK2 = OSCULP32K / 32, about 1024 Hz.
// enable(ms): the early warning comes ms after the last reset() (rounded down to a power
// of two of WDT cycles, as SleepyDog did for the timeout), the watchdog reset at twice
// that. The early warning calls onWatchdogEarlyWarning() (engage the brakes), then
// resets the MCU at once through the watchdog (reset cause: watchdog).

#ifndef _WATCHDOG_h
#define _WATCHDOG_h

#include <stdint.h>

class WatchdogTimer
{
public:
    // Returns the time to the early warning in ms.
    int enable(int earlyWarningMs);
    void reset();
    void disable();

private:
    bool initialized = false;
};

extern WatchdogTimer Watchdog;

// Called from the early warning interrupt; must not use the driver SPI. Defined in utils.cpp.
void onWatchdogEarlyWarning();

#endif
