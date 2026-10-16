// watchdog.h
// SAMD21 watchdog with an early warning interrupt. Replaces Adafruit SleepyDog, whose
// WDT_Handler cannot be replaced.
//
// The clock is the same as with SleepyDog: GCLK2 = OSCULP32K / 32, about 1024 Hz.
// enable(ms): the early warning comes ms after the last reset() (rounded down to a power
// of two of WDT cycles, as SleepyDog did for the timeout). The early warning calls
// onWatchdogEarlyWarning() (engage the brakes and wait for them, up to 1 s), then resets
// the MCU at once through the watchdog (reset cause: watchdog). The watchdog period,
// twice the early warning time, resets the MCU only when the early warning cannot run
// (e.g. a hang with the interrupts disabled).

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
    // For the brake wait of the early warning (interrupt): restarts the watchdog period
    // like reset(), outside the HIL interval measurement.
    void clearFromIsr();

private:
    bool initialized = false;
};

extern WatchdogTimer Watchdog;

// The early warning clears the watchdog this often during its brake wait (up to 1 s),
// well within the shortest period.
#define WATCHDOG_BRAKE_WAIT_CLEAR_MS 100

// Called from the early warning interrupt; must not use the driver SPI. Defined in utils.cpp.
void onWatchdogEarlyWarning();

#ifdef HIL_PROBE
// HIL: the longest interval between two reset() calls while the watchdog is enabled
// (disable() ends an interval, enable() starts one), with the return addresses of the
// two calls (resolve them with addr2line on the ELF). Returns it and clears it ('p').
void watchdogGapTake(uint32_t &maxUs, uint32_t &from, uint32_t &to);
#endif

#endif
