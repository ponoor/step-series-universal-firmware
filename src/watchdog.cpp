// watchdog.cpp
// See watchdog.h.

#include "watchdog.h"
#include <Arduino.h>
#include "hilProbe.h"

WatchdogTimer Watchdog;

int WatchdogTimer::enable(int earlyWarningMs)
{
    if (!initialized)
    {
        // Generic clock generator 2: OSCULP32K / 32 (DIVSEL: 2^(DIV+1)) = 1024 Hz
        GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
        GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN |
                            GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
        while (GCLK->STATUS.bit.SYNCBUSY)
            ;
        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;
        NVIC_DisableIRQ(WDT_IRQn);
        NVIC_ClearPendingIRQ(WDT_IRQn);
        NVIC_SetPriority(WDT_IRQn, 0); // top priority
        NVIC_EnableIRQ(WDT_IRQn);
        initialized = true;
    }
    WDT->CTRL.reg = 0; // disabled for the configuration
    while (WDT->STATUS.bit.SYNCBUSY)
        ;
    // Cycles of the early warning: 8 << bits, bits 0 to 0xA (8 s); the period is the
    // next setting (twice as long, up to 16 s).
    const long cycles = ((long)earlyWarningMs * 1024L + 500) / 1000;
    uint8_t bits = 0;
    while ((bits < 0xA) && ((8L << (bits + 1)) <= cycles))
        bits++;
    WDT->CONFIG.reg = WDT_CONFIG_PER(bits + 1);
    WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET(bits);
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;
    WDT->INTENSET.reg = WDT_INTENSET_EW;
    reset();
    WDT->CTRL.bit.ENABLE = 1;
    while (WDT->STATUS.bit.SYNCBUSY)
        ;
    return (int)(((8L << bits) * 1000L + 512) / 1024);
}

void WatchdogTimer::reset()
{
    while (WDT->STATUS.bit.SYNCBUSY)
        ;
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

void WatchdogTimer::disable()
{
    WDT->CTRL.bit.ENABLE = 0;
    while (WDT->STATUS.bit.SYNCBUSY)
        ;
    WDT->INTENCLR.reg = WDT_INTENCLR_EW;
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;
}

void WDT_Handler(void)
{
    WDT->INTFLAG.reg = WDT_INTFLAG_EW;
    onWatchdogEarlyWarning();
    PROBE_HIGH(0); // HIL: the last P0 edge marks the MCU reset (the pin floats then)
    // Any value but the key resets the MCU at once (reset cause: watchdog).
    while (WDT->STATUS.bit.SYNCBUSY)
        ;
    WDT->CLEAR.reg = 0;
    while (true)
        ;
}
