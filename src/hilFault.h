// hilFault.h
// Fault and stop records for the HIL test bench (TASK_usb_unplug), HIL_PROBE builds only.
// Without HIL_PROBE nothing here is compiled and the hooks are empty macros, so normal
// builds are not affected.
//
// The record is in RAM that neither the startup code (it clears .bss only) nor the
// bootloader (its own .data / .bss at the start of RAM, the double-tap word at the end)
// writes, so it survives a watchdog or software reset (not a power-up: the magic is then
// wrong and the record is cleared).
//
// - HardFault: HardFault_Handler (faultReset.cpp) calls hilFaultRecordHardFault() after
//   the brakes and the fault marker; it records the exception frame (R0-R3, R12, LR, PC,
//   xPSR), the stack pointer and EXC_RETURN and sets P2 high. P0 goes high right before
//   the reset (hilFaultBeforeReset()).
// - Early warning: WDT_Handler records the frame of the code it interrupted, the USB
//   registers and the NVIC state before the brakes are engaged. The USB interrupt keeps
//   the core's priority (0, as the early warning, which then cannot interrupt it), as in
//   the product builds.
// - USB interrupt: USB_Handler (overrides the core's weak handler, calls the same
//   UDD_Handler) counts the entries, keeps the last flags and whether it is running.
// - loop(): hilFaultLoopMark() keeps the time and the USB interrupt count of the last pass.
//
// 'H' prints the record and clears it, 'X' causes a HardFault (read of an invalid address).

#ifndef _HILFAULT_h
#define _HILFAULT_h

#ifdef HIL_PROBE

#include <stdint.h>


void hilFaultInit();      // in setup(): boot count
void hilFaultLoopMark();  // every loop() pass
void hilFaultPrint();     // 'H': prints and clears the record
void hilFaultTrigger();   // 'X': HardFault
extern "C" void hilFaultOnEarlyWarning(uint32_t excReturn, uint32_t msp, uint32_t psp);
extern "C" void hilFaultRecordHardFault(uint32_t excReturn, uint32_t msp, uint32_t psp);
extern "C" void hilFaultBeforeReset(void);

#define HIL_FAULT_INIT() hilFaultInit()
#define HIL_FAULT_LOOP_MARK() hilFaultLoopMark()

#else

#define HIL_FAULT_INIT() do {} while (0)
#define HIL_FAULT_LOOP_MARK() do {} while (0)

#endif // HIL_PROBE

#endif
