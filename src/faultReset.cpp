// faultReset.cpp
// See faultReset.h.

#include "faultReset.h"
#include <stddef.h>
#include "boardsDef.h"

extern "C"
{
volatile uint32_t faultBrakeWaitMs = 0;
}

// Two words, the second the complement of the first: a power-up leaves random RAM, and
// the reset cause ignores the marker after a power-up or a brown-out anyway. A macro
// without a suffix, for the handler in assembly.
#define CPU_FAULT_MAGIC_VALUE 0xC9F4A75E
namespace
{
constexpr uint32_t CPU_FAULT_MAGIC = CPU_FAULT_MAGIC_VALUE;
} // namespace

// The marker is in the RAM after .bss that neither the startup code (it clears .bss only)
// nor the bootloader writes: the .noinit section, as the HIL fault record (hilFault.cpp).
// A NOBITS section of its own (GCC 7 would make a section(".noinit") variable PROGBITS,
// which the linker puts between .data and .bss, in the RAM the bootloader uses).
asm(".section .noinit,\"aw\",%nobits\n"
    ".balign 4\n"
    ".global cpuFaultMarker\n"
    "cpuFaultMarker:\n"
    ".space 8\n"
    ".previous\n");
extern "C" volatile uint32_t cpuFaultMarker[2];

bool cpuFaultMarkerTake()
{
    const bool set = (cpuFaultMarker[0] == CPU_FAULT_MAGIC) && (cpuFaultMarker[1] == ~CPU_FAULT_MAGIC);
    cpuFaultMarker[0] = 0;
    cpuFaultMarker[1] = 0;
    return set;
}

#if defined(__SAMD21G18A__) || defined(__SAMD21__)

// The constants of the handler below, checked against the device header where it can be.
static_assert(sizeof(PortGroup) == 0x80, "PORT group size");
static_assert(offsetof(PortGroup, DIRSET) == 0x08, "PORT DIRSET offset");
static_assert(offsetof(PortGroup, OUTCLR) == 0x14, "PORT OUTCLR offset");
static_assert(offsetof(PortGroup, OUTSET) == 0x18, "PORT OUTSET offset");
static_assert(offsetof(Wdt, STATUS) == 0x07, "WDT STATUS offset");
static_assert(offsetof(Wdt, CLEAR) == 0x08, "WDT CLEAR offset");
static_assert(WDT_STATUS_SYNCBUSY == 0x80, "WDT SYNCBUSY bit");
static_assert(WDT_CLEAR_CLEAR_KEY_Val == 0xA5, "WDT clear key");

#if defined(HAVE_BRAKE) && !defined(STEP400_R1) && !defined(STEP800_R1)
#error "faultReset.cpp: the brake outputs of this board are not set in HardFault_Handler"
#endif

#define FAULT_STR_(x) #x
#define FAULT_STR(x) FAULT_STR_(x)
// The wait loop below runs from RAM (no flash wait states): subs (1 cycle) and a taken
// bne (2 cycles), 3 cycles per pass, so 16000 passes are 1 ms at 48 MHz.
#define FAULT_WAIT_LOOPS_PER_MS 16000

// HardFault_Handler is in a .data input section, which the startup code copies to RAM
// with the initialized data (the linker script places .data* in RAM, loaded from flash).
// Its literal pool (.ltorg) is copied with it, so the handler reads nothing from the
// flash once the CPU has fetched its address from the vector table. All of it is in
// assembly so that no compiler-generated code or call runs before the brakes; r4-r6 hold
// the values across the calls of the HIL build (callee-saved).
asm(".section .data.faultReset,\"aw\",%progbits\n"
    ".syntax unified\n"
    ".balign 4\n"
    ".global HardFault_Handler\n"
    ".type HardFault_Handler, %function\n"
    ".thumb_func\n"
    "HardFault_Handler:\n"
    // 1. Every brake output to engaged: output low, then the pin direction to output (in
    // case the fault comes before setup() has set the pins).
#if defined(STEP400_R1)
    // EM_1 PB08 (A1), EM_2 PA06 (D8), EM_3 PA15 (D5), EM_4 PA10 (D1): low engages.
    "ldr r0, =0x41004400\n"     // PORT group A
    "ldr r1, =0x8440\n"         // PA06 | PA10 | PA15
    "str r1, [r0, #0x14]\n"     // OUTCLR
    "str r1, [r0, #0x08]\n"     // DIRSET
    "mov r3, r0\n"
    "adds r3, #0x80\n"          // PORT group B
    "movs r1, #1\n"
    "lsls r1, r1, #8\n"         // PB08
    "str r1, [r3, #0x14]\n"     // OUTCLR
    "str r1, [r3, #0x08]\n"     // DIRSET
#elif defined(STEP800_R1)
    // As engageAllBrakeOutputsFromIsr(): SHIFTOUT_ENABLE PA15 (D5) low clears the shift
    // register of the 74HC595 at once, and a rising edge of the latch PB02 (A5) puts the
    // zeros (all engaged) on its outputs. No SPI.
    "ldr r0, =0x41004400\n"     // PORT group A
    "movs r1, #1\n"
    "lsls r1, r1, #15\n"        // PA15
    "str r1, [r0, #0x14]\n"     // OUTCLR
    "str r1, [r0, #0x08]\n"     // DIRSET
    "mov r3, r0\n"
    "adds r3, #0x80\n"          // PORT group B
    "movs r1, #4\n"             // PB02
    "str r1, [r3, #0x14]\n"     // OUTCLR
    "str r1, [r3, #0x08]\n"     // DIRSET
    "movs r2, #16\n"            // about 1 us: the clear and the latch pulse width
    "1: subs r2, r2, #1\n"
    "bne 1b\n"
    "str r1, [r3, #0x18]\n"     // OUTSET: the rising edge latches the zeros
#endif
    // 2. The marker for readResetCause().
    "ldr r0, =cpuFaultMarker\n"
    "ldr r1, =" FAULT_STR(CPU_FAULT_MAGIC_VALUE) "\n"
    "mvns r2, r1\n"
    "str r1, [r0]\n"
    "str r2, [r0, #4]\n"
#ifdef HIL_PROBE
    // HIL: the exception frame and EXC_RETURN (in LR, untouched so far) into the fault
    // record, P2 high. This code is in the flash.
    "mov r0, lr\n"
    "mrs r1, msp\n"
    "mrs r2, psp\n"
    "ldr r3, =hilFaultRecordHardFault\n"
    "blx r3\n"
#endif
    // 3. Wait faultBrakeWaitMs, 1 ms per pass. The watchdog keeps running and the early
    // warning cannot interrupt a HardFault, so each pass clears the watchdog unless the
    // previous clear is still being synchronized (2-3 ms).
    "ldr r4, =faultBrakeWaitMs\n"
    "ldr r4, [r4]\n"
    "ldr r5, =0x40001000\n"     // WDT
    "movs r6, #0xA5\n"          // the clear key
    "2: cmp r4, #0\n"
    "beq 4f\n"
    "ldrb r0, [r5, #7]\n"       // STATUS
    "lsls r0, r0, #24\n"        // SYNCBUSY (bit 7) to N
    "bmi 3f\n"
    "strb r6, [r5, #8]\n"       // CLEAR
    "3: ldr r1, =" FAULT_STR(FAULT_WAIT_LOOPS_PER_MS) "\n"
    "5: subs r1, r1, #1\n"
    "bne 5b\n"
    "subs r4, r4, #1\n"
    "b 2b\n"
    "4:\n"
#ifdef HIL_PROBE
    "ldr r3, =hilFaultBeforeReset\n" // HIL: P0 high, the mark of the MCU reset
    "blx r3\n"
#endif
    // 4. NVIC_SystemReset()
    "ldr r0, =0xE000ED0C\n"     // SCB->AIRCR
    "ldr r1, =0x05FA0004\n"     // VECTKEY | SYSRESETREQ
    "dsb\n"
    "str r1, [r0]\n"
    "dsb\n"
    "6: b 6b\n"
    ".ltorg\n"
    ".size HardFault_Handler, . - HardFault_Handler\n"
    ".syntax divided\n" // as GCC 7 emits this file
    ".previous\n");

#endif
