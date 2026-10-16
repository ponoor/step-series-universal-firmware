// hilFault.cpp
// See hilFault.h. HIL_PROBE builds only.

#include "hilFault.h"

#ifdef HIL_PROBE

#include <Arduino.h>
#include <string.h>
#include "hilProbe.h"
#include "utils.h"

namespace
{
constexpr uint32_t HIL_FAULT_MAGIC = 0x48464C54; // "HFLT"
constexpr uint32_t FLAG_HARDFAULT = 1;
constexpr uint32_t FLAG_EARLY_WARNING = 2;
constexpr uint8_t USB_EP_COUNT = 8;
constexpr uint8_t USB_LAST_COUNT = 8;
constexpr uint32_t RAM_START = 0x20000000;
constexpr uint32_t RAM_END = 0x20008000;

struct Frame
{
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
};

struct UsbState
{
    uint16_t intflag, intenset, epintsmry, ctrlb;
    uint8_t ctrla, status, fsmstatus, pad;
    uint8_t epintflag[USB_EP_COUNT], epintenset[USB_EP_COUNT], epstatus[USB_EP_COUNT];
};

struct Record
{
    uint32_t magic;
    uint32_t bootCount;
    uint32_t flags;
    uint32_t bootOfFault; // bootCount when the last fault was recorded
    // HardFault
    Frame hf;
    uint32_t hfSp, hfExcReturn, hfIcsr, hfUsbInIsr;
    // Early warning
    Frame ew;
    uint32_t ewSp, ewExcReturn, ewIcsr, ewIspr, ewIser;
    uint32_t ewMillis, ewLoopMillis, ewUsbCount, ewLoopUsbCount, ewUsbInIsr;
    uint32_t ewUsbLast[USB_LAST_COUNT]; // INTFLAG | EPINTSMRY << 16 at the last USB interrupts, oldest first
    UsbState ewUsb;
};

constexpr unsigned RECORD_STORAGE = 256;
static_assert(sizeof(Record) <= RECORD_STORAGE, "HIL fault record: enlarge RECORD_STORAGE");
} // namespace

// The storage is a NOBITS section of its own (GCC 7 would make a section(".noinit")
// variable PROGBITS, which the linker puts between .data and .bss, in the RAM the
// bootloader uses). The linker places it after .bss, before the heap: not cleared by the
// startup code (.bss only), above the bootloader's .data / .bss. See hilFault.h.
asm(".section .noinit,\"aw\",%nobits\n"
    ".balign 4\n"
    ".global hilFaultRecordStorage\n"
    "hilFaultRecordStorage:\n"
    ".space 256\n"
    ".previous\n");
extern "C" uint32_t hilFaultRecordStorage[];

namespace
{
Record &record = *reinterpret_cast<Record *>(hilFaultRecordStorage);

volatile uint32_t usbIsrCount = 0;
volatile uint32_t usbInIsr = 0;
volatile uint32_t usbLast[USB_LAST_COUNT];
volatile uint8_t usbLastIndex = 0;
volatile uint32_t loopMillis = 0;
volatile uint32_t loopUsbCount = 0;

bool validFrame(uint32_t sp)
{
    return (sp >= RAM_START) && (sp <= RAM_END - sizeof(Frame)) && !(sp & 3);
}

// The stack of the frame. Only an EXC_RETURN value (0xFFFFFFF1 / F9 / FD) can select PSP:
// a HardFault at a USB unplug has shown other values in LR (the handler's first
// instructions seem to run while the flash reads are still wrong), and this firmware uses
// MSP only.
uint32_t frameStack(uint32_t excReturn, uint32_t msp, uint32_t psp)
{
    return (excReturn == 0xFFFFFFFD) ? psp : msp;
}

void readUsb(UsbState &u)
{
    u.intflag = USB->DEVICE.INTFLAG.reg;
    u.intenset = USB->DEVICE.INTENSET.reg;
    u.epintsmry = USB->DEVICE.EPINTSMRY.reg;
    u.ctrlb = USB->DEVICE.CTRLB.reg;
    u.ctrla = USB->DEVICE.CTRLA.reg;
    u.status = USB->DEVICE.STATUS.reg;
    u.fsmstatus = USB->DEVICE.FSMSTATUS.reg;
    for (uint8_t i = 0; i < USB_EP_COUNT; i++)
    {
        u.epintflag[i] = USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg;
        u.epintenset[i] = USB->DEVICE.DeviceEndpoint[i].EPINTENSET.reg;
        u.epstatus[i] = USB->DEVICE.DeviceEndpoint[i].EPSTATUS.reg;
    }
}

void printFrame(const char *name, const Frame &f, uint32_t sp, uint32_t excReturn, uint32_t icsr)
{
    dbgPrint("HIL %s: pc 0x%08lX lr 0x%08lX xpsr 0x%08lX (exception %lu) sp 0x%08lX exc_return 0x%08lX\n", name,
             f.pc, f.lr, f.xpsr, f.xpsr & 0x3F, sp, excReturn);
    dbgPrint("HIL %s: r0 0x%08lX r1 0x%08lX r2 0x%08lX r3 0x%08lX r12 0x%08lX icsr 0x%08lX\n", name, f.r0, f.r1,
             f.r2, f.r3, f.r12, icsr);
}
} // namespace

extern "C" void UDD_Handler(void); // USBCore.cpp, set with USB_SetHandler() by the core

// Overrides the core's weak USB_Handler (cortex_handlers.c), which calls the handler set by
// USB_SetHandler(): UDD_Handler, the only one in device mode.
extern "C" void USB_Handler(void)
{
    usbInIsr = 1;
    usbIsrCount++;
    usbLast[usbLastIndex] = USB->DEVICE.INTFLAG.reg | ((uint32_t)USB->DEVICE.EPINTSMRY.reg << 16);
    usbLastIndex = (usbLastIndex + 1) % USB_LAST_COUNT;
    UDD_Handler();
    usbInIsr = 0;
}

void hilFaultInit()
{
    if (record.magic != HIL_FAULT_MAGIC)
    {
        memset(&record, 0, sizeof(record));
        record.magic = HIL_FAULT_MAGIC;
    }
    record.bootCount++;
    // The core sets the USB interrupt to 0 in USBDevice.init() (before setup()), the same
    // as the early warning, which then cannot interrupt it. Nothing sets it again.
    NVIC_SetPriority(USB_IRQn, HIL_FAULT_USB_PRIORITY);
}

void hilFaultLoopMark()
{
    loopMillis = millis();
    loopUsbCount = usbIsrCount;
}

// Called first in WDT_Handler (the trampoline below) with the stack of the interrupted code.
extern "C" void hilFaultOnEarlyWarning(uint32_t excReturn, uint32_t msp, uint32_t psp)
{
    const uint32_t sp = frameStack(excReturn, msp, psp);
    if (validFrame(sp))
        memcpy(&record.ew, (const void *)sp, sizeof(Frame));
    else
        memset(&record.ew, 0, sizeof(Frame));
    record.ewSp = sp;
    record.ewExcReturn = excReturn;
    record.ewIcsr = SCB->ICSR;
    record.ewIspr = NVIC->ISPR[0];
    record.ewIser = NVIC->ISER[0];
    record.ewMillis = millis();
    record.ewLoopMillis = loopMillis;
    record.ewUsbCount = usbIsrCount;
    record.ewLoopUsbCount = loopUsbCount;
    record.ewUsbInIsr = usbInIsr;
    for (uint8_t i = 0; i < USB_LAST_COUNT; i++)
        record.ewUsbLast[i] = usbLast[(usbLastIndex + i) % USB_LAST_COUNT];
    readUsb(record.ewUsb);
    record.flags |= FLAG_EARLY_WARNING;
    record.bootOfFault = record.bootCount;
}

extern "C" void hilWdtHandlerBody(void); // watchdog.cpp

// The early warning: the frame of the interrupted code is at the stack pointer on entry,
// before the compiler pushes anything, so the entry is in assembly. EXC_RETURN is kept
// in LR for the handler body (which does not return: it resets the MCU).
extern "C" __attribute__((naked)) void WDT_Handler(void)
{
    __asm volatile(
        "mov r0, lr\n"
        "mrs r1, msp\n"
        "mrs r2, psp\n"
        "push {r0, lr}\n"
        "bl hilFaultOnEarlyWarning\n"
        "pop {r0, r1}\n"
        "mov lr, r1\n"
        "ldr r0, =hilWdtHandlerBody\n"
        "bx r0\n"
        ".ltorg\n");
}

extern "C" __attribute__((noreturn, used)) void hilFaultOnHardFault(uint32_t excReturn, uint32_t msp, uint32_t psp)
{
    const uint32_t sp = frameStack(excReturn, msp, psp);
    if (validFrame(sp))
        memcpy(&record.hf, (const void *)sp, sizeof(Frame));
    else
        memset(&record.hf, 0, sizeof(Frame));
    record.hfSp = sp;
    record.hfExcReturn = excReturn;
    record.hfIcsr = SCB->ICSR;
    record.hfUsbInIsr = usbInIsr;
    record.flags |= FLAG_HARDFAULT;
    record.bootOfFault = record.bootCount;
    // As the core's handler: loop until the watchdog period resets the MCU (the early
    // warning cannot interrupt a HardFault). P2 toggles meanwhile (the early warning holds
    // it high). delayMicroseconds() is a cycle loop and runs here.
    while (true)
    {
        PROBE_TOGGLE(2);
        delayMicroseconds(HIL_FAULT_P2_TOGGLE_US);
    }
}

extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "mov r0, lr\n"
        "mrs r1, msp\n"
        "mrs r2, psp\n"
        "ldr r3, =hilFaultOnHardFault\n"
        "bx r3\n"
        ".ltorg\n");
}

void hilFaultPrint()
{
    const Record &r = record;
    dbgPrint("HIL record: boot %lu flags 0x%lX (1 HardFault, 2 early warning) recorded in boot %lu\n", r.bootCount,
             r.flags, r.bootOfFault);
    dbgPrint("HIL now: USB priority %lu, USB interrupts %lu\n", NVIC_GetPriority(USB_IRQn), usbIsrCount);
    if (r.flags & FLAG_HARDFAULT)
    {
        printFrame("hardfault", r.hf, r.hfSp, r.hfExcReturn, r.hfIcsr);
        dbgPrint("HIL hardfault: in USB interrupt %lu\n", r.hfUsbInIsr);
    }
    if (r.flags & FLAG_EARLY_WARNING)
    {
        printFrame("early warning", r.ew, r.ewSp, r.ewExcReturn, r.ewIcsr);
        dbgPrint("HIL early warning: nvic ispr 0x%08lX iser 0x%08lX millis %lu loop millis %lu\n", r.ewIspr,
                 r.ewIser, r.ewMillis, r.ewLoopMillis);
        dbgPrint("HIL early warning: in USB interrupt %lu, USB interrupts %lu, since loop %lu\n", r.ewUsbInIsr,
                 r.ewUsbCount, r.ewUsbCount - r.ewLoopUsbCount);
        dbgPrint("HIL early warning: USB last (INTFLAG | EPINTSMRY << 16, oldest first):");
        for (uint8_t i = 0; i < USB_LAST_COUNT; i++)
            dbgPrint(" 0x%08lX", r.ewUsbLast[i]);
        dbgPrint("\n");
        const UsbState &u = r.ewUsb;
        dbgPrint("HIL usb: INTFLAG 0x%04X INTENSET 0x%04X EPINTSMRY 0x%04X CTRLA 0x%02X CTRLB 0x%04X STATUS 0x%02X "
                 "FSMSTATUS 0x%02X\n", u.intflag, u.intenset, u.epintsmry, u.ctrla, u.ctrlb, u.status, u.fsmstatus);
        for (uint8_t i = 0; i < USB_EP_COUNT; i++)
            dbgPrint("HIL usb ep %u: EPINTFLAG 0x%02X EPINTENSET 0x%02X EPSTATUS 0x%02X\n", i, u.epintflag[i],
                     u.epintenset[i], u.epstatus[i]);
    }
    record.flags = 0;
    dbgPrint("HIL record done\n");
}

void hilFaultTrigger()
{
    dbgPrint("HIL: HardFault (unaligned read)\n");
    SerialUSB.flush();
    delay(20);
    // A word load from an odd address; in C, GCC splits a known unaligned access into byte
    // loads, which do not fault.
    uint32_t addr = RAM_START + 1;
    __asm volatile("ldr %0, [%0]" : "+r"(addr));
}

#endif // HIL_PROBE
