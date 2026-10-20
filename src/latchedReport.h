// latchedReport.h
// Rate limit of the reports of latched driver flags (/overCurrent, /stall), without
// hardware access, so that it can be tested on the host.
//
// The flags (OCD, STEP_LOSS_A/B or STALL_A/B) are latched in the driver and cleared by
// every status read; while the condition lasts they are set again at every read. A flag
// read set makes the report pending. A pending report is sent at once if the report of
// this motor and kind has never been sent, or LATCHED_REPORT_INTERVAL_MS or more after the
// last one; otherwise it waits (several detections in between give one report). While the
// report is disabled nothing is pending, so enabling it does not send an old detection.

#ifndef _LATCHEDREPORT_h
#define _LATCHEDREPORT_h

#include <stdint.h>

constexpr uint32_t LATCHED_REPORT_INTERVAL_MS = 200;

// State of one motor and one kind of report. Zero-initialized: nothing pending, never sent.
struct LatchedReport
{
    bool pending;
    bool sent;           // sent at least once
    uint32_t lastSentMs; // millis() of the last report
};

// Called at every status read with the flag just read, whether the report is enabled,
// and millis(). Returns true if the report is to be sent now (the state is updated).
bool latchedReportPoll(LatchedReport &r, bool flagSet, bool enabled, uint32_t nowMs);

// The report was disabled: drop a pending detection.
void latchedReportCancel(LatchedReport &r);

#endif
