// latchedReport.cpp

#include "latchedReport.h"

bool latchedReportPoll(LatchedReport &r, bool flagSet, bool enabled, uint32_t nowMs)
{
    if (!enabled)
    {
        r.pending = false;
        return false;
    }
    if (flagSet)
        r.pending = true;
    // Unsigned difference: correct across the wrap of millis().
    if (r.pending && (!r.sent || (uint32_t)(nowMs - r.lastSentMs) >= LATCHED_REPORT_INTERVAL_MS))
    {
        r.pending = false;
        r.sent = true;
        r.lastSentMs = nowMs;
        return true;
    }
    return false;
}

void latchedReportCancel(LatchedReport &r)
{
    r.pending = false;
}
