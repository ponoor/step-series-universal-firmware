// perfStats.cpp

#include <string.h>
#include "perfStats.h"
#include "utils.h"

PerfStats perfStats;

void perfStatsReset()
{
    memset(&perfStats, 0, sizeof(perfStats));
}

static void printStat(const char *name, const PerfStat &stat)
{
    uint32_t avg = stat.count ? (uint32_t)(stat.sumUs / stat.count) : 0;
    dbgPrint("%-14s max %6lu us  avg %6lu us  n %lu\n", name,
             (unsigned long)stat.maxUs, (unsigned long)avg, (unsigned long)stat.count);
}

void perfStatsPrint()
{
    PerfStats s = perfStats; // snapshot, then reset
    perfStatsReset();
    printStat("loop", s.loop);
    printStat("oscReceive", s.oscReceive);
    printStat("updateServo", s.updateServo);
    printStat("checkStatus", s.checkStatus);
    printStat("servoInterval", s.servoInterval);
    dbgPrint("oscSlowPath    %lu\n", (unsigned long)s.oscSlowPathCount);
    dbgPrint("oscNotMatched  %lu\n", (unsigned long)s.oscNotMatchedCount);
    dbgPrint("slotLate       %lu\n", (unsigned long)s.slotLateCount);
    dbgPrint("reportClockRealign %lu\n", (unsigned long)s.reportClockRealignCount);
    dbgPrint("udpSendFailed  %lu\n", (unsigned long)s.udpSendFailedCount);
    dbgPrint("udpSendDropped %lu\n", (unsigned long)s.udpSendDroppedCount);
    dbgPrint("arpRefresh     %lu\n", (unsigned long)s.arpRefreshCount);
    dbgPrint("arpRefreshFailed %lu\n", (unsigned long)s.arpRefreshFailedCount);
}
