// perfStats.h
// Lightweight timing statistics for the main loop (shown by the 'p' diagnosis command).

#ifndef _PERFSTATS_h
#define _PERFSTATS_h

#include <stdint.h>

struct PerfStat
{
    uint32_t maxUs;
    uint32_t count;
    uint64_t sumUs;
};

inline void perfAdd(PerfStat &stat, uint32_t us)
{
    if (us > stat.maxUs)
        stat.maxUs = us;
    stat.sumUs += us;
    stat.count++;
}

struct PerfStats
{
    PerfStat loop;          // interval between loop() starts
    PerfStat oscReceive;    // OSCMsgReceive(), only when a packet was received
    PerfStat updateServo;   // updateServo(), only when the servo was updated
    PerfStat checkStatus;   // checkStatus()
    PerfStat servoInterval; // actual interval between servo updates
    uint32_t oscSlowPathCount;   // messages dispatched by pattern matching (no exact address match)
    uint32_t oscNotMatchedCount; // messages answered with MessageNotMatch
    uint32_t slotLateCount;      // slots started a whole slot period or more late (rescheduled)
    uint32_t reportClockRealignCount; // position report clock more than 20 ms off millis() (realigned)
    uint32_t udpSendFailedCount;  // UDP sends that failed (destination did not answer ARP)
    uint32_t udpSendDroppedCount; // messages dropped while the sends to the destination were paused
};

extern PerfStats perfStats;

void perfStatsReset();
void perfStatsPrint();

#endif
