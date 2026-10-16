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
    // Resolution of the MAC address of destIp (arpResolve.h)
    uint32_t arpResolveCount;     // resolutions started (PENDING entered, with the dummy broadcast)
    uint32_t arpProbeCount;       // probes sent
    uint32_t arpProbeFailedCount; // of these, failed (no ARP reply in time)
    uint32_t arpResolvedCount;    // resolutions ended by a successful probe
    uint32_t arpTimeoutCount;     // resolutions given up after 2 s
    uint32_t arpDestIpSentCount;  // held /destIp replies sent (resolved or given up)
    uint32_t arpDroppedCount;     // messages to destIp dropped while PENDING
};

extern PerfStats perfStats;

void perfStatsReset();
void perfStatsPrint();

#endif
