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
};

extern PerfStats perfStats;

void perfStatsReset();
void perfStatsPrint();

#endif
