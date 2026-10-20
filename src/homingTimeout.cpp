// homingTimeout.cpp

#include "homingTimeout.h"
#include "elapsedMs.h"

bool homingTimeoutElapsed(uint32_t startMs, uint32_t nowMs, uint32_t timeoutMs)
{
    if (timeoutMs == 0)
        return false;
    return millisElapsed(startMs, nowMs, timeoutMs);
}
