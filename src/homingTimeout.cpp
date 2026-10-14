// homingTimeout.cpp

#include "homingTimeout.h"

bool homingTimeoutElapsed(uint32_t startMs, uint32_t nowMs, uint32_t timeoutMs)
{
    if (timeoutMs == 0)
        return false;
    // Signed difference: negative when the step started after nowMs was taken.
    const int32_t elapsed = (int32_t)(nowMs - startMs);
    if (elapsed < 0)
        return false;
    return (uint32_t)elapsed >= timeoutMs;
}
