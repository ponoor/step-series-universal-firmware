// elapsedMs.cpp

#include "elapsedMs.h"

bool millisElapsed(uint32_t startMs, uint32_t nowMs, uint32_t durationMs)
{
    // Signed difference: negative when the timer started after nowMs was taken.
    const int32_t elapsed = (int32_t)(nowMs - startMs);
    if (elapsed < 0)
        return false;
    return (uint32_t)elapsed >= durationMs;
}
