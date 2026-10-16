// sendBackoff.cpp

#include "sendBackoff.h"

bool sendBackoffShouldSend(const SendBackoff &b, uint32_t ip, uint32_t nowMs)
{
    if (!b.paused || ip != b.ip)
        return true;
    // Unsigned difference: correct across the wrap of millis().
    return (uint32_t)(nowMs - b.failedMs) >= SEND_BACKOFF_MS;
}

SendBackoffEvent sendBackoffSent(SendBackoff &b, uint32_t ip, bool ok, uint32_t nowMs)
{
    if (ip == SEND_BACKOFF_BROADCAST)
        return SEND_BACKOFF_NONE;
    const bool isPausedIp = b.paused && (ip == b.ip);
    if (ok)
    {
        if (!isPausedIp)
            return SEND_BACKOFF_NONE;
        b.paused = false;
        return SEND_BACKOFF_RESUMED;
    }
    b.failedMs = nowMs;
    if (isPausedIp)
        return SEND_BACKOFF_NONE; // failed retry: paused again
    b.paused = true;
    b.ip = ip;
    return SEND_BACKOFF_PAUSED;
}

SendBackoffEvent sendBackoffReceived(SendBackoff &b, uint32_t ip)
{
    if (!b.paused || ip != b.ip)
        return SEND_BACKOFF_NONE;
    b.paused = false;
    return SEND_BACKOFF_RESUMED;
}

void sendBackoffClear(SendBackoff &b)
{
    b.paused = false;
}
