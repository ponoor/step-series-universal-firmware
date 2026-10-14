// arpRefresh.cpp

#include "arpRefresh.h"

bool arpRefreshReceived(ArpRefresh &r, uint32_t srcIp, uint32_t destIp, bool destIpSet, uint32_t nowMs)
{
    if (srcIp != destIp)
        return false; // not the destination: the record is kept
    // Unsigned difference: correct across the wrap of millis().
    const bool gap = !r.valid || r.ip != srcIp || (uint32_t)(nowMs - r.lastRxMs) >= ARP_REFRESH_GAP_MS;
    r.valid = true;
    r.ip = srcIp;
    r.lastRxMs = nowMs;
    return gap && destIpSet;
}
