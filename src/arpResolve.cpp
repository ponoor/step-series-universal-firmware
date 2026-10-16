// arpResolve.cpp

#include "arpResolve.h"

void arpResolveStart(ArpResolve &r, uint32_t ip, bool holdReply, uint32_t nowMs)
{
    r.replyHeld = holdReply || (r.pending && r.replyHeld);
    r.pending = true;
    r.probed = false;
    r.ip = ip;
    r.startMs = nowMs;
}

bool arpResolveShouldSend(const ArpResolve &r, uint32_t ip)
{
    return !r.pending || ip != r.ip;
}

ArpResolveAction arpResolvePoll(ArpResolve &r, uint32_t nowMs)
{
    if (!r.pending)
        return ARP_RESOLVE_IDLE;
    // Unsigned differences: correct across the wrap of millis().
    if ((uint32_t)(nowMs - r.startMs) >= ARP_RESOLVE_TIMEOUT_MS)
    {
        r.pending = false;
        return ARP_RESOLVE_TIMEOUT;
    }
    if (r.probed && (uint32_t)(nowMs - r.lastProbeMs) < ARP_PROBE_INTERVAL_MS)
        return ARP_RESOLVE_WAIT;
    r.probed = true;
    r.lastProbeMs = nowMs;
    return ARP_RESOLVE_PROBE;
}

bool arpResolveProbeResult(ArpResolve &r, bool ok)
{
    if (!r.pending || !ok)
        return false;
    r.pending = false;
    return true;
}

bool arpResolveTakeReply(ArpResolve &r)
{
    if (r.pending || !r.replyHeld)
        return false;
    r.replyHeld = false;
    return true;
}
