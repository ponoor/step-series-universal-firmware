// arpResolve.cpp

#include "arpResolve.h"

void arpResolveStart(ArpResolve &r, uint32_t ip, bool holdReply, uint32_t nowMs)
{
    r.replyHeld = holdReply || (r.pending && r.replyHeld);
    if (!r.pending)
        r.linkDown = false;
    else if (r.linkDown)
        r.downMs = nowMs; // the time to the timeout starts when the link is up again
    r.pending = true;
    r.failed = false;
    r.ip = ip;
    r.startMs = nowMs;
}

bool arpResolveHoldReply(ArpResolve &r)
{
    if (!r.pending)
        return false;
    r.replyHeld = true;
    return true;
}

bool arpResolveShouldSend(const ArpResolve &r, uint32_t ip)
{
    return !r.pending || ip != r.ip;
}

ArpResolveAction arpResolvePoll(ArpResolve &r, uint32_t nowMs, bool linkUp)
{
    if (!r.pending)
        return ARP_RESOLVE_IDLE;
    if (!linkUp)
    {
        if (!r.linkDown)
        {
            r.linkDown = true;
            r.downMs = nowMs;
        }
        return ARP_RESOLVE_WAIT;
    }
    if (r.linkDown)
    {
        r.linkDown = false;
        r.startMs += nowMs - r.downMs; // the time to the timeout did not run
    }
    // Unsigned differences: correct across the wrap of millis(). A probe 49.7 days ago
    // looks recent: the next one waits up to ARP_PROBE_INTERVAL_MS more, not more.
    if ((uint32_t)(nowMs - r.startMs) >= ARP_RESOLVE_TIMEOUT_MS)
    {
        r.pending = false;
        r.failed = true;
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

void arpResolveSent(ArpResolve &r, uint32_t ip)
{
    if (!r.pending && ip == r.ip)
        r.failed = false;
}

bool arpResolveTakeReply(ArpResolve &r)
{
    if (r.pending || !r.replyHeld)
        return false;
    r.replyHeld = false;
    return true;
}

bool arpRxStartsResolution(const ArpRxRecord &rec, uint32_t srcIp, uint32_t destIp, uint32_t nowMs)
{
    return srcIp != destIp || !rec.valid || rec.ip != srcIp || (uint32_t)(nowMs - rec.lastRxMs) >= ARP_RX_GAP_MS;
}

void arpRxReceived(ArpRxRecord &rec, uint32_t srcIp, uint32_t destIp, uint32_t nowMs)
{
    if (srcIp != destIp)
        return;
    rec.valid = true;
    rec.ip = srcIp;
    rec.lastRxMs = nowMs;
}

void arpLinkWatchRequest(ArpLinkWatch &w)
{
    w.startWanted = true;
}

bool arpLinkWatchPoll(ArpLinkWatch &w, bool linkUp, uint32_t nowMs)
{
    if (!linkUp)
    {
        if (w.wasUp)
            w.startWanted = true; // the link went down: resolve again once it is back
        w.wasUp = false;
        w.linkedMs = nowMs;
        return false;
    }
    w.wasUp = true;
    if (!w.startWanted || (uint32_t)(nowMs - w.linkedMs) < ARP_LINK_WAIT_MS)
        return false;
    w.startWanted = false;
    return true;
}
