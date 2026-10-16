// arpResolve.h
// Resolution of the MAC address of the destination, without hardware access, so that it
// can be tested on the host.
//
// The W5500 keeps the MAC address it resolved by ARP for the destination IP of a UDP
// socket and resolves it again only when a send goes to another destination IP than the
// send before. It keeps it across link changes, and ARP requests or gratuitous ARP from
// the client do not change it (measured, tools/hil "arp"). So when the client is replaced
// by another device with the same IP address (another PC or USB-LAN adapter), the board
// keeps sending to the old MAC address until the resolution is started again.
//
// A send that needs ARP blocks Udp.endPacket() until the reply comes or the W5500 gives
// up (RTR x RCR, about 41 ms). A Wi-Fi client in power save may answer later than that
// (the access point holds the broadcast ARP request until the next DTIM beacon), so one
// try often fails, while tries spread over time are likely to hit a moment when the
// client answers in time.
//
// States: RESOLVED (pending false), the sends to destIp go as before (sendBackoff.h); and
// PENDING (pending true), the MAC address of ip is being resolved. The firmware starts
// the resolution (arpResolveStart()) on /setDestIp, at the start and when the ID changes
// (initEthernet(), if isDestIpSet), and sends one byte to the broadcast address then, so
// that the next send to destIp resolves its MAC address again also for the same IP.
// While PENDING:
// - the messages to ip are dropped, not given to the W5500 (arpResolveShouldSend());
// - every ARP_PROBE_INTERVAL_MS, the first one at once, one byte of UDP is sent to port 9
//   (discard) of ip (a probe, ARP_RESOLVE_PROBE from arpResolvePoll()); a probe that
//   succeeds makes the state RESOLVED, one that fails leaves it PENDING;
// - ARP_RESOLVE_TIMEOUT_MS after the start without success, arpResolvePoll() gives up
//   (ARP_RESOLVE_TIMEOUT) and the state is RESOLVED again;
// - a start again (e.g. another /setDestIp) restarts the time, with the new address.
// The reply to /setDestIp (/destIp) is held while PENDING (holdReply): one reply only, a
// second /setDestIp replaces it. arpResolveTakeReply() tells whether one is held once the
// state is RESOLVED again (on success or on the timeout) and clears it.
// Addresses are IPAddress values converted to uint32_t.

#ifndef _ARPRESOLVE_h
#define _ARPRESOLVE_h

#include <stdint.h>

// 150 ms: not a multiple of the beacon interval (102.4 ms), so the probes fall at
// different phases of the DTIM period.
constexpr uint32_t ARP_PROBE_INTERVAL_MS = 150;
constexpr uint32_t ARP_RESOLVE_TIMEOUT_MS = 2000;

// Zero-initialized: RESOLVED, no reply held.
struct ArpResolve
{
    bool pending;
    bool probed;          // a probe was sent since startMs
    bool replyHeld;       // the reply to /setDestIp is held
    uint32_t ip;          // the address being resolved while pending
    uint32_t startMs;     // millis() of the start
    uint32_t lastProbeMs; // millis() of the last probe
};

enum ArpResolveAction : uint8_t
{
    ARP_RESOLVE_IDLE,    // RESOLVED: nothing to do
    ARP_RESOLVE_WAIT,    // PENDING, no probe due
    ARP_RESOLVE_PROBE,   // PENDING: send a probe to ip now, then call arpResolveProbeResult()
    ARP_RESOLVE_TIMEOUT, // gave up: RESOLVED again
};

// Enters PENDING for ip (also when already PENDING: the time restarts). holdReply: the
// reply to /setDestIp is to be held (replaces a held one); false keeps a held reply if
// PENDING, e.g. on a change of the ID during the resolution.
void arpResolveStart(ArpResolve &r, uint32_t ip, bool holdReply, uint32_t nowMs);

// Whether a message to ip may be given to the W5500 now (false: drop it). False only for
// ip while PENDING.
bool arpResolveShouldSend(const ArpResolve &r, uint32_t ip);

// Called periodically; see ArpResolveAction. The timeout is checked before the probe.
ArpResolveAction arpResolvePoll(ArpResolve &r, uint32_t nowMs);

// Result of the probe sent after ARP_RESOLVE_PROBE (the result of endPacket()). Returns
// true if the state became RESOLVED.
bool arpResolveProbeResult(ArpResolve &r, bool ok);

// Whether a reply to /setDestIp is held, cleared by the call. Always false while PENDING.
bool arpResolveTakeReply(ArpResolve &r);

#endif
