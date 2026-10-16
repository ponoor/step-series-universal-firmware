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
// the resolution (arpResolveStart()) and sends one byte to the broadcast address then, so
// that the next send to destIp resolves its MAC address again also for the same IP:
// - on /setDestIp when arpRxStartsResolution() is true (another address than destIp, or
//   no packet from destIp for ARP_RX_GAP_MS; a client that sends /setDestIp every second
//   as a heartbeat starts none after the first), or while isDestIpSet is false;
// - with isDestIpSet, when the link is up after the start or a change of the ID
//   (initEthernet()), and when the link comes up again after it was down
//   (arpLinkWatch(): ARP_LINK_WAIT_MS after the link came up, as /booted).
// While PENDING:
// - the messages to ip are dropped, not given to the W5500 (arpResolveShouldSend());
// - one byte of UDP is sent to port 9 (discard) of ip (a probe, ARP_RESOLVE_PROBE from
//   arpResolvePoll()) at once and then every ARP_PROBE_INTERVAL_MS; a probe that
//   succeeds makes the state RESOLVED, one that fails leaves it PENDING. Two probes are
//   always ARP_PROBE_INTERVAL_MS or more apart, also across a start again and across two
//   resolutions, so frequent starts do not block the loop more often;
// - ARP_RESOLVE_TIMEOUT_MS after the start without success, arpResolvePoll() gives up
//   (ARP_RESOLVE_TIMEOUT) and the state is RESOLVED again;
// - while the link is down no probe is sent and the time to the timeout does not run;
// - a start again (another /setDestIp that starts a resolution) restarts the time to the
//   timeout, with the new address, and replaces the held reply.
// The reply to /setDestIp (/destIp) is held while PENDING: one reply only, a later
// /setDestIp replaces it (arpResolveStart() with holdReply, or arpResolveHoldReply() for
// a /setDestIp that starts no resolution: the time to the timeout is kept).
// arpResolveTakeReply() tells whether one is held once the state is RESOLVED again (on
// success or on the timeout) and clears it.
// Addresses are IPAddress values converted to uint32_t.

#ifndef _ARPRESOLVE_h
#define _ARPRESOLVE_h

#include <stdint.h>

// 150 ms: not a multiple of the beacon interval (102.4 ms), so the probes fall at
// different phases of the DTIM period.
constexpr uint32_t ARP_PROBE_INTERVAL_MS = 150;
constexpr uint32_t ARP_RESOLVE_TIMEOUT_MS = 2000;
// A /setDestIp from destIp after a gap this long in the packets from it starts a
// resolution (a replaced client: the replacement takes longer than this).
constexpr uint32_t ARP_RX_GAP_MS = 1500;
// The link must be up this long before the resolution at the start or after the link
// came up again (BOOT_MSG_WAIT_TIME of /booted).
constexpr uint32_t ARP_LINK_WAIT_MS = 1000;

// Zero-initialized: RESOLVED, no reply held, no probe sent yet.
struct ArpResolve
{
    bool pending;
    bool probed;          // a probe was sent (lastProbeMs valid), kept across resolutions
    bool replyHeld;       // the reply to /setDestIp is held
    bool linkDown;        // PENDING and the link was down at the last poll (since downMs)
    uint32_t ip;          // the address being resolved while pending
    uint32_t startMs;     // millis() of the start, moved on by the time the link was down
    uint32_t lastProbeMs; // millis() of the last probe
    uint32_t downMs;      // millis() since the link was down (or of a start while it was)
};

enum ArpResolveAction : uint8_t
{
    ARP_RESOLVE_IDLE,    // RESOLVED: nothing to do
    ARP_RESOLVE_WAIT,    // PENDING, no probe due (or the link is down)
    ARP_RESOLVE_PROBE,   // PENDING: send a probe to ip now, then call arpResolveProbeResult()
    ARP_RESOLVE_TIMEOUT, // gave up: RESOLVED again
};

// Enters PENDING for ip (also when already PENDING: the time to the timeout restarts; the
// probe interval runs on). holdReply: the reply to /setDestIp is to be held (replaces a
// held one); false keeps a held reply if PENDING, e.g. on a change of the ID during the
// resolution.
void arpResolveStart(ArpResolve &r, uint32_t ip, bool holdReply, uint32_t nowMs);

// A /setDestIp that starts no resolution: while PENDING the reply is held (replaces a held
// one, the time to the timeout is kept) and true is returned; when RESOLVED, false: send
// the reply now.
bool arpResolveHoldReply(ArpResolve &r);

// Whether a message to ip may be given to the W5500 now (false: drop it). False only for
// ip while PENDING.
bool arpResolveShouldSend(const ArpResolve &r, uint32_t ip);

// Called periodically (also while the link is down); see ArpResolveAction. The timeout is
// checked before the probe.
ArpResolveAction arpResolvePoll(ArpResolve &r, uint32_t nowMs, bool linkUp);

// Result of the probe sent after ARP_RESOLVE_PROBE (the result of endPacket()). Returns
// true if the state became RESOLVED.
bool arpResolveProbeResult(ArpResolve &r, bool ok);

// Whether a reply to /setDestIp is held, cleared by the call. Always false while PENDING.
bool arpResolveTakeReply(ArpResolve &r);

// The last packet from destIp. Zero-initialized: no record.
struct ArpRxRecord
{
    bool valid;
    uint32_t ip;       // its source address (destIp then)
    uint32_t lastRxMs; // its millis()
};

// For a /setDestIp from srcIp, before it changes destIp: whether it starts a resolution
// (srcIp is not destIp, or no packet from it for ARP_RX_GAP_MS or more, or no record).
bool arpRxStartsResolution(const ArpRxRecord &rec, uint32_t srcIp, uint32_t destIp, uint32_t nowMs);

// After every packet, with destIp as it is after the packet was handled: records the
// packet if it came from destIp (a /setDestIp from another address too).
void arpRxReceived(ArpRxRecord &rec, uint32_t srcIp, uint32_t destIp, uint32_t nowMs);

// Link state for the resolution at the start and after the link came up again.
// Zero-initialized: no start wanted, the link not seen up.
struct ArpLinkWatch
{
    bool wasUp;        // the link was up at the last call
    bool startWanted;  // a resolution is to start once the link is up long enough
    uint32_t linkedMs; // millis() of the last call with the link down (0 at first, as /booted)
};

// A resolution is wanted once the link is up (initEthernet()).
void arpLinkWatchRequest(ArpLinkWatch &w);

// Called with the link state at every housekeeping run (as /booted waits for the link).
// A link that comes up after it was up and then down wants a resolution. Returns true
// (and clears the wish) when a wanted resolution is to start now: the link up for
// ARP_LINK_WAIT_MS or more. The firmware starts it only with isDestIpSet.
bool arpLinkWatchPoll(ArpLinkWatch &w, bool linkUp, uint32_t nowMs);

#endif
