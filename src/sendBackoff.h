// sendBackoff.h
// Pause of the sends to a destination that does not answer, without hardware access, so
// that it can be tested on the host.
//
// A UDP send to a unicast address that does not answer ARP blocks Udp.endPacket() until
// the W5500 gives up (about 41 ms, measured), and endPacket() then returns 0. After such a
// failure the destination is unreachable: the messages to it are dropped (not queued) for
// SEND_BACKOFF_MS from the failure. The first message after that is sent (one retry): a
// success resumes the sends, a failure pauses them again for SEND_BACKOFF_MS from this
// failure. A packet received from the destination resumes the sends at once.
//
// One destination is tracked (the firmware sends unicast to destIp only); a failure to
// another address replaces it. The broadcast address is never paused (no ARP).
// Addresses are IPAddress values converted to uint32_t.

#ifndef _SENDBACKOFF_h
#define _SENDBACKOFF_h

#include <stdint.h>

constexpr uint32_t SEND_BACKOFF_MS = 1000;
constexpr uint32_t SEND_BACKOFF_BROADCAST = 0xFFFFFFFFu; // 255.255.255.255

// Zero-initialized: no destination paused.
struct SendBackoff
{
    bool paused;
    uint32_t ip;       // the unreachable destination while paused
    uint32_t failedMs; // millis() of the last failed send to it
};

enum SendBackoffEvent : uint8_t
{
    SEND_BACKOFF_NONE,
    SEND_BACKOFF_PAUSED,  // the sends to ip were paused (not on a failed retry)
    SEND_BACKOFF_RESUMED, // the sends to the paused destination were resumed
};

// Whether a message to ip is to be sent now (false: drop it). True for any address other
// than the paused one, and for the paused one SEND_BACKOFF_MS or more after the failure.
bool sendBackoffShouldSend(const SendBackoff &b, uint32_t ip, uint32_t nowMs);

// Result of a send made after sendBackoffShouldSend() returned true; nowMs is taken
// after the send (the failure time).
SendBackoffEvent sendBackoffSent(SendBackoff &b, uint32_t ip, bool ok, uint32_t nowMs);

// A packet was received from ip.
SendBackoffEvent sendBackoffReceived(SendBackoff &b, uint32_t ip);

#endif
