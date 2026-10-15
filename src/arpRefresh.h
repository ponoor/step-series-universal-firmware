// arpRefresh.h
// When to make the W5500 resolve the MAC address of the destination again, without
// hardware access, so that it can be tested on the host.
//
// The W5500 keeps the MAC address it resolved by ARP for the destination IP of a UDP
// socket and resolves it again only when a send goes to another destination IP than the
// send before. It keeps it across link changes, and ARP requests or gratuitous ARP from
// the client do not change it (measured, tools/hil "arp"). So when the client is replaced
// by another device with the same IP address (another PC or USB-LAN adapter), the board
// keeps sending to the old MAC address until it restarts.
//
// Rule: when a packet is received from destIp while isDestIpSet, and ARP_REFRESH_GAP_MS
// or more have passed since the last packet from that address (or there is no record, or
// the record is of another address), the firmware sends one byte to the broadcast
// address (another destination IP) before it handles the packet; the next send to destIp
// then resolves its MAC address again. The record (address, time) is updated on every
// packet from destIp, also when isDestIpSet is false; packets from other addresses do
// not change it. A client that sends at intervals shorter than ARP_REFRESH_GAP_MS (e.g.
// /setDestIp every second as a heartbeat) causes no refresh after the first one; a gap of
// exactly ARP_REFRESH_GAP_MS already causes one.
// Addresses are IPAddress values converted to uint32_t.

#ifndef _ARPREFRESH_h
#define _ARPREFRESH_h

#include <stdint.h>

constexpr uint32_t ARP_REFRESH_GAP_MS = 1500;

// Zero-initialized: no record.
struct ArpRefresh
{
    bool valid;
    uint32_t ip;       // address of the last packet from destIp
    uint32_t lastRxMs; // millis() of that packet
};

// A packet was received from srcIp. Returns true if the MAC address of destIp is to be
// resolved again (send the dummy broadcast before handling the packet).
bool arpRefreshReceived(ArpRefresh &r, uint32_t srcIp, uint32_t destIp, bool destIpSet, uint32_t nowMs);

#endif
