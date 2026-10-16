// Host tests of the resolution of the MAC address of the destination
// (src/arpResolve.cpp).

#include <unity.h>
#include "arpResolve.h"

void setUp() {}
void tearDown() {}

static const uint32_t DEST = 0x0A00000Au;      // 10.0.0.10 (IPAddress byte order)
static const uint32_t OTHER = 0x0B00000Au;     // 10.0.0.11
static const uint32_t BROADCAST = 0xFFFFFFFFu; // 255.255.255.255

// Polls at 10 ms steps (the housekeeping period) from fromMs to toMs inclusive, answering
// every probe with ok; returns the number of probes and the last action in last.
static int pollRange(ArpResolve &r, uint32_t fromMs, uint32_t toMs, bool ok, ArpResolveAction &last)
{
    int probes = 0;
    last = ARP_RESOLVE_IDLE;
    for (uint32_t t = fromMs; (int32_t)(toMs - t) >= 0; t += 10)
    {
        last = arpResolvePoll(r, t, true);
        if (last == ARP_RESOLVE_PROBE)
        {
            probes++;
            arpResolveProbeResult(r, ok);
        }
        if (last == ARP_RESOLVE_TIMEOUT)
            break;
    }
    return probes;
}

void test_resolved_by_default()
{
    ArpResolve r = {};
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_IDLE, arpResolvePoll(r, 1000, true));
    TEST_ASSERT_FALSE(arpResolveProbeResult(r, true));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_pending_drops_messages_to_the_destination_only()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_FALSE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, OTHER));
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, BROADCAST));
}

void test_first_probe_at_once()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000, true));
}

void test_successful_probe_resolves()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1005, true));
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_FALSE(r.pending);
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_IDLE, arpResolvePoll(r, 1010, true));
}

void test_failed_probe_stays_pending_until_the_interval()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000, true));
    TEST_ASSERT_FALSE(arpResolveProbeResult(r, false));
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_FALSE(arpResolveShouldSend(r, DEST));
    // the failed probe blocked about 41 ms; the interval runs from the probe start
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1041, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS - 1, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS + 1, true));
}

void test_resolved_by_a_later_probe()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    ArpResolveAction last;
    TEST_ASSERT_EQUAL(3, pollRange(r, 1000, 1000 + 3 * ARP_PROBE_INTERVAL_MS - 1, false, last));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000 + 3 * ARP_PROBE_INTERVAL_MS, true));
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r)); // one reply only
}

void test_timeout_gives_up()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    ArpResolveAction last;
    const int probes = pollRange(r, 1000, 1000 + ARP_RESOLVE_TIMEOUT_MS - 1, false, last);
    // 0, 150, ..., 1950 ms
    TEST_ASSERT_EQUAL((ARP_RESOLVE_TIMEOUT_MS - 1) / ARP_PROBE_INTERVAL_MS + 1, probes);
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true));
    TEST_ASSERT_FALSE(r.pending);
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_IDLE, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS + 10, true));
}

void test_timeout_is_checked_before_the_probe()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    // no poll for the whole time (a long block): no probe, the timeout
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true));
}

void test_probe_result_after_the_timeout_is_ignored()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true);
    TEST_ASSERT_FALSE(arpResolveProbeResult(r, true));
}

void test_reply_held_until_resolved()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r)); // not while pending
    arpResolvePoll(r, 1000, true);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_reply_held_on_the_timeout()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_no_reply_without_set_dest_ip()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000); // start or change of the ID
    arpResolvePoll(r, 1000, true);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_restart_while_pending_restarts_the_time()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    ArpResolveAction last;
    pollRange(r, 1000, 2500, false, last);        // the last probe at 2500
    arpResolveStart(r, DEST, true, 2500);         // /setDestIp again
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 2500, true)); // the interval runs on
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 2500 + ARP_PROBE_INTERVAL_MS - 1, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 2500 + ARP_PROBE_INTERVAL_MS, true));
    arpResolveProbeResult(r, false);
    // 2000 ms from the restart, not from the first start (3000)
    pollRange(r, 2500 + ARP_PROBE_INTERVAL_MS + 10, 2500 + ARP_RESOLVE_TIMEOUT_MS - 1, false, last);
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 2500 + ARP_RESOLVE_TIMEOUT_MS, true));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r)); // one reply for the two /setDestIp
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_restart_with_another_address()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    arpResolvePoll(r, 1000, true);
    arpResolveProbeResult(r, false);
    arpResolveStart(r, OTHER, true, 1100); // /setDestIp from another address
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_FALSE(arpResolveShouldSend(r, OTHER));
    TEST_ASSERT_EQUAL_UINT32(OTHER, r.ip);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1100, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1150, true));
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
}

void test_start_without_reply_keeps_a_held_reply()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    arpResolveStart(r, DEST, false, 1050); // the ID changed: initEthernet()
    arpResolvePoll(r, 1050, true);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
}

void test_held_reply_does_not_survive_into_the_next_resolution()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    arpResolvePoll(r, 1000, true);
    arpResolveProbeResult(r, true);
    // not taken (cannot happen in the firmware); a later start without reply
    arpResolveStart(r, DEST, false, 5000);
    arpResolvePoll(r, 5000, true);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_wrap_of_millis()
{
    ArpResolve r = {};
    const uint32_t t0 = 0xFFFFFFFFu - 99; // 100 ms before the wrap
    arpResolveStart(r, DEST, false, t0);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, t0, true));
    arpResolveProbeResult(r, false);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, t0 + ARP_PROBE_INTERVAL_MS - 1, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, t0 + ARP_PROBE_INTERVAL_MS, true));
    arpResolveProbeResult(r, false);
    TEST_ASSERT_NOT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, t0 + ARP_RESOLVE_TIMEOUT_MS - 1, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, t0 + ARP_RESOLVE_TIMEOUT_MS, true));
}

// --- probe interval across starts ---

void test_frequent_starts_keep_the_probe_interval()
{
    // /setDestIp every 50 ms (check C8) from a client that does not answer ARP
    ArpResolve r = {};
    uint32_t lastProbe = 0;
    bool any = false;
    int probes = 0;
    for (uint32_t t = 1000; t < 5000; t += 10)
    {
        if ((t - 1000) % 50 == 0)
            arpResolveStart(r, DEST, true, t);
        if (arpResolvePoll(r, t, true) == ARP_RESOLVE_PROBE)
        {
            if (any)
                TEST_ASSERT_GREATER_OR_EQUAL_UINT32(ARP_PROBE_INTERVAL_MS, t - lastProbe);
            any = true;
            lastProbe = t;
            probes++;
            arpResolveProbeResult(r, false);
        }
    }
    TEST_ASSERT_TRUE(r.pending); // never timed out: each start restarts the time
    TEST_ASSERT_EQUAL(4000 / ARP_PROBE_INTERVAL_MS + 1, probes);
}

void test_new_resolution_keeps_the_probe_interval()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    arpResolvePoll(r, 1000, true);
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true)); // resolved at 1000
    arpResolveStart(r, OTHER, true, 1050);            // a new resolution 50 ms later
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1050, true));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS, true));
    // long after the last probe: the first probe at once
    arpResolveProbeResult(r, true);
    arpResolveStart(r, DEST, false, 9000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 9000, true));
}

void test_start_again_renews_the_timeout_origin()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    ArpResolveAction last;
    pollRange(r, 1000, 2900, false, last);
    TEST_ASSERT_TRUE(r.pending);
    arpResolveStart(r, DEST, true, 2900);
    TEST_ASSERT_EQUAL_UINT32(2900, r.startMs);
    pollRange(r, 2910, 2900 + ARP_RESOLVE_TIMEOUT_MS - 10, false, last);
    TEST_ASSERT_TRUE(r.pending); // past 1000 + 2000
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 2900 + ARP_RESOLVE_TIMEOUT_MS, true));
}

// --- /setDestIp that starts no resolution ---

void test_hold_reply_while_pending_keeps_the_timeout()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000); // at the start (no reply held)
    TEST_ASSERT_TRUE(arpResolveHoldReply(r)); // /setDestIp that starts none
    TEST_ASSERT_EQUAL_UINT32(1000, r.startMs);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_hold_reply_when_resolved_sends_now()
{
    ArpResolve r = {};
    TEST_ASSERT_FALSE(arpResolveHoldReply(r));
    TEST_ASSERT_FALSE(r.pending);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

// --- the record of the packets from destIp (arpRx) ---

// /setDestIp from src at t: the decision, then the record as the firmware updates it.
static bool setDestIpAt(ArpRxRecord &rec, uint32_t &destIp, uint32_t src, uint32_t t)
{
    const bool start = arpRxStartsResolution(rec, src, destIp, t);
    destIp = src;
    arpRxReceived(rec, src, destIp, t);
    return start;
}

void test_set_dest_ip_every_second_starts_once()
{
    ArpRxRecord rec = {};
    uint32_t destIp = DEST;
    int n = 0;
    for (uint32_t i = 0; i < 30; i++)
        n += setDestIpAt(rec, destIp, DEST, 1000 + i * 1000);
    TEST_ASSERT_EQUAL(1, n); // the first one (no record)
}

void test_set_dest_ip_after_a_gap_starts()
{
    ArpRxRecord rec = {};
    uint32_t destIp = DEST;
    setDestIpAt(rec, destIp, DEST, 1000);
    TEST_ASSERT_FALSE(setDestIpAt(rec, destIp, DEST, 1000 + ARP_RX_GAP_MS - 1));
    TEST_ASSERT_TRUE(setDestIpAt(rec, destIp, DEST, 1000 + 2 * ARP_RX_GAP_MS - 1)); // 1500 ms
    int n = 0;
    for (uint32_t i = 1; i <= 10; i++) // every 2.0 s
        n += setDestIpAt(rec, destIp, DEST, 10000 + i * 2000);
    TEST_ASSERT_EQUAL(10, n);
}

void test_other_packets_from_dest_ip_count_against_the_gap()
{
    ArpRxRecord rec = {};
    uint32_t destIp = DEST;
    setDestIpAt(rec, destIp, DEST, 1000);
    for (uint32_t t = 2000; t <= 6000; t += 1000) // /getVersion every second
        arpRxReceived(rec, DEST, destIp, t);
    TEST_ASSERT_FALSE(setDestIpAt(rec, destIp, DEST, 6500));
    // packets from another address do not
    arpRxReceived(rec, OTHER, destIp, 7500);
    TEST_ASSERT_TRUE(setDestIpAt(rec, destIp, DEST, 8000));
}

void test_set_dest_ip_from_another_address_starts()
{
    ArpRxRecord rec = {};
    uint32_t destIp = DEST;
    setDestIpAt(rec, destIp, DEST, 1000);
    TEST_ASSERT_TRUE(setDestIpAt(rec, destIp, OTHER, 1100));
    TEST_ASSERT_EQUAL_UINT32(OTHER, destIp);
    // the new destIp was recorded by its /setDestIp: its next one 1 s later starts none
    TEST_ASSERT_FALSE(setDestIpAt(rec, destIp, OTHER, 2100));
    TEST_ASSERT_TRUE(setDestIpAt(rec, destIp, DEST, 2200)); // and back
}

void test_set_dest_ip_without_resolution_while_pending()
{
    // the resolution at the start runs; a /setDestIp from destIp within 1.5 s of its
    // last packet: no new start, the reply held, the timeout origin kept
    ArpRxRecord rec = {};
    ArpResolve r = {};
    uint32_t destIp = DEST;
    arpRxReceived(rec, DEST, destIp, 900); // e.g. /getVersion
    arpResolveStart(r, destIp, false, 1000);
    arpResolvePoll(r, 1000, true);
    arpResolveProbeResult(r, false);
    TEST_ASSERT_FALSE(setDestIpAt(rec, destIp, DEST, 1200));
    TEST_ASSERT_TRUE(arpResolveHoldReply(r));
    TEST_ASSERT_EQUAL_UINT32(1000, r.startMs);
    TEST_ASSERT_FALSE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1150, true));
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
}

void test_rx_gap_wrap_of_millis()
{
    ArpRxRecord rec = {};
    uint32_t destIp = DEST;
    const uint32_t t0 = 0xFFFFFFFFu - 499;
    setDestIpAt(rec, destIp, DEST, t0);
    TEST_ASSERT_FALSE(setDestIpAt(rec, destIp, DEST, 999)); // 1499 ms
    rec.lastRxMs = t0;
    TEST_ASSERT_TRUE(setDestIpAt(rec, destIp, DEST, 1000)); // 1500 ms
}

// --- link ---

void test_link_down_pauses_probes_and_timeout()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000, true));
    arpResolveProbeResult(r, false);
    // down from 1100 to 6100: no probe, no timeout
    for (uint32_t t = 1100; t < 6100; t += 10)
        TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, t, false));
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 6100, true));
    arpResolveProbeResult(r, false);
    // 100 ms ran before the link went down: 1900 ms left from 6100
    ArpResolveAction last;
    pollRange(r, 6110, 6100 + 1900 - 10, false, last);
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 6100 + 1900, true));
}

void test_start_while_link_down_starts_the_time_at_link_up()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    arpResolvePoll(r, 1000, false); // down
    arpResolvePoll(r, 3000, false);
    arpResolveStart(r, DEST, true, 3000); // /setDestIp while down
    arpResolvePoll(r, 4000, false);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 5000, true));
    arpResolveProbeResult(r, false);
    TEST_ASSERT_NOT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 5000 + ARP_RESOLVE_TIMEOUT_MS - 1, true));
    arpResolveProbeResult(r, false);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 5000 + ARP_RESOLVE_TIMEOUT_MS, true));
}

void test_link_watch_at_the_start()
{
    ArpLinkWatch w = {};
    arpLinkWatchRequest(w); // initEthernet()
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, false, 500));
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, false, 2500)); // link still down
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 2510));
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 2500 + ARP_LINK_WAIT_MS - 1));
    TEST_ASSERT_TRUE(arpLinkWatchPoll(w, true, 2500 + ARP_LINK_WAIT_MS));
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 2500 + ARP_LINK_WAIT_MS + 10)); // once
}

void test_link_watch_link_up_at_once_as_booted()
{
    // link already up at the first poll: from millis() 0, as /booted
    ArpLinkWatch w = {};
    arpLinkWatchRequest(w);
    TEST_ASSERT_TRUE(arpLinkWatchPoll(w, true, ARP_LINK_WAIT_MS + 200));
}

void test_link_watch_link_comes_back()
{
    ArpLinkWatch w = {};
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 5000)); // up, nothing wanted
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, false, 6000)); // cable out
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, false, 8990)); // polled every 10 ms while down
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 9000));
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, false, 9100)); // bounce
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 9200));
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 9100 + ARP_LINK_WAIT_MS - 1));
    TEST_ASSERT_TRUE(arpLinkWatchPoll(w, true, 9100 + ARP_LINK_WAIT_MS));
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 20000));
}

void test_link_watch_cleared_by_another_start()
{
    ArpLinkWatch w = {};
    arpLinkWatchRequest(w);
    w.startWanted = false; // /setDestIp started a resolution first (startArpResolve())
    TEST_ASSERT_FALSE(arpLinkWatchPoll(w, true, 5000));
}

// --- after a resolution that gave up ---

void test_failed_after_timeout_until_a_send_succeeds()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    TEST_ASSERT_FALSE(r.failed);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true));
    TEST_ASSERT_TRUE(r.failed); // the next /setDestIp starts a resolution in any case
    arpResolveSent(r, OTHER);   // another address: still failed
    TEST_ASSERT_TRUE(r.failed);
    arpResolveSent(r, DEST);    // e.g. the held /destIp went out after the timeout
    TEST_ASSERT_FALSE(r.failed);
}

void test_failed_cleared_by_a_start_and_kept_by_success()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS, true);
    TEST_ASSERT_TRUE(r.failed);
    arpResolveStart(r, DEST, true, 5000);
    TEST_ASSERT_FALSE(r.failed);
    arpResolveSent(r, DEST); // ignored while pending
    TEST_ASSERT_TRUE(r.pending);
    arpResolvePoll(r, 5000, true);
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_FALSE(r.failed);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_resolved_by_default);
    RUN_TEST(test_pending_drops_messages_to_the_destination_only);
    RUN_TEST(test_first_probe_at_once);
    RUN_TEST(test_successful_probe_resolves);
    RUN_TEST(test_failed_probe_stays_pending_until_the_interval);
    RUN_TEST(test_resolved_by_a_later_probe);
    RUN_TEST(test_timeout_gives_up);
    RUN_TEST(test_timeout_is_checked_before_the_probe);
    RUN_TEST(test_probe_result_after_the_timeout_is_ignored);
    RUN_TEST(test_reply_held_until_resolved);
    RUN_TEST(test_reply_held_on_the_timeout);
    RUN_TEST(test_no_reply_without_set_dest_ip);
    RUN_TEST(test_restart_while_pending_restarts_the_time);
    RUN_TEST(test_restart_with_another_address);
    RUN_TEST(test_start_without_reply_keeps_a_held_reply);
    RUN_TEST(test_held_reply_does_not_survive_into_the_next_resolution);
    RUN_TEST(test_wrap_of_millis);
    RUN_TEST(test_frequent_starts_keep_the_probe_interval);
    RUN_TEST(test_new_resolution_keeps_the_probe_interval);
    RUN_TEST(test_start_again_renews_the_timeout_origin);
    RUN_TEST(test_hold_reply_while_pending_keeps_the_timeout);
    RUN_TEST(test_hold_reply_when_resolved_sends_now);
    RUN_TEST(test_set_dest_ip_every_second_starts_once);
    RUN_TEST(test_set_dest_ip_after_a_gap_starts);
    RUN_TEST(test_other_packets_from_dest_ip_count_against_the_gap);
    RUN_TEST(test_set_dest_ip_from_another_address_starts);
    RUN_TEST(test_set_dest_ip_without_resolution_while_pending);
    RUN_TEST(test_rx_gap_wrap_of_millis);
    RUN_TEST(test_link_down_pauses_probes_and_timeout);
    RUN_TEST(test_start_while_link_down_starts_the_time_at_link_up);
    RUN_TEST(test_link_watch_at_the_start);
    RUN_TEST(test_link_watch_link_up_at_once_as_booted);
    RUN_TEST(test_link_watch_link_comes_back);
    RUN_TEST(test_link_watch_cleared_by_another_start);
    RUN_TEST(test_failed_after_timeout_until_a_send_succeeds);
    RUN_TEST(test_failed_cleared_by_a_start_and_kept_by_success);
    return UNITY_END();
}
