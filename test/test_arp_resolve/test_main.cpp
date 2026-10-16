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
        last = arpResolvePoll(r, t);
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
    TEST_ASSERT_EQUAL(ARP_RESOLVE_IDLE, arpResolvePoll(r, 1000));
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
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000));
}

void test_successful_probe_resolves()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1005));
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_FALSE(r.pending);
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_IDLE, arpResolvePoll(r, 1010));
}

void test_failed_probe_stays_pending_until_the_interval()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000));
    TEST_ASSERT_FALSE(arpResolveProbeResult(r, false));
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_FALSE(arpResolveShouldSend(r, DEST));
    // the failed probe blocked about 41 ms; the interval runs from the probe start
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1041));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS - 1));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 1000 + ARP_PROBE_INTERVAL_MS + 1));
}

void test_resolved_by_a_later_probe()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    ArpResolveAction last;
    TEST_ASSERT_EQUAL(3, pollRange(r, 1000, 1000 + 3 * ARP_PROBE_INTERVAL_MS - 1, false, last));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1000 + 3 * ARP_PROBE_INTERVAL_MS));
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
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS));
    TEST_ASSERT_FALSE(r.pending);
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_IDLE, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS + 10));
}

void test_timeout_is_checked_before_the_probe()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    // no poll for the whole time (a long block): no probe, the timeout
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS));
}

void test_probe_result_after_the_timeout_is_ignored()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000);
    arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS);
    TEST_ASSERT_FALSE(arpResolveProbeResult(r, true));
}

void test_reply_held_until_resolved()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r)); // not while pending
    arpResolvePoll(r, 1000);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_reply_held_on_the_timeout()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 1000 + ARP_RESOLVE_TIMEOUT_MS));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_no_reply_without_set_dest_ip()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, false, 1000); // start or change of the ID
    arpResolvePoll(r, 1000);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_restart_while_pending_restarts_the_time()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    ArpResolveAction last;
    pollRange(r, 1000, 2500, false, last);
    arpResolveStart(r, DEST, true, 2500); // /setDestIp again
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 2500)); // probe at once
    arpResolveProbeResult(r, false);
    // 2000 ms from the restart, not from the first start
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, 2500 + ARP_PROBE_INTERVAL_MS - 1));
    pollRange(r, 2500 + ARP_PROBE_INTERVAL_MS, 2500 + ARP_RESOLVE_TIMEOUT_MS - 1, false, last);
    TEST_ASSERT_TRUE(r.pending);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, 2500 + ARP_RESOLVE_TIMEOUT_MS));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r)); // one reply for the two /setDestIp
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_restart_with_another_address()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    arpResolvePoll(r, 1000);
    arpResolveProbeResult(r, false);
    arpResolveStart(r, OTHER, true, 1100); // /setDestIp from another address
    TEST_ASSERT_TRUE(arpResolveShouldSend(r, DEST));
    TEST_ASSERT_FALSE(arpResolveShouldSend(r, OTHER));
    TEST_ASSERT_EQUAL_UINT32(OTHER, r.ip);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, 1100));
    TEST_ASSERT_TRUE(arpResolveProbeResult(r, true));
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
}

void test_start_without_reply_keeps_a_held_reply()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    arpResolveStart(r, DEST, false, 1050); // the ID changed: initEthernet()
    arpResolvePoll(r, 1050);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_TRUE(arpResolveTakeReply(r));
}

void test_held_reply_does_not_survive_into_the_next_resolution()
{
    ArpResolve r = {};
    arpResolveStart(r, DEST, true, 1000);
    arpResolvePoll(r, 1000);
    arpResolveProbeResult(r, true);
    // not taken (cannot happen in the firmware); a later start without reply
    arpResolveStart(r, DEST, false, 5000);
    arpResolvePoll(r, 5000);
    arpResolveProbeResult(r, true);
    TEST_ASSERT_FALSE(arpResolveTakeReply(r));
}

void test_wrap_of_millis()
{
    ArpResolve r = {};
    const uint32_t t0 = 0xFFFFFFFFu - 99; // 100 ms before the wrap
    arpResolveStart(r, DEST, false, t0);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, t0));
    arpResolveProbeResult(r, false);
    TEST_ASSERT_EQUAL(ARP_RESOLVE_WAIT, arpResolvePoll(r, t0 + ARP_PROBE_INTERVAL_MS - 1));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_PROBE, arpResolvePoll(r, t0 + ARP_PROBE_INTERVAL_MS));
    arpResolveProbeResult(r, false);
    TEST_ASSERT_NOT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, t0 + ARP_RESOLVE_TIMEOUT_MS - 1));
    TEST_ASSERT_EQUAL(ARP_RESOLVE_TIMEOUT, arpResolvePoll(r, t0 + ARP_RESOLVE_TIMEOUT_MS));
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
    return UNITY_END();
}
