// Host tests of the ARP refresh after a gap in the packets from the destination
// (src/arpRefresh.cpp).

#include <unity.h>
#include "arpRefresh.h"

void setUp() {}
void tearDown() {}

static const uint32_t DEST = 0x0A00000Au;  // 10.0.0.10 (IPAddress byte order)
static const uint32_t OTHER = 0x0B00000Au; // 10.0.0.11

void test_first_packet_from_the_destination_refreshes()
{
    ArpRefresh r = {};
    TEST_ASSERT_TRUE(arpRefreshReceived(r, DEST, DEST, true, 1000));
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT32(DEST, r.ip);
    TEST_ASSERT_EQUAL_UINT32(1000, r.lastRxMs);
}

void test_gap_boundary_1500_ms()
{
    ArpRefresh r = {};
    arpRefreshReceived(r, DEST, DEST, true, 1000);
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, true, 2499)); // 1499 ms
    // the record moved to 2499: 1500 ms from there
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, true, 3998));
    TEST_ASSERT_TRUE(arpRefreshReceived(r, DEST, DEST, true, 5498)); // 1500 ms
}

void test_heartbeat_every_1000_ms_refreshes_once()
{
    ArpRefresh r = {};
    int n = 0;
    for (uint32_t i = 0; i < 100; i++)
        n += arpRefreshReceived(r, DEST, DEST, true, 500 + i * 1000);
    TEST_ASSERT_EQUAL(1, n);
}

void test_every_2000_ms_refreshes_each_time()
{
    ArpRefresh r = {};
    for (uint32_t i = 0; i < 20; i++)
        TEST_ASSERT_TRUE(arpRefreshReceived(r, DEST, DEST, true, 500 + i * 2000));
}

void test_other_source_neither_refreshes_nor_changes_the_record()
{
    ArpRefresh r = {};
    arpRefreshReceived(r, DEST, DEST, true, 1000);
    TEST_ASSERT_FALSE(arpRefreshReceived(r, OTHER, DEST, true, 3000));
    TEST_ASSERT_EQUAL_UINT32(DEST, r.ip);
    TEST_ASSERT_EQUAL_UINT32(1000, r.lastRxMs);
    // still judged from the packet at 1000
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, true, 2000));
    TEST_ASSERT_TRUE(arpRefreshReceived(r, DEST, DEST, true, 3500));
}

void test_new_destination_refreshes_on_its_first_packet()
{
    ArpRefresh r = {};
    arpRefreshReceived(r, DEST, DEST, true, 1000);
    // /setDestIp from OTHER: received while destIp is still DEST
    TEST_ASSERT_FALSE(arpRefreshReceived(r, OTHER, DEST, true, 1100));
    // destIp is OTHER now; the record is of DEST
    TEST_ASSERT_TRUE(arpRefreshReceived(r, OTHER, OTHER, true, 1200));
    TEST_ASSERT_FALSE(arpRefreshReceived(r, OTHER, OTHER, true, 1300));
}

void test_dest_ip_not_set_records_without_refresh()
{
    ArpRefresh r = {};
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, false, 1000));
    TEST_ASSERT_TRUE(r.valid);
    // set right after (e.g. this packet was /setDestIp): no refresh within 1500 ms
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, true, 2000));
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, true, 3400));
    TEST_ASSERT_TRUE(arpRefreshReceived(r, DEST, DEST, true, 4900));
}

void test_wrap_of_millis()
{
    ArpRefresh r = {};
    const uint32_t t0 = 0xFFFFFFFFu - 499; // 500 ms before the wrap
    arpRefreshReceived(r, DEST, DEST, true, t0);
    TEST_ASSERT_FALSE(arpRefreshReceived(r, DEST, DEST, true, 999)); // 1499 ms after
    r.lastRxMs = t0;
    TEST_ASSERT_TRUE(arpRefreshReceived(r, DEST, DEST, true, 1000)); // 1500 ms after
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_packet_from_the_destination_refreshes);
    RUN_TEST(test_gap_boundary_1500_ms);
    RUN_TEST(test_heartbeat_every_1000_ms_refreshes_once);
    RUN_TEST(test_every_2000_ms_refreshes_each_time);
    RUN_TEST(test_other_source_neither_refreshes_nor_changes_the_record);
    RUN_TEST(test_new_destination_refreshes_on_its_first_packet);
    RUN_TEST(test_dest_ip_not_set_records_without_refresh);
    RUN_TEST(test_wrap_of_millis);
    return UNITY_END();
}
