// Host tests of the pause of the sends to an unreachable destination (src/sendBackoff.cpp).

#include <unity.h>
#include "sendBackoff.h"

void setUp() {}
void tearDown() {}

static const uint32_t DEST = 0xFA00000Au;  // 10.0.0.250 (IPAddress byte order)
static const uint32_t OTHER = 0x1C00000Au; // 10.0.0.28

// A send that was made and failed at t: the state is paused on DEST from t.
static SendBackoff pausedAt(uint32_t t)
{
    SendBackoff b = {};
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, t));
    TEST_ASSERT_EQUAL(SEND_BACKOFF_PAUSED, sendBackoffSent(b, DEST, false, t));
    return b;
}

void test_sends_while_not_paused()
{
    SendBackoff b = {};
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 0));
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffSent(b, DEST, true, 0));
    TEST_ASSERT_FALSE(b.paused);
}

void test_no_send_for_1000_ms_after_a_failure()
{
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 5000));
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 5500));
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 5999));
}

void test_retry_exactly_1000_ms_after_the_failure()
{
    // 1000 ms after the failure counts: the next message is sent.
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 6000));
}

void test_successful_retry_resumes()
{
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 6010));
    TEST_ASSERT_EQUAL(SEND_BACKOFF_RESUMED, sendBackoffSent(b, DEST, true, 6010));
    TEST_ASSERT_FALSE(b.paused);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 6011));
}

void test_failed_retry_pauses_for_another_1000_ms()
{
    // The retry at 6000 blocks until 6041 and fails: paused from 6041, no new PAUSED event.
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 6000));
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffSent(b, DEST, false, 6041));
    TEST_ASSERT_TRUE(b.paused);
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 6041));
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 7040));
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 7041));
}

void test_one_retry_per_interval()
{
    // A message every 10 ms for 5 s, each send to DEST failing after blocking 41 ms (the
    // clock goes on during the block): one send per 1000 ms + the block.
    SendBackoff b = {};
    int sends = 0, paused = 0, dropped = 0;
    for (uint32_t t = 0; t < 5000; t += 10)
    {
        if (!sendBackoffShouldSend(b, DEST, t))
        {
            dropped++;
            continue;
        }
        sends++;
        t += 41;
        if (sendBackoffSent(b, DEST, false, t) == SEND_BACKOFF_PAUSED)
            paused++;
    }
    TEST_ASSERT_EQUAL(5, sends); // at 0, 1041, 2082, 3123 and 4164
    TEST_ASSERT_EQUAL(1, paused);
    TEST_ASSERT_EQUAL(500 - 5 * 5, dropped); // a send takes its tick and blocks the next 4
}

void test_receive_from_the_destination_resumes_at_once()
{
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_EQUAL(SEND_BACKOFF_RESUMED, sendBackoffReceived(b, DEST));
    TEST_ASSERT_FALSE(b.paused);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 5001));
    // Nothing to resume a second time.
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffReceived(b, DEST));
}

void test_receive_from_another_address_does_not_resume()
{
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffReceived(b, OTHER));
    TEST_ASSERT_TRUE(b.paused);
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 5001));
}

void test_other_destinations_are_not_affected()
{
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, OTHER, 5001));
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffSent(b, OTHER, true, 5001));
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, SEND_BACKOFF_BROADCAST, 5002));
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffSent(b, SEND_BACKOFF_BROADCAST, true, 5002));
    TEST_ASSERT_TRUE(b.paused);
    TEST_ASSERT_EQUAL_UINT32(DEST, b.ip);
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 5003));
}

void test_broadcast_is_never_paused()
{
    SendBackoff b = {};
    TEST_ASSERT_EQUAL(SEND_BACKOFF_NONE, sendBackoffSent(b, SEND_BACKOFF_BROADCAST, false, 100));
    TEST_ASSERT_FALSE(b.paused);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, SEND_BACKOFF_BROADCAST, 101));
}

void test_failure_to_another_address_replaces_the_paused_one()
{
    SendBackoff b = pausedAt(5000);
    TEST_ASSERT_EQUAL(SEND_BACKOFF_PAUSED, sendBackoffSent(b, OTHER, false, 5100));
    TEST_ASSERT_EQUAL_UINT32(OTHER, b.ip);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 5101));
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, OTHER, 5101));
}

void test_wrap_of_millis()
{
    const uint32_t t0 = 0xFFFFFFFFu - 499; // fails 500 ms before the wrap
    SendBackoff b = pausedAt(t0);
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 0xFFFFFFFFu));
    TEST_ASSERT_FALSE(sendBackoffShouldSend(b, DEST, 499)); // 999 ms after
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 500));  // 1000 ms after
}

void test_clear_resumes_without_an_event()
{
    SendBackoff b = pausedAt(5000);
    sendBackoffClear(b);
    TEST_ASSERT_FALSE(b.paused);
    TEST_ASSERT_TRUE(sendBackoffShouldSend(b, DEST, 5001));
    // a later failure pauses again as the first one
    TEST_ASSERT_EQUAL(SEND_BACKOFF_PAUSED, sendBackoffSent(b, DEST, false, 5100));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_sends_while_not_paused);
    RUN_TEST(test_no_send_for_1000_ms_after_a_failure);
    RUN_TEST(test_retry_exactly_1000_ms_after_the_failure);
    RUN_TEST(test_successful_retry_resumes);
    RUN_TEST(test_failed_retry_pauses_for_another_1000_ms);
    RUN_TEST(test_one_retry_per_interval);
    RUN_TEST(test_receive_from_the_destination_resumes_at_once);
    RUN_TEST(test_receive_from_another_address_does_not_resume);
    RUN_TEST(test_other_destinations_are_not_affected);
    RUN_TEST(test_broadcast_is_never_paused);
    RUN_TEST(test_failure_to_another_address_replaces_the_paused_one);
    RUN_TEST(test_wrap_of_millis);
    RUN_TEST(test_clear_resumes_without_an_event);
    return UNITY_END();
}
