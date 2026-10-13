// Host tests of the rate limit of /overCurrent and /stall (src/latchedReport.cpp).

#include <unity.h>
#include "latchedReport.h"

void setUp() {}
void tearDown() {}

static const uint32_t SLOT_MS = 5; // status read period of the firmware

// Polls every SLOT_MS from t0 for n reads, with the flag given by flagAt(t).
// Returns the number of reports sent and stores their times.
template <typename F>
static int run(LatchedReport &r, uint32_t t0, int n, F flagAt, uint32_t *sentAt = nullptr,
               bool enabled = true)
{
    int count = 0;
    for (int k = 0; k < n; k++)
    {
        const uint32_t t = t0 + (uint32_t)k * SLOT_MS;
        if (latchedReportPoll(r, flagAt(t), enabled, t))
        {
            if (sentAt)
                sentAt[count] = t;
            count++;
        }
    }
    return count;
}

void test_first_detection_is_sent_at_once()
{
    LatchedReport r = {};
    TEST_ASSERT_FALSE(latchedReportPoll(r, false, true, 1000));
    TEST_ASSERT_TRUE(latchedReportPoll(r, true, true, 1005));
    TEST_ASSERT_FALSE(r.pending);
}

void test_continuous_condition_every_200_ms()
{
    // Set at every read for 1 s (reads at 0, 5, ..., 995 ms): reports at 0, 200, 400, 600
    // and 800 ms. A report is due exactly 200 ms after the last one (200 ms counts).
    LatchedReport r = {};
    uint32_t sent[16];
    const int n = run(r, 0, 200, [](uint32_t) { return true; }, sent);
    TEST_ASSERT_EQUAL(5, n);
    for (int k = 0; k < n; k++)
        TEST_ASSERT_EQUAL_UINT32(200u * k, sent[k]);
    // The read at 1000 ms (a 201st read) would send the 6th.
    TEST_ASSERT_TRUE(latchedReportPoll(r, true, true, 1000));
}

void test_second_detection_waits_for_the_interval()
{
    // Set once at 100 ms and once at 250 ms: the first at once, the second at 300 ms.
    LatchedReport r = {};
    uint32_t sent[4];
    const int n = run(r, 100, 100, [](uint32_t t) { return t == 100 || t == 250; }, sent);
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_EQUAL_UINT32(100, sent[0]);
    TEST_ASSERT_EQUAL_UINT32(300, sent[1]);
}

void test_several_detections_in_the_interval_give_one_report()
{
    LatchedReport r = {};
    uint32_t sent[4];
    const int n = run(r, 0, 100, [](uint32_t t) { return t == 0 || t == 50 || t == 100 || t == 150; }, sent);
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_EQUAL_UINT32(0, sent[0]);
    TEST_ASSERT_EQUAL_UINT32(200, sent[1]);
}

void test_single_detection_is_sent_once()
{
    LatchedReport r = {};
    TEST_ASSERT_EQUAL(1, run(r, 0, 400, [](uint32_t t) { return t == 20; }));
}

void test_detection_while_disabled_is_not_sent()
{
    LatchedReport r = {};
    // Set while the report is disabled
    TEST_ASSERT_EQUAL(0, run(r, 0, 10, [](uint32_t) { return true; }, nullptr, false));
    // Enabled, the flag no longer set: nothing is sent
    TEST_ASSERT_EQUAL(0, run(r, 50, 100, [](uint32_t) { return false; }));
    // A new detection after enabling is sent at once
    TEST_ASSERT_TRUE(latchedReportPoll(r, true, true, 600));
}

void test_disable_drops_a_pending_detection()
{
    LatchedReport r = {};
    TEST_ASSERT_TRUE(latchedReportPoll(r, true, true, 0));
    TEST_ASSERT_FALSE(latchedReportPoll(r, true, true, 100)); // pending until 200 ms
    TEST_ASSERT_TRUE(r.pending);
    // /enable...Report 0 then 1 between two reads (latchedReportCancel in the handler)
    latchedReportCancel(r);
    TEST_ASSERT_FALSE(latchedReportPoll(r, false, true, 300));
    // Disabled at a read: the detection of that read is dropped too
    TEST_ASSERT_FALSE(latchedReportPoll(r, true, false, 400));
    TEST_ASSERT_FALSE(latchedReportPoll(r, false, true, 405));
    TEST_ASSERT_FALSE(r.pending);
}

void test_wrap_of_millis()
{
    // Reads across the wrap of the 32-bit millis() (at 2^32 ms)
    LatchedReport r = {};
    uint32_t sent[8];
    const uint32_t t0 = 0xFFFFFFFFu - 302; // 0xFFFFFED1
    const int n = run(r, t0, 200, [](uint32_t) { return true; }, sent);
    TEST_ASSERT_EQUAL(5, n);
    for (int k = 1; k < n; k++)
        TEST_ASSERT_EQUAL_UINT32(200, sent[k] - sent[k - 1]);
    TEST_ASSERT_TRUE(sent[2] < t0); // past the wrap
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_detection_is_sent_at_once);
    RUN_TEST(test_continuous_condition_every_200_ms);
    RUN_TEST(test_second_detection_waits_for_the_interval);
    RUN_TEST(test_several_detections_in_the_interval_give_one_report);
    RUN_TEST(test_single_detection_is_sent_once);
    RUN_TEST(test_detection_while_disabled_is_not_sent);
    RUN_TEST(test_disable_drops_a_pending_detection);
    RUN_TEST(test_wrap_of_millis);
    return UNITY_END();
}
