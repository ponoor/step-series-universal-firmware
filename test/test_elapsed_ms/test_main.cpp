// Host tests of the elapsed time test of the timers (src/elapsedMs.cpp).

#include <unity.h>
#include "elapsedMs.h"

void setUp() {}
void tearDown() {}

void test_boundary_of_the_duration()
{
    // brakeTransitionDuration 100 ms, RXL_TXL_BLINK_DURATION 30 ms
    TEST_ASSERT_FALSE(millisElapsed(5000, 5000, 100));
    TEST_ASSERT_FALSE(millisElapsed(5000, 5099, 100));
    TEST_ASSERT_TRUE(millisElapsed(5000, 5100, 100));
    TEST_ASSERT_TRUE(millisElapsed(5000, 5101, 100));
    TEST_ASSERT_FALSE(millisElapsed(5000, 5029, 30));
    TEST_ASSERT_TRUE(millisElapsed(5000, 5030, 30));
}

void test_start_after_now_is_not_elapsed()
{
    // The timer started in this slot after millis() ticked: 0 ms elapsed, not 49.7 days.
    TEST_ASSERT_FALSE(millisElapsed(5001, 5000, 100));
    TEST_ASSERT_FALSE(millisElapsed(5003, 5000, 30));
    TEST_ASSERT_FALSE(millisElapsed(5003, 5000, 1));
    // The next slots count from the start time.
    TEST_ASSERT_FALSE(millisElapsed(5003, 5005, 30));
    TEST_ASSERT_FALSE(millisElapsed(5003, 5032, 30));
    TEST_ASSERT_TRUE(millisElapsed(5003, 5033, 30));
}

void test_zero_duration()
{
    // 0 is "at once" (unlike the homing timeouts, where 0 disables the timeout), except
    // for a start time later than the slot time: elapsed from the next slot on.
    TEST_ASSERT_TRUE(millisElapsed(5000, 5001, 0));
    TEST_ASSERT_TRUE(millisElapsed(5000, 5000, 0));
    TEST_ASSERT_FALSE(millisElapsed(5001, 5000, 0));
    TEST_ASSERT_TRUE(millisElapsed(5001, 5005, 0));
}

void test_duration_of_2_31_or_more_never_elapses()
{
    TEST_ASSERT_FALSE(millisElapsed(0, 0x7FFFFFFFu, 0x80000000u));
    TEST_ASSERT_FALSE(millisElapsed(0, 0x7FFFFFFFu, 0xFFFFFFFFu));
    // Just below 2^31: elapses at 2^31 - 1 ms.
    TEST_ASSERT_FALSE(millisElapsed(0, 0x7FFFFFFEu, 0x7FFFFFFFu));
    TEST_ASSERT_TRUE(millisElapsed(0, 0x7FFFFFFFu, 0x7FFFFFFFu));
}

void test_across_the_wrap_of_millis()
{
    const uint32_t start = 0xFFFFFFF0u;
    TEST_ASSERT_FALSE(millisElapsed(start, start + 100u - 1u, 100));
    TEST_ASSERT_TRUE(millisElapsed(start, start + 100u, 100));
    TEST_ASSERT_TRUE(millisElapsed(start, 0x00000000u, 16));
    TEST_ASSERT_FALSE(millisElapsed(start, 0x00000000u, 17));
    // Start just after the wrap, slot time just before it.
    TEST_ASSERT_FALSE(millisElapsed(0x00000001u, 0xFFFFFFFFu, 30));
    TEST_ASSERT_FALSE(millisElapsed(0x00000001u, 0xFFFFFFFFu, 0));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_boundary_of_the_duration);
    RUN_TEST(test_start_after_now_is_not_elapsed);
    RUN_TEST(test_zero_duration);
    RUN_TEST(test_duration_of_2_31_or_more_never_elapses);
    RUN_TEST(test_across_the_wrap_of_millis);
    return UNITY_END();
}
