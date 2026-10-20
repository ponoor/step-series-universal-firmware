// Host tests of the homing timeout test (src/homingTimeout.cpp).

#include <unity.h>
#include "homingTimeout.h"

void setUp() {}
void tearDown() {}

void test_not_elapsed_before_the_timeout()
{
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5000, 5000, 10000));
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5000, 6000, 10000));
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5000, 14999, 10000));
}

void test_elapsed_at_and_after_the_timeout()
{
    TEST_ASSERT_TRUE(homingTimeoutElapsed(5000, 15000, 10000));
    TEST_ASSERT_TRUE(homingTimeoutElapsed(5000, 15001, 10000));
    TEST_ASSERT_TRUE(homingTimeoutElapsed(5000, 100000, 10000));
}

void test_zero_timeout_is_disabled()
{
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5000, 5000, 0));
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5000, 1000000, 0));
    // 2^31 ms or more (a negative OSC argument): never elapsed.
    TEST_ASSERT_FALSE(homingTimeoutElapsed(0, 0x7FFFFFFFu, 0x80000000u));
    TEST_ASSERT_FALSE(homingTimeoutElapsed(0, 0x7FFFFFFFu, 0xFFFFFFFFu));
}

void test_start_after_now_is_not_elapsed()
{
    // The bug of C4-2: the slot time is taken at 5000, then releaseSw is sent in the same
    // slot after millis() ticked (start 5001). The unsigned difference was 0xFFFFFFFF.
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5001, 5000, 10000));
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5003, 5000, 1));
    // The next slot counts from the start time as usual.
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5001, 5005, 10000));
    TEST_ASSERT_TRUE(homingTimeoutElapsed(5001, 15001, 10000));
}

void test_short_timeout()
{
    TEST_ASSERT_FALSE(homingTimeoutElapsed(5000, 5000, 1));
    TEST_ASSERT_TRUE(homingTimeoutElapsed(5000, 5001, 1));
}

void test_across_the_wrap_of_millis()
{
    const uint32_t start = 0xFFFFFF00u;
    TEST_ASSERT_FALSE(homingTimeoutElapsed(start, start + 1000u - 1u, 1000));
    TEST_ASSERT_TRUE(homingTimeoutElapsed(start, start + 1000u, 1000));
    // Start just after the wrap, slot time just before it.
    TEST_ASSERT_FALSE(homingTimeoutElapsed(0x00000001u, 0xFFFFFFFFu, 10000));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_not_elapsed_before_the_timeout);
    RUN_TEST(test_elapsed_at_and_after_the_timeout);
    RUN_TEST(test_zero_timeout_is_disabled);
    RUN_TEST(test_start_after_now_is_not_elapsed);
    RUN_TEST(test_short_timeout);
    RUN_TEST(test_across_the_wrap_of_millis);
    return UNITY_END();
}
