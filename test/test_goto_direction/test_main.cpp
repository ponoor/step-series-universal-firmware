// Host tests of goToDirection() (src/goToDirection.h).

#include <unity.h>
#include <initializer_list>
#include "goToDirection.h"

void setUp() {}
void tearDown() {}

constexpr int32_t HALF = 1 << 21; // half of the 22-bit range

void test_sign_extend()
{
    TEST_ASSERT_EQUAL_INT32(0, signExtend22(0));
    TEST_ASSERT_EQUAL_INT32(1, signExtend22(1));
    TEST_ASSERT_EQUAL_INT32(HALF - 1, signExtend22(HALF - 1));
    TEST_ASSERT_EQUAL_INT32(-HALF, signExtend22(HALF));
    TEST_ASSERT_EQUAL_INT32(-1, signExtend22(0x3FFFFF));
    TEST_ASSERT_EQUAL_INT32(0, signExtend22(0x400000));
    TEST_ASSERT_EQUAL_INT32(-1, signExtend22(-1));
    TEST_ASSERT_EQUAL_INT32(-1, signExtend22(0x7FFFFFFF >> 9)); // 0x3FFFFF
}

void test_short_moves()
{
    // the target above the position: forward; below: reverse (the old code had it the other way round for |d| < 2048)
    TEST_ASSERT_TRUE(goToDirection(0, 1));
    TEST_ASSERT_FALSE(goToDirection(1, 0));
    TEST_ASSERT_TRUE(goToDirection(0, 100));
    TEST_ASSERT_FALSE(goToDirection(0, -100));
    TEST_ASSERT_TRUE(goToDirection(5000, 5100));
    TEST_ASSERT_FALSE(goToDirection(5000, 4900));
    TEST_ASSERT_FALSE(goToDirection(-100, -101));
    TEST_ASSERT_TRUE(goToDirection(-100, -99));
}

void test_no_move()
{
    TEST_ASSERT_FALSE(goToDirection(0, 0));
    TEST_ASSERT_FALSE(goToDirection(-77, -77));
    TEST_ASSERT_FALSE(goToDirection(12345, 12345 + (1 << 22))); // the same position modulo 2^22
}

void test_around_2048()
{
    // |d| = 2047, 2048, 2049: the old threshold flipped the direction at 2048
    for (int32_t d : {2047, 2048, 2049})
    {
        TEST_ASSERT_TRUE(goToDirection(0, d));
        TEST_ASSERT_FALSE(goToDirection(0, -d));
        TEST_ASSERT_TRUE(goToDirection(1000, 1000 + d));
        TEST_ASSERT_FALSE(goToDirection(1000, 1000 - d));
    }
}

void test_shortest_way_wraps()
{
    // d = 2^21 - 1 forward is the shortest; 2^21 + 1 goes the other way round
    TEST_ASSERT_TRUE(goToDirection(0, HALF - 1));
    TEST_ASSERT_FALSE(goToDirection(0, HALF + 1)); // = -(2^21 - 1) in 22 bits
    TEST_ASSERT_FALSE(goToDirection(0, -(HALF - 1)));
    TEST_ASSERT_TRUE(goToDirection(0, -(HALF + 1)));
    // exactly half a turn: reverse
    TEST_ASSERT_FALSE(goToDirection(0, HALF));
    TEST_ASSERT_FALSE(goToDirection(0, -HALF));
    TEST_ASSERT_FALSE(goToDirection(100, 100 + HALF));
}

void test_range_ends()
{
    // the position at the top of the range, the target just past it (wraps to the bottom)
    TEST_ASSERT_TRUE(goToDirection(HALF - 1, -HALF));        // +1 step
    TEST_ASSERT_FALSE(goToDirection(-HALF, HALF - 1));       // -1 step
    TEST_ASSERT_TRUE(goToDirection(HALF - 10, HALF + 10));   // 20 forward, the target 22-bit folded
    TEST_ASSERT_FALSE(goToDirection(-HALF + 10, -HALF - 10));
}

void test_extreme_int32_targets()
{
    // only the low 22 bits of the target count; no overflow
    TEST_ASSERT_FALSE(goToDirection(0, INT32_MIN));          // low 22 bits 0
    TEST_ASSERT_FALSE(goToDirection(0, INT32_MAX));          // low 22 bits 0x3FFFFF = -1: reverse
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_sign_extend);
    RUN_TEST(test_short_moves);
    RUN_TEST(test_no_move);
    RUN_TEST(test_around_2048);
    RUN_TEST(test_shortest_way_wraps);
    RUN_TEST(test_range_ends);
    RUN_TEST(test_extreme_int32_targets);
    return UNITY_END();
}
