// Host tests of rawSpeed.h: the conversions of the driver libraries (accCalc() etc. of
// Ponoor_L6470_Library and Ponoor_PowerSTEP01_Library release/1.2.0, private there) are
// copied here, and calc(fromRaw(n)) must be n for every register value.
#pragma STDC FP_CONTRACT OFF

#include <unity.h>
#include <math.h>
#include "rawSpeed.h"

void setUp() {}
void tearDown() {}

typedef unsigned long ul;

// ---- Ponoor_L6470_Library -----------------------------------------------------
static ul l6470AccCalc(float v)
{
    float t = v * 0.068719477F;
    if (t >= 4094.5F) return 0x00000FFE;
    ul r = (t > 0.0F) ? (ul)(t + 0.5F) : 0;
    return r < 1 ? 1 : r;
}
static ul l6470MaxCalc(float v)
{
    ul t = ceil(v * 0.065536F);
    return t > 0x3FF ? 0x3FF : t;
}
static ul l6470MinCalc(float v)
{
    float t = v * 4.194304F;
    return (ul)long(t) > 0xFFF ? 0xFFF : (ul)long(t);
}
static ul l6470FsCalc(float v)
{
    float t = (v * 0.065536F) - 0.5F;
    return (ul)long(t) > 0x3FF ? 0x3FF : (ul)long(t);
}
// ---- Ponoor_PowerSTEP01_Library -----------------------------------------------
static ul psDecCalc(float v)
{
    float t = v * 0.068719477F;
    if (t >= 4095.5F) return 0x00000FFF;
    ul r = (t > 0.0F) ? (ul)(t + 0.5F) : 0;
    return r < 1 ? 1 : r;
}
static ul psMaxCalc(float v)
{
    float t = ceil(v * 0.065536F);
    if (t < 1.0F) return 1;
    if (t > 1023.0F) return 0x3FF;
    return (ul)t;
}

void test_acc_dec_round_trip()
{
    for (ul n = 1; n <= 0xFFE; n++)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, l6470AccCalc(accFromRaw(n)), "L6470 / PowerSTEP01 ACC");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, l6470AccCalc(decFromRaw(n)), "L6470 DEC");
    }
    for (ul n = 1; n <= 0xFFF; n++)
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, psDecCalc(decFromRaw(n)), "PowerSTEP01 DEC");
    // the documented exceptions
    TEST_ASSERT_EQUAL_UINT32(1, l6470AccCalc(accFromRaw(0)));
    TEST_ASSERT_EQUAL_UINT32(0xFFE, l6470AccCalc(accFromRaw(0xFFF)));
}

void test_max_speed_round_trip()
{
    for (ul n = 1; n <= 0x3FF; n++)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, l6470MaxCalc(maxSpeedFromRaw(n)), "L6470 MAX_SPEED");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, psMaxCalc(maxSpeedFromRaw(n)), "PowerSTEP01 MAX_SPEED");
    }
}

void test_min_speed_round_trip()
{
    for (ul n = 0; n <= 0xFFF; n++)
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, l6470MinCalc(minSpeedFromRaw(n)), "MIN_SPEED");
    // bit 12 (LSPD_OPT) is not part of the speed
    TEST_ASSERT_EQUAL_FLOAT(minSpeedFromRaw(0x0123), minSpeedFromRaw(0x1123));
}

void test_full_step_speed_round_trip()
{
    for (ul n = 0; n <= 0x3FF; n++)
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(n, l6470FsCalc(fullStepSpeedFromRaw(n)), "FS_SPD");
    TEST_ASSERT_EQUAL_FLOAT(fullStepSpeedFromRaw(0x0027), fullStepSpeedFromRaw(0x0427)); // BOOST_MODE
}

void test_values_near_the_parse_functions()
{
    // within half an LSB of what /getMinSpeed and /getFullstepSpeed return for the register
    TEST_ASSERT_FLOAT_WITHIN(0.24F * 0.5F + 0.001F, 1077 * 0.23841858F, minSpeedFromRaw(1077));
    TEST_ASSERT_FLOAT_WITHIN(15.26F * 0.5F + 0.01F, (538 + 0.5F) * 15.258789F, fullStepSpeedFromRaw(538));
    TEST_ASSERT_EQUAL_FLOAT(0.0F, accFromRaw(0));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_acc_dec_round_trip);
    RUN_TEST(test_max_speed_round_trip);
    RUN_TEST(test_min_speed_round_trip);
    RUN_TEST(test_full_step_speed_round_trip);
    RUN_TEST(test_values_near_the_parse_functions);
    return UNITY_END();
}
