// Host tests of the servo mode switch rule (src/servoSwitch.cpp).

#include <unity.h>
#include "servoSwitch.h"

void setUp() {}
void tearDown() {}

static const bool FWD = true;
static const bool REV = false;

// Arguments in the order of servoSwitchBlocks(); homing direction REV unless given.
static bool blocks(bool homeSw, bool limitSw, bool prohibitHome, bool prohibitLimit,
                   bool homeSwMode, bool limitSwMode, bool dir, bool homingDir = REV)
{
    return servoSwitchBlocks(homeSw, limitSw, prohibitHome, prohibitLimit, homeSwMode,
                             limitSwMode, homingDir, dir);
}

void test_no_switch_never_blocks()
{
    for (int bits = 0; bits < 32; bits++)
    {
        const bool ph = bits & 1, pl = bits & 2, hm = bits & 4, lm = bits & 8, d = bits & 16;
        TEST_ASSERT_FALSE(blocks(false, false, ph, pl, hm, lm, d));
        TEST_ASSERT_FALSE(blocks(false, false, ph, pl, hm, lm, d, FWD));
    }
}

void test_home_switch_blocks_the_homing_direction()
{
    // prohibited, user mode
    TEST_ASSERT_TRUE(blocks(true, false, true, false, true, true, REV));
    TEST_ASSERT_FALSE(blocks(true, false, true, false, true, true, FWD));
    // not prohibited, hard stop mode
    TEST_ASSERT_TRUE(blocks(true, false, false, false, false, true, REV));
    TEST_ASSERT_FALSE(blocks(true, false, false, false, false, true, FWD));
    // not prohibited, user mode: free
    TEST_ASSERT_FALSE(blocks(true, false, false, false, true, true, REV));
    TEST_ASSERT_FALSE(blocks(true, false, false, false, true, true, FWD));
    // homing direction FWD
    TEST_ASSERT_TRUE(blocks(true, false, true, false, true, true, FWD, FWD));
    TEST_ASSERT_FALSE(blocks(true, false, true, false, true, true, REV, FWD));
}

void test_limit_switch_blocks_the_other_direction()
{
    // prohibited, user mode
    TEST_ASSERT_TRUE(blocks(false, true, false, true, true, true, FWD));
    TEST_ASSERT_FALSE(blocks(false, true, false, true, true, true, REV));
    // not prohibited, hard stop mode
    TEST_ASSERT_TRUE(blocks(false, true, false, false, true, false, FWD));
    TEST_ASSERT_FALSE(blocks(false, true, false, false, true, false, REV));
    // not prohibited, user mode: free
    TEST_ASSERT_FALSE(blocks(false, true, false, false, true, true, FWD));
    TEST_ASSERT_FALSE(blocks(false, true, false, false, true, true, REV));
    // homing direction FWD
    TEST_ASSERT_TRUE(blocks(false, true, false, true, true, true, REV, FWD));
    TEST_ASSERT_FALSE(blocks(false, true, false, true, true, true, FWD, FWD));
}

// Both pressed: each switch is judged by its own rule. Before, a pressed home switch hid
// the limit switch.
void test_both_switches_are_judged_separately()
{
    // home free (user mode, not prohibited), limit prohibited: away from home blocked
    TEST_ASSERT_TRUE(blocks(true, true, false, true, true, true, FWD));
    TEST_ASSERT_FALSE(blocks(true, true, false, true, true, true, REV));
    // home prohibited, limit free: towards home blocked
    TEST_ASSERT_TRUE(blocks(true, true, true, false, true, true, REV));
    TEST_ASSERT_FALSE(blocks(true, true, true, false, true, true, FWD));
    // both active: both directions blocked
    TEST_ASSERT_TRUE(blocks(true, true, true, true, true, true, REV));
    TEST_ASSERT_TRUE(blocks(true, true, true, true, true, true, FWD));
    TEST_ASSERT_TRUE(blocks(true, true, false, false, false, false, REV));
    TEST_ASSERT_TRUE(blocks(true, true, false, false, false, false, FWD));
    // both free
    TEST_ASSERT_FALSE(blocks(true, true, false, false, true, true, REV));
    TEST_ASSERT_FALSE(blocks(true, true, false, false, true, true, FWD));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_switch_never_blocks);
    RUN_TEST(test_home_switch_blocks_the_homing_direction);
    RUN_TEST(test_limit_switch_blocks_the_other_direction);
    RUN_TEST(test_both_switches_are_judged_separately);
    return UNITY_END();
}
