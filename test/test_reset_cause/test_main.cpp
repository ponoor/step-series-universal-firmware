// Host tests of resetCauseFromSamd21Rcause() (src/resetCause.cpp).
// SAMD21 PM.RCAUSE bits: POR 0, BOD12 1, BOD33 2, EXT 4, WDT 5, SYST 6.

#include <unity.h>
#include "resetCause.h"

void setUp() {}
void tearDown() {}

static const uint8_t POR = 1 << 0, BOD12 = 1 << 1, BOD33 = 1 << 2, EXT = 1 << 4, WDT = 1 << 5,
                     SYST = 1 << 6;

void test_numbers_are_fixed()
{
    // part of the OSC specification
    TEST_ASSERT_EQUAL(0, RESET_CAUSE_UNKNOWN);
    TEST_ASSERT_EQUAL(1, RESET_CAUSE_POWER_ON);
    TEST_ASSERT_EQUAL(2, RESET_CAUSE_BROWN_OUT);
    TEST_ASSERT_EQUAL(3, RESET_CAUSE_EXTERNAL);
    TEST_ASSERT_EQUAL(4, RESET_CAUSE_WATCHDOG);
    TEST_ASSERT_EQUAL(5, RESET_CAUSE_SOFTWARE);
}

void test_single_bits()
{
    TEST_ASSERT_EQUAL(RESET_CAUSE_POWER_ON, resetCauseFromSamd21Rcause(POR));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD12));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD33));
    TEST_ASSERT_EQUAL(RESET_CAUSE_EXTERNAL, resetCauseFromSamd21Rcause(EXT));
    TEST_ASSERT_EQUAL(RESET_CAUSE_WATCHDOG, resetCauseFromSamd21Rcause(WDT));
    TEST_ASSERT_EQUAL(RESET_CAUSE_SOFTWARE, resetCauseFromSamd21Rcause(SYST));
}

void test_no_known_bit()
{
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause(0));
    // bits 3 and 7 are reserved
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause(1 << 3));
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause(1 << 7));
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause((1 << 3) | (1 << 7)));
}

// Reference: brown-out > power-on > watchdog > reset pin > software
static ResetCause reference(uint8_t raw)
{
    if (raw & (BOD12 | BOD33))
        return RESET_CAUSE_BROWN_OUT;
    if (raw & POR)
        return RESET_CAUSE_POWER_ON;
    if (raw & WDT)
        return RESET_CAUSE_WATCHDOG;
    if (raw & EXT)
        return RESET_CAUSE_EXTERNAL;
    if (raw & SYST)
        return RESET_CAUSE_SOFTWARE;
    return RESET_CAUSE_UNKNOWN;
}

void test_priority_examples()
{
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(POR | BOD33));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD12 | WDT | SYST));
    TEST_ASSERT_EQUAL(RESET_CAUSE_POWER_ON, resetCauseFromSamd21Rcause(POR | WDT | EXT | SYST));
    TEST_ASSERT_EQUAL(RESET_CAUSE_WATCHDOG, resetCauseFromSamd21Rcause(WDT | EXT));
    TEST_ASSERT_EQUAL(RESET_CAUSE_WATCHDOG, resetCauseFromSamd21Rcause(WDT | SYST));
    TEST_ASSERT_EQUAL(RESET_CAUSE_EXTERNAL, resetCauseFromSamd21Rcause(EXT | SYST));
}

void test_all_combinations()
{
    for (int raw = 0; raw < 256; raw++)
        TEST_ASSERT_EQUAL_MESSAGE(reference((uint8_t)raw), resetCauseFromSamd21Rcause((uint8_t)raw), "raw");
}

void test_read_on_host()
{
    // not a SAMD21: unknown, raw 0
    uint8_t raw = 0xAA;
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, readResetCause(&raw));
    TEST_ASSERT_EQUAL(0, raw);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_numbers_are_fixed);
    RUN_TEST(test_single_bits);
    RUN_TEST(test_no_known_bit);
    RUN_TEST(test_priority_examples);
    RUN_TEST(test_all_combinations);
    RUN_TEST(test_read_on_host);
    return UNITY_END();
}
