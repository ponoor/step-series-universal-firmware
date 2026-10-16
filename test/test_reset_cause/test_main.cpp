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
    TEST_ASSERT_EQUAL(6, RESET_CAUSE_CPU_FAULT);
}

void test_single_bits()
{
    TEST_ASSERT_EQUAL(RESET_CAUSE_POWER_ON, resetCauseFromSamd21Rcause(POR, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD12, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD33, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_EXTERNAL, resetCauseFromSamd21Rcause(EXT, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_WATCHDOG, resetCauseFromSamd21Rcause(WDT, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_SOFTWARE, resetCauseFromSamd21Rcause(SYST, false));
}

void test_no_known_bit()
{
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause(0, false));
    // bits 3 and 7 are reserved
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause(1 << 3, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause(1 << 7, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_UNKNOWN, resetCauseFromSamd21Rcause((1 << 3) | (1 << 7), false));
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
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(POR | BOD33, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD12 | WDT | SYST, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_POWER_ON, resetCauseFromSamd21Rcause(POR | WDT | EXT | SYST, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_WATCHDOG, resetCauseFromSamd21Rcause(WDT | EXT, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_WATCHDOG, resetCauseFromSamd21Rcause(WDT | SYST, false));
    TEST_ASSERT_EQUAL(RESET_CAUSE_EXTERNAL, resetCauseFromSamd21Rcause(EXT | SYST, false));
}

void test_all_combinations()
{
    for (int raw = 0; raw < 256; raw++)
        TEST_ASSERT_EQUAL_MESSAGE(reference((uint8_t)raw), resetCauseFromSamd21Rcause((uint8_t)raw, false), "raw");
}

// The CPU fault marker: first of all causes, except that it is ignored with POR or BOD
// (the RAM that holds it is not kept then).
void test_cpu_fault_marker()
{
    TEST_ASSERT_EQUAL(RESET_CAUSE_CPU_FAULT, resetCauseFromSamd21Rcause(SYST, true));
    TEST_ASSERT_EQUAL(RESET_CAUSE_CPU_FAULT, resetCauseFromSamd21Rcause(WDT, true));
    TEST_ASSERT_EQUAL(RESET_CAUSE_CPU_FAULT, resetCauseFromSamd21Rcause(EXT | SYST, true));
    TEST_ASSERT_EQUAL(RESET_CAUSE_CPU_FAULT, resetCauseFromSamd21Rcause(0, true));
    TEST_ASSERT_EQUAL(RESET_CAUSE_POWER_ON, resetCauseFromSamd21Rcause(POR, true));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD33, true));
    TEST_ASSERT_EQUAL(RESET_CAUSE_BROWN_OUT, resetCauseFromSamd21Rcause(BOD12 | SYST, true));
    for (int raw = 0; raw < 256; raw++)
    {
        const bool powerLost = raw & (POR | BOD12 | BOD33);
        TEST_ASSERT_EQUAL_MESSAGE(powerLost ? reference((uint8_t)raw) : RESET_CAUSE_CPU_FAULT,
                                  resetCauseFromSamd21Rcause((uint8_t)raw, true), "raw");
    }
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
    RUN_TEST(test_cpu_fault_marker);
    RUN_TEST(test_read_on_host);
    return UNITY_END();
}
