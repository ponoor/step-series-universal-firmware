// Host tests of the brake state machine (src/brakeControl.cpp).
// Every cell of the command table and every row of the poll table of the specification.

#include <unity.h>
#include <stdio.h>
#include "brakeControl.h"

void setUp() {}
void tearDown() {}

// Short names for the tables
static const BrakeState EG = BRAKE_ENGAGED, RLG = BRAKE_RELEASING, RLD = BRAKE_RELEASED,
                        STP = BRAKE_STOPPING, EGG = BRAKE_ENGAGING, FR = BRAKE_FREE;
static const BrakeDriverCommand NO = BRAKE_DRIVER_NONE, HS = BRAKE_DRIVER_HARD_STOP,
                                SS = BRAKE_DRIVER_SOFT_STOP, HZ = BRAKE_DRIVER_HARD_HIZ;
static const BrakeError OK = BRAKE_ERROR_NONE, BE = BRAKE_ERROR_ENGAGED, CI = BRAKE_ERROR_IGNORED,
                        RF = BRAKE_ERROR_RELEASE_FAILED, EH = BRAKE_ERROR_ENGAGED_ON_HIZ;

struct Expect
{
    BrakeState next;
    BrakeDriverCommand driver;
    BrakeError error;
    bool execute;
    bool startTimer;
};

static const char *const stateNames[] = {"ENGAGED", "RELEASING", "RELEASED", "STOPPING", "ENGAGING", "FREE"};
static const char *const commandNames[] = {"activate 1", "activate 0", "softHiZ", "hardHiZ",
                                           "free", "motion", "stop", "resetMotorDriver"};

static void check(const char *where, const BrakeAction &a, const Expect &e)
{
    char msg[96];
    snprintf(msg, sizeof(msg), "%s: next", where);
    TEST_ASSERT_EQUAL_MESSAGE(e.next, a.next, msg);
    snprintf(msg, sizeof(msg), "%s: driver", where);
    TEST_ASSERT_EQUAL_MESSAGE(e.driver, a.driver, msg);
    snprintf(msg, sizeof(msg), "%s: error", where);
    TEST_ASSERT_EQUAL_MESSAGE(e.error, a.error, msg);
    snprintf(msg, sizeof(msg), "%s: execute", where);
    TEST_ASSERT_EQUAL_MESSAGE(e.execute, a.execute, msg);
    snprintf(msg, sizeof(msg), "%s: startTimer", where);
    TEST_ASSERT_EQUAL_MESSAGE(e.startTimer, a.startTimer, msg);
}

void test_state_numbers_are_fixed()
{
    // part of the OSC specification (/brakeStatus)
    TEST_ASSERT_EQUAL(0, BRAKE_ENGAGED);
    TEST_ASSERT_EQUAL(1, BRAKE_RELEASING);
    TEST_ASSERT_EQUAL(2, BRAKE_RELEASED);
    TEST_ASSERT_EQUAL(3, BRAKE_STOPPING);
    TEST_ASSERT_EQUAL(4, BRAKE_ENGAGING);
    TEST_ASSERT_EQUAL(5, BRAKE_FREE);
}

void test_output()
{
    TEST_ASSERT_FALSE(brakeOutputReleased(EG));
    TEST_ASSERT_FALSE(brakeOutputReleased(RLG));
    TEST_ASSERT_TRUE(brakeOutputReleased(RLD));
    TEST_ASSERT_TRUE(brakeOutputReleased(STP));
    TEST_ASSERT_FALSE(brakeOutputReleased(EGG));
    TEST_ASSERT_TRUE(brakeOutputReleased(FR));
}

// Command table. Columns: ENGAGED, RELEASING, RELEASED, STOPPING, ENGAGING, FREE.
// {next, driver command, error, execute the command, start the timer}
static const Expect commandTable[BRAKE_CMD_COUNT][BRAKE_STATE_COUNT] = {
    // /activate 1
    {{RLG, HS, OK, false, true}, {RLG, NO, OK, false, false}, {RLD, NO, OK, false, false},
     {RLD, NO, OK, false, false}, {RLG, NO, OK, false, true}, {RLD, HS, OK, false, false}},
    // /activate 0
    {{EG, NO, OK, false, false}, {EGG, NO, OK, false, true}, {STP, SS, OK, false, false},
     {STP, NO, OK, false, false}, {EGG, NO, OK, false, false}, {EG, NO, OK, false, false}},
    // /softHiZ
    {{EG, NO, OK, false, false}, {EGG, NO, OK, false, true}, {STP, SS, OK, false, false},
     {STP, NO, OK, false, false}, {EGG, NO, OK, false, false}, {EG, NO, OK, false, false}},
    // /hardHiZ
    {{EG, NO, OK, false, false}, {EGG, NO, OK, false, true}, {EGG, HS, OK, false, true},
     {EGG, HS, OK, false, true}, {EGG, NO, OK, false, false}, {EG, NO, OK, false, false}},
    // /free
    {{FR, HZ, OK, false, false}, {FR, HZ, OK, false, false}, {FR, HZ, OK, false, false},
     {FR, HZ, OK, false, false}, {FR, HZ, OK, false, false}, {FR, NO, OK, false, false}},
    // motion commands
    {{EG, NO, BE, false, false}, {RLG, NO, BE, false, false}, {RLD, NO, OK, true, false},
     {RLD, NO, OK, true, false}, {EGG, NO, BE, false, false}, {RLD, NO, OK, true, false}},
    // /softStop, /hardStop, /enableServoMode 0
    {{EG, NO, OK, false, false}, {RLG, NO, OK, true, false}, {RLD, NO, OK, true, false},
     {RLD, NO, OK, true, false}, {EGG, NO, OK, true, false}, {FR, NO, OK, false, false}},
    // /resetMotorDriver
    {{EG, NO, OK, true, false}, {RLG, NO, CI, false, false}, {RLD, NO, CI, false, false},
     {STP, NO, CI, false, false}, {EGG, NO, CI, false, false}, {FR, NO, OK, true, false}},
};

void test_command_table()
{
    char where[64];
    for (int c = 0; c < BRAKE_CMD_COUNT; c++)
    {
        for (int s = 0; s < BRAKE_STATE_COUNT; s++)
        {
            snprintf(where, sizeof(where), "%s in %s", commandNames[c], stateNames[s]);
            const BrakeAction a = brakeOnCommand((BrakeState)s, (BrakeCommand)c);
            check(where, a, commandTable[c][s]);
            // Commands never cancel through the action (the caller cancels), nor warn.
            TEST_ASSERT_FALSE_MESSAGE(a.cancelPending, where);
            TEST_ASSERT_FALSE_MESSAGE(a.warn, where);
        }
    }
}

// Invariant 1: no command energizes the driver in ENGAGED or FREE, except /activate 1 and
// a motion command from FREE; no command makes the motor turn while the output is engaged.
void test_no_energizing_while_hiz()
{
    for (int c = 0; c < BRAKE_CMD_COUNT; c++)
    {
        const BrakeState hizStates[] = {EG, FR};
        for (BrakeState s : hizStates)
        {
            const BrakeAction a = brakeOnCommand(s, (BrakeCommand)c);
            const bool energizes = (a.driver == HS) || (a.driver == SS) ||
                                   (a.execute && (c == BRAKE_CMD_MOTION || c == BRAKE_CMD_STOP));
            const bool allowed = (c == BRAKE_CMD_ACTIVATE) || (c == BRAKE_CMD_MOTION && s == FR);
            if (!allowed)
                TEST_ASSERT_FALSE_MESSAGE(energizes, commandNames[c]);
        }
        const BrakeState engagedOutput[] = {EG, RLG, EGG};
        for (BrakeState s : engagedOutput)
        {
            const BrakeAction a = brakeOnCommand(s, (BrakeCommand)c);
            if (c == BRAKE_CMD_MOTION)
                TEST_ASSERT_FALSE(a.execute);
        }
    }
}

// Poll table
struct PollRow
{
    const char *name;
    BrakeState state;
    BrakePollInput in; // hiz, stopped, statusFresh, elapsed
    BrakeState next;
    BrakeDriverCommand driver;
    BrakeError error;
    bool startTimer;
    bool cancelPending;
    bool warn;
};

static const PollRow pollTable[] = {
    // RELEASING
    {"RELEASING HiZ (fresh)", RLG, {true, true, true, false}, EG, NO, RF, false, false, false},
    {"RELEASING HiZ (fresh, elapsed)", RLG, {true, true, true, true}, EG, NO, RF, false, false, false},
    {"RELEASING HiZ (not fresh)", RLG, {true, true, false, true}, RLG, NO, OK, false, false, false},
    {"RELEASING elapsed, energized (fresh)", RLG, {false, true, true, true}, RLD, NO, OK, false, false, false},
    {"RELEASING elapsed, energized (not fresh)", RLG, {false, true, false, true}, RLG, NO, OK, false, false, false},
    {"RELEASING not elapsed", RLG, {false, true, true, false}, RLG, NO, OK, false, false, false},
    // RELEASED / STOPPING: HiZ
    {"RELEASED HiZ", RLD, {true, true, true, false}, EG, NO, EH, false, true, false},
    {"STOPPING HiZ", STP, {true, false, true, false}, EG, NO, EH, false, true, false},
    {"STOPPING HiZ and stopped", STP, {true, true, true, false}, EG, NO, EH, false, true, false},
    {"RELEASED HiZ (not fresh)", RLD, {true, true, false, false}, RLD, NO, OK, false, false, false},
    {"RELEASED energized", RLD, {false, true, true, true}, RLD, NO, OK, false, false, false},
    // STOPPING: stop
    {"STOPPING stopped", STP, {false, true, true, false}, EGG, NO, OK, true, false, false},
    {"STOPPING stopped (not fresh)", STP, {false, true, false, false}, STP, NO, OK, false, false, false},
    {"STOPPING moving", STP, {false, false, true, true}, STP, NO, OK, false, false, false},
    // ENGAGING
    {"ENGAGING elapsed", EGG, {false, true, true, true}, EG, HZ, OK, false, false, false},
    {"ENGAGING elapsed (not fresh)", EGG, {false, true, false, true}, EG, HZ, OK, false, false, false},
    {"ENGAGING HiZ", EGG, {true, true, true, false}, EG, NO, OK, false, false, false},
    {"ENGAGING not elapsed", EGG, {false, true, true, false}, EGG, NO, OK, false, false, false},
    // ENGAGED
    {"ENGAGED energized", EG, {false, true, true, false}, EG, HZ, OK, false, false, true},
    {"ENGAGED energized (not fresh)", EG, {false, true, false, false}, EG, NO, OK, false, false, false},
    {"ENGAGED HiZ", EG, {true, true, true, true}, EG, NO, OK, false, false, false},
    // FREE
    {"FREE energized", FR, {false, true, true, false}, RLD, NO, OK, false, false, true},
    {"FREE energized (not fresh)", FR, {false, true, false, false}, FR, NO, OK, false, false, false},
    {"FREE HiZ", FR, {true, true, true, true}, FR, NO, OK, false, false, false},
};

void test_poll_table()
{
    for (const PollRow &r : pollTable)
    {
        const BrakeAction a = brakeOnPoll(r.state, r.in);
        check(r.name, a, {r.next, r.driver, r.error, false, r.startTimer});
        TEST_ASSERT_EQUAL_MESSAGE(r.cancelPending, a.cancelPending, r.name);
        TEST_ASSERT_EQUAL_MESSAGE(r.warn, a.warn, r.name);
    }
}

// Invariant 3: in RELEASED or STOPPING, a fresh HiZ always engages the brake.
void test_hiz_engages_at_once()
{
    const BrakeState states[] = {RLD, STP};
    for (BrakeState s : states)
        for (int stopped = 0; stopped < 2; stopped++)
            for (int elapsed = 0; elapsed < 2; elapsed++)
            {
                const BrakeAction a = brakeOnPoll(s, {true, (bool)stopped, true, (bool)elapsed});
                TEST_ASSERT_EQUAL(EG, a.next);
                TEST_ASSERT_FALSE(brakeOutputReleased(a.next));
            }
}

// Invariant 2: the poll releases the brake only with a fresh status showing HiZ = 0.
void test_release_needs_fresh_energized_status()
{
    for (int s = 0; s < BRAKE_STATE_COUNT; s++)
        for (int bits = 0; bits < 16; bits++)
        {
            const BrakePollInput in = {(bits & 1) != 0, (bits & 2) != 0, (bits & 4) != 0, (bits & 8) != 0};
            const BrakeAction a = brakeOnPoll((BrakeState)s, in);
            if (!brakeOutputReleased((BrakeState)s) && brakeOutputReleased(a.next))
            {
                TEST_ASSERT_TRUE(in.statusFresh);
                TEST_ASSERT_FALSE(in.hiz);
            }
        }
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_state_numbers_are_fixed);
    RUN_TEST(test_output);
    RUN_TEST(test_command_table);
    RUN_TEST(test_no_energizing_while_hiz);
    RUN_TEST(test_poll_table);
    RUN_TEST(test_hiz_engages_at_once);
    RUN_TEST(test_release_needs_fresh_energized_status);
    return UNITY_END();
}
