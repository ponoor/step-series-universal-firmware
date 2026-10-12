// Host tests of OscRouter (src/oscRouter.cpp) with the real address table (src/oscRoutes.h).
//
// The slow path must call the same handlers, in the same order, as the former chain of
// CNMAT OSCMessage::route() calls. The router was also compared with that CNMAT code on
// about 1,700,000 exact, wildcard, suffixed and mutated addresses (no difference).

#include <unity.h>
#include <string.h>
#include <string>
#include <vector>
#include "oscRouter.h"

void setUp() {}
void tearDown() {}

// ---- tables with all optional features and with none -----------------------------

#define HAVE_BRAKE
#define DRIVER_POWERSTEP01
#define HAVE_LIMIT_ADC
#define OSC_ROUTE(address, handler) address,
static const char *const fullNames[] = {
#include "oscRoutes.h"
};
#undef HAVE_BRAKE
#undef DRIVER_POWERSTEP01
#undef HAVE_LIMIT_ADC
static const char *const minimalNames[] = {
#include "oscRoutes.h"
};
#undef OSC_ROUTE
constexpr int FULL_COUNT = sizeof(fullNames) / sizeof(fullNames[0]);
constexpr int MINIMAL_COUNT = sizeof(minimalNames) / sizeof(minimalNames[0]);

// Each table entry gets its own handler that records its table index.
static std::vector<int> calls;
static int lastIntArg;
template <int I>
static void handler(OscMessage &msg)
{
    calls.push_back(I);
    lastIntArg = msg.getInt(0);
}
template <int I>
struct Handlers
{
    static void fill(OscHandler *h)
    {
        h[I] = handler<I>;
        Handlers<I - 1>::fill(h);
    }
};
template <>
struct Handlers<-1>
{
    static void fill(OscHandler *) {}
};

struct Table
{
    OscRoute routes[FULL_COUNT];
    uint8_t index[FULL_COUNT];
    OscRouter router;
    const char *const *names;
    int count;
    void begin(const char *const *n, int c)
    {
        names = n;
        count = c;
        OscHandler h[FULL_COUNT];
        Handlers<FULL_COUNT - 1>::fill(h);
        for (int i = 0; i < c; i++)
            routes[i] = {n[i], h[i]};
        router.begin(routes, (uint8_t)c, index);
    }
    int indexOf(const char *address) const
    {
        for (int i = 0; i < count; i++)
            if (!strcmp(names[i], address))
                return i;
        return -1;
    }
};
static Table full, minimal;

// Receive buffer with guards, as in OSCMsgReceive()
static uint8_t storage[OSC_RX_GUARD_BEFORE + 128 + OSC_RX_GUARD_AFTER];
static uint8_t *const rx = storage + OSC_RX_GUARD_BEFORE;
static OscMessage msg;

static OscDispatchResult send(Table &t, const char *address, int32_t arg = 7)
{
    uint16_t n = 0;
    const size_t len = strlen(address) + 1;
    memcpy(rx, address, len);
    n = (uint16_t)len;
    while (n % 4)
        rx[n++] = 0;
    memcpy(rx + n, ",i\0\0", 4);
    n += 4;
    rx[n++] = arg >> 24;
    rx[n++] = arg >> 16;
    rx[n++] = arg >> 8;
    rx[n++] = arg;
    oscSetRxGuards(rx, n);
    msg.decode(rx, n);
    TEST_ASSERT_FALSE(msg.hasError());
    calls.clear();
    return t.router.dispatch(msg);
}

// ---- table properties --------------------------------------------------------------

static void checkTable(const char *const *names, int count)
{
    for (int i = 0; i < count; i++)
    {
        // no pattern characters: an exact match cannot match another entry as a pattern
        TEST_ASSERT_NULL_MESSAGE(strpbrk(names[i], "*?[]{},"), names[i]);
        TEST_ASSERT_EQUAL_INT('/', names[i][0]);
        for (int j = 0; j < count; j++)
        {
            if (i == j)
                continue;
            TEST_ASSERT_NOT_EQUAL_MESSAGE(0, strcmp(names[i], names[j]), names[i]);
            // no entry is a '/'-prefix of another (e.g. "/a" and "/a/b")
            const size_t len = strlen(names[i]);
            const bool prefix = !strncmp(names[i], names[j], len) && names[j][len] == '/';
            TEST_ASSERT_FALSE_MESSAGE(prefix, names[j]);
        }
    }
}

void test_table_properties()
{
    TEST_ASSERT_TRUE(FULL_COUNT <= 255);
    checkTable(fullNames, FULL_COUNT);
    checkTable(minimalNames, MINIMAL_COUNT);
}

// ---- fast path ---------------------------------------------------------------------

void test_every_address_exact_match()
{
    for (int i = 0; i < FULL_COUNT; i++)
    {
        TEST_ASSERT_EQUAL_MESSAGE(OSC_DISPATCH_FAST, send(full, fullNames[i], i), fullNames[i]);
        TEST_ASSERT_EQUAL(1, (int)calls.size());
        TEST_ASSERT_EQUAL(i, calls[0]);
        TEST_ASSERT_EQUAL(i, lastIntArg);
    }
}

void test_similar_addresses()
{
    // "/run" and "/runRaw" do not match each other
    TEST_ASSERT_EQUAL(OSC_DISPATCH_FAST, send(full, "/run"));
    TEST_ASSERT_EQUAL(full.indexOf("/run"), calls[0]);
    TEST_ASSERT_EQUAL(OSC_DISPATCH_FAST, send(full, "/runRaw"));
    TEST_ASSERT_EQUAL(full.indexOf("/runRaw"), calls[0]);
}

// ---- slow path ---------------------------------------------------------------------

static std::vector<int> expected(Table &t, std::initializer_list<const char *> names)
{
    std::vector<int> v;
    for (const char *n : names)
        v.push_back(t.indexOf(n));
    return v;
}

void test_star()
{
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/enable*Report"));
    // table order
    std::vector<int> e = expected(full, {"/enableBusyReport", "/enableHizReport", "/enableHomeSwReport",
                                         "/enableDirReport", "/enableMotorStatusReport", "/enableSwEventReport",
                                         "/enableUvloReport", "/enableThermalStatusReport",
                                         "/enableOverCurrentReport", "/enableStallReport",
                                         "/enableLimitSwReport"});
    TEST_ASSERT_EQUAL(e.size(), calls.size());
    TEST_ASSERT_EQUAL_INT_ARRAY(e.data(), calls.data(), e.size());
}

void test_question_mark()
{
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/setAc?"));
    TEST_ASSERT_EQUAL(1, (int)calls.size());
    TEST_ASSERT_EQUAL(full.indexOf("/setAcc"), calls[0]);
}

void test_brackets()
{
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/set[AD][ce]c"));
    std::vector<int> e = expected(full, {"/setAcc", "/setDec"});
    TEST_ASSERT_EQUAL(e.size(), calls.size());
    TEST_ASSERT_EQUAL_INT_ARRAY(e.data(), calls.data(), e.size());
    // range and negation
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/set[A-C]cc"));
    TEST_ASSERT_EQUAL(1, (int)calls.size());
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/set[!A]ec"));
    TEST_ASSERT_EQUAL(full.indexOf("/setDec"), calls[0]);
}

void test_braces()
{
    // in table order (/setAcc comes before /setDec), not in the order of the pattern
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/set{Dec,Acc}"));
    std::vector<int> e = expected(full, {"/setAcc", "/setDec"});
    TEST_ASSERT_EQUAL(e.size(), calls.size());
    TEST_ASSERT_EQUAL_INT_ARRAY(e.data(), calls.data(), e.size());
}

void test_trailing_path()
{
    // an address followed by "/..." matches (CNMAT partial match)
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/run/foo"));
    TEST_ASSERT_EQUAL(1, (int)calls.size());
    TEST_ASSERT_EQUAL(full.indexOf("/run"), calls[0]);
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(full, "/combine/move/x/y"));
    TEST_ASSERT_EQUAL(full.indexOf("/combine/move"), calls[0]);
}

void test_not_matched()
{
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(full, "/foo"));
    TEST_ASSERT_EQUAL(0, (int)calls.size());
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(full, "/ru"));     // prefix only
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(full, "/runRawX")); // longer
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(full, "/combine")); // no "/combine" entry
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(full, "/"));
}

void test_ifdef_disabled()
{
    // entries removed by #ifdef are not matched, by exact address or by pattern
    TEST_ASSERT_TRUE(full.indexOf("/activate") >= 0);
    TEST_ASSERT_EQUAL(-1, minimal.indexOf("/activate"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_FAST, send(full, "/activate"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(minimal, "/activate"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(minimal, "/setTval"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(minimal, "/getLimitSw"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(minimal, "/enableElectromagnet*"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_FAST, send(full, "/getBrakeStatus"));
    TEST_ASSERT_EQUAL(OSC_DISPATCH_NOT_MATCHED, send(minimal, "/getBrakeStatus"));
    // /enable*Report without /enableLimitSwReport
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, send(minimal, "/enable*Report"));
    TEST_ASSERT_EQUAL(10, (int)calls.size());
}

// The handlers of the slow path share one message, so an index error of one handler is
// seen by the next ones (as with the former route() chain).
static std::vector<int32_t> seen;
static void readsPastEnd(OscMessage &m)
{
    seen.push_back(m.getInt(0));
    m.getInt(5);
}
static void readsFirst(OscMessage &m) { seen.push_back(m.getInt(0)); }

void test_shared_message_in_slow_path()
{
    OscRoute routes[] = {{"/aa", readsPastEnd}, {"/ab", readsFirst}};
    uint8_t index[2];
    OscRouter router;
    router.begin(routes, 2, index);
    const uint8_t packet[] = {'/', 'a', '?', 0, ',', 'i', 0, 0, 0, 0, 0, 9};
    msg.decode(packet, sizeof(packet));
    seen.clear();
    TEST_ASSERT_EQUAL(OSC_DISPATCH_SLOW, router.dispatch(msg));
    TEST_ASSERT_EQUAL(2, (int)seen.size());
    TEST_ASSERT_EQUAL_INT32(9, seen[0]);
    TEST_ASSERT_EQUAL_INT32(0, seen[1]);
}

void test_unclosed_bracket_is_safe()
{
    // osc_match() scans past the end of an unclosed '[' or '{'; the guards stop it.
    // (CNMAT read outside the address in this case; any result is accepted, but it must
    // terminate inside the buffer.)
    send(full, "/set[Acc");
    send(full, "/set{Acc");
    send(full, "/s[e");
    send(full, "/{a,");
    TEST_PASS();
}

int main(int, char **)
{
    full.begin(fullNames, FULL_COUNT);
    minimal.begin(minimalNames, MINIMAL_COUNT);
    UNITY_BEGIN();
    RUN_TEST(test_table_properties);
    RUN_TEST(test_every_address_exact_match);
    RUN_TEST(test_similar_addresses);
    RUN_TEST(test_star);
    RUN_TEST(test_question_mark);
    RUN_TEST(test_brackets);
    RUN_TEST(test_braces);
    RUN_TEST(test_trailing_path);
    RUN_TEST(test_not_matched);
    RUN_TEST(test_ifdef_disabled);
    RUN_TEST(test_shared_message_in_slow_path);
    RUN_TEST(test_unclosed_bracket_is_safe);
    return UNITY_END();
}
