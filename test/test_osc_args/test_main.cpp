// Host tests of oscCheckArgs() (src/oscArgs.cpp) and of the argument list of every route
// of the real address table (src/oscRoutes.h).

#include <unity.h>
#include <string.h>
#include <string>
#include <vector>
#include <math.h>
#include "oscArgs.h"

void setUp() {}
void tearDown() {}

struct Packet
{
    std::vector<uint8_t> bytes;
    Packet &str(const char *s)
    {
        for (const char *p = s; *p; p++)
            bytes.push_back((uint8_t)*p);
        bytes.push_back(0);
        while (bytes.size() % 4)
            bytes.push_back(0);
        return *this;
    }
    Packet &i32(uint32_t v)
    {
        bytes.push_back(v >> 24);
        bytes.push_back(v >> 16);
        bytes.push_back(v >> 8);
        bytes.push_back(v);
        return *this;
    }
    Packet &f32(float f)
    {
        uint32_t v;
        memcpy(&v, &f, 4);
        return i32(v);
    }
};

static OscMessage msg;
static std::vector<uint8_t> current;
static void decode(const Packet &p)
{
    current = p.bytes;
    msg.decode(current.data(), (uint16_t)current.size());
}
static int32_t bad;
static OscArgStatus check(const char *args, uint8_t motors = 4)
{
    bad = 12345;
    return oscCheckArgs(msg, args, motors, &bad);
}

void test_no_args_needed()
{
    decode(Packet().str("/getVersion").str(","));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check(""));
    decode(Packet().str("/getVersion").str(",i").i32(5)); // extra arguments are ignored
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check(""));
}

void test_missing()
{
    decode(Packet().str("/setKval").str(",i").i32(1));
    TEST_ASSERT_EQUAL(OSC_ARGS_MISSING, check("mnnnn"));
    decode(Packet().str("/setKval").str(","));
    TEST_ASSERT_EQUAL(OSC_ARGS_MISSING, check("m"));
    // the count comes before the types: a wrong type does not hide a missing argument
    decode(Packet().str("/x").str(",s").str("a"));
    TEST_ASSERT_EQUAL(OSC_ARGS_MISSING, check("mn"));
}

void test_ok_int_float_mix()
{
    decode(Packet().str("/setKval").str(",ififf").i32(1).f32(2.5f).i32(3).f32(4).f32(5));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("mnnnn"));
}

void test_wrong_type()
{
    decode(Packet().str("/setKval").str(",isi").i32(1).str("x").i32(3));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("mnn"));
    decode(Packet().str("/setKval").str(",s").str("1"));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("m"));
    decode(Packet().str("/setConfigName").str(",i").i32(1));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("s"));
    decode(Packet().str("/setConfigName").str(",s").str("name"));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("s"));
}

void test_nan_and_inf()
{
    decode(Packet().str("/run").str(",if").i32(1).f32(NAN));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("mn"));
    decode(Packet().str("/run").str(",if").i32(1).f32(INFINITY));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("mn"));
    decode(Packet().str("/run").str(",fi").f32(NAN).i32(1));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("mn"));
}

void test_motor_id_range()
{
    const int32_t ok[] = {0, 1, 8, 255};
    for (int32_t v : ok)
    {
        decode(Packet().str("/getPosition").str(",i").i32((uint32_t)v));
        TEST_ASSERT_EQUAL_MESSAGE(OSC_ARGS_OK, check("m"), std::to_string(v).c_str());
    }
    const int32_t ng[] = {-1, 256, 257, 511, 65536, INT32_MIN, INT32_MAX};
    for (int32_t v : ng)
    {
        decode(Packet().str("/getPosition").str(",i").i32((uint32_t)v));
        TEST_ASSERT_EQUAL_MESSAGE(OSC_ARGS_BAD_MOTOR_ID, check("m"), std::to_string(v).c_str());
        TEST_ASSERT_EQUAL_INT32(v, bad);
    }
    // floats are truncated like the int conversion of the handlers
    decode(Packet().str("/getPosition").str(",f").f32(1.9f));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("m"));
    decode(Packet().str("/getPosition").str(",f").f32(256.0f));
    TEST_ASSERT_EQUAL(OSC_ARGS_BAD_MOTOR_ID, check("m"));
    TEST_ASSERT_EQUAL_INT32(256, bad);
    decode(Packet().str("/getPosition").str(",f").f32(1e20f));
    TEST_ASSERT_EQUAL(OSC_ARGS_BAD_MOTOR_ID, check("m"));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, bad);
    decode(Packet().str("/getPosition").str(",f").f32(-5.0f));
    TEST_ASSERT_EQUAL(OSC_ARGS_BAD_MOTOR_ID, check("m"));
}

void test_bool_like()
{
    decode(Packet().str("/activate").str(",iT").i32(1));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("mb"));
    decode(Packet().str("/activate").str(",iF").i32(1));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("mb"));
    decode(Packet().str("/activate").str(",if").i32(1).f32(0.5f));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("mb"));
    decode(Packet().str("/activate").str(",is").i32(1).str("1"));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("mb"));
    // a boolean is not a number
    decode(Packet().str("/run").str(",iT").i32(1));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("mn"));
}

void test_list()
{
    decode(Packet().str("/setTargetPositionList").str(",iiii").i32(1).i32(2).i32(3).i32(4));
    TEST_ASSERT_EQUAL(OSC_ARGS_OK, check("L", 4));
    TEST_ASSERT_EQUAL(OSC_ARGS_MISSING, check("L", 8));
    decode(Packet().str("/setTargetPositionList").str(",iiis").i32(1).i32(2).i32(3).str("x"));
    TEST_ASSERT_EQUAL(OSC_ARGS_WRONG_TYPE, check("L", 4));
    decode(Packet().str("/setTargetPositionList").str(",iii").i32(1).i32(2).i32(3));
    TEST_ASSERT_EQUAL(OSC_ARGS_MISSING, check("L", 4));
}

// ---- the table -------------------------------------------------------------------

struct Entry
{
    const char *address;
    const char *args;
};
#define HAVE_BRAKE
#define DRIVER_POWERSTEP01
#define HAVE_LIMIT_ADC
#define OSC_ROUTE(address, handler, args) {address, args},
static const Entry table[] = {
#include "oscRoutes.h"
};
#undef OSC_ROUTE
constexpr int TABLE_COUNT = sizeof(table) / sizeof(table[0]);

void test_table_specs_are_valid()
{
    for (int i = 0; i < TABLE_COUNT; i++)
    {
        TEST_ASSERT_NOT_NULL_MESSAGE(table[i].args, table[i].address);
        for (const char *p = table[i].args; *p; p++)
            TEST_ASSERT_NOT_EQUAL_MESSAGE(nullptr, strchr("mnbsL", *p), table[i].address);
        // the motor ID is the first argument when there is one (except set/get without)
        const char *m = strchr(table[i].args, 'm');
        if (m)
            TEST_ASSERT_TRUE_MESSAGE(m == table[i].args || table[i].args[0] == 'm', table[i].address);
    }
}

void test_table_known_entries()
{
    auto spec = [](const char *address) -> const char * {
        for (int i = 0; i < TABLE_COUNT; i++)
            if (!strcmp(table[i].address, address))
                return table[i].args;
        return nullptr;
    };
    TEST_ASSERT_EQUAL_STRING("mnnnn", spec("/setKval"));
    TEST_ASSERT_EQUAL_STRING("mnnn", spec("/setSpeedProfile"));
    TEST_ASSERT_EQUAL_STRING("L", spec("/setTargetPositionList"));
    TEST_ASSERT_EQUAL_STRING("s", spec("/setConfigName"));
    TEST_ASSERT_EQUAL_STRING("mbb", spec("/releaseSw"));
    TEST_ASSERT_EQUAL_STRING("m", spec("/getPosition"));
    TEST_ASSERT_EQUAL_STRING("", spec("/getVersion"));
    TEST_ASSERT_EQUAL_STRING("mnn", spec("/setElPos"));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_args_needed);
    RUN_TEST(test_missing);
    RUN_TEST(test_ok_int_float_mix);
    RUN_TEST(test_wrong_type);
    RUN_TEST(test_nan_and_inf);
    RUN_TEST(test_motor_id_range);
    RUN_TEST(test_bool_like);
    RUN_TEST(test_list);
    RUN_TEST(test_table_specs_are_valid);
    RUN_TEST(test_table_known_entries);
    return UNITY_END();
}
