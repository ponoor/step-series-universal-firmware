// Host tests of OscWriter (src/oscWriter.cpp).
//
// Expected bytes are derived from CNMAT OSC 6cea108 OSCMessage::send():
//  - address, NUL, padded with NULs to a multiple of 4
//  - ',' + one tag per argument, then padSize(count + 1) NULs, or 4 NULs if that is 0
//  - 'i'/'f': 4 bytes big endian; 's': string, NUL, padded to a multiple of 4
// The writer was also compared with that CNMAT code on 300,000 random messages of
// int/float/string arguments (identical bytes). Exception by specification: bool is sent
// as 'i' (0/1) instead of the CNMAT 'T'/'F'.

#include <unity.h>
#include <string.h>
#include <vector>
#include "oscWriter.h"

void setUp() {}
void tearDown() {}

static uint8_t buf[256];

static void assertBytes(const std::vector<uint8_t> &expected, const OscWriter &w)
{
    const uint16_t len = w.encode(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT16(expected.size(), len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), buf, expected.size());
}

void test_two_ints_position()
{
    // /position 1 -1234 (sendTwoData with uint8_t and int32_t)
    OscWriter w("/position");
    w.add((uint8_t)1).add((int32_t)-1234);
    assertBytes({'/', 'p', 'o', 's', 'i', 't', 'i', 'o', 'n', 0, 0, 0, // 10 -> 12
                 ',', 'i', 'i', 0,                                     // padSize(3) = 1
                 0, 0, 0, 1,
                 0xFF, 0xFF, 0xFB, 0x2E},
                w);
}

void test_type_tag_padding_when_aligned()
{
    // ",iii" is 4 bytes: CNMAT adds 4 NULs so that the tag string is NUL-terminated.
    // (/homeSw motorID state dir from sendThreeInt)
    OscWriter w("/homeSw");
    w.add(2).add(1).add(0);
    assertBytes({'/', 'h', 'o', 'm', 'e', 'S', 'w', 0,
                 ',', 'i', 'i', 'i', 0, 0, 0, 0,
                 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0, 0},
                w);
}

void test_string_error_osc()
{
    // /error/osc "MessageNotMatch": 15 chars + NUL = 16, no padding
    OscWriter w("/error/osc");
    w.add("MessageNotMatch");
    assertBytes({'/', 'e', 'r', 'r', 'o', 'r', '/', 'o', 's', 'c', 0, 0,
                 ',', 's', 0, 0,
                 'M', 'e', 's', 's', 'a', 'g', 'e', 'N', 'o', 't', 'M', 'a', 't', 'c', 'h', 0},
                w);
}

void test_string_and_int_error_command()
{
    // /error/command "InServoMode" 3
    OscWriter w("/error/command");
    w.add("InServoMode").add((uint8_t)3);
    assertBytes({'/', 'e', 'r', 'r', 'o', 'r', '/', 'c', 'o', 'm', 'm', 'a', 'n', 'd', 0, 0,
                 ',', 's', 'i', 0,
                 'I', 'n', 'S', 'e', 'r', 'v', 'o', 'M', 'o', 'd', 'e', 0,
                 0, 0, 0, 3},
                w);
}

void test_floats_speed_profile()
{
    // /speedProfile 1 1000.0 500.0 620.0 (int + 3 floats)
    OscWriter w("/speedProfile");
    w.add((int32_t)1).add(1000.0f).add(500.0f).add(620.0f);
    assertBytes({'/', 's', 'p', 'e', 'e', 'd', 'P', 'r', 'o', 'f', 'i', 'l', 'e', 0, 0, 0,
                 ',', 'i', 'f', 'f', 'f', 0, 0, 0,
                 0, 0, 0, 1,
                 0x44, 0x7A, 0x00, 0x00,  // 1000.0f
                 0x43, 0xFA, 0x00, 0x00,  // 500.0f
                 0x44, 0x1B, 0x00, 0x00}, // 620.0f
                w);
}

void test_config_name()
{
    // /configName name sd open parse filename
    OscWriter w("/configName");
    char filename[16] = "/config.txt";
    w.add("abc").add(1).add(1).add(0).add(filename);
    assertBytes({'/', 'c', 'o', 'n', 'f', 'i', 'g', 'N', 'a', 'm', 'e', 0,
                 ',', 's', 'i', 'i', 'i', 's', 0, 0,
                 'a', 'b', 'c', 0,
                 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
                 '/', 'c', 'o', 'n', 'f', 'i', 'g', '.', 't', 'x', 't', 0},
                w);
}

void test_empty_string_and_no_args()
{
    OscWriter w("/a");
    w.add("");
    assertBytes({'/', 'a', 0, 0, ',', 's', 0, 0, 0, 0, 0, 0}, w);
    OscWriter n("/booted");
    // no arguments: "," + 3 NULs
    assertBytes({'/', 'b', 'o', 'o', 't', 'e', 'd', 0, ',', 0, 0, 0}, n);
}

// bool must become 'i' 0/1, also through templates like sendTwoData().
template <class T, class U>
static void addTwo(OscWriter &w, T a, U b) { w.add(a).add(b); }

void test_bool_is_int()
{
    // /uvlo motorID state, with the state as bool (formerly sent as 'T'/'F')
    OscWriter w("/uvlo");
    bool state = true;
    addTwo(w, (uint8_t)1, state);
    assertBytes({'/', 'u', 'v', 'l', 'o', 0, 0, 0,
                 ',', 'i', 'i', 0,
                 0, 0, 0, 1, 0, 0, 0, 1},
                w);
    OscWriter f("/x");
    f.add(false);
    assertBytes({'/', 'x', 0, 0, ',', 'i', 0, 0, 0, 0, 0, 0}, f);
}

void test_low_speed_optimize_threshold()
{
    // /lowSpeedOptimizeThreshold motorID threshold enabled(bool -> i)
    OscWriter w("/lowSpeedOptimizeThreshold");
    bool enabled = false;
    w.add((int32_t)2).add(20.0f).add(enabled);
    const uint16_t len = w.encode(buf, sizeof(buf));
    // address: 26 chars + NUL = 27 -> 28; ",ifi" is aligned -> 4 NULs
    TEST_ASSERT_EQUAL_UINT16(28 + 8 + 12, len);
    TEST_ASSERT_EQUAL_MEMORY(",ifi\0\0\0\0", buf + 28, 8);
    const uint8_t enabledBytes[] = {0, 0, 0, 0};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(enabledBytes, buf + 44, 4);
}

enum TestEnum : uint8_t
{
    TEST_ENUM_THREE = 3
};

void test_integer_types()
{
    // every integer type of up to 4 bytes and enums are 'i'
    OscWriter w("/t");
    w.add((int8_t)-1).add((uint16_t)0xFFFF).add((int16_t)-2).add((unsigned)7).add(TEST_ENUM_THREE).add('A');
    const uint16_t len = w.encode(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT16(4 + 8 + 24, len);
    TEST_ASSERT_EQUAL_MEMORY(",iiiiii\0", buf + 4, 8);
    const uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
                            0, 0, 0, 7, 0, 0, 0, 3, 0, 0, 0, 'A'};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data, buf + 12, sizeof(data));
}

void test_position_list()
{
    // /positionList with 8 ints: ",iiiiiiii" = 9 -> 12
    OscWriter w("/positionList");
    for (int32_t i = 0; i < 8; i++)
        w.add(i);
    const uint16_t len = w.encode(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT16(16 + 12 + 32, len);
    TEST_ASSERT_EQUAL_MEMORY(",iiiiiiii\0\0\0", buf + 16, 12);
}

void test_overflow()
{
    // does not fit: nothing is written, 0 is returned
    OscWriter w("/abc");
    w.add("hello");
    // "/abc" 8 + ",s" 4 + "hello" 8 = 20 bytes
    TEST_ASSERT_EQUAL_UINT16(0, w.encode(buf, 19));
    TEST_ASSERT_EQUAL_UINT16(20, w.encode(buf, 20));
    // too many arguments
    OscWriter m("/m");
    for (int i = 0; i <= OSC_TX_MAX_ARGS; i++)
        m.add(i);
    TEST_ASSERT_EQUAL_UINT16(0, m.encode(buf, sizeof(buf)));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_two_ints_position);
    RUN_TEST(test_type_tag_padding_when_aligned);
    RUN_TEST(test_string_error_osc);
    RUN_TEST(test_string_and_int_error_command);
    RUN_TEST(test_floats_speed_profile);
    RUN_TEST(test_config_name);
    RUN_TEST(test_empty_string_and_no_args);
    RUN_TEST(test_bool_is_int);
    RUN_TEST(test_low_speed_optimize_threshold);
    RUN_TEST(test_integer_types);
    RUN_TEST(test_position_list);
    RUN_TEST(test_overflow);
    return UNITY_END();
}
