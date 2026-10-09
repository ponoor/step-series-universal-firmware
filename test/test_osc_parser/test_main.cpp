// Host tests of OscMessage (src/oscParser.cpp).
//
// The expected results of the malformed cases follow CNMAT OSC 6cea108
// (OSCMessage::decode()/hasError() and the getters). The parser was also compared
// with that CNMAT code on 1,000,000 random, truncated and mutated packets (no
// difference except more than OSC_MAX_ARGS arguments, see test_too_many_args).

#include <unity.h>
#include <string.h>
#include <string>
#include <vector>
#include "oscParser.h"

void setUp() {}
void tearDown() {}

// Small helper to build packets.
struct Packet
{
    std::vector<uint8_t> bytes;
    Packet &str(const char *s) // NUL-terminated, padded to 4 bytes
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
    Packet &raw(const char *s, size_t n)
    {
        bytes.insert(bytes.end(), (const uint8_t *)s, (const uint8_t *)s + n);
        return *this;
    }
};

static OscMessage msg;
// The parser does not copy: keep the bytes alive while msg is used.
static std::vector<uint8_t> current;
static void decode(const Packet &p)
{
    current = p.bytes;
    msg.decode(current.data(), (uint16_t)current.size());
}

// ---- normal cases ----------------------------------------------------------

void test_int_float_string()
{
    decode(Packet().str("/setKval").str(",ifs").i32(3).f32(1.5f).str("abc"));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_STRING("/setKval", msg.getAddress());
    TEST_ASSERT_EQUAL(3, msg.size());
    TEST_ASSERT_TRUE(msg.isInt(0));
    TEST_ASSERT_EQUAL_INT32(3, msg.getInt(0));
    TEST_ASSERT_TRUE(msg.isFloat(1));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, msg.getFloat(1));
    TEST_ASSERT_TRUE(msg.isString(2));
    TEST_ASSERT_EQUAL(4, msg.getDataLength(2)); // strlen + 1, like CNMAT OSCData::bytes
    char buf[8];
    TEST_ASSERT_EQUAL(4, msg.getString(2, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("abc", buf);
    TEST_ASSERT_FALSE(msg.hasError());
}

void test_negative_int_and_types()
{
    decode(Packet().str("/run").str(",if").i32((uint32_t)-1234).f32(-200.25f));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_INT32(-1234, msg.getInt(0));
    // wrong type: CNMAT returns 0 and keeps no error
    TEST_ASSERT_FALSE(msg.isFloat(0));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, msg.getFloat(0));
    TEST_ASSERT_EQUAL_INT32(0, msg.getInt(1));
    TEST_ASSERT_EQUAL_FLOAT(-200.25f, msg.getFloat(1));
    TEST_ASSERT_EQUAL('f', msg.getType(1));
    TEST_ASSERT_FALSE(msg.hasError());
}

void test_true_false()
{
    // T and F have no data bytes.
    decode(Packet().str("/enableBusyReport").str(",iTF").i32(1));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL(3, msg.size());
    TEST_ASSERT_TRUE(msg.isBoolean(1));
    TEST_ASSERT_TRUE(msg.getBoolean(1));
    TEST_ASSERT_TRUE(msg.isBoolean(2));
    TEST_ASSERT_FALSE(msg.getBoolean(2));
    TEST_ASSERT_FALSE(msg.isBoolean(0));
    TEST_ASSERT_FALSE(msg.getBoolean(0)); // not a bool: false
    TEST_ASSERT_EQUAL(0, msg.getDataLength(1));
    TEST_ASSERT_EQUAL_INT32(1, msg.getInt(0));
}

void test_max_udpsend()
{
    // Max [udpsend] for the message "/goTo 1 -1000": address padded with NULs to 8 bytes,
    // ",ii" + NUL, then two big-endian int32.
    const uint8_t bytes[] = {'/', 'g', 'o', 'T', 'o', 0, 0, 0,
                             ',', 'i', 'i', 0,
                             0x00, 0x00, 0x00, 0x01,
                             0xFF, 0xFF, 0xFC, 0x18};
    msg.decode(bytes, sizeof(bytes));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_STRING("/goTo", msg.getAddress());
    TEST_ASSERT_EQUAL_INT32(1, msg.getInt(0));
    TEST_ASSERT_EQUAL_INT32(-1000, msg.getInt(1));
    // "/run 2 1.5": float argument
    decode(Packet().str("/run").str(",if").i32(2).f32(1.5f));
    TEST_ASSERT_EQUAL_FLOAT(1.5f, msg.getFloat(1));
    // symbol argument: "/setConfigName test"
    decode(Packet().str("/setConfigName").str(",s").str("test"));
    TEST_ASSERT_TRUE(msg.isString(0));
    TEST_ASSERT_EQUAL(5, msg.getDataLength(0));
}

void test_string_lengths()
{
    // string of length 3 (no padding) and 4 (4 NUL padding bytes) followed by an int
    decode(Packet().str("/a").str(",sisi").str("abc").i32(7).str("abcd").i32(8));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_INT32(7, msg.getInt(1));
    TEST_ASSERT_EQUAL_INT32(8, msg.getInt(3));
    char buf[8];
    TEST_ASSERT_EQUAL(5, msg.getString(2, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("abcd", buf);
    // buffer too small: CNMAT copies nothing and returns 0 (no error)
    TEST_ASSERT_EQUAL(0, msg.getString(2, buf, 4));
    // not a string: 0
    TEST_ASSERT_EQUAL(0, msg.getString(1, buf, sizeof(buf)));
    TEST_ASSERT_FALSE(msg.hasError());
}

void test_no_arguments()
{
    // "/getPositionList" with an empty type tag string
    decode(Packet().str("/getPositionList").str(","));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL(0, msg.size());
}

void test_other_types_are_skipped()
{
    // h (int64), d (double), t (timetag) and b (blob) are decoded with their lengths and
    // are not visible as i/f/s/T/F.
    decode(Packet().str("/x").str(",hdbti").i32(0).i32(5).i32(0x3FF00000).i32(0)
               .i32(3).raw("xyz\0", 4).i32(1).i32(2).i32(42));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL(5, msg.size());
    TEST_ASSERT_FALSE(msg.isInt(0));
    TEST_ASSERT_EQUAL_INT32(0, msg.getInt(0));
    TEST_ASSERT_EQUAL(8, msg.getDataLength(0));
    TEST_ASSERT_FALSE(msg.isFloat(1));
    TEST_ASSERT_EQUAL(8, msg.getDataLength(1));
    TEST_ASSERT_EQUAL('b', msg.getType(2));
    TEST_ASSERT_EQUAL(7, msg.getDataLength(2)); // CNMAT: blob length + 4
    TEST_ASSERT_EQUAL(8, msg.getDataLength(3));
    TEST_ASSERT_EQUAL_INT32(42, msg.getInt(4));
    TEST_ASSERT_FALSE(msg.hasError());
}

void test_args_limit()
{
    // OSC_MAX_ARGS arguments are accepted.
    Packet p;
    p.str("/setTargetPositionList");
    std::string tags = ",";
    for (int i = 0; i < OSC_MAX_ARGS; i++)
        tags += 'i';
    p.str(tags.c_str());
    for (int i = 0; i < OSC_MAX_ARGS; i++)
        p.i32(i * 100);
    decode(p);
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL(OSC_MAX_ARGS, msg.size());
    TEST_ASSERT_EQUAL_INT32((OSC_MAX_ARGS - 1) * 100, msg.getInt(OSC_MAX_ARGS - 1));
}

// ---- index out of range ------------------------------------------------------

void test_index_out_of_range_is_sticky()
{
    // CNMAT getOSCData() sets INDEX_OUT_OF_BOUNDS on the message; afterwards every getter
    // returns its default value, even for valid positions (utils getInt() then sends
    // WrongDataType for each).
    decode(Packet().str("/goTo").str(",ii").i32(1).i32(2));
    TEST_ASSERT_EQUAL_INT32(2, msg.getInt(1));
    TEST_ASSERT_FALSE(msg.isInt(2));
    TEST_ASSERT_TRUE(msg.hasError());
    TEST_ASSERT_FALSE(msg.isInt(0));
    TEST_ASSERT_EQUAL_INT32(0, msg.getInt(0));
    TEST_ASSERT_EQUAL(0, msg.getDataLength(0));
    char buf[4];
    TEST_ASSERT_EQUAL(-1, msg.getString(0, buf, 4));
    TEST_ASSERT_EQUAL('\0', msg.getType(0));
    // a new packet clears it
    decode(Packet().str("/goTo").str(",ii").i32(1).i32(2));
    TEST_ASSERT_FALSE(msg.hasError());
}

void test_missing_argument_same_as_wrong_type()
{
    // "/getPosition" without arguments: isFloat(0) and isInt(0) are false, as for a
    // wrong type, so getInt() of utils sends WrongDataType in both cases.
    decode(Packet().str("/getPosition").str(","));
    TEST_ASSERT_FALSE(msg.isFloat(0));
    TEST_ASSERT_FALSE(msg.isInt(0));
    TEST_ASSERT_EQUAL_INT32(0, msg.getInt(0));
}

// ---- malformed packets (results of CNMAT 6cea108) -----------------------------

void test_empty_and_no_address()
{
    // No '/' at all: the address is never decoded -> error (OscSyntaxError).
    msg.decode(nullptr, 0);
    TEST_ASSERT_TRUE(msg.hasError());
    decode(Packet().raw("abc\0", 4));
    TEST_ASSERT_TRUE(msg.hasError());
}

void test_address_without_nul()
{
    // The address must be NUL-terminated -> error.
    decode(Packet().raw("/run", 4));
    TEST_ASSERT_TRUE(msg.hasError());
}

void test_no_type_tag()
{
    // No type tag string: CNMAT accepts it as a message without arguments.
    decode(Packet().str("/getPositionList"));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL(0, msg.size());
    // length not a multiple of 4 is not checked either
    decode(Packet().raw("/getPositionList\0", 17));
    TEST_ASSERT_FALSE(msg.hasError());
}

void test_truncated()
{
    // data shorter than the type tags -> error
    decode(Packet().str("/goTo").str(",ii").i32(1).raw("\0\0", 2));
    TEST_ASSERT_TRUE(msg.hasError());
    // no data at all -> error
    decode(Packet().str("/goTo").str(",ii"));
    TEST_ASSERT_TRUE(msg.hasError());
    // type tag string cut before its padding -> error
    decode(Packet().raw("/goTo\0\0\0,ii", 11));
    TEST_ASSERT_TRUE(msg.hasError());
    // unterminated string -> error
    decode(Packet().str("/setConfigName").str(",s").raw("abcd", 4));
    TEST_ASSERT_TRUE(msg.hasError());
}

void test_more_data_than_tags()
{
    // extra bytes after the last argument are ignored
    decode(Packet().str("/goTo").str(",i").i32(1).i32(2).raw("xy", 2));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL(1, msg.size());
    TEST_ASSERT_EQUAL_INT32(1, msg.getInt(0));
}

void test_unsupported_types()
{
    // CNMAT has no decoder for 'c', 'N', 'I': the argument never gets data -> error
    decode(Packet().str("/x").str(",N"));
    TEST_ASSERT_TRUE(msg.hasError());
    decode(Packet().str("/x").str(",ci").i32(1).i32(2));
    TEST_ASSERT_TRUE(msg.hasError());
}

void test_bundle()
{
    // CNMAT has no bundle support in OSCMessage: the bytes before the first '/' stay in
    // its buffer, so the address becomes "#bundle" and the first element's arguments
    // are decoded. No route matches "#bundle" -> MessageNotMatch.
    Packet inner;
    inner.str("/run").str(",if").i32(1).f32(100.0f);
    Packet p;
    p.str("#bundle").i32(0).i32(1).i32((uint32_t)inner.bytes.size());
    p.bytes.insert(p.bytes.end(), inner.bytes.begin(), inner.bytes.end());
    decode(p);
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_STRING("#bundle", msg.getAddress());
    TEST_ASSERT_EQUAL(2, msg.size());
    TEST_ASSERT_EQUAL_INT32(1, msg.getInt(0));
}

void test_garbage_before_address()
{
    // Bytes before the '/' become part of the address (CNMAT keeps them in its buffer).
    decode(Packet().raw("xy/run\0\0", 8).str(",i").i32(5));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_STRING("xy/run", msg.getAddress());
}

void test_string_followed_by_true_quirk()
{
    // CNMAT quirk kept for compatibility: after a string, the padding is counted for the
    // last decoded argument. With a later T/F (0 bytes, already valid) the padding bytes
    // are not skipped and become part of the next int.
    decode(Packet().str("/x").str(",siT").str("ab").i32(0x01020304));
    TEST_ASSERT_FALSE(msg.hasError());
    TEST_ASSERT_EQUAL_INT32(0x00010203, msg.getInt(1));
}

void test_too_many_args()
{
    // Difference from CNMAT (unlimited): more than OSC_MAX_ARGS type tags is an error.
    Packet p;
    p.str("/x");
    std::string tags = ",";
    for (int i = 0; i < OSC_MAX_ARGS + 1; i++)
        tags += 'T';
    p.str(tags.c_str());
    decode(p);
    TEST_ASSERT_TRUE(msg.hasError());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_int_float_string);
    RUN_TEST(test_negative_int_and_types);
    RUN_TEST(test_true_false);
    RUN_TEST(test_max_udpsend);
    RUN_TEST(test_string_lengths);
    RUN_TEST(test_no_arguments);
    RUN_TEST(test_other_types_are_skipped);
    RUN_TEST(test_args_limit);
    RUN_TEST(test_index_out_of_range_is_sticky);
    RUN_TEST(test_missing_argument_same_as_wrong_type);
    RUN_TEST(test_empty_and_no_address);
    RUN_TEST(test_address_without_nul);
    RUN_TEST(test_no_type_tag);
    RUN_TEST(test_truncated);
    RUN_TEST(test_more_data_than_tags);
    RUN_TEST(test_unsupported_types);
    RUN_TEST(test_bundle);
    RUN_TEST(test_garbage_before_address);
    RUN_TEST(test_string_followed_by_true_quirk);
    RUN_TEST(test_too_many_args);
    return UNITY_END();
}
