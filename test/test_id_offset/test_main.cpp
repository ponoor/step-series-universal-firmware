// Host tests of the DIP switch ID added to the IP address and port (src/idOffset.cpp).

#include <unity.h>
#include "idOffset.h"

void setUp() {}
void tearDown() {}

static void assertIp(uint8_t configured, uint8_t id, bool added, uint8_t expected)
{
    uint8_t out = 0x55;
    TEST_ASSERT_EQUAL(added, addIdToIpLastByte(configured, id, out));
    TEST_ASSERT_EQUAL_UINT8(expected, out);
}

void test_ip_default_100()
{
    assertIp(100, 0, true, 100);
    assertIp(100, 1, true, 101);
    assertIp(100, 154, true, 254);  // the largest ID with the default address
    assertIp(100, 155, false, 100); // 255: broadcast address
    assertIp(100, 156, false, 100); // would wrap around to 0
    assertIp(100, 255, false, 100);
}

void test_ip_configured_0()
{
    assertIp(0, 0, true, 0); // 0 + 0: the configured value as is
    assertIp(0, 1, true, 1);
    assertIp(0, 254, true, 254);
    assertIp(0, 255, false, 0);
}

void test_ip_configured_254()
{
    assertIp(254, 0, true, 254);
    assertIp(254, 1, false, 254);
    assertIp(254, 255, false, 254);
}

void test_ip_configured_255()
{
    // A configured broadcast byte is not changed here (only the ID is checked).
    assertIp(255, 0, false, 255);
}

void test_port()
{
    uint16_t out = 0;
    TEST_ASSERT_TRUE(addIdToPort(50100, 0, out));
    TEST_ASSERT_EQUAL_UINT16(50100, out);
    TEST_ASSERT_TRUE(addIdToPort(50100, 255, out));
    TEST_ASSERT_EQUAL_UINT16(50355, out);
    TEST_ASSERT_TRUE(addIdToPort(65280, 255, out));
    TEST_ASSERT_EQUAL_UINT16(65535, out);
    TEST_ASSERT_FALSE(addIdToPort(65281, 255, out));
    TEST_ASSERT_EQUAL_UINT16(65281, out);
    TEST_ASSERT_TRUE(addIdToPort(65535, 0, out));
    TEST_ASSERT_EQUAL_UINT16(65535, out);
    TEST_ASSERT_FALSE(addIdToPort(65535, 1, out));
    TEST_ASSERT_EQUAL_UINT16(65535, out);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_ip_default_100);
    RUN_TEST(test_ip_configured_0);
    RUN_TEST(test_ip_configured_254);
    RUN_TEST(test_ip_configured_255);
    RUN_TEST(test_port);
    return UNITY_END();
}
