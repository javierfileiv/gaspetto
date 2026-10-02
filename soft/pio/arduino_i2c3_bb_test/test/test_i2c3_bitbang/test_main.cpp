#include "i2c3_bitbang.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <unity.h>

static const uint8_t kAds1115Address      = 0x4A;
static const uint16_t kAds1115ResetConfig = 0x8583;
static const uint16_t kRailSettleDelayMs  = 250;

void setUp(void)
{
    i2c3_bb_init();
    delay(10);
}

void tearDown(void)
{
    i2c3_bb_stop();
    delay(10);
}

void test_init_does_not_crash(void)
{
    i2c3_bb_init();
    TEST_ASSERT_TRUE(true);
}

void test_start_succeeds_on_idle_bus(void)
{
    bool result = i2c3_bb_start();
    TEST_ASSERT_TRUE(result);
    i2c3_bb_stop();
}

void test_probe_detects_ads1115(void)
{
    bool found = i2c3_bb_probe(kAds1115Address);
    TEST_ASSERT_TRUE_MESSAGE(found, "ADS1115 at 0x4A not found on I2C3");
}

void test_probe_rejects_invalid_address(void)
{
    bool found = i2c3_bb_probe(0x7F);
    TEST_ASSERT_FALSE_MESSAGE(found, "No device should respond at 0x7F");
}

void test_read_config_register(void)
{
    uint16_t config = 0;
    bool ok         = i2c3_bb_read_register(kAds1115Address, 0x01, config);
    TEST_ASSERT_TRUE_MESSAGE(ok, "Failed to read config register");
    TEST_ASSERT_EQUAL_HEX16(kAds1115ResetConfig, config);
}

void test_write_and_read_config_register(void)
{
    uint16_t original = 0;
    bool ok           = i2c3_bb_read_register(kAds1115Address, 0x01, original);
    TEST_ASSERT_TRUE(ok);

    uint16_t testValue = 0xC0E3;
    ok                 = i2c3_bb_write_register(kAds1115Address, 0x01, testValue);
    TEST_ASSERT_TRUE_MESSAGE(ok, "Failed to write config register");

    uint16_t readBack = 0;
    ok                = i2c3_bb_read_register(kAds1115Address, 0x01, readBack);
    TEST_ASSERT_TRUE_MESSAGE(ok, "Failed to read back config register");
    TEST_ASSERT_EQUAL_HEX16(testValue & 0x7FFF, readBack & 0x7FFF);

    i2c3_bb_write_register(kAds1115Address, 0x01, original);
}

void test_full_transaction_sequence(void)
{
    bool ok = i2c3_bb_start();
    TEST_ASSERT_TRUE(ok);

    ok = i2c3_bb_write_byte(kAds1115Address << 1);
    TEST_ASSERT_TRUE(ok);

    ok = i2c3_bb_write_byte(0x01);
    TEST_ASSERT_TRUE(ok);

    ok = i2c3_bb_start();
    TEST_ASSERT_TRUE(ok);

    ok = i2c3_bb_write_byte((kAds1115Address << 1) | 0x01);
    TEST_ASSERT_TRUE(ok);

    uint8_t high   = i2c3_bb_read_byte(true);
    uint8_t low    = i2c3_bb_read_byte(false);
    uint16_t value = (high << 8) | low;

    TEST_ASSERT_NOT_EQUAL(0x0000, value);
    TEST_ASSERT_NOT_EQUAL(0xFFFF, value);

    i2c3_bb_stop();
}

void setup()
{
    delay(3000);
    Serial.begin(115200);

    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    delay(100);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
    delay(kRailSettleDelayMs);

    i2c3_bb_init();
    delay(50);

    UNITY_BEGIN();
    RUN_TEST(test_init_does_not_crash);
    RUN_TEST(test_start_succeeds_on_idle_bus);
    RUN_TEST(test_probe_detects_ads1115);
    RUN_TEST(test_probe_rejects_invalid_address);
    RUN_TEST(test_read_config_register);
    RUN_TEST(test_write_and_read_config_register);
    RUN_TEST(test_full_transaction_sequence);
    UNITY_END();
}

void loop()
{
    delay(1000);
}
