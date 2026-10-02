# I2C3 Bit-Bang Test

Hardware validation project for the I2C3 bit-bang implementation on STM32F411CE (BlackPill).

## Purpose

The STM32F411CE's hardware I2C3 peripheral on PB4/PA8 pins has reliability issues. While initialization and device detection work, actual ADC conversions hang during reads. This project validates the bit-bang workaround that provides reliable I2C communication.

## What It Tests

- Device detection at address 0x4A (ADS1115)
- Register read/write operations
- ADC channel reads (all 4 channels)
- Full I2C transaction sequences

## Hardware Setup

- **Board**: STM32F411CE BlackPill
- **I2C3 Pins**: PB4 (SDA), PA8 (SCL)
- **Device**: ADS1115 ADC module at address 0x4A (ADDR pin connected to SDA)
- **Power**: 3V3 sensor rail controlled by PB15 (PIN_MOSFET_3V3_SENSORS)

## Building and Flashing

```bash
cd soft/pio/arduino_i2c3_bb_test

# Put board in DFU mode (hold BOOT0 while pressing RESET)
pio run -e arduino_i2c3_bb_test -t upload

# Wait ~8 seconds for serial port to stabilize after DFU exit
pio test -e arduino_i2c3_bb_test --without-uploading
```

**Note**: The `--without-uploading` flag is required because PlatformIO tries to read serial output immediately after upload, but the board needs time to boot and stabilize the USB CDC serial port.

## Expected Output

```
test/test_i2c3_bitbang/test_main.cpp:119: test_init_does_not_crash      [PASSED]
test/test_i2c3_bitbang/test_main.cpp:120: test_start_succeeds_on_idle_bus [PASSED]
test/test_i2c3_bitbang/test_main.cpp:121: test_probe_detects_ads1115     [PASSED]
test/test_i2c3_bitbang/test_main.cpp:122: test_probe_rejects_invalid_address [PASSED]
test/test_i2c3_bitbang/test_main.cpp:123: test_read_config_register      [PASSED]
test/test_i2c3_bitbang/test_main.cpp:124: test_write_and_read_config_register [PASSED]
test/test_i2c3_bitbang/test_main.cpp:125: test_full_transaction_sequence [PASSED]
```

## Implementation Details

- Uses bit-bang implementation from `soft/emu-pc/targets/gbox/src/i2c3_bitbang.cpp` (symlinked)
- Header from `soft/emu-pc/targets/gbox/include/i2c3_bitbang.h` (symlinked)
- Pin definitions from `soft/emu-pc/targets/gbox/include/pin_definitions.h` (symlinked)
- I2C speed: ~10kHz (50μs delay between transitions)

## Related Projects

- **GBox_pio**: Production firmware that uses `BitBangWire` wrapper to integrate this with Adafruit_ADS1X15 library
- **arduino_box_hw_test**: Full hardware test including I2C1 and I2C3 buses

## See Also

- [I2C3 Bit-Bang Documentation](../../emu-pc/README.md#i2c3-bit-bang-implementation)
- [BitBangWire Wrapper](../GBox_pio/include/BitBangWire.h)
