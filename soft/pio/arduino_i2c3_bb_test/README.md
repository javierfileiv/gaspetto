# I2C3 Bit-Bang Test

Hardware validation project for the I2C3 bit-bang implementation and BitBangWire wrapper on STM32F411CE (BlackPill).

## Purpose

The STM32F411CE's hardware I2C3 peripheral on PB4/PA8 pins has reliability issues. While initialization and device detection work, actual ADC conversions hang during reads. This project validates the bit-bang workaround and the BitBangWire wrapper that integrates it with the Adafruit_ADS1X15 library.

## What It Tests

1. **BitBangWire probe** — device detection at address 0x4A (ADS1115) via `beginTransmission`/`endTransmission`
2. **BitBangWire register read** — read config register, compare with reset value 0x8583
3. **BitBangWire register write** — write + read-back config register
4. **Adafruit_ADS1115 integration** — `ads.begin(0x4A, &gI2c3)` with BitBangWire as `TwoWire*`
5. **ADC channel reads** — read all 4 channels via `ads.readADC_SingleEnded()`, each cross-checked against a direct bit-bang read of the conversion register
6. **Adafruit register write** — seed a known CONFIG through the bit-bang, ask Adafruit to rewrite it with MUX = AIN3, read it back through the bit-bang

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
pio device monitor -b 115200
```

## Expected Output

```
--- BITBANGWIRE + ADS1115 TEST STARTED ---
3V3 sensor rail ON

1. BitBangWire probe
   [OK]   probe 0x4A responds ACK

2. BitBangWire read config register
      config = 0x8583
   [OK]   write register pointer
   [OK]   read 2 bytes
   [OK]   config matches reset value 0x8583

3. BitBangWire write + read-back config register
      wrote 0xC0E3, read back 0x40E3
   [OK]   write config register
   [OK]   read back matches

4. Adafruit_ADS1115 integration via BitBangWire
   [OK]   ads.begin(0x4A, &gI2c3)

5. ADC channel reads via Adafruit_ADS1115
      ch0: <raw count>
      ch1: <raw count>
      ch2: <raw count>
      ch3: <raw count>
   [OK]   all 4 channels match bit-bang reads

6. Adafruit register write reaches bit-bang
      CONFIG = 0xF380
   [OK]   seed CONFIG register
   [OK]   Adafruit write reaches bit-bang bus

--- SUMMARY ---
PASSED: 10
FAILED: 0
TOTAL:  10
ALL TESTS PASSED
--- DONE ---
```

Channel counts depend on the voltage present on AIN0 to AIN3 and change from one run to
the next, so they are left as placeholders. What the test asserts is that each count
equals a direct bit-bang read of the conversion register, not a particular value.
`CONFIG = 0xF380` is stable: it is `startADCReading()` writing MUX single-ended AIN3 with
`CQUE_1CONV` (low two bits at 0, not `CQUE_NONE`) and the library default data rate.

## Implementation Details

- Uses bit-bang implementation from `soft/emu-pc/targets/gbox/src/i2c3_bitbang.cpp` (symlinked)
- Header from `soft/emu-pc/targets/gbox/include/i2c3_bitbang.h` (symlinked)
- Pin definitions from `soft/emu-pc/targets/gbox/include/pin_definitions.h` (symlinked)
- BitBangWire wrapper from `soft/pio/GBox_pio/include/BitBangWire.h` (symlinked)
- I2C speed: ~33kHz (10us delay between transitions, 30us per bit)

## Related Projects

- **GBox_pio**: Production firmware for the box, uses this same bit-bang driver
- **arduino_box_hw_test**: Full hardware test including I2C1 and I2C3 buses

## See Also

- [I2C3 Bit-Bang Documentation](../../emu-pc/README.md#i2c3-bit-bang-implementation)
