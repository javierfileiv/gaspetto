# NRF Sender - Radio Testing Utility

A simple utility for testing and debugging nRF24L01+ radio communication. Useful for verifying hardware setup and troubleshooting radio issues.

## Overview

The NRF Sender is a minimal test program that:
- Initializes the RF24 radio module
- Sends test packets at regular intervals
- Helps verify radio hardware and configuration
- Assists in debugging communication issues

## Structure

```
nrf_sender/
├── CMakeLists.txt
├── src/
│   └── nrf_sender.cpp    # Main test program
└── tools/
    └── ...               # Additional utilities
```

## Usage

### Building

```bash
cd build-nrf
cmake -DNRF_SENDER=ON -DCMAKE_BUILD_TYPE=Release ..
make nrf_sender
```

### Running on PC

```bash
./targets/nrf_sender/nrf_sender
```

### Running on Embedded Target

```bash
# Flash to device
st-flash write nrf_sender.bin 0x8000000

# Monitor output
minicom -D /dev/ttyUSB0 -b 115200
```

## Features

### Basic Transmission Test

Sends test packets with incrementing counter:

```cpp
void loop() {
    TestPacket packet;
    packet.counter = packetCount++;
    packet.timestamp = millis();

    if (radio.write(&packet, sizeof(packet))) {
        Serial.println("Packet sent successfully");
    } else {
        Serial.println("Transmission failed");
    }

    delay(1000);  // Send every second
}
```

### Configuration Testing

Test different radio configurations:

```cpp
// Test different power levels
radio.setPALevel(RF24_PA_LOW);
testTransmission("Low Power");

radio.setPALevel(RF24_PA_HIGH);
testTransmission("High Power");

radio.setPALevel(RF24_PA_MAX);
testTransmission("Max Power");

// Test different data rates
radio.setDataRate(RF24_250KBPS);
testTransmission("250Kbps");

radio.setDataRate(RF24_1MBPS);
testTransmission("1Mbps");

radio.setDataRate(RF24_2MBPS);
testTransmission("2Mbps");
```

### Range Testing

Verify communication range:

```cpp
void rangeTest() {
    uint32_t successCount = 0;
    uint32_t failCount = 0;

    for (int i = 0; i < 100; i++) {
        if (radio.write(&testData, sizeof(testData))) {
            successCount++;
        } else {
            failCount++;
        }
        delay(50);
    }

    Serial.print("Success rate: ");
    Serial.print(100.0 * successCount / (successCount + failCount));
    Serial.println("%");
}
```

## Test Scenarios

### 1. Basic Connectivity

Verify radio module is working:

```cpp
void testConnectivity() {
    if (!radio.begin()) {
        Serial.println("ERROR: Radio initialization failed");
        Serial.println("Check SPI connections");
        return;
    }

    Serial.println("Radio initialized successfully");
    radio.printDetails();
}
```

### 2. Loopback Test

Test with two radios (sender and receiver):

**Sender:**
```cpp
void sendLoop() {
    uint32_t data = random(1000);
    Serial.print("Sending: ");
    Serial.println(data);

    radio.stopListening();
    radio.write(&data, sizeof(data));
    radio.startListening();
}
```

**Receiver:**
```cpp
void receiveLoop() {
    if (radio.available()) {
        uint32_t data;
        radio.read(&data, sizeof(data));

        Serial.print("Received: ");
        Serial.println(data);
    }
}
```

### 3. Interference Test

Monitor for interference and packet loss:

```cpp
void interferenceTest() {
    const int TEST_DURATION = 60000;  // 1 minute
    unsigned long startTime = millis();
    uint32_t sent = 0, received = 0;

    while (millis() - startTime < TEST_DURATION) {
        // Send packet
        uint32_t data = sent++;
        radio.write(&data, sizeof(data));

        // Check for ACK
        if (radio.txStandBy(100)) {
            received++;
        }

        delay(10);
    }

    Serial.print("Packet loss: ");
    Serial.print(100.0 * (sent - received) / sent);
    Serial.println("%");
}
```

### 4. Channel Scan

Find clearest channel:

```cpp
void scanChannels() {
    Serial.println("Scanning channels...");

    for (int channel = 0; channel < 126; channel++) {
        radio.setChannel(channel);
        radio.startListening();
        delayMicroseconds(128);

        bool interference = radio.testCarrier();

        Serial.print("Channel ");
        Serial.print(channel);
        Serial.print(": ");
        Serial.println(interference ? "BUSY" : "Clear");
    }
}
```

## Packet Formats

### Test Packet
```cpp
struct TestPacket {
    uint32_t counter;      // Packet sequence number
    uint32_t timestamp;    // Timestamp in milliseconds
    uint8_t data[24];      // Test payload
};
```

### Diagnostic Packet
```cpp
struct DiagnosticPacket {
    uint8_t channel;       // Radio channel
    uint8_t paLevel;       // Power level
    uint8_t dataRate;      // Data rate
    uint32_t lostPackets;  // Lost packet count
    int8_t rssi;          // Signal strength (if available)
};
```

## Troubleshooting Guide

### Radio Won't Initialize

**Symptoms:**
- `radio.begin()` returns false
- No response from radio module

**Solutions:**
1. Check SPI connections (MOSI, MISO, SCK)
2. Verify CE and CSN pin connections
3. Check 3.3V power supply
4. Add 10µF capacitor across VCC/GND
5. Try different CE/CSN pins

### No Packets Received

**Symptoms:**
- Transmitter reports success
- Receiver sees no data

**Solutions:**
1. Verify pipe addresses match
2. Check channel configuration
3. Test with radios close together
4. Verify power supply stability
5. Check for address swap (TX/RX pipes)

### Intermittent Communication

**Symptoms:**
- Sometimes works, sometimes doesn't
- High packet loss

**Solutions:**
1. Add decoupling capacitor
2. Use shorter wires
3. Change to less crowded channel
4. Reduce transmission power initially
5. Check for loose connections

### Low Range

**Symptoms:**
- Only works at very short distance
- Signal drops quickly

**Solutions:**
1. Increase power level to MAX
2. Change data rate to 250Kbps (longer range)
3. Add external antenna
4. Check antenna is properly connected
5. Test in open area (avoid obstacles)

## Command Line Options

Example with configurable parameters:

```bash
# Send 100 packets on channel 76
./nrf_sender --count=100 --channel=76

# Continuous transmission at max power
./nrf_sender --continuous --power=max

# Range test mode
./nrf_sender --range-test
```

## Output Examples

### Successful Transmission
```
Radio initialized successfully
Channel: 76
Power Level: MAX
Data Rate: 1Mbps
Pipe 0: 0x4361724143 (Listening)
Pipe 1: 0x426F784143 (Writing)

Sending packet 0... OK
Sending packet 1... OK
Sending packet 2... OK
Success rate: 100%
```

### Failed Transmission
```
Radio initialized successfully
Sending packet 0... FAILED
Sending packet 1... FAILED
Sending packet 2... FAILED

Troubleshooting:
- Check receiver is powered on
- Verify pipe addresses match
- Ensure receiver is listening
- Check channel configuration
```

## Integration with Car/Box

Use NRF Sender to verify communication before testing full system:

```bash
# Terminal 1: Run NRF Sender (as transmitter)
./nrf_sender --tx --pipe=Car01

# Terminal 2: Run Car receiver
./gaspetto_car

# Or use as receiver to test box transmission
./nrf_sender --rx --pipe=Box01
```

## Development Uses

### Protocol Development
Test new packet formats before implementing in main firmware.

### Performance Testing
Measure throughput and latency under various conditions.

### Hardware Validation
Verify radio modules before assembly into final product.

### Field Testing
Quick tool for testing communication range in deployment environment.

## Building for Production

Disable debug output for faster execution:

```cpp
#define DEBUG_OUTPUT 0

#if DEBUG_OUTPUT
    Serial.println("Debug info");
#endif
```

## Safety Notes

- **RF Regulations**: Ensure compliance with local regulations (2.4GHz ISM band)
- **Power Limits**: Don't exceed maximum transmission power for your region
- **Antenna**: Always use appropriate antenna (don't transmit without antenna)
- **Duty Cycle**: Avoid continuous transmission for extended periods

## Advanced Features

### Packet Sniffer
Monitor all traffic on a channel:

```cpp
void snifferMode() {
    radio.setAutoAck(false);
    radio.setChannel(MONITOR_CHANNEL);
    radio.startListening();

    while (true) {
        if (radio.available()) {
            uint8_t packet[32];
            radio.read(packet, 32);

            Serial.print("Packet captured: ");
            printHex(packet, 32);
        }
    }
}
```

### Signal Strength Measurement
Approximate RSSI measurement:

```cpp
int getRSSI() {
    // Read RPD (Received Power Detector)
    // Returns true if signal > -64dBm
    return radio.testRPD() ? -64 : -70;
}
```

## Related Tools

- **Radio Analyzer**: Real-time packet capture and analysis
- **Configuration Tool**: GUI for radio parameter adjustment
- **Spectrum Analyzer**: 2.4GHz spectrum visualization

## References

- [nRF24L01+ Product Specification](https://www.nordicsemi.com/products/nrf24l01)
- [RF24 Library Documentation](https://nrf24.github.io/RF24/)
- [Debugging nRF24L01+](https://forum.arduino.cc/t/nrf24l01-hardware-debugging/)
