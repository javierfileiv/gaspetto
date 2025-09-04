// Simple NRF24L01 protocol definitions for command & telemetry + optional ACK payloads
#pragma once
#include <Arduino.h>

// 5-byte addresses (nRF24 treats them as char arrays; keep 5 chars + null for readability)
// NOTE: nRF24 uses fixed 5-byte addresses. We provide a 6-byte array only so the
// trailing null makes them readable when printed as C strings. Do NOT use strlen
// on these; always transmit the full 5 bytes.
// Use 5 non-null bytes (no embedded 0) for robust addressing. Last element of the 6-byte array
// is unused/for readability only; RF24 uses first 5 bytes.
static const uint8_t RADIO_ADDR_CMD[6] = {'C','M','D','C','1','\0'}; // Commands TO vehicle
static const uint8_t RADIO_ADDR_TLM[6] = {'T','L','M','T','1','\0'}; // Telemetry FROM vehicle

// Command: ASCII text up to 15 chars + null (matches existing serial commands)
struct __attribute__((packed)) CommandPacket {
    char text[16];
};

// Telemetry: fits into 32 byte payload (nRF24 max)
struct __attribute__((packed)) TelemetryPacket {
    float yaw;          // 4 - Current yaw angle
    float err;          // 4 - PID error
    float pwmFreq;      // 4 - Current PWM frequency
    float kp;           // 4 - Current Kp parameter
    float ki;           // 4 - Current Ki parameter
    float kd;           // 4 - Current Kd parameter
    uint8_t imuOk;      // 1 - IMU status (0/1)
    uint8_t padding[7]; // 7 - Alignment padding to reach 32 bytes
    // Total: 32 bytes exactly
};

static_assert(sizeof(TelemetryPacket) == 32, "TelemetryPacket must be exactly 32 bytes");

// ACK payload returned with every received command (if enabled). Helps debug link.
struct __attribute__((packed)) AckPayload {
    char text[12]; // e.g. "ACK:f" or truncated command
};
static_assert(sizeof(AckPayload) <= 32, "AckPayload too large");
