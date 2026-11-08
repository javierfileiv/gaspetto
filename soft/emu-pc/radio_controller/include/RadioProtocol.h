/* Simple NRF24L01 protocol definitions for command & telemetry + optional ACK payloads. */
#pragma once
#include "PacketTag.h"

#include <stdint.h>

/*
 * 5-byte addresses (nRF24 treats them as char arrays; keep 5 chars + null for readability).
 * NOTE: nRF24 uses fixed 5-byte addresses. We provide a 6-byte array only so the
 * trailing null makes them readable when printed as C strings. Do NOT use strlen
 * on these; always transmit the full 5 bytes.
 * Use 5 non-null bytes (no embedded 0) for robust addressing. Last element of the 6-byte array
 * is unused/for readability only; RF24 uses first 5 bytes.
 */
/* Telemetry: fits into 32 byte payload (nRF24 max). It carries the
 * PacketTag::Telemetry byte so the box can tell it from a program or a log
 * fragment with a single comparison; err is gone (the receiver recomputes
 * targetYaw - yaw) because the tag needed its byte. */
struct __attribute__((packed)) TelemetryPacket {
    uint8_t tag; /* 1 - PacketTag::Telemetry, stamped by RadioController::sendTelemetry */
    float targetYaw; /* 4 - Target yaw angle. */
    float yaw; /* 4 - Current yaw angle. */
    float pwmFreq; /* 4 - Current PWM frequency. */
    float kp; /* 4 - Current Kp parameter. */
    float ki; /* 4 - Current Ki parameter. */
    float kd; /* 4 - Current Kd parameter. */
    uint8_t imuOk; /* 1 - IMU status (0/1). */
    uint8_t state; /* 1 - 0=IDLE, 1=STRAIGHT, 2=TURNING */
    uint8_t speed; /* 1 - baseSpeed (0-100). */
    int8_t trim; /* 1 - trimOffset (-100 to 100). */
    uint8_t reserved[3]; /* 3 - pad the packet to the fixed 32 byte payload */
    /* ======= 32 bytes*/
};

static_assert(sizeof(TelemetryPacket) == 32, "TelemetryPacket must be exactly 32 bytes");

/* ACK payload returned with every received command (if enabled). Helps debug link. */
struct __attribute__((packed)) AckPayload {
    char text[12]; /* E.g. "ACK:f" or truncated command. */
};
static_assert(sizeof(AckPayload) <= 32, "AckPayload too large");
