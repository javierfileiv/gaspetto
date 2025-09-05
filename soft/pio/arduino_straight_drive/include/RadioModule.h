// RadioModule.h - abstraction layer for RF24 command reception & telemetry send
#pragma once
#include <Arduino.h>
#include <RadioProtocol.h>

// Callback invoked for each received command string (already trimmed, non-empty)
typedef void (*RadioCmdCallback)(const String &tok);

// Initialize radio; returns true if OK
bool radioInit(RadioCmdCallback cb);
// Poll/service radio receive (call each loop)
void radioService();
// Send telemetry packet (handles stop/start listening). Returns true if (ack or noAck fallback) accepted.
bool radioSendTelemetry(const TelemetryPacket &tp);
// Try re-initialize radio (after failure); returns success
bool radioTryReinit();
// Print low-level register details
void radioPrintDetails();
// Set minimal mode (no acks / fixed payload) on/off
void radioSetMinimal(bool on);
bool radioIsMinimal();
// Send a ping command packet (for diagnostics)
bool radioSendPing(const char *text = "ping");
// Stats / state accessors
bool radioIsOk();
uint32_t radioGetCmdRx();
uint32_t radioGetTlmOk();
uint32_t radioGetTlmFail();
