// RadioModule.cpp - implementation
#include <RF24.h>
#include <SPI.h>
#include <cstring>
#include "RadioModule.h"

#ifndef RADIO_CE_PIN
#define RADIO_CE_PIN PB0
#endif
#ifndef RADIO_CSN_PIN
#define RADIO_CSN_PIN PB1
#endif

static RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
static bool radioOk = false;
static bool minimalMode = false;
static RadioCmdCallback cmdCb = nullptr;
static uint32_t cmdRxCount = 0;
static uint32_t tlmOk = 0;
static uint32_t tlmFail = 0;

bool radioInit(RadioCmdCallback cb) {
    cmdCb = cb;
    if (radio.begin()) {
        radio.setAddressWidth(5);
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_250KBPS);
        radio.setChannel(108);
        radio.setRetries(5,5);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
        // Use pipe 0 for command reception (more reliable ACK payload behavior)
        radio.openReadingPipe(0, RADIO_ADDR_CMD);
        radio.startListening();
        radioOk = true;
    } else {
        radioOk = false;
    }
    return radioOk;
}

void radioService() {
    if (!radioOk) return;
    while (radio.available()) {
        uint8_t pipeNum;
        radio.available(&pipeNum);
        uint8_t len = 0;
        if (radio.isPVariant()) { // still safe; use dynamic size API
            len = radio.getDynamicPayloadSize();
        }
        if (len == 0 || len > sizeof(CommandPacket)) {
            // Read & discard to clear FIFO
            uint8_t dump[32];
            radio.read(&dump, 32);
            continue;
        }
        CommandPacket cp{};
        radio.read(&cp, len);
        if (len < sizeof(cp.text)) cp.text[len] = '\0'; else cp.text[sizeof(cp.text)-1] = '\0';
        String tok = String(cp.text);
        tok.trim();
        if (!tok.length()) continue; // ignore empty/noise
        cmdRxCount++;
        if (!minimalMode) {
            AckPayload ap{};
            const char prefix[] = "ACK:";
            size_t idx=0; while (idx < sizeof(prefix)-1 && idx < sizeof(ap.text)-1) { ap.text[idx]=prefix[idx]; idx++; }
            for (size_t i=0;i<(size_t)tok.length() && idx < sizeof(ap.text)-1;i++,idx++) ap.text[idx]=tok.charAt(i);
            ap.text[idx]='\0';
            radio.writeAckPayload(0,&ap,sizeof(ap));
        }
        if (cmdCb) cmdCb(tok);
    }
}

bool radioSendTelemetry(const TelemetryPacket &tp) {
    if (!radioOk) return false;
    radio.stopListening();
    radio.openWritingPipe(RADIO_ADDR_TLM);
    bool ok=true;
    if (minimalMode) {
        radio.write(&tp, sizeof(tp), true);
    } else {
        ok = radio.write(&tp, sizeof(tp));
        if (!ok) {
            bool ok2 = radio.write(&tp, sizeof(tp), true);
            if (ok2) ok = true;
        }
    }
    radio.startListening();
    if (ok) tlmOk++; else tlmFail++;
    return ok;
}

bool radioTryReinit() {
    radioOk = radioInit(cmdCb);
    return radioOk;
}

void radioPrintDetails() {
    if (!radioOk) return;
    radio.printDetails();
}

void radioSetMinimal(bool on) {
    if (minimalMode == on) return;
    minimalMode = on;
    if (!radioOk) return;
    radio.stopListening();
    if (minimalMode) {
        radio.setAutoAck(false);
        radio.disableDynamicPayloads();
    } else {
        radio.setAutoAck(true);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
        radio.setRetries(5,5);
    }
    radio.openReadingPipe(1, RADIO_ADDR_CMD);
    radio.startListening();
}

bool radioIsMinimal() { return minimalMode; }
bool radioIsOk() { return radioOk; }
uint32_t radioGetCmdRx() { return cmdRxCount; }
uint32_t radioGetTlmOk() { return tlmOk; }
uint32_t radioGetTlmFail() { return tlmFail; }

bool radioSendPing(const char *text) {
    if (!radioOk) return false;
    CommandPacket cp{};
    if (!text) text = "ping";
    for (size_t i=0;i<sizeof(cp.text)-1 && text[i];++i) cp.text[i]=text[i];
    radio.stopListening();
    radio.openWritingPipe(RADIO_ADDR_CMD);
    bool ok = radio.write(&cp, sizeof(cp));
    radio.startListening();
    return ok;
}
