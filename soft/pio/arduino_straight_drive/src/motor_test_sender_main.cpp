// Motor PWM threshold test program - sender side
// Sends PWM commands via nRF24L01+ to test motor response
// Usage: Type commands like "L50", "R-30", "B100", "S" (stop)

#include <Arduino.h>
#include <RF24.h>
#include <RadioProtocol.h>
#include <SPI.h>

#ifndef RADIO_CE_PIN
#define RADIO_CE_PIN 9 // Arduino Uno pins
#endif
#ifndef RADIO_CSN_PIN
#define RADIO_CSN_PIN 10
#endif

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
bool radioOk = false;

void printHelp()
{
    Serial.println(F("PWM Test Commands:"));
    Serial.println(F("  L<n>  - Set left motor PWM (e.g., L50)"));
    Serial.println(F("  R<n>  - Set right motor PWM (e.g., R-30)"));
    Serial.println(F("  B<n>  - Set both motors PWM (e.g., B100)"));
    Serial.println(F("  F<n>  - Straight driving with IMU+PID (e.g., F75)"));
    Serial.println(F("  Z     - Reset yaw to zero"));
    Serial.println(F("  E     - Send debug telemetry (IMU, PID status)"));
    Serial.println(F("  M<ms> - Set movement timer (e.g., M5000 for 5 seconds, M0 disables)"));
    Serial.println(F("PID Movement Controls (WASDX):"));
    Serial.println(F("  W<n>  - Forward with PID (e.g., W30)"));
    Serial.println(F("  S<n>  - Backward with PID (e.g., S30)"));
    Serial.println(F("  A<n>  - Turn left 90° (e.g., A50)"));
    Serial.println(F("  D<n>  - Turn right 90° (e.g., D50)"));
    Serial.println(F("  X     - Emergency stop all movement"));
    Serial.println(F("PID Individual Tuning:"));
    Serial.println(F("  K<n>  - Set Kp (e.g., K50 = 0.05, K100 = 0.1)"));
    Serial.println(F("  I<n>  - Set Ki (e.g., I500 = 0.5, I100 = 0.1)"));
    Serial.println(F("  V<n>  - Set Kd (e.g., V200 = 0.2, V100 = 0.1)"));
    Serial.println(F("Range: -255 to +255 (negative = reverse)"));
    Serial.println(F("Suggested test sequence:"));
    Serial.println(F("  Try: W30, S30, A50, D50 for PID movement"));
    Serial.println(F("  Use E to monitor IMU and PID status while testing"));
    Serial.println(F("  Tune PID: Start with K50, I100, V50 and adjust based on response"));
    Serial.println(F("  Use X for emergency stop if needed"));
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    printHelp();

    // Radio setup
    if (radio.begin())
    {
        radio.setAddressWidth(5);
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_250KBPS);
        radio.setChannel(108);
        radio.setRetries(5, 5);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();

        // Set up for telemetry reception
        radio.openReadingPipe(1, RADIO_ADDR_TLM);
        radio.startListening();

        radioOk = true;
        Serial.println(F("Radio OK - Listening for telemetry"));
    }
    else
    {
        Serial.println(F("Radio init FAILED"));
    }

    Serial.println(F("Ready to send PWM commands..."));
    Serial.print(F("> "));
}

void displayTelemetry(const TelemetryPacket &t)
{
    Serial.print(F("TELEM IMU:"));
    if (t.imuOk)
    {
        // Validate yaw is within reasonable range
        if (t.yaw >= -180.0 && t.yaw <= 180.0) {
            Serial.print(F("Y="));
            Serial.print(t.yaw, 1);
            Serial.print(F("°"));
        } else {
            Serial.print(F("Y=INVALID"));
        }
    }
    else
    {
        Serial.print(F("FAIL"));
    }
    Serial.print(F(" PID:"));

    // Validate PID values are reasonable
    if (t.err >= -360.0 && t.err <= 360.0 &&
        t.kp >= 0.0 && t.kp <= 100.0 &&
        t.ki >= 0.0 && t.ki <= 100.0 &&
        t.kd >= 0.0 && t.kd <= 100.0) {
        Serial.print(F("err="));
        Serial.print(t.err, 2);
        Serial.print(F(" kp="));
        Serial.print(t.kp, 3);
        Serial.print(F(" ki="));
        Serial.print(t.ki, 3);
        Serial.print(F(" kd="));
        Serial.print(t.kd, 3);
    } else {
        Serial.print(F("CORRUPTED"));
    }

    Serial.print(F(" PWM_freq:"));
    if (t.pwmFreq >= 0.0 && t.pwmFreq <= 1000.0) {
        Serial.print(t.pwmFreq, 1);
        Serial.println(F("Hz"));
    } else {
        Serial.println(F("INVALIDHz"));
    }
}

void loop()
{
    // Check for incoming telemetry data
    if (radioOk && radio.available())
    {
        TelemetryPacket telemetry;
        radio.read(&telemetry, sizeof(telemetry));
        displayTelemetry(telemetry);
    }

    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if (cmd.length() == 0)
        {
            Serial.print(F("> "));
            return;
        }

        Serial.print(F("Sending: "));
        Serial.println(cmd);

        if (radioOk)
        {
            CommandPacket cp{};
            cmd.toCharArray(cp.text, sizeof(cp.text));

            radio.stopListening();
            radio.openWritingPipe(RADIO_ADDR_CMD);

            bool success = radio.write(&cp, sizeof(cp));

            if (success)
            {
                Serial.print(F("TX: OK"));

                // Check for ACK payload
                if (radio.isAckPayloadAvailable())
                {
                    AckPayload ap{};
                    memset(&ap, 0, sizeof(ap)); // Clear buffer first
                    radio.read(&ap, sizeof(ap));
                    Serial.print(F(" - ACK: "));

                    // Ensure null termination and print safely
                    ap.text[sizeof(ap.text)-1] = '\0';
                    for (int i = 0; i < sizeof(ap.text)-1; i++) {
                        if (ap.text[i] >= 32 && ap.text[i] <= 126) {
                            Serial.print(ap.text[i]);
                        } else if (ap.text[i] == 0) {
                            break;
                        } else {
                            Serial.print('?'); // Replace non-printable chars
                        }
                    }
                }
                Serial.println();
            }
            else
            {
                // Try without ACK as fallback
                bool fallback = radio.write(&cp, sizeof(cp), true);
                if (fallback)
                {
                    Serial.println(F("TX: FALLBACK"));
                }
                else
                {
                    Serial.println(F("TX: FAILED"));
                }
            }
            radio.startListening();
        }
        else
        {
            Serial.println(F("Radio not available"));
        }
        Serial.print(F("> "));
    }
}
