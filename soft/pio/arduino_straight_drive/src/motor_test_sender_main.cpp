// Motor PWM threshold test program - sender side
// Sends PWM commands via nRF24L01+ to test motor response
// Usage: Type commands like "L50", "R-30", "B100", "S" (stop)

#include <Arduino.h>
#include <RF24.h>
#include <RadioProtocol.h>
#include <SPI.h>
#include <config_radio.h>

#define SERIAL_BAUD        115200
#define SERIAL_TIMEOUT_MS  3000
#define RF_CHANNEL         108
#define RF_RETRIES_DELAY   5
#define RF_RETRIES_COUNT   5
#define TELEMETRY_THROTTLE 10

struct CommandPacket
{
    char text[16];
};

RF24 radio(NRF24_CE, NRF24_CSN);
bool radioOk                 = false;
static int telemetryCount    = 0;
static bool telemetryEnabled = true;
static bool expectTelemetry  = false;

#define PROGRAM_MAX 10
#define CMD_LEN     16
static char programQueue[PROGRAM_MAX][CMD_LEN];
static int programCount = 0;

void sendCmd(const String &cmd);
void displayTelemetry(const TelemetryPacket &t);
bool isValidCmd(char c);

void printHelp()
{
    Serial.println(F("=== Motor Test Sender ==="));
    Serial.println(F("Direct motor (PWM 500ms):"));
    Serial.println(F("  L<v>  Left motor"));
    Serial.println(F("  R<v>  Right motor"));
    Serial.println(F("  B<v>  Both motors"));
    Serial.println(F("  M     Full test all dirs"));
    Serial.println(F("PID movement (5s timer):"));
    Serial.println(F("  W<v>  Forward"));
    Serial.println(F("  S<v>  Backward"));
    Serial.println(F("  A<v>  Turn left 90 deg"));
    Serial.println(F("  D<v>  Turn right 90 deg"));
    Serial.println(F("  X     Stop all"));
    Serial.println(F("  Z     Zero yaw"));
    Serial.println(F("PID tuning:"));
    Serial.println(F("  K<v>  Kp (K50 = 0.05)"));
    Serial.println(F("  I<v>  Ki (I100 = 0.10)"));
    Serial.println(F("  V<v>  Kd (V50 = 0.05)"));
    Serial.println(F("  Space Debug (receiver)"));
    Serial.println(F("  B     Debug telemetry"));
    Serial.println(F("  O<v>  Trim offset (±100)"));
    Serial.println(F("  M<v>  Set movement timer (ms)"));
    Serial.println(F("Program (uppercase adds to queue, * sends):"));
    Serial.println(F("  W<v>  Forward"));
    Serial.println(F("  S<v>  Backward"));
    Serial.println(F("  A<v>  Turn left 90 deg"));
    Serial.println(F("  D<v>  Turn right 90 deg"));
    Serial.println(F("  K<v>  Kp (K50 = 0.05)"));
    Serial.println(F("  I<v>  Ki (I100 = 0.10)"));
    Serial.println(F("  V<v>  Kd (V50 = 0.05)"));
    Serial.println(F("  *     Send program queue"));
    Serial.println(F("  -     Clear program queue"));
    Serial.println(F("  T     Toggle telemetry display"));
    Serial.println(F("  ?     Print this help"));
}

bool isValidCmd(char c)
{
    switch (c)
    {
    case 'W':
    case 'S':
    case 'A':
    case 'D':
    case 'L':
    case 'R':
    case 'B':
    case 'M':
    case 'X':
    case 'Z':
    case 'K':
    case 'I':
    case 'O':
    case 'V':
        return true;
    default:
        return false;
    }
}

void sendCmd(const String &cmd)
{
    if (!radioOk)
    {
        Serial.println(F("Radio not available"));
        return;
    }

    Serial.print(millis());
    Serial.print(F(" TX: "));
    Serial.println(cmd);

    CommandPacket cp{};
    cmd.toCharArray(cp.text, sizeof(cp.text));

    radio.stopListening();
    radio.openWritingPipe(gcar_pipe_name);

    bool success = radio.write(&cp, sizeof(cp));

    if (success)
    {
        Serial.print(millis());
        Serial.print(F(" OK"));

        if (radio.isAckPayloadAvailable())
        {
            uint8_t ackLen = radio.isPVariant() ? radio.getDynamicPayloadSize() :
                                                  sizeof(AckPayload);
            if (ackLen >= sizeof(TelemetryPacket))
            {
                TelemetryPacket telemetry;
                radio.read(&telemetry, sizeof(telemetry));
                Serial.println();
                displayTelemetry(telemetry);
            }
            else
            {
                AckPayload ap{};
                memset(&ap, 0, sizeof(ap));
                radio.read(&ap, sizeof(ap));
                Serial.print(F(" ACK="));
                ap.text[sizeof(ap.text) - 1] = '\0';
                for (uint8_t i = 0; i < sizeof(ap.text) - 1; i++)
                {
                    if (ap.text[i] >= 32 && ap.text[i] <= 126)
                        Serial.print((char)ap.text[i]);
                    else if (ap.text[i] == 0)
                        break;
                    else
                        Serial.print('?');
                }
            }
        }
        Serial.println();
    }
    else
    {
        bool fallback = radio.write(&cp, sizeof(cp), true);
        Serial.print(millis());
        if (fallback)
            Serial.println(F(" FALLBACK"));
        else
            Serial.println(F(" FAILED"));
    }
    radio.startListening();
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < SERIAL_TIMEOUT_MS)
    {
    }

    printHelp();

    // Radio setup
    if (radio.begin())
    {
        radio.setAddressWidth(5);
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_250KBPS);
        radio.setChannel(RF_CHANNEL);
        radio.setRetries(RF_RETRIES_DELAY, RF_RETRIES_COUNT);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();

        // Set up for telemetry reception
        radio.openReadingPipe(1, gcar_telemetry_pipe_name);
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
    Serial.print(millis());
    switch (t.state)
    {
    case 1:
        Serial.print(F(" FWD"));
        break;
    case 2:
        Serial.print(F(" BWD"));
        break;
    case 3:
        Serial.print(F(" TL"));
        break;
    case 4:
        Serial.print(F(" TR"));
        break;
    default:
        Serial.print(F(" IDL"));
        break;
    }
    Serial.print(t.speed);
    Serial.print(F(" Y="));
    if (t.imuOk && t.yaw >= -180.0 && t.yaw <= 180.0)
        Serial.print(t.yaw, 1);
    else
        Serial.print(F("?"));
    Serial.print(F(" e="));
    Serial.print(t.err, 1);
    Serial.print(F(" Kp="));
    Serial.print(t.kp, 3);
    Serial.print(F(" Ki="));
    Serial.print(t.ki, 3);
    Serial.print(F(" Kd="));
    Serial.print(t.kd, 3);
    Serial.print(F(" O="));
    Serial.print(t.trim);
    Serial.println();
}

void loop()
{
    // Check for incoming telemetry data
    while (radioOk && radio.available())
    {
        TelemetryPacket telemetry;
        radio.read(&telemetry, sizeof(telemetry));
        bool byExpect   = expectTelemetry;
        bool byThrottle = telemetryEnabled && (++telemetryCount % TELEMETRY_THROTTLE == 0);
        if (byExpect || byThrottle)
        {
            displayTelemetry(telemetry);
        }
        if (byExpect)
        {
            expectTelemetry = false;
            telemetryCount  = 0;
        }
    }

    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.length() == 0)
        {
            Serial.print(F("> "));
            return;
        }

        if (cmd == "T")
        {
            telemetryEnabled = !telemetryEnabled;
            Serial.print(F("Telemetry display "));
            Serial.println(telemetryEnabled ? F("ON") : F("OFF"));
            Serial.print(F("> "));
            return;
        }

        if (cmd == "?")
        {
            printHelp();
            Serial.print(F("> "));
            return;
        }

        if (cmd == "*")
        {
            for (int i = 0; i < programCount; i++)
            {
                Serial.print(F("Program: "));
                Serial.println(programQueue[i]);
                sendCmd(String(programQueue[i]));
                delay(50);
            }
            programCount = 0;
            Serial.print(F("> "));
            return;
        }

        if (cmd == "-")
        {
            programCount = 0;
            Serial.println(F("Program cleared"));
            Serial.print(F("> "));
            return;
        }

        char first = cmd.charAt(0);
        if (first >= 'A' && first <= 'Z')
        {
            if (!isValidCmd(first))
            {
                Serial.println(F("Unknown command"));
            }
            else if (programCount >= PROGRAM_MAX)
            {
                Serial.println(F("Program full!"));
            }
            else if (cmd.length() >= CMD_LEN)
            {
                Serial.println(F("Command too long"));
            }
            else
            {
                cmd.toCharArray(programQueue[programCount], CMD_LEN);
                programCount++;
                Serial.print(F("Added to program ("));
                Serial.print(programCount);
                Serial.print(F("/"));
                Serial.print(PROGRAM_MAX);
                Serial.println(F(")"));
            }
        }
        else if (first >= 'a' && first <= 'z')
        {
            if (isValidCmd(first - 32))
            {
                sendCmd(cmd);
            }
            else
            {
                Serial.println(F("Unknown command"));
            }
        }
        else
        {
            Serial.println(F("Unknown command"));
        }

        Serial.print(F("> "));
    }
}
