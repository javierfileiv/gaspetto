// Motor PWM threshold test program - receiver side

// grhh a7dda9c17f03fe99db358a1872fe2944d57a3a26
// drive straight program F35, A30 or D30 working fine
// A70 , turn 90 degrees to left
// D70 , turn 90 degrees to right

#include "IMUOrientation.h"
#include "MotorControl.h"
#include "MovementController.h"

#include <Arduino.h>
#include <RF24.h>
#include <RadioProtocol.h>
#include <SPI.h>
#include <config_radio.h>
#include <pin_definitions.h>

#define SERIAL_BAUD           115200
#define SERIAL_TIMEOUT_MS     3000
#define PWM_FREQ_HZ           17
#define MOVEMENT_TIMEOUT_MS   1500
#define TELEMETRY_INTERVAL_MS 500
#define LED_BLINK_INTERVAL_MS 1000
#define RF_CHANNEL            108
#define RF_RETRIES_DELAY      5
#define RF_RETRIES_COUNT      5
#define MAX_PACKET_SIZE       32
#define TURN_ANGLE_DEG        90.0f
#define YAW_WRAP_AROUND       180.0f
#define YAW_FULL_CIRCLE       360.0f

struct CommandPacket
{
    char text[16];
};

#define MOTOR_TEST_MS       500
#define MOTOR_TEST_DELAY_MS 200

RF24 radio(NRF24_CE, NRF24_CSN);
bool radioOk = false;

MotorControl motorControl(MOTOR_LEFT_BWD, MOTOR_LEFT_FWD, MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD);
IMUOrientation imu;
MovementController movementController(motorControl, imu);

unsigned long lastTelemetryMs           = 0;
const unsigned long telemetryIntervalMs = TELEMETRY_INTERVAL_MS;
static int movementTimeoutMs            = MOVEMENT_TIMEOUT_MS;

void sendTelemetry();

void processCommand(const String &cmd)
{
    if (cmd.length() < 1)
        return;

    char motor = cmd.charAt(0);
    int value  = 0;
    if (cmd.length() > 1)
        value = cmd.substring(1).toInt();

    Serial.print(F("CMD: "));
    Serial.print(motor);
    if (cmd.length() > 1)
    {
        Serial.print(F(" = "));
        Serial.println(value);
    }
    else
    {
        Serial.println();
    }

    switch (motor)
    {
    case 'L':
    case 'l': {
        uint32_t pwm = value > 0 ? map(value, 0, 100, 0, 255) : 255;
        analogWrite(MOTOR_LEFT_BWD, pwm);
        analogWrite(MOTOR_LEFT_FWD, 0);
        delay(MOTOR_TEST_MS);
        analogWrite(MOTOR_LEFT_BWD, 0);
        Serial.print(F("Left "));
        Serial.print(pwm);
        Serial.println(F("/255"));
        break;
    }

    case 'R':
    case 'r': {
        uint32_t pwm = value > 0 ? map(value, 0, 100, 0, 255) : 255;
        analogWrite(MOTOR_RIGHT_FWD, pwm);
        analogWrite(MOTOR_RIGHT_BWD, 0);
        delay(MOTOR_TEST_MS);
        analogWrite(MOTOR_RIGHT_FWD, 0);
        Serial.print(F("Right "));
        Serial.print(pwm);
        Serial.println(F("/255"));
        break;
    }

    case 'B':
    case 'b':
        if (cmd.length() == 1)
        {
            TelemetryPacket telemetry = movementController.buildTelemetryPacket();
            radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
            Serial.println(F("Debug telemetry sent"));
        }
        else
        {
            uint32_t pwm = map(value, 0, 100, 0, 255);
            analogWrite(MOTOR_LEFT_BWD, pwm);
            analogWrite(MOTOR_LEFT_FWD, 0);
            analogWrite(MOTOR_RIGHT_FWD, pwm);
            analogWrite(MOTOR_RIGHT_BWD, 0);
            delay(MOTOR_TEST_MS);
            analogWrite(MOTOR_LEFT_BWD, 0);
            analogWrite(MOTOR_RIGHT_FWD, 0);
            Serial.print(F("Both "));
            Serial.print(pwm);
            Serial.println(F("/255"));
        }
        break;

    case 'W':
    case 'w':
        if (value > 0)
        {
            movementController.startStraightDriving((float)value, movementTimeoutMs);
            Serial.print(F("Forward "));
            Serial.print(value);
            Serial.print(F(" for "));
            Serial.print(movementTimeoutMs);
            Serial.println(F("ms"));
        }
        else
        {
            movementController.stopBothMotors();
            Serial.println(F("Stop"));
        }
        break;

    case 'S':
    case 's':
        if (value > 0)
        {
            movementController.startStraightDriving(-(float)value, movementTimeoutMs);
            Serial.print(F("Backward "));
            Serial.print(value);
            Serial.print(F(" for "));
            Serial.print(movementTimeoutMs);
            Serial.println(F("ms"));
        }
        else
        {
            movementController.stopBothMotors();
            Serial.println(F("Stop"));
        }
        break;

    case 'A':
    case 'a':
        if (value > 0)
        {
            movementController.startTurningInPlace(movementController._imu.yaw() + TURN_ANGLE_DEG,
                                                   (float)value);
            Serial.print(F("Turn left 90° at speed "));
            Serial.println(value);
        }
        break;

    case 'D':
    case 'd':
        if (cmd.length() > 1)
        {
            movementController.startTurningInPlace(movementController._imu.yaw() - TURN_ANGLE_DEG,
                                                   (float)value);
            Serial.print(F("Turn right 90° at speed "));
            Serial.println(value);
        }
        break;
        break;

    case 'M':
    case 'm':
        if (cmd.length() > 1)
        {
            movementTimeoutMs = value;
            Serial.print(F("Timeout set to "));
            Serial.print(value);
            Serial.println(F("ms"));
            break;
        }
        Serial.println(F("Motor test: Left FWD"));
        analogWrite(MOTOR_LEFT_BWD, 255);
        analogWrite(MOTOR_LEFT_FWD, 0);
        delay(MOTOR_TEST_MS);
        analogWrite(MOTOR_LEFT_BWD, 0);
        delay(MOTOR_TEST_DELAY_MS);

        Serial.println(F("Motor test: Left BWD"));
        analogWrite(MOTOR_LEFT_BWD, 0);
        analogWrite(MOTOR_LEFT_FWD, 255);
        delay(MOTOR_TEST_MS);
        analogWrite(MOTOR_LEFT_FWD, 0);
        delay(MOTOR_TEST_DELAY_MS);

        Serial.println(F("Motor test: Right FWD"));
        analogWrite(MOTOR_RIGHT_FWD, 255);
        analogWrite(MOTOR_RIGHT_BWD, 0);
        delay(MOTOR_TEST_MS);
        analogWrite(MOTOR_RIGHT_FWD, 0);
        delay(MOTOR_TEST_DELAY_MS);

        Serial.println(F("Motor test: Right BWD"));
        analogWrite(MOTOR_RIGHT_FWD, 0);
        analogWrite(MOTOR_RIGHT_BWD, 255);
        delay(MOTOR_TEST_MS);
        analogWrite(MOTOR_RIGHT_BWD, 0);

        Serial.println(F("Motor test done"));
        break;

    case 'X':
    case 'x':
        movementController.stopBothMotors();
        Serial.println(F("Emergency stop"));
        break;

    case 'Z':
    case 'z':
        movementController._imu.zeroYaw();
        Serial.println(F("Yaw reset to zero"));
        break;

    case 'K':
    case 'k':
        movementController.setTunings(value / 1000.0, movementController.getKi(),
                                      movementController.getKd());
        Serial.print(F("Kp="));
        Serial.println(value / 1000.0, 3);
        break;

    case 'I':
    case 'i':
        movementController.setTunings(movementController.getKp(), value / 1000.0,
                                      movementController.getKd());
        Serial.print(F("Ki="));
        Serial.println(value / 1000.0, 3);
        break;

    case 'V':
    case 'v':
        movementController.setTunings(movementController.getKp(), movementController.getKi(),
                                      value / 1000.0);
        Serial.print(F("Kd="));
        Serial.println(value / 1000.0, 3);
        break;

    case 'O':
    case 'o':
        movementController.setTrim(static_cast<float>(value));
        Serial.print(F("Trim="));
        Serial.println(value);
        break;

    default:
        Serial.print(F("Unknown: "));
        Serial.println(motor);
        break;
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < SERIAL_TIMEOUT_MS)
    {
    }

    pinMode(PIN_LED, OUTPUT);

    Serial.println(F("=== Motor Test Receiver ==="));
    Serial.println(F("Direct motor (digital test 500ms):"));
    Serial.println(F("  L<v>  Left motor"));
    Serial.println(F("  R<v>  Right motor"));
    Serial.println(F("  B<v>  Both motors"));
    Serial.println(F("  M     Full test all dirs"));
    Serial.println(F("PID movement:"));
    Serial.println(F("  W<v>  Forward (M<v> for timer)"));
    Serial.println(F("  S<v>  Backward"));
    Serial.println(F("  A<v>  Turn left 90 deg"));
    Serial.println(F("  D<v>  Turn right 90 deg"));
    Serial.println(F("  X     Stop all"));
    Serial.println(F("  Z     Zero yaw"));
    Serial.println(F("  M<v>  Set movement timer (ms)"));
    Serial.println(F("PID tuning:"));
    Serial.println(F("  K<v>  Kp (K50 = 0.05)"));
    Serial.println(F("  I<v>  Ki (I100 = 0.10)"));
    Serial.println(F("  V<v>  Kd (V50 = 0.05)"));
    Serial.println(F("  Space Debug telemetry"));
    Serial.println(F("  O<v>  Trim offset (±100)"));

    movementController.init(PWM_FREQ_HZ);

    if (!movementController.isImuOk())
    {
        Serial.println(F("IMU FAILED - halting"));
        while (1)
            ;
    }
    Serial.println(F("IMU OK"));

    movementController.setTelemetryCallback([](const TelemetryPacket &) { sendTelemetry(); });

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
        radio.openReadingPipe(0, gcar_pipe_name);
        radio.startListening();
        radioOk = true;
        Serial.println(F("Radio OK"));
    }
    else
    {
        Serial.println(F("Radio init FAILED"));
    }

    Serial.println(F("Ready"));
}

void sendTelemetry()
{
    TelemetryPacket telemetry = movementController.buildTelemetryPacket();

    delay(10);
    radio.stopListening();
    radio.openWritingPipe(gcar_telemetry_pipe_name);
    bool result = radio.write(&telemetry, sizeof(telemetry));
    radio.openWritingPipe(gcar_pipe_name);
    radio.startListening();

    if (!result)
        Serial.println(F("Telemetry send failed"));
    else
        Serial.println(F("Telemetry sent"));
}

void loop()
{
    // Update PID + motor control
    movementController.updateMovement();

    if (movementController.isMoving())
    {
        unsigned long now = millis();
        if (now - lastTelemetryMs >= telemetryIntervalMs)
        {
            lastTelemetryMs = now;
        }
    }

    // Handle serial commands
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == ' ')
        {
            sendTelemetry();
        }
        else
        {
            String cmd;
            cmd += c;
            cmd += Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd.length() > 0)
                processCommand(cmd);
        }
    }

    // Handle radio commands
    if (radioOk)
    {
        while (radio.available())
        {
            uint8_t len = 0;
            if (radio.isPVariant())
                len = radio.getDynamicPayloadSize();
            if (len == 0 || len > MAX_PACKET_SIZE)
            {
                uint8_t dump[MAX_PACKET_SIZE];
                radio.read(&dump, MAX_PACKET_SIZE);
                continue;
            }

            CommandPacket cp{};
            radio.read(&cp, len);
            if (len < sizeof(cp.text))
                cp.text[len] = '\0';
            else
                cp.text[sizeof(cp.text) - 1] = '\0';

            String cmd = String(cp.text);
            cmd.trim();
            if (cmd.length() > 0)
            {
                Serial.print(F("RADIO: "));
                processCommand(cmd);

                if (cmd != "D" && cmd != "d")
                {
                    AckPayload ap{};
                    String ack = "OK:" + cmd;
                    ack.toCharArray(ap.text, sizeof(ap.text));
                    radio.writeAckPayload(0, &ap, sizeof(ap));
                }
            }
        }
    }

    // Blink LED
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > LED_BLINK_INTERVAL_MS)
    {
        lastBlink = millis();
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
}
