/*To understand the context and purpose of this code, visit:
 * https://www.instructables.com/How-to-Make-a-Robot-Car-Drive-Straight-and-Turn-Ex/
 * This code makes references to steps on this Instructables website
 * written by square1a on 7th July 2022
 *
 * Acknowledgement:
 * some of the MPU6050-raw-data-extraction code in void loop() and most in calculateError() are
 * written by Dejan from:
 * https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 */
#include <Wire.h>

// L9110S control pins for left motor
// These pins control both direction and speed (via PWM) for the left motor
const int L9110_left_IN1 = PB15; // Connect to A-IA or INA on your L9110S module for the left motor
const int L9110_left_IN2 = PB14; // Connect to A-IB or INB on your L9110S module for the left motor

// L9110S control pins for right motor
// These pins control both direction and speed (via PWM) for the right motor
const int L9110_right_IN1 = PB11; // Connect to B-IA or INC on your L9110S module for the right
                                  // motor
const int L9110_right_IN2 = PB10; // Connect to B-IB or IND on your L9110S module for the right
                                  // motor

// Note: The original 'leftSpeed' (pin 9) and 'rightSpeed' (pin 5) are no longer used as separate
// Enable pins. Speed control (PWM) is now applied directly to the active input pin of the L9110S.

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; // linear acceleration
float GyroX, GyroY, GyroZ; // angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; // max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; // min PWM value at which motor moves
float angle; // due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 248; // rough estimate of PWM at the speed pin of the stronger motor, while
                            // driving straight
// and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; // it the car driving forward OR rotate/stationary
bool prevIsDriving = true; // equals isDriving in the previous iteration of void loop()
bool paused = false; // is the program paused
void driving();
void controlSpeed();
void rotate();
int changeSpeed(int motorSpeed, int increment);
void calculateError();
void readAcceleration();
void readGyro();
void stopCar();
// Removed forward(), left(), right() as they are now integrated directly into driving() and
// rotate()

void setup()
{
    Serial.begin(115200);
    Wire.begin(); // Initialize comunication
    Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B); // Talk to the register 6B
    Wire.write(0x00); // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true); // end the transmission
    // Call this function if you need to get the IMU error values for your module
    calculateError();
    delay(20);

    // Initialize L9110S motor control pins as outputs
    pinMode(L9110_left_IN1, OUTPUT);
    pinMode(L9110_left_IN2, OUTPUT);
    pinMode(L9110_right_IN1, OUTPUT);
    pinMode(L9110_right_IN2, OUTPUT);

    // No need for separate speed pins (like leftSpeed, rightSpeed) with L9110S
    // as PWM is applied directly to the direction input pins.

    currentTime = micros();
}

void loop()
{
    // === Read accelerometer (on the MPU6050) data === //
    readAcceleration();
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) -
                AccErrorX; // AccErrorX is calculated in the calculateError() function
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

    // === Read gyroscope (on the MPU6050) data === //
    previousTime = currentTime;
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
    readGyro();
    // Correct the outputs with the calculated error values
    GyroX -= GyroErrorX; // GyroErrorX is calculated in the calculateError() function
    GyroY -= GyroErrorY;
    GyroZ -= GyroErrorZ;
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by
    // sendonds (s) to get the angle in degrees
    gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;
    // combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined
    // through trial and error by other people
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    angle = roll; // if you mounted MPU6050 in a different orientation to me, angle may not = roll.
                  // It can roll, pitch, yaw or minus version of the three
    // for me, turning right reduces angle. Turning left increases angle.

    // Print the values on the serial monitor
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'w') { // drive forward
            Serial.println("forward");
            isDriving = true;
        } else if (c == 'a') { // turn left
            Serial.println("left");
            targetAngle += 90;
            if (targetAngle > 180) {
                targetAngle -= 360;
            }
            isDriving = false;
        } else if (c == 'd') { // turn right
            Serial.println("right");
            targetAngle -= 90;
            if (targetAngle <= -180) {
                targetAngle += 360;
            }
            isDriving = false;
        } else if (c == 'q') { // stop or brake
            Serial.println("stop");
            isDriving = false;
        } else if (c == 'i') { // print information. When car is stationary, GyroX should approx. =
                               // 0.
            Serial.print("angle: ");
            Serial.println(angle);
            Serial.print("targetAngle: ");
            Serial.println(targetAngle);
            Serial.print("GyroX: ");
            Serial.println(GyroX);
            Serial.print("elapsedTime (in ms): "); // estimates time to run void loop() once
            Serial.println(elapsedTime * pow(10, 3));
            Serial.print("equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
        } else if (c == 'p') { // pause the program
            paused = !paused;
            stopCar();
            isDriving = false;
            Serial.println("key p was pressed, which pauses/unpauses the program");
        }
    }

    static int count;
    static int countStraight;
    if (count < 6) {
        count++;
    } else { // runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this
             // else condition runs every 19.6ms or 50 times/second
        count = 0;
        if (!paused) {
            if (isDriving != prevIsDriving) {
                leftSpeedVal = equilibriumSpeed;
                countStraight = 0;
                Serial.print("mode changed, isDriving: ");
                Serial.println(isDriving);
            }
            if (isDriving) {
                if (abs(targetAngle - angle) < 3) {
                    if (countStraight < 20) {
                        countStraight++;
                    } else {
                        countStraight = 0;
                        equilibriumSpeed = leftSpeedVal; // to find equilibrium speed, 20
                                                         // consecutive readings need to indicate
                                                         // car is going straight
                        Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
                        Serial.println(equilibriumSpeed);
                    }
                } else {
                    countStraight = 0;
                }
                driving();
            } else {
                rotate();
            }
            prevIsDriving = isDriving;
        }
    }
}

void driving()
{ // called by void loop(), which isDriving = true
    int deltaAngle = round(targetAngle - angle); // rounding is neccessary, since you never get
                                                 // exact values in reality

    // Set motor directions and apply speed for forward movement
    // Right motor forward direction
    digitalWrite(L9110_right_IN2, LOW); // Set inactive pin LOW
    analogWrite(L9110_right_IN1, rightSpeedVal); // Apply PWM to active pin

    // Left motor forward direction
    digitalWrite(L9110_left_IN2, LOW); // Set inactive pin LOW
    analogWrite(L9110_left_IN1, leftSpeedVal); // Apply PWM to active pin

    if (deltaAngle != 0) {
        controlSpeed();
        rightSpeedVal = maxSpeed; // Assuming the right motor is set to max speed when correcting
    }
    // The analogWrite calls are already done above.
}

void controlSpeed()
{ // this function is called by driving ()
    int deltaAngle = round(targetAngle - angle);
    int targetGyroX;

    // setting up propoertional control, see Step 3 on the website
    if (deltaAngle > 30) {
        targetGyroX = 60;
    } else if (deltaAngle < -30) {
        targetGyroX = -60;
    } else {
        targetGyroX = 2 * deltaAngle;
    }

    if (round(targetGyroX - GyroX) == 0) {
        ;
    } else if (targetGyroX > GyroX) {
        leftSpeedVal = changeSpeed(leftSpeedVal, -1); // would increase GyroX
    } else {
        leftSpeedVal = changeSpeed(leftSpeedVal, +1);
    }
}

void rotate()
{ // called by void loop(), which isDriving = false
    int deltaAngle = round(targetAngle - angle);
    int targetGyroX;

    if (abs(deltaAngle) <= 1) {
        stopCar();
    } else {
        if (angle > targetAngle) { // turn left (Left motor forward, Right motor reverse)
            // Left motor forward direction
            digitalWrite(L9110_left_IN2, LOW); // Set inactive pin LOW
            analogWrite(L9110_left_IN1, leftSpeedVal); // Apply PWM to active pin

            // Right motor reverse direction
            digitalWrite(L9110_right_IN1, LOW); // Set inactive pin LOW
            analogWrite(L9110_right_IN2, rightSpeedVal); // Apply PWM to active pin

        } else if (angle < targetAngle) { // turn right (Left motor reverse, Right motor forward)
            // Left motor reverse direction
            digitalWrite(L9110_left_IN1, LOW); // Set inactive pin LOW
            analogWrite(L9110_left_IN2, leftSpeedVal); // Apply PWM to active pin

            // Right motor forward direction
            digitalWrite(L9110_right_IN2, LOW); // Set inactive pin LOW
            analogWrite(L9110_right_IN1, rightSpeedVal); // Apply PWM to active pin
        }

        // setting up proportional control, see Step 3 on the website
        if (abs(deltaAngle) > 30) {
            targetGyroX = 60;
        } else {
            targetGyroX = 2 * abs(deltaAngle);
        }

        if (round(targetGyroX - abs(GyroX)) == 0) {
            ;
        } else if (targetGyroX > abs(GyroX)) {
            leftSpeedVal = changeSpeed(leftSpeedVal, +1); // would increase abs(GyroX)
        } else {
            leftSpeedVal = changeSpeed(leftSpeedVal, -1);
        }
        rightSpeedVal = leftSpeedVal; // Assuming speeds are synchronized for rotation
        // The analogWrite calls are now inside the if/else if blocks for direction.
    }
}

int changeSpeed(int motorSpeed, int increment)
{
    motorSpeed += increment;
    if (motorSpeed > maxSpeed) { // to prevent motorSpeed from exceeding 255, which is a problem
                                 // when using analogWrite
        motorSpeed = maxSpeed;
    } else if (motorSpeed < minSpeed) {
        motorSpeed = minSpeed;
    }
    return motorSpeed;
}

void calculateError()
{
    // When this function is called, ensure the car is stationary. See Step 2 for more info

    // Read accelerometer values 200 times
    c = 0;
    while (c < 200) {
        readAcceleration();
        // Sum all readings
        AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
        AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
        c++;
    }
    // Divide the sum by 200 to get the error value, since expected value of reading is zero
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;

    // Read gyro values 200 times
    while (c < 200) {
        readGyro();
        // Sum all readings
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
        c++;
    }
    // Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2
                                    // registers
    // For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050
    // datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}

void readGyro()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void stopCar()
{
    // For L9110S, setting both input pins LOW effectively stops the motor (braking)
    digitalWrite(L9110_right_IN1, LOW);
    digitalWrite(L9110_right_IN2, LOW);
    digitalWrite(L9110_left_IN1, LOW);
    digitalWrite(L9110_left_IN2, LOW);
}
