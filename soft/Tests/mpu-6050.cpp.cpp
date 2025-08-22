/*the closed-loop feedback control can be achieved with MPU6050
-----------  A5 - SCL ;  A4 - SDA; interupt 2;
*/

#include "I2Cdev.h"
#include "Wire.h"
#define MPU6050_ADDR 0x68 // Alternatively set AD0 to HIGH  --> Address = 0x69

// define DRIVE module IO Pin
#define STBY 3 // STBY  <  ENR || ENL
#define IN1 8 // Left
#define ENL 6 // PWML

#define IN3 7 // Right
#define ENR 5 // PWMR

int highspeed = 255;
int lowspeed = 10;
int mySpeed = 200; // speed

int corrSpeedL; // Left Motor
int corrSpeedR; // Right Motor
unsigned long t1 = 0;
unsigned long t2 = 0;

//***** MPU6050 **********
// A5 - SCL ;  A4 - SDA; interupt 2;
#define MPU6050_RA_GYRO_ZOUT_H 0x47
// uint8_t buffer[14];
float gyroZ; // Raw register values  gyroscope

unsigned long lastTime = 0;
float dt; // Differential time
long gyroZ0 = 0; // Gyro offset = mean value
float yaw;
float yawOld = 0;
float gyroAngleZ = 0; // Angle variable

// before execute loop() function,
void setup()
{
    Serial.begin(9600);
    pinMode(IN1, OUTPUT); // before useing io pin, pin mode must be set first
    pinMode(ENR, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(ENL, OUTPUT);

    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // wake up!
    Wire.endTransmission(true);

    delay(2000);
    Serial.print("   ***   straight_line_4 ****");

    // Call this function if you need to get the IMU error values for your module
    calibration();
}

void loop()
{
    // ev control getspeed=mySpeed, gerDirection0forward/back
    //

    t1 = millis(); // The millis function returns the number of milliseconds since Arduino board has
                   // been powered up.
    t2 = t1;

    while (abs(t2 - t1) < 2500) {
        // driveStraight();
        forward(mySpeed); // go forward
        t2 = millis();
    }

    Serial.print("   ***   Yaw:");
    Serial.println(yaw);
    Serial.print("Forward: Speed L ");
    Serial.print(corrSpeedL);
    Serial.print("   |   Speed R:");
    Serial.println(corrSpeedR);

    Serial.println();

    yawOld = yaw;

    t1 = millis();
    t2 = t1;

    while (abs(t2 - t1) < 2500) {
        // driveStraight;
        back(mySpeed); // go back -invers
        t2 = millis();
    }
    /*
      Serial.println("Back");
      Serial.print("   |   Yaw:");
      Serial.println(yaw);
      Serial.println();
      yawOld = yaw;
     */
}

//************

int16_t getRotationZ()
{
    uint8_t buffer[14];
    I2Cdev::readBytes(MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
// routine from MPU6050.cpp

void calibration()
{
    // Here we do 100 readings for gyro sensitiv Z-axis output -gyroZ, we sum them and divide them
    // by 100.
    //  gyrZ0 mean value of gyroZ raw register values in 100 ms

    unsigned short times = 100; // Sampling times
    for (int i = 0; i < times; i++) {
        gyroZ = getRotationZ(); // gyroZ - Raw register values gyroscope Z axis
        gyroZ0 += gyroZ; // sum all measured values gyroZ
    }
    gyroZ0 /= times;
}

void calcYaw()
{
    unsigned long currentTime = millis(); // current time(ms)
    // get the current timestamp in Milliseconds since epoch time which is
    dt = (currentTime - lastTime) / 1000.0; // Differential time(s)
    lastTime = currentTime; // Last sampling time(ms)

    gyroZ = getRotationZ();

    float angularZ = (gyroZ - gyroZ0) / 131.0 * dt; // angular z: =t
    if (fabs(angularZ) < 0.05) //
    {
        angularZ = 0.00;
    }
    gyroAngleZ += angularZ; // returns the absolute value of the z-axis rotazion integral
    yaw = -gyroAngleZ;

    /*
    Serial.print(" | GyZo = "); Serial.print(toStr(gyroZ0));
    Serial.println();
    Serial.print(" | GyroAngle = "); Serial.print (gyroAngleZ);
    Serial.println();
    */
}

//++++++++++++++++++++++++

void driveStraight(int speed)
{
    static unsigned long onTime;
    // to compute corrSpeed(A/B) The yaw data is acquired at the first startup, and the data is
    // updated every ten millisecundes.
    if (millis() - onTime > 10) {
        corrSpeed(speed);
        onTime = millis();
    }
}

void corrSpeed(int myspeed)
{
    calcYaw();
    int kp = 15; ////Add proportional constant - p( ???)
    // if drive direktion changes: corrSpeedA = mySpeedA - (yawOld - yaw) * kp;
    corrSpeedR = mySpeed + (yaw - yawOld) * kp; // maintain speed by speeding up right motor
    if (corrSpeedR > highspeed) {
        corrSpeedR = highspeed;
    } else if (corrSpeedR < lowspeed) {
        corrSpeedR = lowspeed;
    }
    // if drive direktion changes:corrSpeedB = mySpeedB + (yawOld - yaw) * kp;
    corrSpeedL = mySpeed - (yaw - yawOld) * kp; // if error is positive, slow down left motor
    if (corrSpeedL > highspeed) {
        corrSpeedL = highspeed;
    } else if (corrSpeedL < lowspeed) {
        corrSpeedL = lowspeed;
    }
}

//*****
void forward(int is_speed)
{
    driveStraight(is_speed);

    // digitalWrite(ENA,HIGH);
    digitalWrite(STBY, HIGH);
    // A ... LEFT
    digitalWrite(IN1, HIGH); // set IN1 hight level enable A channel CW (automatic IN2 LOW)
    analogWrite(ENL, corrSpeedL); // set EN for A RIGHT side speed
    //  B ... RIGHT
    digitalWrite(IN3, HIGH); // set IN3 hight level enable B channel CW (automatisch IN4 LOW)
    analogWrite(ENR, corrSpeedR); // set EN for B LEFT side speed
    // Serial.println("Forward");//send message to serial monitor
}

void back(int is_speed)
{
    corrSpeed(is_speed);

    // digitalWrite(ENA,HIGH);
    digitalWrite(STBY, HIGH);
    // A ... LEFT
    digitalWrite(IN1, LOW); // set IN1 LOW level enable A channel CCW (automatic IN2 HIGH)
    analogWrite(ENL, corrSpeedR); // set EN for A RIGHT side speed
    //  B ... RIGHT
    digitalWrite(IN3, LOW); // set IN3 hight level enable B channel CCW (automatic IN4 HIGH)
    analogWrite(ENR, corrSpeedL); // set EN for B LEFT side speed
    // Serial.println("Back");//send message to serial monitor
}

void left(int is_speed)
{
    digitalWrite(STBY, HIGH);
    //  A ... LEFT
    digitalWrite(IN1, LOW); // set IN1 hight level enable
    analogWrite(ENL, is_speed); // set EN for A RIGHT side speed
                                //  B ... RIGHT
    digitalWrite(IN3, HIGH); // set IN3 hight level enable
    analogWrite(ENR, is_speed); // set EN for B LEFT side speed
    // Serial.println("Left");
}

void right(int is_speed)
{
    digitalWrite(STBY, HIGH);
    //  A ... LEFT
    digitalWrite(IN1, HIGH); // set IN1 LOW level enable A channel CCW (automatic IN2 HIGH)
    analogWrite(ENL, is_speed); // set EN for A RIGHT side speed
                                //  B ... RIGHT
    digitalWrite(IN3, LOW); // set IN3 hight level enable B channel CW (automatisch IN4 LOW)
    analogWrite(ENR, is_speed); // set EN for B LEFT side speed
    // Serial.println("Right");
}

void halt()
{
    digitalWrite(STBY, LOW);
    analogWrite(ENL, 0); // set A side speed 0

    analogWrite(ENL, 0); // set B side speed 0
    // Serial.println("stop");
}

/*
  LeftForward,
  LeftBackward,
  RightForward,
  RightBackward,
  */
