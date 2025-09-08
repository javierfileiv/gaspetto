#include "IMUOrientation.h"

#include <cmath>

// Conversion
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.2957795f
#endif

bool IMUOrientation::begin(uint8_t addr, TwoWire *theWire)
{
    if (!mpu.begin(addr, theWire)) {
        return false;
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    prevMicros = micros();
    return true;
}

void IMUOrientation::calibrate(bool print)
{
    // Warm up
    for (int i = 0; i < 50; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        delay(5);
    }
    const int N = 600;
    double sumAx = 0, sumAy = 0, sumAz = 0, sumGx = 0, sumGy = 0, sumGz = 0;
    for (int i = 0; i < N; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumAx += a.acceleration.x;
        sumAy += a.acceleration.y;
        sumAz += a.acceleration.z;
        sumGx += g.gyro.x;
        sumGy += g.gyro.y;
        sumGz += g.gyro.z;
        delay(2);
    }
    offsets.accX = sumAx / N;
    offsets.accY = sumAy / N;
    offsets.accZ = (sumAz / N) - SENSORS_GRAVITY_STANDARD; // expect 1g on Z
    offsets.gyroX = sumGx / N;
    offsets.gyroY = sumGy / N;
    offsets.gyroZ = sumGz / N;
    if (print) {
        Serial.println(F("IMU Calibration complete"));
        Serial.print(F("Acc Offsets: "));
        Serial.print(offsets.accX, 4);
        Serial.print(',');
        Serial.print(offsets.accY, 4);
        Serial.print(',');
        Serial.println(offsets.accZ, 4);
        Serial.print(F("Gyro Offsets: "));
        Serial.print(offsets.gyroX, 4);
        Serial.print(',');
        Serial.print(offsets.gyroY, 4);
        Serial.print(',');
        Serial.println(offsets.gyroZ, 4);
    }
    // Initialize orientation from accel
    sensors_event_t a2, g2, t2;
    mpu.getEvent(&a2, &g2, &t2);
    rollDeg = atan2(a2.acceleration.y, a2.acceleration.z) * RAD_TO_DEG;
    pitchDeg = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y +
                                              a2.acceleration.z * a2.acceleration.z)) *
               RAD_TO_DEG;
    yawDeg = 0.0f;
}

void IMUOrientation::update()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = micros();
    float dt = (now - prevMicros) / 1000000.0f;
    prevMicros = now;
    // Apply offsets
    float ax = a.acceleration.x - offsets.accX;
    float ay = a.acceleration.y - offsets.accY;
    float az = a.acceleration.z - offsets.accZ;
    float gx = (g.gyro.x - offsets.gyroX) * RAD_TO_DEG; // deg/s
    float gy = (g.gyro.y - offsets.gyroY) * RAD_TO_DEG;
    float rawGz = (g.gyro.z - offsets.gyroZ) * RAD_TO_DEG;

    // Spike rejection
    bool spike = fabs(rawGz) > IMU_GYROZ_SPIKE_REJECT_DEG_S;
    if (spike) {
        rawGz = lastGyroZDeg; // hold previous
    }
    lastGyroSpike = spike;

    // Bias estimation: update only if not a spike and magnitude moderate
    if (!spike && fabs(rawGz) < IMU_GYROZ_CLAMP_DEG_S * 0.6f) {
        gyroZBias = gyroZBias + IMU_GYROZ_BIAS_ALPHA * (rawGz - gyroZBias);
    }
    float gz = rawGz - gyroZBias;
    // Clamp
    if (gz > IMU_GYROZ_CLAMP_DEG_S)
        gz = IMU_GYROZ_CLAMP_DEG_S;
    if (gz < -IMU_GYROZ_CLAMP_DEG_S)
        gz = -IMU_GYROZ_CLAMP_DEG_S;
    lastGyroZDeg = gz;

    // Integrate yaw directly from gyro (will drift without mag)
    yawDeg += gz * dt;

    // Compute accel angles
    float accelRoll = atan2(ay, az) * RAD_TO_DEG;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Complementary filter for roll/pitch
    rollDeg = alpha * (rollDeg + gx * dt) + (1.0f - alpha) * accelRoll;
    pitchDeg = alpha * (pitchDeg + gy * dt) + (1.0f - alpha) * accelPitch;

    // Normalize yaw within -180..180
    if (yawDeg > 180.0f)
        yawDeg -= 360.0f;
    if (yawDeg < -180.0f)
        yawDeg += 360.0f;
}

void IMUOrientation::idleYawDampen(float dt)
{
    // Exponential decay toward zero (simple high-pass like correction when idle)
    float factor = 1.0f - (IMU_IDLE_YAW_DAMP_RATE * dt);
    if (factor < 0)
        factor = 0;
    yawDeg *= factor;
    if (yawDeg > 180.0f)
        yawDeg -= 360.0f; // keep normalization
    if (yawDeg < -180.0f)
        yawDeg += 360.0f;
}
