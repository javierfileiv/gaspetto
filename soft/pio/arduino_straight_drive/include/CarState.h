// Shared vehicle state & enums exposed to command processor
#pragma once

#include <Arduino.h>
#include <IMUOrientation.h>

enum class Mode { Idle, Forward, Backward, Turn };

// Global state (defined in car_main.cpp)
extern IMUOrientation imu;
extern Mode mode;
extern float targetYaw;
extern bool turningToAngle;
extern float turnStopThreshold;
extern unsigned long movementExpireMs;
extern int turnDir;
extern unsigned long lastMoveStartMs;
extern bool baselineSet;
extern float baselineYaw;
extern float Kp, Ki, Kd, integral, prevErr;
extern float baseSpeed;
extern float turnSpeed;
extern bool manualOverride;
extern unsigned long manualExpireMs;
extern float lastCmdLeft, lastCmdRight;
extern bool autoResetYawOnMove;
extern float autoResetThresholdDeg;

// Functions implemented in car_main.cpp
void drive(float left, float right);
void resetYawAll(const __FlashStringHelper *reason);
void printHelp();
