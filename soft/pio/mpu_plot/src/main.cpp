#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdint.h>

// ====================================================================
//                 LOCAL LIGHTWEIGHT MATH IMPLEMENTATIONS
//   (Avoid pulling full <math.h> / libm for size & speed considerations)
//   Functions: fast_fabsf, fast_sqrtf, fast_atan2f (float versions)
//   Accuracy: Sufficient for orientation estimation (few mdeg typical)
// ====================================================================
static inline float fast_fabsf(float x) { return x < 0.0f ? -x : x; }

// Fast inverse square root (Quake style) + one Newton iteration
static inline float fast_invsqrtf(float x) {
  union { float f; uint32_t i; } conv { x };
  conv.i = 0x5f3759df - (conv.i >> 1);
  float y = conv.f;
  // One Newton-Raphson iteration
  y = y * (1.5f - 0.5f * x * y * y);
  return y;
}

static inline float fast_sqrtf(float x) {
  if (x <= 0.0f) return 0.0f;
  return x * fast_invsqrtf(x);
}

// Fast atan2 approximation (based on minimax & conditional polynomial)
// Max error ~0.005 rad (~0.29 deg) in practice
static inline float fast_atan2f(float y, float x) {
  const float ONEQTR_PI = 0.78539816339f;   // PI/4
  const float THRQTR_PI = 2.35619449019f;   // 3PI/4
  if (x == 0.0f) {
    if (y > 0.0f) return 1.57079632679f;    // PI/2
    if (y < 0.0f) return -1.57079632679f;   // -PI/2
    return 0.0f;                            // undefined, return 0
  }
  float abs_y = fast_fabsf(y) + 1e-10f; // prevent 0/0
  float r;
  float angle;
  if (x >= 0.0f) {
    r = (x - abs_y) / (x + abs_y);
    angle = ONEQTR_PI - ONEQTR_PI * r;
  } else {
    r = (x + abs_y) / (abs_y - x);
    angle = THRQTR_PI - ONEQTR_PI * r;
  }
  return (y < 0.0f) ? -angle : angle;
}

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ====================================================================
//                          OBJECT INITIALIZATION
// ====================================================================
Adafruit_MPU6050 mpu; // MPU6050 connected via I2C1 (default for Wire library)

// ====================================================================
//                          KALMAN FILTER VARIABLES
// ====================================================================
// Process noise variance for the gyroscope angle
float Q_angle = 0.001;
// Process noise variance for the gyro bias
float Q_bias = 0.003;
// Measurement noise variance
float R_measure = 0.03;

// State vector variables (angle and gyro bias) for roll and pitch
float kalman_roll_angle = 0.0;
float kalman_roll_bias = 0.0;
float kalman_pitch_angle = 0.0;
float kalman_pitch_bias = 0.0;
// Error covariance matrix (P) for roll and pitch. Initialized for roll, copied for pitch
float P_00_roll = 0.0, P_01_roll = 0.0, P_10_roll = 0.0, P_11_roll = 0.0;
float P_00_pitch = 0.0, P_01_pitch = 0.0, P_10_pitch = 0.0, P_11_pitch = 0.0;


// ====================================================================
//                          GLOBAL VARIABLES
// ====================================================================
sensors_event_t a, g, temp; // MPU6050 raw sensor events
float integratedYaw = 0.0;  // Integrated yaw angle from gyro Z-axis

// MPU6050 Calibration offsets
float accOffset_X = 0.0;
float accOffset_Y = 0.0;
float accOffset_Z = 0.0;
float gyroOffset_X = 0.0;
float gyroOffset_Y = 0.0;
float gyroOffset_Z = 0.0;

// Variables for dynamic dt calculation
unsigned long previousMicros = 0;
float dt = 0.0; // Delta time in seconds
static unsigned long seqCounter = 0; // Sequence counter for each sample line

// ====================================================================
//                          FUNCTION PROTOTYPES
// ====================================================================
void calibrateMPU6050();
float kalmanFilter(float newAngle, float newRate, float& currentAngle, float& currentBias, float& P00, float& P01, float& P10, float& P11, float dt);

// ====================================================================
//                              SETUP
// ====================================================================
void setup() {
  // Start logging via USART1 (Serial1) at 115200 baud
  Serial1.begin(115200);
  while (!Serial1) {
    // Wait for Serial1 to be ready, if needed (especially for some boards/debuggers)
  }
  Serial1.println("MPU6050 Data Streamer Starting...");

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial1.println("Failed to find MPU6050 chip! Check wiring.");
    while (1) {
      delay(10); // Halt if MPU6050 not found
    }
  }
  Serial1.println("MPU6050 found and initialized.");

  // Configure MPU6050 settings (optional, can be default)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // +/- 8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // +/- 500 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Low pass filter at 21 Hz
  Serial1.println("MPU6050 sensor configured.");

  // Perform MPU6050 calibration
  calibrateMPU6050();

  // Get initial MPU6050 reading for Kalman filter initialization
  // after calibration to get a clean baseline.
  mpu.getEvent(&a, &g, &temp);

  // Initialize Kalman filter for roll and pitch with calibrated accelerometer readings
  kalman_roll_angle = fast_atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  kalman_pitch_angle = fast_atan2f(-a.acceleration.x, fast_sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

  // Initialize previousMicros for dt calculation
  previousMicros = micros();

  Serial1.println("Streaming Seq,Micros,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,KalmanRoll,KalmanPitch,IntegratedYaw");
}

// ====================================================================
//                          KALMAN FILTER FUNCTION
// ====================================================================
float kalmanFilter(float newAngle, float newRate, float& currentAngle, float& currentBias, float& P00, float& P01, float& P10, float& P11, float dt) {
  // Prediction step
  currentAngle += dt * (newRate - currentBias);
  P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
  P01 -= dt * P11;
  P10 -= dt * P11;
  P11 += dt * Q_bias;

  // Update step
  float y = newAngle - currentAngle;
  float S = P00 + R_measure;
  float K_0 = P00 / S;
  float K_1 = P10 / S;

  currentAngle += K_0 * y;
  currentBias += K_1 * y;

  P00 -= K_0 * P00;
  P01 -= K_0 * P01;
  P10 -= K_1 * P00;
  P11 -= K_1 * P01;

  // The filtered angle is now in currentAngle (passed by reference)
  return currentAngle;
}

// ====================================================================
//                          MPU6050 CALIBRATION
// ====================================================================
void calibrateMPU6050() {
  Serial1.println("Calibrating MPU6050... Keep the sensor absolutely still!");
  delay(2000); // Give user time to place sensor still

  // Variables for accumulating sensor readings
  float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  // Read 1000 samples for calibration
  const int numSamples = 1000;
  for (int i = 0; i < numSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumAccX += a.acceleration.x;
    sumAccY += a.acceleration.y;
    sumAccZ += a.acceleration.z;
    sumGyroX += g.gyro.x;
    sumGyroY += g.gyro.y;
    sumGyroZ += g.gyro.z;
    delay(2); // Small delay between readings
  }

  // Calculate average offsets
  accOffset_X = sumAccX / numSamples;
  accOffset_Y = sumAccY / numSamples;
  // For AccZ, assume it should ideally read 1g (gravity) when still and upright.
  // So, the offset is the average reading minus the ideal 1g.
  // MPU6050 returns values in m/s^2. SENSORS_GRAVITY_STANDARD is ~9.80665 m/s^2.
  accOffset_Z = (sumAccZ / numSamples) - SENSORS_GRAVITY_STANDARD;

  gyroOffset_X = sumGyroX / numSamples;
  gyroOffset_Y = sumGyroY / numSamples;
  gyroOffset_Z = sumGyroZ / numSamples;

  Serial1.println("MPU6050 Calibration complete!");
  Serial1.print("Acc Offsets (m/s^2): X="); Serial1.print(accOffset_X);
  Serial1.print(", Y="); Serial1.print(accOffset_Y);
  Serial1.print(", Z="); Serial1.println(accOffset_Z);
  Serial1.print("Gyro Offsets (rad/s): X="); Serial1.print(gyroOffset_X);
  Serial1.print(", Y="); Serial1.print(gyroOffset_Y);
  Serial1.print(", Z="); Serial1.println(gyroOffset_Z);

  delay(1000); // Pause to display calibration results
}

// ====================================================================
//                             LOOP
// ====================================================================
void loop() {
  // Dynamically calculate dt
  unsigned long currentMicros = micros();
  dt = (float)(currentMicros - previousMicros) / 1000000.0; // dt in seconds
  previousMicros = currentMicros;

  // Get new raw sensor event data
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration offsets to raw readings
  a.acceleration.x -= accOffset_X;
  a.acceleration.y -= accOffset_Y;
  a.acceleration.z -= accOffset_Z;
  g.gyro.x -= gyroOffset_X;
  g.gyro.y -= gyroOffset_Y;
  g.gyro.z -= gyroOffset_Z;

  // Calculate roll and pitch from accelerometer (calibrated) for Kalman filter
  float accel_roll = fast_atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
  float accel_pitch = fast_atan2f(-a.acceleration.x, fast_sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0f / PI;

  // Run Kalman filter for roll and pitch. Note: Gyro data is converted to degrees/s.
  kalman_roll_angle = kalmanFilter(accel_roll, g.gyro.x * 180 / PI, kalman_roll_angle, kalman_roll_bias, P_00_roll, P_01_roll, P_10_roll, P_11_roll, dt);
  kalman_pitch_angle = kalmanFilter(accel_pitch, g.gyro.y * 180 / PI, kalman_pitch_angle, kalman_pitch_bias, P_00_pitch, P_01_pitch, P_10_pitch, P_11_pitch, dt);

  // Integrate calibrated gyro Z for yaw (will still drift)
  integratedYaw += g.gyro.z * dt * 180 / PI; // Convert rad/s to deg/s and integrate

  // Normalize yaw to -180 to 180 degrees
  while (integratedYaw > 180) integratedYaw -= 360;
  while (integratedYaw < -180) integratedYaw += 360;

  // Print CSV line: Seq,Micros,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,KalmanRoll,KalmanPitch,IntegratedYaw
  Serial1.print(seqCounter); Serial1.print(',');
  Serial1.print(currentMicros); Serial1.print(',');
  Serial1.print(a.acceleration.x, 6); Serial1.print(',');
  Serial1.print(a.acceleration.y, 6); Serial1.print(',');
  Serial1.print(a.acceleration.z, 6); Serial1.print(',');
  Serial1.print(g.gyro.x, 6); Serial1.print(',');
  Serial1.print(g.gyro.y, 6); Serial1.print(',');
  Serial1.print(g.gyro.z, 6); Serial1.print(',');
  Serial1.print(kalman_roll_angle, 4); Serial1.print(',');
  Serial1.print(kalman_pitch_angle, 4); Serial1.print(',');
  Serial1.println(integratedYaw, 4);
  seqCounter++;
}
