#pragma once

#include "Adafruit_Sensor.h"
#include "Wire.h"

#include <array>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <vector>

/* Constants for PC emulation. */
#define SENSORS_GRAVITY_STANDARD (9.80665f)

/* MPU6050 Range constants. */
typedef enum {
    MPU6050_RANGE_2_G = 0,
    MPU6050_RANGE_4_G,
    MPU6050_RANGE_8_G,
    MPU6050_RANGE_16_G
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_RANGE_250_DEG = 0,
    MPU6050_RANGE_500_DEG,
    MPU6050_RANGE_1000_DEG,
    MPU6050_RANGE_2000_DEG
} mpu6050_gyro_range_t;

typedef enum {
    MPU6050_BAND_260_HZ = 0,
    MPU6050_BAND_184_HZ,
    MPU6050_BAND_94_HZ,
    MPU6050_BAND_44_HZ,
    MPU6050_BAND_21_HZ,
    MPU6050_BAND_10_HZ,
    MPU6050_BAND_5_HZ
} mpu6050_bandwidth_t;

/* Forward declarations. */
typedef struct {
    char name[12];
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    float max_value;
    float min_value;
    float resolution;
    int32_t min_delay;
} sensor_t;

/* Stub sensor class for PC emulation. */
class Adafruit_Sensor {
public:
    virtual ~Adafruit_Sensor() = default;
    virtual bool getEvent(sensors_event_t *)
    {
        return true;
    }
    virtual void getSensor(sensor_t *)
    {
    }
};

/* Stub implementation of Adafruit_MPU6050 for PC emulation. */
class Adafruit_MPU6050 {
public:
    Adafruit_MPU6050() = default;
    ~Adafruit_MPU6050() = default;

    bool begin(uint8_t addr = 0x68, TwoWire *theWire = &Wire)
    {
        (void)addr;
        (void)theWire;
        load_csv_samples();
        return true;
    }

    void setAccelerometerRange(mpu6050_accel_range_t range)
    {
        /* Stub: do nothing. */
        accel_range = range;
    }

    void setGyroRange(mpu6050_gyro_range_t range)
    {
        /* Stub: do nothing. */
        gyro_range = range;
    }

    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth)
    {
        /* Stub: do nothing. */
        filter_bandwidth = bandwidth;
    }

    bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)
    {
        if (has_csv_samples()) {
            const Sample &sample = csvSamples[sampleIndex];
            if (accel) {
                accel->acceleration.x = sample.ax;
                accel->acceleration.y = sample.ay;
                accel->acceleration.z = sample.az;
            }
            if (gyro) {
                gyro->gyro.x = sample.gx;
                gyro->gyro.y = sample.gy;
                gyro->gyro.z = sample.gz;
            }
            if (temp) {
                temp->data[0] = sample.tempC;
            }

            next_sample_index();
            return true;
        }

        /* Default deterministic stub values when no CSV is configured. */
        if (accel) {
            accel->acceleration.x = 0.1f;
            accel->acceleration.y = 0.1f;
            accel->acceleration.z = 9.8f;
        }
        if (gyro) {
            gyro->gyro.x = 0.01f;
            gyro->gyro.y = 0.01f;
            gyro->gyro.z = 0.01f;
        }
        if (temp) {
            temp->data[0] = 25.0f;
        }
        return true;
    }

private:
    struct Sample {
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
        float tempC;
    };

    static bool split_csv_line(const std::string &line, std::vector<std::string> &tokens)
    {
        tokens.clear();
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        return !tokens.empty();
    }

    static bool parse_float(const std::string &src, float &out)
    {
        std::stringstream ss(src);
        ss >> out;
        return !ss.fail() && ss.eof();
    }

    bool has_csv_samples() const
    {
        return !csvSamples.empty();
    }

    void next_sample_index()
    {
        if (csvSamples.empty()) {
            return;
        }

        if (csv_loop_enabled) {
            sampleIndex = (sampleIndex + 1U) % csvSamples.size();
            return;
        }

        if ((sampleIndex + 1U) < csvSamples.size()) {
            sampleIndex++;
        }
    }

    void load_csv_samples()
    {
        const char *path = std::getenv("IMU_CSV");
        if ((path == nullptr) || (path[0] == '\0')) {
            return;
        }

        const char *loop = std::getenv("IMU_LOOP");
        csv_loop_enabled = (loop != nullptr) && (std::string(loop) != "0");

        std::ifstream file(path);
        if (!file.is_open()) {
            return;
        }

        std::string line;
        std::vector<std::string> tokens;
        std::vector<Sample> loaded;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            if (!split_csv_line(line, tokens)) {
                continue;
            }

            /* Supported rows:
             * ax,ay,az,gx,gy,gz
             * ax,ay,az,gx,gy,gz,tempC
             * t,ax,ay,az,gx,gy,gz[,tempC]
             */
            if ((tokens.size() < 6U) || (tokens.size() > 8U)) {
                continue;
            }

            std::array<float, 8> values{};
            bool allNumbers = true;
            for (size_t i = 0; i < tokens.size(); ++i) {
                if (!parse_float(tokens[i], values[i])) {
                    allNumbers = false;
                    break;
                }
            }
            if (!allNumbers) {
                /* Allow one header line such as: ax,ay,az,gx,gy,gz,tempC. */
                continue;
            }

            Sample sample{};
            if (tokens.size() == 6U) {
                sample.ax = values[0];
                sample.ay = values[1];
                sample.az = values[2];
                sample.gx = values[3];
                sample.gy = values[4];
                sample.gz = values[5];
                sample.tempC = 25.0f;
            } else if (tokens.size() == 7U) {
                sample.ax = values[0];
                sample.ay = values[1];
                sample.az = values[2];
                sample.gx = values[3];
                sample.gy = values[4];
                sample.gz = values[5];
                sample.tempC = values[6];
            } else {
                sample.ax = values[1];
                sample.ay = values[2];
                sample.az = values[3];
                sample.gx = values[4];
                sample.gy = values[5];
                sample.gz = values[6];
                sample.tempC = (tokens.size() == 8U) ? values[7] : 25.0f;
            }
            loaded.push_back(sample);
        }

        if (!loaded.empty()) {
            csvSamples = loaded;
            sampleIndex = 0;
        }
    }

    mpu6050_accel_range_t accel_range = MPU6050_RANGE_2_G;
    mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_250_DEG;
    mpu6050_bandwidth_t filter_bandwidth = MPU6050_BAND_260_HZ;
    std::vector<Sample> csvSamples;
    size_t sampleIndex = 0;
    bool csv_loop_enabled = true;
};
