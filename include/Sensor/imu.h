#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "mpu6050.h"

// ============================================================================
//  IMU CONFIG
// ============================================================================

#ifndef IMU_TASK_RATE_HZ
#define IMU_TASK_RATE_HZ 400.0f
#endif

#ifndef IMU_TASK_STACK_SIZE
#define IMU_TASK_STACK_SIZE 4096
#endif

#ifndef IMU_TASK_PRIORITY
#define IMU_TASK_PRIORITY 2
#endif

#ifndef IMU_MUTEX_TIMEOUT_MS
#define IMU_MUTEX_TIMEOUT_MS 5
#endif

#ifndef IMU_DEFAULT_SDA_PIN
#define IMU_DEFAULT_SDA_PIN 21
#endif

#ifndef IMU_DEFAULT_SCL_PIN
#define IMU_DEFAULT_SCL_PIN 22
#endif

// ============================================================================
//  DATA IMU UNTUK MAIN / TASK LAIN
// ============================================================================

struct ImuData {
    uint8_t address = MPU6050_ADDR_LOW;
    uint32_t sequence = 0;
    uint32_t timestamp_us = 0;
    float dt_s = 0.0f;

    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 0.0f;

    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;

    float gx_rad_s = 0.0f;
    float gy_rad_s = 0.0f;
    float gz_rad_s = 0.0f;

    float temperature_c = 0.0f;

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    Mpu6050Status status = Mpu6050Status::NOT_INITIALIZED;
};

// ============================================================================
//  IMU WRAPPER
// ============================================================================

class Imu {
public:
    explicit Imu(uint8_t address = MPU6050_ADDR_LOW)
        : _sensor(address)
    {
        _data.address = address;
        _data.status = Mpu6050Status::NOT_INITIALIZED;
    }

    Mpu6050Status begin(
        TwoWire& wire = Wire,
        int sdaPin = IMU_DEFAULT_SDA_PIN,
        int sclPin = IMU_DEFAULT_SCL_PIN,
        uint32_t i2cClock = MPU6050_FAST_I2C_CLOCK,
        bool autoCalibrateGyro = true,
        uint16_t gyroCalibSamples = 1000,
        uint16_t gyroCalibDelayMs = 2
    ) {
        ensureMutex();

        _sensor.setTargetRateHz(IMU_TASK_RATE_HZ);

        Mpu6050Status status = _sensor.begin(
            wire,
            sdaPin,
            sclPin,
            i2cClock
        );

        if (status != Mpu6050Status::OK) {
            writeStatus(status);
            return status;
        }

        if (autoCalibrateGyro) {
            status = _sensor.calibrateGyro(
                gyroCalibSamples,
                gyroCalibDelayMs
            );

            if (status != Mpu6050Status::OK) {
                writeStatus(status);
                return status;
            }
        }

        status = update();

        if (status != Mpu6050Status::OK) {
            writeStatus(status);
            return status;
        }

        return Mpu6050Status::OK;
    }

    Mpu6050Status calibrateGyro(
        uint16_t samples = 1000,
        uint16_t delayMs = 2
    ) {
        ensureMutex();

        Mpu6050Status status = _sensor.calibrateGyro(samples, delayMs);

        if (status != Mpu6050Status::OK) {
            writeStatus(status);
        }

        return status;
    }

    Mpu6050Status update() {
        ensureMutex();

        Mpu6050Status status = _sensor.process();

        if (status != Mpu6050Status::OK) {
            writeStatus(status);
            return status;
        }

        ImuDataMpu6050 latest = _sensor.getDataCopy();
        ImuData converted = convertFromMpu6050(latest);

        writeData(converted);

        return Mpu6050Status::OK;
    }

    ImuData getData() {
        ensureMutex();

        ImuData copy = _data;

        if (_mutex != nullptr) {
            if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(IMU_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                copy = _data;
                xSemaphoreGive(_mutex);
            }
        }

        return copy;
    }

    bool readData(ImuData& out) {
        ensureMutex();

        if (_mutex != nullptr) {
            if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(IMU_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                out = _data;
                xSemaphoreGive(_mutex);
                return true;
            }

            return false;
        }

        out = _data;
        return true;
    }

    Mpu6050Status status() {
        ImuData copy = getData();
        return copy.status;
    }

    bool isInitialized() const {
        return _sensor.isInitialized();
    }

    MPU6050& sensor() {
        return _sensor;
    }

    const MPU6050& sensor() const {
        return _sensor;
    }

    SemaphoreHandle_t mutex() const {
        return _mutex;
    }

private:
    MPU6050 _sensor;
    ImuData _data;
    SemaphoreHandle_t _mutex = nullptr;

    void ensureMutex() {
        if (_mutex == nullptr) {
            _mutex = xSemaphoreCreateMutex();
        }
    }

    static ImuData convertFromMpu6050(const ImuDataMpu6050& src) {
        ImuData dst;

        dst.address = src.address;
        dst.sequence = src.sequence;
        dst.timestamp_us = src.timestamp_us;
        dst.dt_s = src.dt_s;

        dst.ax_g = src.ax_g;
        dst.ay_g = src.ay_g;
        dst.az_g = src.az_g;

        dst.gx_dps = src.gx_dps;
        dst.gy_dps = src.gy_dps;
        dst.gz_dps = src.gz_dps;

        dst.gx_rad_s = src.gx_rad_s;
        dst.gy_rad_s = src.gy_rad_s;
        dst.gz_rad_s = src.gz_rad_s;

        dst.temperature_c = src.temperature_c;

        dst.roll_rad = src.roll_rad;
        dst.pitch_rad = -src.pitch_rad;
        dst.yaw_rad = src.yaw_rad;

        dst.roll_deg = src.roll_deg;
        dst.pitch_deg = src.pitch_deg;
        dst.yaw_deg = src.yaw_deg;

        dst.status = src.status;

        return dst;
    }

    void writeData(const ImuData& newData) {
        ensureMutex();

        if (_mutex != nullptr) {
            if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(IMU_MUTEX_TIMEOUT_MS)) == pdTRUE) {
                _data = newData;
                xSemaphoreGive(_mutex);
            }
        } else {
            _data = newData;
        }
    }

    void writeStatus(Mpu6050Status status) {
        ensureMutex();

        ImuData temp = _data;

        temp.status = status;
        temp.timestamp_us = micros();
        temp.sequence++;

        writeData(temp);
    }
};

// ============================================================================
//  GLOBAL OBJECT
// ============================================================================

static Imu imu;

// ============================================================================
//  TASK IMU
// ============================================================================

static void TaskImu(void* parameter) {
    Imu* imuPtr = static_cast<Imu*>(parameter);

    if (imuPtr == nullptr) {
        imuPtr = &imu;
    }

    const uint32_t periodUs = static_cast<uint32_t>(1000000.0f / IMU_TASK_RATE_HZ);
    uint32_t nextTimeUs = micros();

    while (true) {
        imuPtr->update();

        nextTimeUs += periodUs;

        int32_t remainingUs = static_cast<int32_t>(nextTimeUs - micros());

        if (remainingUs > 1000) {
            vTaskDelay(pdMS_TO_TICKS(remainingUs / 1000));
        } else if (remainingUs > 0) {
            delayMicroseconds(static_cast<uint32_t>(remainingUs));
        } else {
            taskYIELD();
            nextTimeUs = micros();
        }
    }
}