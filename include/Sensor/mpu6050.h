#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

// ============================================================================
//  ALAMAT I2C
// ============================================================================

static constexpr uint8_t MPU6050_ADDR_LOW  = 0x68;
static constexpr uint8_t MPU6050_ADDR_HIGH = 0x69;

#ifndef MPU6050_FAST_I2C_CLOCK
#define MPU6050_FAST_I2C_CLOCK 400000UL
#endif

// ============================================================================
//  STATUS HARDWARE
// ============================================================================

enum class Mpu6050Status : uint8_t {
    OK = 0,
    I2C_ERROR,
    WHO_AM_I_ERROR,
    NOT_INITIALIZED
};

// ============================================================================
//  KONFIGURASI SENSOR
// ============================================================================

enum class Mpu6050GyroRange : uint8_t {
    DPS_250  = 0,
    DPS_500  = 1,
    DPS_1000 = 2,
    DPS_2000 = 3
};

enum class Mpu6050AccelRange : uint8_t {
    G_2  = 0,
    G_4  = 1,
    G_8  = 2,
    G_16 = 3
};

enum class Mpu6050Dlpf : uint8_t {
    BW_260HZ = 0,
    BW_184HZ = 1,
    BW_94HZ  = 2,
    BW_44HZ  = 3,
    BW_21HZ  = 4,
    BW_10HZ  = 5,
    BW_5HZ   = 6
};

// ============================================================================
//  DATA MENTAH DAN DATA TERSKALAKAN
// ============================================================================

struct Mpu6050RawData {
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t temperature = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    uint32_t timestamp_us = 0;
};

struct Mpu6050ScaledData {
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 0.0f;

    float temperature_c = 0.0f;

    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;

    uint32_t timestamp_us = 0;
};

// ============================================================================
//  OUTPUT INTERNAL MPU6050
//  Nama dibuat ImuDataMpu6050 agar tidak bentrok dengan ImuData di imu.h
// ============================================================================

struct ImuDataMpu6050 {
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
//  MPU6050Fast
//  Driver I2C level rendah
// ============================================================================

class MPU6050Fast {
public:
    explicit MPU6050Fast(uint8_t address = MPU6050_ADDR_LOW)
        : _address(address) {}

    Mpu6050Status begin(
        TwoWire& wire = Wire,
        int sdaPin = -1,
        int sclPin = -1,
        uint32_t i2cClock = MPU6050_FAST_I2C_CLOCK
    ) {
        _wire = &wire;

        if (sdaPin >= 0 && sclPin >= 0) {
            _wire->begin(sdaPin, sclPin);
        } else {
            _wire->begin();
        }

        _wire->setClock(i2cClock);
        delay(50);

        uint8_t who = 0;
        if (!readRegister(REG_WHO_AM_I, who)) {
            return Mpu6050Status::I2C_ERROR;
        }

        if (who != 0x68) {
            return Mpu6050Status::WHO_AM_I_ERROR;
        }

        if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return Mpu6050Status::I2C_ERROR;
        delay(100);

        if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return Mpu6050Status::I2C_ERROR;
        if (!writeRegister(REG_PWR_MGMT_2, 0x00)) return Mpu6050Status::I2C_ERROR;
        delay(10);

        if (!setDlpf(Mpu6050Dlpf::BW_94HZ))         return Mpu6050Status::I2C_ERROR;
        if (!setSampleRateDiv(1))                   return Mpu6050Status::I2C_ERROR;
        if (!setGyroRange(Mpu6050GyroRange::DPS_500)) return Mpu6050Status::I2C_ERROR;
        if (!setAccelRange(Mpu6050AccelRange::G_4)) return Mpu6050Status::I2C_ERROR;

        _initialized = true;
        return Mpu6050Status::OK;
    }

    bool setSampleRateDiv(uint8_t divider) {
        return writeRegister(REG_SMPLRT_DIV, divider);
    }

    bool setDlpf(Mpu6050Dlpf dlpf) {
        _dlpf = dlpf;
        return writeRegister(REG_CONFIG, static_cast<uint8_t>(dlpf));
    }

    bool setGyroRange(Mpu6050GyroRange range) {
        _gyroRange = range;
        return writeRegister(REG_GYRO_CONFIG, static_cast<uint8_t>(range) << 3);
    }

    bool setAccelRange(Mpu6050AccelRange range) {
        _accelRange = range;
        return writeRegister(REG_ACCEL_CONFIG, static_cast<uint8_t>(range) << 3);
    }

    Mpu6050Status readRaw(Mpu6050RawData& out) {
        if (!_initialized) return Mpu6050Status::NOT_INITIALIZED;

        uint8_t b[14];

        if (!readBytes(REG_ACCEL_XOUT_H, b, 14)) {
            return Mpu6050Status::I2C_ERROR;
        }

        out.ax          = combine(b[0],  b[1]);
        out.ay          = combine(b[2],  b[3]);
        out.az          = combine(b[4],  b[5]);
        out.temperature = combine(b[6],  b[7]);
        out.gx          = combine(b[8],  b[9]);
        out.gy          = combine(b[10], b[11]);
        out.gz          = combine(b[12], b[13]);
        out.timestamp_us = micros();

        return Mpu6050Status::OK;
    }

    Mpu6050Status readScaled(Mpu6050ScaledData& out) {
        Mpu6050RawData raw;
        const Mpu6050Status status = readRaw(raw);

        if (status != Mpu6050Status::OK) {
            return status;
        }

        const float accelScale = accelLsbPerG();
        const float gyroScale  = gyroLsbPerDps();

        out.ax_g = static_cast<float>(raw.ax) / accelScale;
        out.ay_g = static_cast<float>(raw.ay) / accelScale;
        out.az_g = static_cast<float>(raw.az) / accelScale;

        out.temperature_c = static_cast<float>(raw.temperature) / 340.0f + 36.53f;

        out.gx_dps = static_cast<float>(raw.gx) / gyroScale - _gyroOffsetX_dps;
        out.gy_dps = static_cast<float>(raw.gy) / gyroScale - _gyroOffsetY_dps;
        out.gz_dps = static_cast<float>(raw.gz) / gyroScale - _gyroOffsetZ_dps;

        out.timestamp_us = raw.timestamp_us;

        return Mpu6050Status::OK;
    }

    Mpu6050Status calibrateGyro(uint16_t samples = 1000, uint16_t delayMs = 2) {
        if (!_initialized) return Mpu6050Status::NOT_INITIALIZED;

        _gyroOffsetX_dps = 0.0f;
        _gyroOffsetY_dps = 0.0f;
        _gyroOffsetZ_dps = 0.0f;

        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;

        const float gyroScale = gyroLsbPerDps();

        for (uint16_t i = 0; i < samples; ++i) {
            Mpu6050RawData raw;
            const Mpu6050Status status = readRaw(raw);

            if (status != Mpu6050Status::OK) {
                return status;
            }

            sx += static_cast<double>(raw.gx) / gyroScale;
            sy += static_cast<double>(raw.gy) / gyroScale;
            sz += static_cast<double>(raw.gz) / gyroScale;

            delay(delayMs);
        }

        const float invN = 1.0f / static_cast<float>(samples);

        _gyroOffsetX_dps = static_cast<float>(sx) * invN;
        _gyroOffsetY_dps = static_cast<float>(sy) * invN;
        _gyroOffsetZ_dps = static_cast<float>(sz) * invN;

        return Mpu6050Status::OK;
    }

    void setGyroOffsetDps(float ox, float oy, float oz) {
        _gyroOffsetX_dps = ox;
        _gyroOffsetY_dps = oy;
        _gyroOffsetZ_dps = oz;
    }

    uint8_t address() const {
        return _address;
    }

    bool isInitialized() const {
        return _initialized;
    }

private:
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_PWR_MGMT_2   = 0x6C;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;

    TwoWire* _wire = nullptr;
    uint8_t _address = MPU6050_ADDR_LOW;
    bool _initialized = false;

    Mpu6050GyroRange  _gyroRange  = Mpu6050GyroRange::DPS_500;
    Mpu6050AccelRange _accelRange = Mpu6050AccelRange::G_4;
    Mpu6050Dlpf       _dlpf       = Mpu6050Dlpf::BW_94HZ;

    float _gyroOffsetX_dps = 0.0f;
    float _gyroOffsetY_dps = 0.0f;
    float _gyroOffsetZ_dps = 0.0f;

    static int16_t combine(uint8_t high, uint8_t low) {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(high) << 8) | low
        );
    }

    float accelLsbPerG() const {
        switch (_accelRange) {
            case Mpu6050AccelRange::G_2:  return 16384.0f;
            case Mpu6050AccelRange::G_4:  return 8192.0f;
            case Mpu6050AccelRange::G_8:  return 4096.0f;
            case Mpu6050AccelRange::G_16: return 2048.0f;
            default: return 8192.0f;
        }
    }

    float gyroLsbPerDps() const {
        switch (_gyroRange) {
            case Mpu6050GyroRange::DPS_250:  return 131.0f;
            case Mpu6050GyroRange::DPS_500:  return 65.5f;
            case Mpu6050GyroRange::DPS_1000: return 32.8f;
            case Mpu6050GyroRange::DPS_2000: return 16.4f;
            default: return 65.5f;
        }
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        if (_wire == nullptr) return false;

        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->write(value);

        return _wire->endTransmission(true) == 0;
    }

    bool readRegister(uint8_t reg, uint8_t& value) {
        return readBytes(reg, &value, 1);
    }

    bool readBytes(uint8_t reg, uint8_t* buffer, uint8_t length) {
        if (_wire == nullptr || buffer == nullptr || length == 0) {
            return false;
        }

        _wire->beginTransmission(_address);
        _wire->write(reg);

        if (_wire->endTransmission(false) != 0) {
            return false;
        }

        const uint8_t received = _wire->requestFrom(
            static_cast<int>(_address),
            static_cast<int>(length),
            static_cast<int>(true)
        );

        if (received != length) {
            return false;
        }

        for (uint8_t i = 0; i < length; ++i) {
            buffer[i] = static_cast<uint8_t>(_wire->read());
        }

        return true;
    }
};

// ============================================================================
//  MPU6050
//  Proses EKF lengkap: baca sensor, hitung sudut, simpan buffer
// ============================================================================
class MPU6050 {
public:
    explicit MPU6050(uint8_t address = MPU6050_ADDR_LOW)
        : _address(address), _driver(address)
    {
        registerSelf();
        resetEkf();

        _buffers[0].address = address;
        _buffers[1].address = address;
    }

    Mpu6050Status begin(
        TwoWire& wire = Wire,
        int sdaPin = -1,
        int sclPin = -1,
        uint32_t i2cClock = MPU6050_FAST_I2C_CLOCK
    ) {
        const Mpu6050Status status = _driver.begin(
            wire,
            sdaPin,
            sclPin,
            i2cClock
        );

        if (status != Mpu6050Status::OK) {
            writeStatus(status);
            return status;
        }

        _initialized = true;
        _lastUpdateUs = 0;

        resetEkf();
        initializeAngleFromAccel();
        writeStatus(Mpu6050Status::OK);

        return Mpu6050Status::OK;
    }

    Mpu6050Status calibrateGyro(uint16_t samples = 1000, uint16_t delayMs = 2) {
        return _driver.calibrateGyro(samples, delayMs);
    }

    Mpu6050Status process() {
        if (!_initialized) {
            writeStatus(Mpu6050Status::NOT_INITIALIZED);
            return Mpu6050Status::NOT_INITIALIZED;
        }

        Mpu6050ScaledData sample;
        const Mpu6050Status status = _driver.readScaled(sample);

        if (status != Mpu6050Status::OK) {
            writeStatus(status);
            return status;
        }

        float dt = _targetDtS;

        if (_lastUpdateUs != 0) {
            dt = static_cast<float>(sample.timestamp_us - _lastUpdateUs) * 1.0e-6f;
        }

        _lastUpdateUs = sample.timestamp_us;

        if (dt <= 0.0f || dt > 0.1f) {
            dt = _targetDtS;
        }

        const float gx = sample.gx_dps * DEG_TO_RAD_F;
        const float gy = sample.gy_dps * DEG_TO_RAD_F;
        const float gz = sample.gz_dps * DEG_TO_RAD_F;

        ekfPredict(gx, gy, gz, dt);
        ekfCorrectAccel(sample.ax_g, sample.ay_g, sample.az_g);

        const uint8_t nextIndex = _activeIndex ^ 1;

        ImuDataMpu6050 out;

        out.address = _address;
        out.sequence = _sequence + 1;
        out.timestamp_us = sample.timestamp_us;
        out.dt_s = dt;

        out.ax_g = sample.ax_g;
        out.ay_g = sample.ay_g;
        out.az_g = sample.az_g;

        out.gx_dps = sample.gx_dps;
        out.gy_dps = sample.gy_dps;
        out.gz_dps = sample.gz_dps;

        out.gx_rad_s = gx;
        out.gy_rad_s = gy;
        out.gz_rad_s = gz;

        out.temperature_c = sample.temperature_c;

        out.roll_rad = _state[0];
        out.pitch_rad = _state[1];
        out.yaw_rad = _state[2];

        out.roll_deg = _state[0] * RAD_TO_DEG_F;
        out.pitch_deg = _state[1] * RAD_TO_DEG_F;
        out.yaw_deg = _state[2] * RAD_TO_DEG_F;

        out.status = Mpu6050Status::OK;

        _buffers[nextIndex] = out;
        _sequence = out.sequence;
        _activeIndex = nextIndex;

        return Mpu6050Status::OK;
    }

    const ImuDataMpu6050* getDataPtr() const {
        return &_buffers[_activeIndex];
    }

    ImuDataMpu6050 getDataCopy() const {
        return _buffers[_activeIndex];
    }

    void setTargetRateHz(float hz) {
        if (hz > 0.0f) {
            _targetDtS = 1.0f / hz;
        }
    }

    void setEkfNoise(float processRollPitch, float processYaw, float accelMeasurement) {
        _qRollPitch = processRollPitch;
        _qYaw = processYaw;
        _rAccel = accelMeasurement;
    }

    void resetYaw(float yawRad = 0.0f) {
        _state[2] = wrapPi(yawRad);
    }

    void resetEkf() {
        _state[0] = 0.0f;
        _state[1] = 0.0f;
        _state[2] = 0.0f;

        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t col = 0; col < 3; ++col) {
                _covariance[row][col] = (row == col) ? 0.05f : 0.0f;
            }
        }
    }

    MPU6050Fast& driver() {
        return _driver;
    }

    const MPU6050Fast& driver() const {
        return _driver;
    }

    uint8_t address() const {
        return _address;
    }

    bool isInitialized() const {
        return _initialized;
    }

    static MPU6050* findByAddress(uint8_t address) {
        MPU6050** items = registry();

        if (address == MPU6050_ADDR_LOW) {
            return items[0];
        }

        if (address == MPU6050_ADDR_HIGH) {
            return items[1];
        }

        return nullptr;
    }

    static const ImuDataMpu6050* getDataPtrByAddress(uint8_t address) {
        MPU6050* sensor = findByAddress(address);
        return sensor ? sensor->getDataPtr() : nullptr;
    }

private:
    static constexpr float DEG_TO_RAD_F = 0.01745329251994329577f;
    static constexpr float RAD_TO_DEG_F = 57.295779513082320876f;
    static constexpr float PITCH_LIMIT_RAD = 1.553343034f;

    uint8_t _address = MPU6050_ADDR_LOW;
    MPU6050Fast _driver;
    bool _initialized = false;

    ImuDataMpu6050 _buffers[2];
    volatile uint8_t _activeIndex = 0;

    uint32_t _sequence = 0;
    uint32_t _lastUpdateUs = 0;
    float _targetDtS = 1.0f / 400.0f;

    float _state[3] = { 0.0f, 0.0f, 0.0f };

    float _covariance[3][3] = {
        { 0.05f, 0.0f,  0.0f  },
        { 0.0f,  0.05f, 0.0f  },
        { 0.0f,  0.0f,  0.05f }
    };

    float _qRollPitch = 0.001f;
    float _qYaw = 0.0050f;
    float _rAccel = 0.0200f ;

    static MPU6050** registry() {
        static MPU6050* items[2] = { nullptr, nullptr };
        return items;
    }

    void registerSelf() {
        MPU6050** items = registry();

        if (_address == MPU6050_ADDR_LOW) {
            items[0] = this;
        }

        if (_address == MPU6050_ADDR_HIGH) {
            items[1] = this;
        }
    }

    void writeStatus(Mpu6050Status status) {
        const uint8_t nextIndex = _activeIndex ^ 1;

        ImuDataMpu6050 out = _buffers[_activeIndex];

        out.address = _address;
        out.status = status;
        out.sequence = _sequence + 1;
        out.timestamp_us = micros();

        _buffers[nextIndex] = out;
        _sequence = out.sequence;
        _activeIndex = nextIndex;
    }

    void initializeAngleFromAccel() {
        Mpu6050ScaledData sample;

        if (_driver.readScaled(sample) != Mpu6050Status::OK) {
            return;
        }

        float roll = 0.0f;
        float pitch = 0.0f;

        accelToRollPitch(sample.ax_g, sample.ay_g, sample.az_g, roll, pitch);

        _state[0] = roll;
        _state[1] = pitch;
        _state[2] = 0.0f;
    }

    static float clampPitch(float pitch) {
        if (pitch > PITCH_LIMIT_RAD) {
            return PITCH_LIMIT_RAD;
        }

        if (pitch < -PITCH_LIMIT_RAD) {
            return -PITCH_LIMIT_RAD;
        }

        return pitch;
    }

    static float wrapPi(float angle) {
        while (angle > PI) {
            angle -= TWO_PI;
        }

        while (angle < -PI) {
            angle += TWO_PI;
        }

        return angle;
    }

    static void accelToRollPitch(float ax, float ay, float az, float& roll, float& pitch) {
        roll = atan2f(ay, az);
        pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    }

    void ekfPredict(float gx, float gy, float gz, float dt) {
        const float phi = _state[0];
        const float theta = clampPitch(_state[1]);

        const float sPhi = sinf(phi);
        const float cPhi = cosf(phi);
        const float tTheta = tanf(theta);

        float cTheta = cosf(theta);

        if (fabsf(cTheta) < 0.01745f) {
            cTheta = (cTheta >= 0.0f) ? 0.01745f : -0.01745f;
        }

        const float secTheta = 1.0f / cTheta;
        const float secTheta2 = secTheta * secTheta;

        const float rollRate = gx + sPhi * tTheta * gy + cPhi * tTheta * gz;
        const float pitchRate = cPhi * gy - sPhi * gz;
        const float yawRate = sPhi * secTheta * gy + cPhi * secTheta * gz;

        _state[0] = wrapPi(_state[0] + rollRate * dt);
        _state[1] = clampPitch(_state[1] + pitchRate * dt);
        _state[2] = wrapPi(_state[2] + yawRate * dt);

        const float a00 = cPhi * tTheta * gy - sPhi * tTheta * gz;
        const float a01 = (sPhi * gy + cPhi * gz) * secTheta2;
        const float a10 = -sPhi * gy - cPhi * gz;
        const float a20 = cPhi * secTheta * gy - sPhi * secTheta * gz;
        const float a21 = (sPhi * gy + cPhi * gz) * secTheta * tTheta;

        const float jacobian[3][3] = {
            { 1.0f + a00 * dt, a01 * dt, 0.0f },
            { a10 * dt,        1.0f,     0.0f },
            { a20 * dt,        a21 * dt, 1.0f }
        };

        float predictedCovarianceTemp[3][3] = {};

        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t col = 0; col < 3; ++col) {
                predictedCovarianceTemp[row][col] =
                    jacobian[row][0] * _covariance[0][col] +
                    jacobian[row][1] * _covariance[1][col] +
                    jacobian[row][2] * _covariance[2][col];
            }
        }

        float predictedCovariance[3][3] = {};

        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t col = 0; col < 3; ++col) {
                predictedCovariance[row][col] =
                    predictedCovarianceTemp[row][0] * jacobian[col][0] +
                    predictedCovarianceTemp[row][1] * jacobian[col][1] +
                    predictedCovarianceTemp[row][2] * jacobian[col][2];
            }
        }

        predictedCovariance[0][0] += _qRollPitch * dt;
        predictedCovariance[1][1] += _qRollPitch * dt;
        predictedCovariance[2][2] += _qYaw * dt;

        copyCovariance(predictedCovariance);
        symmetrizeCovariance();
    }

    void ekfCorrectAccel(float ax, float ay, float az) {
        const float norm2 = ax * ax + ay * ay + az * az;

        if (norm2 < 0.25f || norm2 > 4.0f) {
            return;
        }

        float rollAcc = 0.0f;
        float pitchAcc = 0.0f;

        accelToRollPitch(ax, ay, az, rollAcc, pitchAcc);

        const float innovationRoll = wrapPi(rollAcc - _state[0]);
        const float innovationPitch = wrapPi(pitchAcc - _state[1]);

        const float measurementNoise = _rAccel * _rAccel;

        const float s00 = _covariance[0][0] + measurementNoise;
        const float s01 = _covariance[0][1];
        const float s10 = _covariance[1][0];
        const float s11 = _covariance[1][1] + measurementNoise;

        const float determinant = s00 * s11 - s01 * s10;

        if (fabsf(determinant) < 1.0e-9f) {
            return;
        }

        const float inv00 =  s11 / determinant;
        const float inv01 = -s01 / determinant;
        const float inv10 = -s10 / determinant;
        const float inv11 =  s00 / determinant;

        float kalmanGain[3][2] = {};

        for (uint8_t index = 0; index < 3; ++index) {
            kalmanGain[index][0] =
                _covariance[index][0] * inv00 +
                _covariance[index][1] * inv10;

            kalmanGain[index][1] =
                _covariance[index][0] * inv01 +
                _covariance[index][1] * inv11;
        }

        _state[0] = wrapPi(
            _state[0] +
            kalmanGain[0][0] * innovationRoll +
            kalmanGain[0][1] * innovationPitch
        );

        _state[1] = clampPitch(
            _state[1] +
            kalmanGain[1][0] * innovationRoll +
            kalmanGain[1][1] * innovationPitch
        );

        _state[2] = wrapPi(
            _state[2] +
            kalmanGain[2][0] * innovationRoll +
            kalmanGain[2][1] * innovationPitch
        );

        float correctedCovariance[3][3] = {};

        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t col = 0; col < 3; ++col) {
                correctedCovariance[row][col] =
                    _covariance[row][col] -
                    kalmanGain[row][0] * _covariance[0][col] -
                    kalmanGain[row][1] * _covariance[1][col];
            }
        }

        copyCovariance(correctedCovariance);
        symmetrizeCovariance();
    }

    void copyCovariance(const float src[3][3]) {
        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t col = 0; col < 3; ++col) {
                _covariance[row][col] = src[row][col];
            }
        }
    }

    void symmetrizeCovariance() {
        const float c01 = 0.5f * (_covariance[0][1] + _covariance[1][0]);
        const float c02 = 0.5f * (_covariance[0][2] + _covariance[2][0]);
        const float c12 = 0.5f * (_covariance[1][2] + _covariance[2][1]);

        _covariance[0][1] = c01;
        _covariance[1][0] = c01;

        _covariance[0][2] = c02;
        _covariance[2][0] = c02;

        _covariance[1][2] = c12;
        _covariance[2][1] = c12;

        if (_covariance[0][0] < 1.0e-8f) {
            _covariance[0][0] = 1.0e-8f;
        }

        if (_covariance[1][1] < 1.0e-8f) {
            _covariance[1][1] = 1.0e-8f;
        }

        if (_covariance[2][2] < 1.0e-8f) {
            _covariance[2][2] = 1.0e-8f;
        }
    }
};