#pragma once
#ifndef FIXEDWING_H
#define FIXEDWING_H

#include <Arduino.h>
#include <math.h>

#include "../include/Communication/radio.h"
#include "../include/Sensor/imu.h"
#include "../include/Actuator/actuator.h"

// ============================================================================
//  UTILITY
// ============================================================================

namespace FWUtil {

inline float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline uint16_t clamp(uint16_t v, uint16_t lo, uint16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace FWUtil

// ============================================================================
//  FIXED WING MODE
// ============================================================================

enum class FixedWingMode : uint8_t {
    MANUAL   = 0,
    FBWA     = 1,
    FAILSAFE = 2
};

// ============================================================================
//  PID PARAMS
// ============================================================================

struct PIDParams {
    float kp;
    float ki;
    float kd;
    float outMin;
    float outMax;
    float integratorLimit;

    PIDParams(
        float kp_ = 0.0f,
        float ki_ = 0.0f,
        float kd_ = 0.0f,
        float outMin_ = -1.0f,
        float outMax_ = 1.0f,
        float integratorLimit_ = 20.0f
    ) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        outMin = outMin_;
        outMax = outMax_;
        integratorLimit = integratorLimit_;
    }
};

// ============================================================================
//  FIXED WING CONFIG
// ============================================================================

struct FixedWingConfig {
    // PWM standard
    uint16_t pwmMin = 1000;
    uint16_t pwmMid = 1500;
    uint16_t pwmMax = 2000;

    // -------------------------------------------------------------------------
    //  Radio channel mapping (input):
    //    CH1 = Roll   / Aileron
    //    CH2 = Pitch  / Elevator
    //    CH3 = Throttle / Motor
    //    CH4 = Yaw    / Rudder
    //    CH5 = Arming  (>= armingThresholdUs → armed)
    //    CH6 = Mode    (>= modeThresholdUs   → FBWA, else MANUAL)
    //
    //  Servo output mapping:
    //    ServoPWM1 = Aileron Right
    //    ServoPWM2 = Aileron Left  (reverseAileronLeft = true by default)
    //    ServoPWM3 = Elevator
    //    ServoPWM4 = Rudder
    //    ServoPWM5 = Motor (throttle — locked to throttleMinUs when disarmed)
    // -------------------------------------------------------------------------

    uint16_t armingThresholdUs = 1500;  // CH5: below = disarmed, above = armed
    uint16_t modeThresholdUs   = 1500;  // CH6: below = MANUAL,   above = FBWA

    // FBWA attitude limits
    float rollLimitDeg  = 45.0f;
    float pitchMaxDeg   = 20.0f;
    float pitchMinDeg   = -15.0f;

    // Rate limits from outer attitude controller
    float rollRateLimitDps  = 120.0f;
    float pitchRateLimitDps = 90.0f;

    // Stick deadband to prevent jitter at center
    uint16_t stickDeadbandUs = 20;

    // -------------------------------------------------------------------------
    //  Servo trims
    // -------------------------------------------------------------------------
    uint16_t aileronRightTrimUs = 1500;  // ServoPWM1
    uint16_t aileronLeftTrimUs  = 1500;  // ServoPWM2
    uint16_t elevatorTrimUs     = 1500;  // ServoPWM3
    uint16_t rudderTrimUs       = 1500;  // ServoPWM4
    uint16_t throttleMinUs      = 1000;  // ServoPWM5 — also used when disarmed

    // -------------------------------------------------------------------------
    //  Servo reverse flags
    //  reverseAileronLeft = true: left servo is physically mounted mirrored
    //  from the right — flip to false if they are already mirrored mechanically.
    // -------------------------------------------------------------------------
    bool reverseAileronRight = false;
    bool reverseAileronLeft  = true;
    bool reverseElevator     = false;
    bool reverseRudder       = false;

    // -------------------------------------------------------------------------
    //  Failsafe outputs (wings level, elevator/rudder neutral, motor off)
    // -------------------------------------------------------------------------
    uint16_t failsafeAileronRightUs = 1500;
    uint16_t failsafeAileronLeftUs  = 1500;
    uint16_t failsafeElevatorUs     = 1500;
    uint16_t failsafeRudderUs       = 1500;
    uint16_t failsafeThrottleUs     = 1000;  // motor off

    // -------------------------------------------------------------------------
    //  PID gains — conservative defaults, tune bertahap di lapangan
    // -------------------------------------------------------------------------

    // Outer loop: attitude error → desired angular rate
    PIDParams rollAttitude  = { 4.0f,   0.0f,  0.0f, -120.0f, 120.0f, 50.0f };
    PIDParams pitchAttitude = { 4.0f,   0.0f,  0.0f,  -90.0f,  90.0f, 50.0f };

    // Inner loop: rate error → normalized servo command
    PIDParams rollRate      = { 0.025f, 0.0f,  0.001f, -1.0f,   1.0f, 20.0f };
    PIDParams pitchRate     = { 0.030f, 0.0f,  0.001f, -1.0f,   1.0f, 20.0f };
};

// ============================================================================
//  PID CONTROLLER
// ============================================================================

class FixedWingPID {
public:
    FixedWingPID() = default;

    FixedWingPID(float kp, float ki, float kd, float outMin, float outMax,
                float integratorLimit = 100.0f)
        : _kp(kp), _ki(ki), _kd(kd)
        , _outMin(outMin), _outMax(outMax)
        , _integratorLimit(fabsf(integratorLimit))
    {}

    void setGains(float kp, float ki, float kd) {
        _kp = kp; _ki = ki; _kd = kd;
    }

    void setOutputLimit(float outMin, float outMax) {
        _outMin = outMin; _outMax = outMax;
    }

    void setIntegratorLimit(float limit) {
        _integratorLimit = fabsf(limit);
    }

    void reset() {
        _integrator = 0.0f;
        _lastError  = 0.0f;
        _firstRun   = true;
    }

    float update(float error, float dt) {
        if (dt <= 0.0f || dt > 1.0f) dt = 0.02f;

        _integrator = FWUtil::clamp(
            _integrator + error * dt,
            -_integratorLimit, _integratorLimit
        );

        float derivative = _firstRun ? 0.0f : (error - _lastError) / dt;
        _firstRun  = false;
        _lastError = error;

        return FWUtil::clamp(
            _kp * error + _ki * _integrator + _kd * derivative,
            _outMin, _outMax
        );
    }

private:
    float _kp = 0.0f, _ki = 0.0f, _kd = 0.0f;
    float _outMin = -1.0f, _outMax = 1.0f;
    float _integrator = 0.0f;
    float _integratorLimit = 100.0f;
    float _lastError = 0.0f;
    bool  _firstRun  = true;
};

// ============================================================================
//  FIXED WING CONTROLLER
// ============================================================================

class FixedWing {
public:
    FixedWing() = default;

    void begin(const ServoPinCofig& servoPins = ServoPinCofig()) {
        servoController.init_actuators(servoPins);
        applyPIDFromConfig();
        resetControllers();
        _lastUpdateUs = micros();
        _armed = false;
        setFailsafeOutput();
        servoController.writeServos(PWM_Servo);
    }

    void setConfig(const FixedWingConfig& config) {
        _config = config;
        applyPIDFromConfig();
    }
    FixedWingConfig getConfig() const { return _config; }
    FixedWingMode   getMode()   const { return _mode;   }
    bool            isArmed()   const { return _armed;  }

    void resetControllers() {
        _rollAttitudePID.reset();
        _rollRatePID.reset();
        _pitchAttitudePID.reset();
        _pitchRatePID.reset();
    }

    // =========================================================================
    //  UPDATE — via Radio object
    // =========================================================================

    ServoPWM update(Radio& radioInput, const ImuData& imuData,
                    bool writeToServo = true) {
        if (radioInput.signalLost || radioInput.failsafe) {
            applyFailsafe(writeToServo);
            return PWM_Servo;
        }
        return update(radioInput.channels, imuData, writeToServo);
    }

    // =========================================================================
    //  UPDATE — via raw channels
    // =========================================================================

    ServoPWM update(const Channel& channels, const ImuData& imuData,
                    bool writeToServo = true) {
        const float dt = computeDt(imuData);

        // --- Arming (CH5) ---------------------------------------------------
        const bool nowArmed = (channels.ch5 >= _config.armingThresholdUs);
        if (nowArmed && !_armed) {
            // Rising edge: just armed — reset integrators so there is no
            // accumulated wind-up from when the vehicle was on the ground.
            resetControllers();
        }
        _armed = nowArmed;

        // --- Mode (CH6) -----------------------------------------------------
        const FixedWingMode newMode = readModeFromChannel(channels);
        if (newMode != _mode) {
            resetControllers();
            _mode = newMode;
        }

        switch (_mode) {
            case FixedWingMode::MANUAL: runManual(channels);             break;
            case FixedWingMode::FBWA:   runFBWA(channels, imuData, dt); break;
            default:                    setFailsafeOutput();              break;
        }

        // --- Motor lockout --------------------------------------------------
        // Overwrite throttle with minimum when disarmed, regardless of mode.
        if (!_armed) {
            PWM_Servo.ServoPWM5 = _config.throttleMinUs;
        }

        if (writeToServo) servoController.writeServos(PWM_Servo);
        return PWM_Servo;
    }

    ServoPWM getOutput() const { return PWM_Servo; }

private:
    FixedWingConfig _config;
    FixedWingMode   _mode  = FixedWingMode::MANUAL;
    bool            _armed = false;
    uint32_t        _lastUpdateUs = 0;

    FixedWingPID _rollAttitudePID;
    FixedWingPID _pitchAttitudePID;
    FixedWingPID _rollRatePID;
    FixedWingPID _pitchRatePID;

    void applyPIDFromConfig() {
        auto make = [](const PIDParams& p) {
            return FixedWingPID(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.integratorLimit);
        };
        _rollAttitudePID  = make(_config.rollAttitude);
        _pitchAttitudePID = make(_config.pitchAttitude);
        _rollRatePID      = make(_config.rollRate);
        _pitchRatePID     = make(_config.pitchRate);
    }

    // ----- mode selection (CH6) ----------------------------------------------

    FixedWingMode readModeFromChannel(const Channel& channels) const {
        return (channels.ch6 >= _config.modeThresholdUs)
            ? FixedWingMode::FBWA
            : FixedWingMode::MANUAL;
    }

    // ----- control loops -----------------------------------------------------

    void runManual(const Channel& channels) {
        const float aileronCmd = pwmToCenteredNormalized(channels.ch1);

        // ServoPWM1 — Aileron Right
        PWM_Servo.ServoPWM1 = surfaceToPwm(aileronCmd,
                                           _config.aileronRightTrimUs,
                                           _config.reverseAileronRight);
        // ServoPWM2 — Aileron Left (same command, reverseAileronLeft mirrors it)
        PWM_Servo.ServoPWM2 = surfaceToPwm(aileronCmd,
                                           _config.aileronLeftTrimUs,
                                           _config.reverseAileronLeft);
        // ServoPWM3 — Elevator
        PWM_Servo.ServoPWM3 = surfaceToPwm(pwmToCenteredNormalized(channels.ch2),
                                           _config.elevatorTrimUs,
                                           _config.reverseElevator);
        // ServoPWM4 — Rudder
        PWM_Servo.ServoPWM4 = surfaceToPwm(pwmToCenteredNormalized(channels.ch4),
                                           _config.rudderTrimUs,
                                           _config.reverseRudder);
        // ServoPWM5 — Motor (may be overwritten to throttleMinUs if disarmed)
        PWM_Servo.ServoPWM5 = FWUtil::clamp(channels.ch3,
                                            _config.throttleMinUs,
                                            _config.pwmMax);
    }

    void runFBWA(const Channel& channels, const ImuData& imuData, float dt) {
        // --- Roll -----------------------------------------------------------
        const float rollStick      = pwmToCenteredNormalized(channels.ch1);
        const float targetRollDeg  = rollStick * _config.rollLimitDeg;
        const float rollErrorDeg   = targetRollDeg - imuData.roll_deg;

        const float desiredRollRate = FWUtil::clamp(
            _rollAttitudePID.update(rollErrorDeg, dt),
            -_config.rollRateLimitDps, _config.rollRateLimitDps
        );
        const float aileronCmd = _rollRatePID.update(
            desiredRollRate - imuData.gx_dps, dt
        );

        // --- Pitch ----------------------------------------------------------
        const float pitchStick     = pwmToCenteredNormalized(channels.ch2);
        const float targetPitchDeg = pitchStick >= 0.0f
            ? pitchStick * _config.pitchMaxDeg
            : pitchStick * fabsf(_config.pitchMinDeg);
        const float pitchErrorDeg  = targetPitchDeg - imuData.pitch_deg;

        const float desiredPitchRate = FWUtil::clamp(
            _pitchAttitudePID.update(pitchErrorDeg, dt),
            -_config.pitchRateLimitDps, _config.pitchRateLimitDps
        );
        const float elevatorCmd = _pitchRatePID.update(
            desiredPitchRate - imuData.gy_dps, dt
        );

        // --- Output mixer ---------------------------------------------------
        // ServoPWM1 — Aileron Right (stabilised)
        PWM_Servo.ServoPWM1 = surfaceToPwm(aileronCmd,
                                           _config.aileronRightTrimUs,
                                           _config.reverseAileronRight);
        // ServoPWM2 — Aileron Left (stabilised, mirrored)
        PWM_Servo.ServoPWM2 = surfaceToPwm(aileronCmd,
                                           _config.aileronLeftTrimUs,
                                           _config.reverseAileronLeft);
        // ServoPWM3 — Elevator (stabilised)
        PWM_Servo.ServoPWM3 = surfaceToPwm(elevatorCmd,
                                           _config.elevatorTrimUs,
                                           _config.reverseElevator);
        // ServoPWM4 — Rudder (manual pass-through)
        PWM_Servo.ServoPWM4 = surfaceToPwm(pwmToCenteredNormalized(channels.ch4),
                                           _config.rudderTrimUs,
                                           _config.reverseRudder);
        // ServoPWM5 — Motor (manual pass-through; may be overwritten if disarmed)
        PWM_Servo.ServoPWM5 = FWUtil::clamp(channels.ch3,
                                            _config.throttleMinUs,
                                            _config.pwmMax);
    }

    void setFailsafeOutput() {
        PWM_Servo.ServoPWM1 = _config.failsafeAileronRightUs;
        PWM_Servo.ServoPWM2 = _config.failsafeAileronLeftUs;
        PWM_Servo.ServoPWM3 = _config.failsafeElevatorUs;
        PWM_Servo.ServoPWM4 = _config.failsafeRudderUs;
        PWM_Servo.ServoPWM5 = _config.failsafeThrottleUs;  // motor off
    }

    void applyFailsafe(bool writeToServo) {
        _armed = false;
        _mode  = FixedWingMode::FAILSAFE;
        setFailsafeOutput();
        if (writeToServo) servoController.writeServos(PWM_Servo);
    }

    // ----- helpers -----------------------------------------------------------

    float computeDt(const ImuData& imuData) {
        if (imuData.dt_s > 0.0005f && imuData.dt_s < 0.1f) {
            return imuData.dt_s;
        }
        const uint32_t nowUs   = micros();
        const uint32_t deltaUs = nowUs - _lastUpdateUs;
        _lastUpdateUs = nowUs;
        const float dt = deltaUs * 1e-6f;
        return (dt > 0.0005f && dt < 0.1f) ? dt : 0.02f;
    }

    float pwmToCenteredNormalized(uint16_t pwm) const {
        pwm = FWUtil::clamp(pwm, _config.pwmMin, _config.pwmMax);

        const int16_t delta    = (int16_t)pwm - (int16_t)_config.pwmMid;
        const int16_t deadband = (int16_t)_config.stickDeadbandUs;

        if (abs(delta) <= deadband) return 0.0f;

        if (delta > 0) {
            return FWUtil::clamp(
                (float)(delta - deadband) /
                (float)(_config.pwmMax - _config.pwmMid - deadband),
                0.0f, 1.0f
            );
        }

        return FWUtil::clamp(
            (float)(delta + deadband) /
            (float)(_config.pwmMid - _config.pwmMin - deadband),
            -1.0f, 0.0f
        );
    }

    uint16_t surfaceToPwm(float cmd, uint16_t trimUs, bool reversed) const {
        cmd = FWUtil::clamp(cmd, -1.0f, 1.0f);
        if (reversed) cmd = -cmd;

        const float halfRange = 0.5f * (float)(_config.pwmMax - _config.pwmMin);
        const float pwm = (float)trimUs + cmd * halfRange;

        return FWUtil::clamp(
            (uint16_t)lroundf(pwm),
            _config.pwmMin, _config.pwmMax
        );
    }
};

// ============================================================================
//  GLOBAL OBJECT
// ============================================================================

static FixedWing fixedWing;

// ============================================================================
//  OPTIONAL FREERTOS TASK
//
//  Mendukung update rate tinggi hingga 400 Hz.
//
//  SYARAT hardware untuk 400 Hz:
//    1. IMU  — harus dikonfigurasi output >= 400 Hz
//    2. Servo — harus servo digital yang terima PWM 400 Hz
//    3. FreeRTOS tick rate — CONFIG_FREERTOS_HZ = 1000 di sdkconfig
//
//  Usage:
//      xTaskCreatePinnedToCore(
//          fixedWingTask, "FixedWingTask", 4096,
//          &fixedWing, 5, NULL, 1
//      );
// ============================================================================

#ifndef FIXEDWING_TASK_RATE_HZ
#define FIXEDWING_TASK_RATE_HZ 400
#endif

static void fixedWingTask(void* parameter) {
    FixedWing& fw = *static_cast<FixedWing*>(parameter);
    fw.begin();

    const TickType_t period = pdMS_TO_TICKS(1000 / FIXEDWING_TASK_RATE_HZ);
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        ImuData data = imu.getData();
        fw.update(radio, data, true);
        vTaskDelayUntil(&lastWakeTime, period);
    }
}

#endif // FIXEDWING_H