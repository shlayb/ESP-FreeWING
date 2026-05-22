#pragma once
#ifndef PARAMS_H
#define PARAMS_H

/**
 * ============================================================
 *  params.h — Parameter Manager
 *  ESP32 | FixedWing Autopilot FC
 * ============================================================
 *
 *  Fitur:
 *    - Tabel param lengkap (42 entry) mencakup ServoPinCofig
 *      dan seluruh field FixedWingConfig
 *    - Persistensi via ESP32 NVS (Preferences library)
 *    - getParamValue / setParamValue terintegrasi MAVLink
 *    - loadAll()  → panggil di setup() sebelum mavHandler.begin()
 *    - saveParam(idx, val) → dipanggil otomatis saat PARAM_SET
 *
 *  Integrasi ke MAV.h:
 *    1. Hapus blok PARAM_TABLE, PARAM_COUNT, ParamDef di MAV.h
 *    2. Tambahkan #include "params.h" di atas MAV.h
 *    3. Ganti getParamValue() dan setParamValue() di MAV.h dengan
 *       wrapper yang memanggil versi dari ParamsManager
 *
 *  NVS namespace : "fwparams"
 *  Key format    : "p00" … "p41"  (3 karakter, aman < 15 char limit)
 * ============================================================
 */

#include <Arduino.h>
#include <Preferences.h>
#include <cstring>

#include "../Control/fixedwing.h"   // FixedWingConfig, PIDParams
#include "../Actuator/actuator.h"   // ServoPinCofig, ServoController

// Extern ke instance global di masing-masing modul

// ============================================================
//  PARAM TABLE  (42 params)
//  Nama max 16 karakter (standar MAVLink PARAM_VALUE)
//  Indeks HARUS sinkron dengan readFromStruct() / applyToStruct()
// ============================================================
struct ParamDef {
    char  id[17];
    float minVal;
    float maxVal;
};

static const uint8_t PARAM_COUNT = 42;

static const ParamDef PARAM_TABLE[PARAM_COUNT] = {

    // ── Servo Pins [0-4] ─────────────────────────────────────
    { "SRV_PIN1",        0,   39 },   //  0 – Aileron Right
    { "SRV_PIN2",        0,   39 },   //  1 – Aileron Left
    { "SRV_PIN3",        0,   39 },   //  2 – Elevator
    { "SRV_PIN4",        0,   39 },   //  3 – Rudder
    { "SRV_PIN5",        0,   39 },   //  4 – Motor

    // ── PWM Range [5-7] ──────────────────────────────────────
    { "PWM_MIN_US",    800, 1200 },   //  5
    { "PWM_MID_US",   1400, 1600 },   //  6
    { "PWM_MAX_US",   1800, 2200 },   //  7

    // ── Arming & Mode thresholds [8-9] ───────────────────────
    { "ARM_THRESH_US",  900, 2100 },  //  8 – CH5: >= armed
    { "MODE_THRESH_US", 900, 2100 },  //  9 – CH6: >= FBWA

    // ── FBWA attitude limits [10-12] ─────────────────────────
    { "ROLL_LIM_DEG",    10,   75 },  // 10
    { "PTCH_MAX_DEG",     5,   45 },  // 11
    { "PTCH_MIN_DEG",   -45,   -2 },  // 12

    // ── Rate limits [13-14] ──────────────────────────────────
    { "ROLL_RATE_LIM",   20,  360 },  // 13 – deg/s
    { "PTCH_RATE_LIM",   10,  270 },  // 14 – deg/s

    // ── Stick deadband [15] ──────────────────────────────────
    { "STICK_DB_US",      0,  100 },  // 15 – µs each side

    // ── Servo Trims [16-20] ──────────────────────────────────
    { "TRIM_AIL_R_US", 1300, 1700 },  // 16 – Aileron Right
    { "TRIM_AIL_L_US", 1300, 1700 },  // 17 – Aileron Left
    { "TRIM_ELEV_US",  1300, 1700 },  // 18 – Elevator
    { "TRIM_RUDR_US",  1300, 1700 },  // 19 – Rudder
    { "THRO_MIN_US",    800, 1200 },  // 20 – Motor minimum

    // ── Reverse Flags (0=normal, 1=reversed) [21-24] ─────────
    { "REV_AIL_R",       0,    1 },  // 21
    { "REV_AIL_L",       0,    1 },  // 22
    { "REV_ELEV",        0,    1 },  // 23
    { "REV_RUDR",        0,    1 },  // 24

    // ── Failsafe Outputs [25-29] ─────────────────────────────
    { "FS_AIL_R_US",   1000, 2000 }, // 25
    { "FS_AIL_L_US",   1000, 2000 }, // 26
    { "FS_ELEV_US",    1000, 2000 }, // 27
    { "FS_RUDR_US",    1000, 2000 }, // 28
    { "FS_THRO_US",     800, 1200 }, // 29

    // ── Outer Attitude PID [30-35] ───────────────────────────
    { "ROLL_ATT_KP",   0.0f, 20.0f }, // 30
    { "ROLL_ATT_KI",   0.0f,  5.0f }, // 31
    { "ROLL_ATT_KD",   0.0f,  2.0f }, // 32
    { "PTCH_ATT_KP",   0.0f, 20.0f }, // 33
    { "PTCH_ATT_KI",   0.0f,  5.0f }, // 34
    { "PTCH_ATT_KD",   0.0f,  2.0f }, // 35

    // ── Inner Rate PID [36-41] ───────────────────────────────
    { "ROLL_RATE_KP",  0.0f,  0.5f  }, // 36
    { "ROLL_RATE_KI",  0.0f,  0.2f  }, // 37
    { "ROLL_RATE_KD",  0.0f,  0.05f }, // 38
    { "PTCH_RATE_KP",  0.0f,  0.5f  }, // 39
    { "PTCH_RATE_KI",  0.0f,  0.2f  }, // 40
    { "PTCH_RATE_KD",  0.0f,  0.05f }, // 41
};

// ============================================================
//  HELPERS
// ============================================================
namespace MavUtil {

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline int findParam(const char* name) {
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(PARAM_TABLE[i].id, name, 16) == 0) return i;
    }
    return -1;
}

// Format NVS key: "p00"…"p41"  (3 char, aman di bawah limit 15)
inline void paramKey(uint8_t idx, char* out) {  // out >= 4 byte
    out[0] = 'p';
    out[1] = '0' + (idx / 10);
    out[2] = '0' + (idx % 10);
    out[3] = '\0';
}

} // namespace MavUtil

// ============================================================
//  ParamsManager
// ============================================================
class ParamsManager {
public:

    // ── Init — panggil di setup() sebelum mavHandler.begin() ─
    void begin() {
        _prefs.begin("fwparams", false);
        Serial.println("[PARAMS] NVS namespace: fwparams");
    }

    void end() { _prefs.end(); }

    // ── Load semua param dari NVS → cfg + pins ────────────────
    //  First boot: key belum ada → default dari struct disimpan
    void loadAll(FixedWingConfig& cfg, ServoPinCofig& pins) {
        char key[4];
        bool firstBoot = !_prefs.isKey("p00");

        for (uint8_t i = 0; i < PARAM_COUNT; i++) {
            MavUtil::paramKey(i, key);
            if (_prefs.isKey(key)) {
                float val = _prefs.getFloat(key, readFromStruct(i, cfg, pins));
                applyToStruct(i, val, cfg, pins);
            } else {
                _prefs.putFloat(key, readFromStruct(i, cfg, pins));
            }
        }

        Serial.println(firstBoot
            ? "[PARAMS] First boot — defaults saved to NVS"
            : "[PARAMS] Params loaded from NVS");
    }

    // ── Simpan satu param ke NVS ──────────────────────────────
    void saveParam(uint8_t idx, float val) {
        if (idx >= PARAM_COUNT) return;
        char key[4];
        MavUtil::paramKey(idx, key);
        _prefs.putFloat(key, val);
    }

    // ── Reset semua ke default (hapus namespace NVS) ──────────
    void resetAll() {
        _prefs.clear();
        Serial.println("[PARAMS] NVS cleared — reboot untuk load defaults");
    }

    // ── Baca nilai param saat ini dari global struct ──────────
    float getParamValue(uint16_t idx) const {
        FixedWingConfig cfg = fixedWing.getConfig();
        return readFromStruct((uint8_t)idx, cfg, servoConfig);
    }

    // ── Set param: clamp → apply ke struct → fixedWing → NVS ─
    void setParamValue(uint16_t idx, float val) {
        if (idx >= PARAM_COUNT) return;
        float v = MavUtil::clampf(val,
                                  PARAM_TABLE[idx].minVal,
                                  PARAM_TABLE[idx].maxVal);

        FixedWingConfig cfg = fixedWing.getConfig();
        applyToStruct((uint8_t)idx, v, cfg, servoConfig);
        fixedWing.setConfig(cfg);

        // Jika servo pin berubah, re-attach hardware
        if (idx <= 4) {
            servoController.attachServos(servoConfig);
        }

        saveParam((uint8_t)idx, v);
    }

private:
    Preferences _prefs;

    // ── Baca nilai dari struct ────────────────────────────────
    static float readFromStruct(uint8_t idx,
                                 const FixedWingConfig& cfg,
                                 const ServoPinCofig&   pins)
    {
        switch (idx) {
            // Servo pins
            case  0: return (float)pins.ServoPin1;
            case  1: return (float)pins.ServoPin2;
            case  2: return (float)pins.ServoPin3;
            case  3: return (float)pins.ServoPin4;
            case  4: return (float)pins.ServoPin5;
            // PWM range
            case  5: return (float)cfg.pwmMin;
            case  6: return (float)cfg.pwmMid;
            case  7: return (float)cfg.pwmMax;
            // Thresholds
            case  8: return (float)cfg.armingThresholdUs;
            case  9: return (float)cfg.modeThresholdUs;
            // FBWA limits
            case 10: return cfg.rollLimitDeg;
            case 11: return cfg.pitchMaxDeg;
            case 12: return cfg.pitchMinDeg;
            // Rate limits
            case 13: return cfg.rollRateLimitDps;
            case 14: return cfg.pitchRateLimitDps;
            // Deadband
            case 15: return (float)cfg.stickDeadbandUs;
            // Trims
            case 16: return (float)cfg.aileronRightTrimUs;
            case 17: return (float)cfg.aileronLeftTrimUs;
            case 18: return (float)cfg.elevatorTrimUs;
            case 19: return (float)cfg.rudderTrimUs;
            case 20: return (float)cfg.throttleMinUs;
            // Reverse flags
            case 21: return cfg.reverseAileronRight ? 1.0f : 0.0f;
            case 22: return cfg.reverseAileronLeft  ? 1.0f : 0.0f;
            case 23: return cfg.reverseElevator     ? 1.0f : 0.0f;
            case 24: return cfg.reverseRudder       ? 1.0f : 0.0f;
            // Failsafe
            case 25: return (float)cfg.failsafeAileronRightUs;
            case 26: return (float)cfg.failsafeAileronLeftUs;
            case 27: return (float)cfg.failsafeElevatorUs;
            case 28: return (float)cfg.failsafeRudderUs;
            case 29: return (float)cfg.failsafeThrottleUs;
            // Roll attitude PID
            case 30: return cfg.rollAttitude.kp;
            case 31: return cfg.rollAttitude.ki;
            case 32: return cfg.rollAttitude.kd;
            // Pitch attitude PID
            case 33: return cfg.pitchAttitude.kp;
            case 34: return cfg.pitchAttitude.ki;
            case 35: return cfg.pitchAttitude.kd;
            // Roll rate PID
            case 36: return cfg.rollRate.kp;
            case 37: return cfg.rollRate.ki;
            case 38: return cfg.rollRate.kd;
            // Pitch rate PID
            case 39: return cfg.pitchRate.kp;
            case 40: return cfg.pitchRate.ki;
            case 41: return cfg.pitchRate.kd;
            default: return 0.0f;
        }
    }

    // ── Tulis nilai ke struct ─────────────────────────────────
    static void applyToStruct(uint8_t idx, float val,
                               FixedWingConfig& cfg,
                               ServoPinCofig&   pins)
    {
        switch (idx) {
            // Servo pins
            case  0: pins.ServoPin1 = (uint8_t)val; break;
            case  1: pins.ServoPin2 = (uint8_t)val; break;
            case  2: pins.ServoPin3 = (uint8_t)val; break;
            case  3: pins.ServoPin4 = (uint8_t)val; break;
            case  4: pins.ServoPin5 = (uint8_t)val; break;
            // PWM range
            case  5: cfg.pwmMin = (uint16_t)val; break;
            case  6: cfg.pwmMid = (uint16_t)val; break;
            case  7: cfg.pwmMax = (uint16_t)val; break;
            // Thresholds
            case  8: cfg.armingThresholdUs = (uint16_t)val; break;
            case  9: cfg.modeThresholdUs   = (uint16_t)val; break;
            // FBWA limits
            case 10: cfg.rollLimitDeg  = val; break;
            case 11: cfg.pitchMaxDeg   = val; break;
            case 12: cfg.pitchMinDeg   = val; break;
            // Rate limits
            case 13: cfg.rollRateLimitDps  = val; break;
            case 14: cfg.pitchRateLimitDps = val; break;
            // Deadband
            case 15: cfg.stickDeadbandUs = (uint16_t)val; break;
            // Trims
            case 16: cfg.aileronRightTrimUs = (uint16_t)val; break;
            case 17: cfg.aileronLeftTrimUs  = (uint16_t)val; break;
            case 18: cfg.elevatorTrimUs     = (uint16_t)val; break;
            case 19: cfg.rudderTrimUs       = (uint16_t)val; break;
            case 20: cfg.throttleMinUs      = (uint16_t)val; break;
            // Reverse flags
            case 21: cfg.reverseAileronRight = (val >= 0.5f); break;
            case 22: cfg.reverseAileronLeft  = (val >= 0.5f); break;
            case 23: cfg.reverseElevator     = (val >= 0.5f); break;
            case 24: cfg.reverseRudder       = (val >= 0.5f); break;
            // Failsafe
            case 25: cfg.failsafeAileronRightUs = (uint16_t)val; break;
            case 26: cfg.failsafeAileronLeftUs  = (uint16_t)val; break;
            case 27: cfg.failsafeElevatorUs     = (uint16_t)val; break;
            case 28: cfg.failsafeRudderUs       = (uint16_t)val; break;
            case 29: cfg.failsafeThrottleUs     = (uint16_t)val; break;
            // Roll attitude PID
            case 30: cfg.rollAttitude.kp = val; break;
            case 31: cfg.rollAttitude.ki = val; break;
            case 32: cfg.rollAttitude.kd = val; break;
            // Pitch attitude PID
            case 33: cfg.pitchAttitude.kp = val; break;
            case 34: cfg.pitchAttitude.ki = val; break;
            case 35: cfg.pitchAttitude.kd = val; break;
            // Roll rate PID
            case 36: cfg.rollRate.kp = val; break;
            case 37: cfg.rollRate.ki = val; break;
            case 38: cfg.rollRate.kd = val; break;
            // Pitch rate PID
            case 39: cfg.pitchRate.kp = val; break;
            case 40: cfg.pitchRate.ki = val; break;
            case 41: cfg.pitchRate.kd = val; break;
            default: break;
        }
    }
};

// ============================================================
//  GLOBAL INSTANCE
// ============================================================
static ParamsManager paramsManager;

// ============================================================
//  PETUNJUK INTEGRASI
// ============================================================
//
//  main.cpp / setup():
//  ───────────────────────────────────────────────────────────
//  ServoPinCofig servoConfig;          // definisi global
//
//  void setup() {
//      paramsManager.begin();
//
//      FixedWingConfig cfg = fixedWing.getConfig();
//      paramsManager.loadAll(cfg, servoConfig);   // load NVS
//      fixedWing.setConfig(cfg);
//
//      servoController.init_actuators(servoConfig);
//      mavHandler.begin();
//  }
//
//  MAV.h — ganti dua method:
//  ───────────────────────────────────────────────────────────
//  float getParamValue(uint16_t idx) const {
//      return paramsManager.getParamValue(idx);
//  }
//  void setParamValue(uint16_t idx, float val) {
//      paramsManager.setParamValue(idx, val);
//  }
//
//  Reset NVS (opsional):
//  ───────────────────────────────────────────────────────────
//  paramsManager.resetAll();
//  ESP.restart();

#endif  // PARAMS_H