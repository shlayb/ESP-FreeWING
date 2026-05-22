#pragma once
#ifndef MAV_H
#define MAV_H

/**
 * ============================================================
 *  MAV.h — MAVLink Handler
 *  ESP32 | FixedWing Autopilot FC
 * ============================================================
 *
 *  Integrasi:
 *    - MAVLink Telemetry (heartbeat, attitude, VFR HUD, battery, nav)
 *    - Mission upload/download (in-memory, up to MAX_WAYPOINTS)
 *    - PID param tuning dari Mission Planner / QGC
 *    - Forward pesan antar port (USB ↔ Telemetry)
 *
 *  RC ditangani oleh Radio (SBUS) — tidak lewat MAVLink di sini.
 *
 *  Serial port:
 *    Serial   (USB)   → GCS (Mission Planner / QGC)
 *    Serial1  (UART1) → Telemetry radio (konfigurasi via define di bawah)
 *    Serial2  (UART2) → SUDAH DIPAKAI SBUS — jangan disentuh di sini
 *
 *  Wiring telemetry radio → ESP32:
 *    Radio TX  →  MAV_TLM_RX_PIN  (default pin 16)
 *    Radio RX  →  MAV_TLM_TX_PIN  (default pin 17)
 *
 *  Usage:
 *    // Di setup():
 *    mavHandler.begin();
 *
 *    // Atau jalankan sebagai FreeRTOS task:
 *    xTaskCreatePinnedToCore(mavlinkTask, "MavTask", 4096, NULL, 2, NULL, 1);
 * ============================================================
 */

#include <Arduino.h>
#include <math.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "TimingUtils.h"
#include "../Sensor/imu.h"
#include "../Control/fixedwing.h"
#include "../Communication/radio.h"
#include <common/mavlink.h>

// ============================================================
//  KONFIGURASI PORT TELEMETRY
//  Override di platformio.ini atau sebelum #include "MAV.h"
// ============================================================
#ifndef MAV_TLM_SERIAL
  #define MAV_TLM_SERIAL   Serial1
#endif
#ifndef MAV_TLM_BAUD
  #define MAV_TLM_BAUD     57600
#endif
#ifndef MAV_TLM_RX_PIN
  #define MAV_TLM_RX_PIN   16
#endif
#ifndef MAV_TLM_TX_PIN
  #define MAV_TLM_TX_PIN   17
#endif

// ============================================================
//  WAYPOINT — simple in-memory store
//  Ganti dengan EEPROM/NVS jika perlu persistensi
// ============================================================
static const uint8_t MAX_WAYPOINTS = 50;

struct WayPoint {
    int32_t lat = 0;  // degE7
    int32_t lng = 0;  // degE7
    int32_t alt = 0;  // cm AGL
};

static WayPoint  g_waypoints[MAX_WAYPOINTS];
static uint16_t  g_wp_sum = 0;



// ============================================================
//  INTERNAL ENUMS
// ============================================================
enum class MavPortId    : uint8_t { USB = 0, TLM = 1 };
enum class MissionState : uint8_t { IDLE, RECEIVING };

// ============================================================
//  MavlinkHandler
// ============================================================
class MavlinkHandler {
public:
    MavlinkHandler() {
        memset(&_st_usb, 0, sizeof(_st_usb));
        memset(&_st_tlm, 0, sizeof(_st_tlm));
    }

    // --------------------------------------------------------
    //  begin() — panggil dari setup()
    // --------------------------------------------------------
    void begin() {
        MAV_TLM_SERIAL.begin(MAV_TLM_BAUD, SERIAL_8N1,
                             MAV_TLM_RX_PIN, MAV_TLM_TX_PIN);
        Serial.println("[MAV] MAVLink handler started");
        Serial.printf("[MAV] TLM baud=%lu RX=%d TX=%d\n",
                      (unsigned long)MAV_TLM_BAUD,
                      MAV_TLM_RX_PIN, MAV_TLM_TX_PIN);
    }

    // --------------------------------------------------------
    //  update() — panggil tiap loop / dari FreeRTOS task
    // --------------------------------------------------------
    void update() {
        handlePorts();
        sendPeriodic();
    }

    // --------------------------------------------------------
    //  Public utilities
    // --------------------------------------------------------
    void notifyWaypointReached(uint16_t seq) { sendMissionItemReached(seq); }
    void sendCustomText(const char* text)    { sendStatusText(MAV_SEVERITY_INFO, text); }
    uint16_t waypointCount() const           { return g_wp_sum; }

private:
    // ── System identity ───────────────────────────────────────
    static const uint8_t  SYSID  = 1;
    static const uint8_t  COMPID = MAV_COMP_ID_AUTOPILOT1;

    // ── Telemetry intervals (ms) ──────────────────────────────
    static const uint32_t HB_MS   = 1000;
    static const uint32_t ATT_MS  = 50;   // 10 Hz
    static const uint32_t VFR_MS  = 250;   //  4 Hz
    static const uint32_t BAT_MS  = 5000;
    static const uint32_t NAV_MS  = 1000;
    static const uint32_t GPOS_MS = 200;   //  5 Hz

    // ── Timers ────────────────────────────────────────────────
    uint32_t _tHb   = 0, _tAtt  = 0, _tVfr  = 0;
    uint32_t _tBat  = 0, _tNav  = 0, _tGpos = 0;

    // ── MAVLink parse state ───────────────────────────────────
    mavlink_status_t  _st_usb, _st_tlm;
    mavlink_message_t _rx_usb, _rx_tlm;

    // ── Mission upload state ──────────────────────────────────
    MissionState _missionState  = MissionState::IDLE;
    uint16_t     _expectedCount = 0;
    uint16_t     _uploadSeq     = 0;
    uint8_t      _upSysId       = 0;
    uint8_t      _upCompId      = 0;

    // ── Param streaming ───────────────────────────────────────
    uint16_t _paramStreamIdx = UINT16_MAX;  // UINT16_MAX = idle

    // =========================================================
    //  OUTPUT
    // =========================================================
    void writePort(HardwareSerial& port, const mavlink_message_t& msg) {
        uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        port.write(buf, len);
    }

    // Kirim ke semua port
    void sendAll(const mavlink_message_t& msg) {
        writePort(Serial,         msg);  // USB  → GCS
        writePort(MAV_TLM_SERIAL, msg);  // UART → telemetry radio
    }

    // =========================================================
    //  PERIODIC TELEMETRY
    // =========================================================
    void sendPeriodic() {
        const uint32_t now     = getMillis();
        const ImuData  imuData = imu.getData();

        if (now - _tHb   >= HB_MS)   { _tHb   = now; sendHeartbeat();           }
        if (now - _tAtt  >= ATT_MS)  { _tAtt  = now; sendAttitude(imuData);     }
        if (now - _tVfr  >= VFR_MS)  { _tVfr  = now; sendVfrHud(imuData);       }
        if (now - _tBat  >= BAT_MS)  { _tBat  = now; sendBattery();              }
        if (now - _tNav  >= NAV_MS)  { _tNav  = now; sendNavController(imuData); }
        if (now - _tGpos >= GPOS_MS) { _tGpos = now; sendGlobalPosition(imuData);}

        pumpParamStream();
    }

    // =========================================================
    //  PARSE INCOMING BYTES
    // =========================================================
    void handlePorts() {
        // USB (GCS)
        while (Serial.available()) {
            uint8_t c = Serial.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &_rx_usb, &_st_usb)) {
                processMsg(_rx_usb, MavPortId::USB);
            }
        }
        // Telemetry radio
        while (MAV_TLM_SERIAL.available()) {
            uint8_t c = MAV_TLM_SERIAL.read();
            if (mavlink_parse_char(MAVLINK_COMM_1, c, &_rx_tlm, &_st_tlm)) {
                processMsg(_rx_tlm, MavPortId::TLM);
            }
        }
    }

    // ── Routing ──────────────────────────────────────────────
    void processMsg(const mavlink_message_t& msg, MavPortId from) {
        // Forward ke port lain terlebih dulu
        if (from == MavPortId::USB) writePort(MAV_TLM_SERIAL, msg);
        else                        writePort(Serial,          msg);

        switch (msg.msgid) {
            case MAVLINK_MSG_ID_COMMAND_LONG:
                handleCommandLong(msg);         break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                handleMissionRequestList(msg);  break;
            case MAVLINK_MSG_ID_MISSION_REQUEST:
                handleMissionRequest(msg);      break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
                handleMissionRequestInt(msg);   break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
                handleMissionCount(msg);        break;
            case MAVLINK_MSG_ID_MISSION_ITEM_INT:
                handleMissionItemInt(msg);      break;
            case MAVLINK_MSG_ID_MISSION_ITEM:
                handleMissionItem(msg);         break;
            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                handleMissionClearAll(msg);     break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                handleParamRequestList(msg);    break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                handleParamRequestRead(msg);    break;
            case MAVLINK_MSG_ID_PARAM_SET:
                handleParamSet(msg);            break;
            default: break;
        }
    }

    // =========================================================
    //  MODE HELPERS (dari FixedWing)
    // =========================================================
    uint8_t getBaseMode() const {
        uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

        if (fixedWing.isArmed()) {
            base |= MAV_MODE_FLAG_SAFETY_ARMED;
        }

        switch (fixedWing.getMode()) {
            case FixedWingMode::MANUAL:
                base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
                break;

            case FixedWingMode::FBWA:
                base |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
                base |= MAV_MODE_FLAG_STABILIZE_ENABLED;
                break;

            case FixedWingMode::FAILSAFE:
                base |= MAV_MODE_FLAG_STABILIZE_ENABLED;
                break;

            default:
                break;
        }

        return base;
    }

        uint32_t getCustomMode() const {
            switch (fixedWing.getMode()) {
                case FixedWingMode::MANUAL:   return 0;  // MANUAL
                case FixedWingMode::FBWA:     return 5;  // FBWA
                case FixedWingMode::FAILSAFE: return 11; // RTL, lebih aman daripada 0
                default:                      return 0;
            }
        }

    // =========================================================
    //  TELEMETRY SENDERS
    // =========================================================

    // ── Heartbeat ────────────────────────────────────────────
    void sendHeartbeat() {
        mavlink_message_t msg;
        uint8_t sys_status = fixedWing.isArmed()
                           ? MAV_STATE_ACTIVE
                           : MAV_STATE_STANDBY;
        mavlink_msg_heartbeat_pack(
            SYSID, COMPID, &msg,
            MAV_TYPE_FIXED_WING,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            getBaseMode(),
            getCustomMode(),
            sys_status);
        sendAll(msg);
    }

    // ── Attitude ─────────────────────────────────────────────
    void sendAttitude(const ImuData& d) {
        mavlink_message_t msg;
        mavlink_msg_attitude_pack(
            SYSID, COMPID, &msg,
            getMillis(),
            d.roll_rad,   d.pitch_rad,   d.yaw_rad,
            d.gx_rad_s,   d.gy_rad_s,    d.gz_rad_s);
        sendAll(msg);
    }

    // ── VFR HUD ──────────────────────────────────────────────
    //  Airspeed & groundspeed diisi 0 (tambahkan sensor jika ada)
    void sendVfrHud(const ImuData& d) {
        mavlink_message_t msg;
        float heading = d.yaw_deg < 0.0f ? d.yaw_deg + 360.0f : d.yaw_deg;
        mavlink_msg_vfr_hud_pack(
            SYSID, COMPID, &msg,
            0.0f,               // airspeed (m/s) — tambahkan sensor
            0.0f,               // groundspeed (m/s) — tambahkan GPS
            (int16_t)heading,
            0,                  // throttle % — opsional
            0.0f,               // alt (m) — tambahkan baro/GPS
            0.0f);              // climb (m/s)
        sendAll(msg);
    }

    // ── Global Position ──────────────────────────────────────
    //  Lat/lon/alt diisi 0 (tambahkan GPS jika ada)
    void sendGlobalPosition(const ImuData& d) {
        mavlink_message_t msg;
        float hdg_f = d.yaw_deg < 0.0f ? d.yaw_deg + 360.0f : d.yaw_deg;
        uint16_t hdg_cdeg = (uint16_t)(hdg_f * 100.0f);
        mavlink_msg_global_position_int_pack(
            SYSID, COMPID, &msg,
            getMillis(),
            0, 0,         // lat, lon  degE7 — GPS
            0, 0,         // alt MSL, alt rel (mm) — baro
            0, 0, 0,      // vx, vy, vz (cm/s) — GPS
            hdg_cdeg);
        sendAll(msg);
    }

    // ── Nav Controller ───────────────────────────────────────
    void sendNavController(const ImuData& d) {
        mavlink_message_t msg;
        mavlink_msg_nav_controller_output_pack(
            SYSID, COMPID, &msg,
            d.roll_deg, d.pitch_deg,
            0, 0,         // nav_bearing, target_bearing
            0,            // wp_dist (m)
            0, 0, 0);     // alt_error, aspd_error, xtrack_error
        sendAll(msg);
    }

    // ── Battery ──────────────────────────────────────────────
    //  Dummy 11.1 V — ganti dengan pembacaan ADC/sensor real
    void sendBattery() {
        uint16_t voltages[10];
        voltages[0] = 11100;  // mV — ganti: (uint16_t)(adcVoltage * 1000.0f)
        for (uint8_t i = 1; i < 10; i++) voltages[i] = UINT16_MAX;

        mavlink_message_t msg;
        mavlink_msg_battery_status_pack(
            SYSID, COMPID, &msg,
            0,
            MAV_BATTERY_FUNCTION_ALL,
            MAV_BATTERY_TYPE_LIPO,
            INT16_MAX,      // suhu unknown
            voltages,
            -1,             // current (cA) unknown
            -1,             // consumed (mAh) unknown
            -1,             // energy (hJ) unknown
            50,             // remaining % — ganti dengan kalkulasi real
            0,
            MAV_BATTERY_CHARGE_STATE_OK);
        sendAll(msg);
    }

    // ── Mission item reached ─────────────────────────────────
    void sendMissionItemReached(uint16_t seq) {
        mavlink_message_t msg;
        mavlink_msg_mission_item_reached_pack(SYSID, COMPID, &msg, seq);
        sendAll(msg);
        Serial.printf("[MAV] WP %d reached\n", seq);
    }

    // ── Status text ──────────────────────────────────────────
    void sendStatusText(uint8_t severity, const char* text) {
        mavlink_message_t msg;
        mavlink_msg_statustext_pack(SYSID, COMPID, &msg, severity, text);
        sendAll(msg);
    }

    // ── Command ACK ──────────────────────────────────────────
    void sendCommandAck(uint16_t command, uint8_t result) {
        mavlink_message_t msg;
        mavlink_msg_command_ack_pack(SYSID, COMPID, &msg,
                                     command, result, 0, 0, 0, 0);
        sendAll(msg);
    }

    // =========================================================
    //  COMMAND LONG
    // =========================================================
    void handleCommandLong(const mavlink_message_t& msg) {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        if (cmd.target_system != SYSID && cmd.target_system != 0) return;

        uint8_t result = MAV_RESULT_UNSUPPORTED;

        switch (cmd.command) {
            // ── Arm / Disarm ─────────────────────────────────
            // Arming dikontrol via RC CH5 — GCS hanya dapat info,
            // tidak bisa force-arm tanpa sinyal radio.
            case MAV_CMD_COMPONENT_ARM_DISARM:
                result = MAV_RESULT_ACCEPTED;
                sendStatusText(MAV_SEVERITY_INFO,
                    cmd.param1 == 1.0f
                    ? "ARM: gunakan RC CH5"
                    : "DISARM: gunakan RC CH5");
                break;

            // ── Set Mode ─────────────────────────────────────
            // Mode dikontrol via RC CH6 (MANUAL / FBWA).
            // GCS dapat mengubah mode via command ini sebagai override.
            case MAV_CMD_DO_SET_MODE: {
                uint32_t custom = (uint32_t)cmd.param2;
                // 0 = MANUAL, 5 = FBWA
                if (custom == 0 || custom == 5) {
                    result = MAV_RESULT_ACCEPTED;
                    // Catatan: untuk apply, kamu perlu memodifikasi
                    // FixedWing agar menerima mode override dari GCS.
                    sendStatusText(MAV_SEVERITY_INFO,
                        custom == 0 ? "MODE: MANUAL (RC override)" : "MODE: FBWA (RC override)");
                } else {
                    result = MAV_RESULT_UNSUPPORTED;
                    sendStatusText(MAV_SEVERITY_WARNING, "Mode tidak didukung");
                }
                break;
            }

            default:
                result = MAV_RESULT_UNSUPPORTED;
                break;
        }

        sendCommandAck(cmd.command, result);
    }

    // =========================================================
    //  MISSION HANDLERS
    // =========================================================
    void handleMissionRequestList(const mavlink_message_t& msg) {
        mavlink_message_t reply;
        mavlink_msg_mission_count_pack(
            SYSID, COMPID, &reply,
            msg.sysid, msg.compid,
            g_wp_sum,
            MAV_MISSION_TYPE_MISSION);
        sendAll(reply);
        Serial.printf("[MAV] Mission count sent: %d\n", g_wp_sum);
    }

    void handleMissionRequest(const mavlink_message_t& msg) {
        mavlink_mission_request_t req;
        mavlink_msg_mission_request_decode(&msg, &req);
        if (req.seq >= g_wp_sum) return;
        sendWaypoint(msg.sysid, msg.compid, req.seq);
    }

    void handleMissionRequestInt(const mavlink_message_t& msg) {
        mavlink_mission_request_int_t req;
        mavlink_msg_mission_request_int_decode(&msg, &req);
        if (req.seq >= g_wp_sum) return;
        sendWaypoint(msg.sysid, msg.compid, req.seq);
    }

    void sendWaypoint(uint8_t sysid, uint8_t compid, uint16_t seq) {
        mavlink_message_t reply;
        mavlink_msg_mission_item_int_pack(
            SYSID, COMPID, &reply,
            sysid, compid,
            seq,
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV_CMD_NAV_WAYPOINT,
            0,    // current
            1,    // autocontinue
            0, 0, 0, 0,
            g_waypoints[seq].lat,
            g_waypoints[seq].lng,
            g_waypoints[seq].alt / 100.0f,
            MAV_MISSION_TYPE_MISSION);
        sendAll(reply);
        Serial.printf("[MAV] Sent WP %d\n", seq);

        // ACK setelah item terakhir
        if (seq == g_wp_sum - 1) {
            mavlink_message_t ack;
            mavlink_msg_mission_ack_pack(
                SYSID, COMPID, &ack,
                sysid, compid,
                MAV_MISSION_ACCEPTED,
                MAV_MISSION_TYPE_MISSION);
            sendAll(ack);
        }
    }

    void handleMissionCount(const mavlink_message_t& msg) {
        mavlink_mission_count_t mc;
        mavlink_msg_mission_count_decode(&msg, &mc);

        if (mc.count > MAX_WAYPOINTS) {
            missionAck(msg.sysid, msg.compid, MAV_MISSION_ERROR);
            sendStatusText(MAV_SEVERITY_WARNING, "Mission terlalu besar");
            return;
        }

        _expectedCount = mc.count;
        _uploadSeq     = 0;
        _upSysId       = msg.sysid;
        _upCompId      = msg.compid;
        _missionState  = MissionState::RECEIVING;

        Serial.printf("[MAV] Receiving %d waypoints\n", _expectedCount);
        requestNextItem();
    }

    void handleMissionItemInt(const mavlink_message_t& msg) {
        if (_missionState != MissionState::RECEIVING) return;
        mavlink_mission_item_int_t item;
        mavlink_msg_mission_item_int_decode(&msg, &item);

        if (!checkSeq(msg, item.seq)) return;

        g_waypoints[item.seq].lat = item.x;
        g_waypoints[item.seq].lng = item.y;
        g_waypoints[item.seq].alt = (int32_t)(item.z * 100.0f);

        Serial.printf("[MAV] WP %d: lat=%d lon=%d alt=%.1fm\n",
                      item.seq, item.x, item.y, item.z);
        advanceUpload(msg.sysid, msg.compid);
    }

    void handleMissionItem(const mavlink_message_t& msg) {
        if (_missionState != MissionState::RECEIVING) return;
        mavlink_mission_item_t item;
        mavlink_msg_mission_item_decode(&msg, &item);

        if (!checkSeq(msg, item.seq)) return;

        g_waypoints[item.seq].lat = (int32_t)(item.x * 1e7f);
        g_waypoints[item.seq].lng = (int32_t)(item.y * 1e7f);
        g_waypoints[item.seq].alt = (int32_t)(item.z * 100.0f);

        Serial.printf("[MAV] WP %d: lat=%.7f lon=%.7f alt=%.1fm\n",
                      item.seq, (double)item.x, (double)item.y, item.z);
        advanceUpload(msg.sysid, msg.compid);
    }

    void handleMissionClearAll(const mavlink_message_t& msg) {
        g_wp_sum = 0;
        missionAck(msg.sysid, msg.compid, MAV_MISSION_ACCEPTED);
        sendStatusText(MAV_SEVERITY_INFO, "Mission cleared");
        Serial.println("[MAV] Mission cleared");
    }

    // ── Upload helpers ────────────────────────────────────────
    bool checkSeq(const mavlink_message_t& msg, uint16_t seq) {
        if (seq != _uploadSeq) {
            missionAck(msg.sysid, msg.compid, MAV_MISSION_INVALID_SEQUENCE);
            _missionState = MissionState::IDLE;
            Serial.printf("[MAV] Bad seq: got %d expected %d\n",
                          seq, _uploadSeq);
            return false;
        }
        return true;
    }

    void advanceUpload(uint8_t sysid, uint8_t compid) {
        _uploadSeq++;
        if (_uploadSeq < _expectedCount) {
            requestNextItem();
        } else {
            g_wp_sum      = _expectedCount;
            _missionState = MissionState::IDLE;
            missionAck(sysid, compid, MAV_MISSION_ACCEPTED);
            sendStatusText(MAV_SEVERITY_INFO, "Mission OK");
            Serial.printf("[MAV] Mission complete: %d WP\n", g_wp_sum);
            // TODO: simpan ke NVS/EEPROM jika perlu persistensi
        }
    }

    void requestNextItem() {
        mavlink_message_t req;
        mavlink_msg_mission_request_int_pack(
            SYSID, COMPID, &req,
            _upSysId, _upCompId,
            _uploadSeq,
            MAV_MISSION_TYPE_MISSION);
        sendAll(req);
    }

    void missionAck(uint8_t sysid, uint8_t compid, uint8_t result) {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            SYSID, COMPID, &ack,
            sysid, compid,
            result,
            MAV_MISSION_TYPE_MISSION);
        sendAll(ack);
    }

    // =========================================================
    //  PARAM SYSTEM
    //  Nilai param dibaca/ditulis ke FixedWingConfig via
    //  getParamValue() / setParamValue() di bawah.
    // =========================================================
    void handleParamRequestList(const mavlink_message_t& msg) {
        _paramStreamIdx = 0;
        Serial.printf("[PARAM] RequestList → stream %d params\n", PARAM_COUNT);
    }

    void handleParamRequestRead(const mavlink_message_t& msg) {
        mavlink_param_request_read_t req;
        mavlink_msg_param_request_read_decode(&msg, &req);

        int idx = -1;
        if (req.param_index >= 0 && req.param_index < PARAM_COUNT) {
            idx = req.param_index;
        } else {
            idx = MavUtil::findParam(req.param_id);
        }

        if (idx >= 0) {
            sendParamValue((uint16_t)idx);
        } else {
            sendStatusText(MAV_SEVERITY_WARNING, "PARAM NOT FOUND");
        }
    }

    void handleParamSet(const mavlink_message_t& msg) {
        mavlink_param_set_t ps;
        mavlink_msg_param_set_decode(&msg, &ps);

        if (ps.target_system != SYSID && ps.target_system != 0) return;

        int idx = MavUtil::findParam(ps.param_id);
        if (idx < 0) {
            sendStatusText(MAV_SEVERITY_WARNING, "PARAM SET: NOT FOUND");
            return;
        }

        float val = MavUtil::clampf(ps.param_value,
                                    PARAM_TABLE[idx].minVal,
                                    PARAM_TABLE[idx].maxVal);

        setParamValue((uint16_t)idx, val);
        sendParamValue((uint16_t)idx);  // echo balik wajib

        char buf[50];
        snprintf(buf, sizeof(buf), "SET %s=%.4f", PARAM_TABLE[idx].id, val);
        sendStatusText(MAV_SEVERITY_INFO, buf);
        Serial.printf("[PARAM] %s\n", buf);
    }

    void pumpParamStream() {
        if (_paramStreamIdx == UINT16_MAX) return;
        if (_paramStreamIdx >= PARAM_COUNT) {
            _paramStreamIdx = UINT16_MAX;
            return;
        }
        sendParamValue(_paramStreamIdx++);
    }

    void sendParamValue(uint16_t idx) {
        if (idx >= PARAM_COUNT) return;
        mavlink_message_t msg;
        mavlink_msg_param_value_pack(
            SYSID, COMPID, &msg,
            PARAM_TABLE[idx].id,
            getParamValue(idx),
            MAV_PARAM_TYPE_REAL32,
            PARAM_COUNT,
            idx);
        sendAll(msg);
    }
    float getParamValue(uint16_t idx) const {
    return paramsManager.getParamValue(idx);
    }

    void setParamValue(uint16_t idx, float val) {
        paramsManager.setParamValue(idx, val);
    }
};

// ============================================================
//  GLOBAL INSTANCE
// ============================================================
static MavlinkHandler mavHandler;

// ============================================================
//  FREERTOS TASK
//  Jalankan dari setup():
//    xTaskCreatePinnedToCore(
//        mavlinkTask, "MavTask",
//        4096, NULL, 2, NULL, 1
//    );
// ============================================================
inline void mavlinkTask(void* pvParameters) {
    mavHandler.begin();

    const TickType_t period    = pdMS_TO_TICKS(10);  // 100 Hz parse loop
    TickType_t       lastWake  = xTaskGetTickCount();

    for (;;) {
        mavHandler.update();
        vTaskDelayUntil(&lastWake, period);
    }
}

#endif  // MAV_H