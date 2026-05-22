#pragma once
#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─── SBUS Protocol Constants ─────────────────────────────────────────────────
#define SBUS_PIN_RX       19
#define SBUS_BAUD         100000
#define SBUS_FRAME_SIZE   25
#define SBUS_HEADER       0x0F
#define SBUS_FOOTER       0x00
#define SBUS_CH_COUNT     16

#define CH_MIN            172
#define CH_MAX            1811
#define PWM_MIN           1000
#define PWM_MAX           2000

// ─── Channel Data ─────────────────────────────────────────────────────────────
struct Channel {
    uint16_t ch1  = 1000;
    uint16_t ch2  = 1000;
    uint16_t ch3  = 1000;
    uint16_t ch4  = 1000;
    uint16_t ch5  = 1000;
    uint16_t ch6  = 1000;
    uint16_t ch7  = 1000;
    uint16_t ch8  = 1000;
    uint16_t ch9  = 1000;
    uint16_t ch10 = 1000;
    uint16_t ch11 = 1000;
    uint16_t ch12 = 1000;
    uint16_t ch13 = 1000;
    uint16_t ch14 = 1000;
    uint16_t ch15 = 1000;
    uint16_t ch16 = 1000;
};

// ─── Radio Class ──────────────────────────────────────────────────────────────
class Radio {
public:
    Channel     channels;
    bool        signalLost   = true;
    bool        failsafe     = false;
    bool        ARMED        = false;

    // ── Init UART2 untuk SBUS (inverted, 8E2) ────────────────────────────────
    void begin() {
        // ESP32: Serial2 = UART2
        // SBUS = 100000 baud, Even parity, 2 stop bit, sinyal dibalik (SERIAL_8E2)
        // invert_logic = true  → ESP32 HardwareSerial mendukung inversi via config
        Serial2.begin(SBUS_BAUD,
                      SERIAL_8E2,   // 8 data bits, Even parity, 2 stop bits
                      SBUS_PIN_RX,  // RX pin 19
                      -1,           // TX tidak dipakai
                      true);        // invert = true (SBUS active-low)
    }

    // ── Update: baca & parse satu frame SBUS ─────────────────────────────────
    // Kembalikan true jika frame valid berhasil di-parse
    bool update() {
        // Kumpulkan byte hingga ketemu header 0x0F
        if (Serial2.available() < SBUS_FRAME_SIZE) return false;

        // Sinkronisasi: cari header
        while (Serial2.available() && Serial2.peek() != SBUS_HEADER) {
            Serial2.read();   // buang byte sampah
        }

        if (Serial2.available() < SBUS_FRAME_SIZE) return false;

        // Baca satu frame penuh
        uint8_t buf[SBUS_FRAME_SIZE];
        Serial2.readBytes(buf, SBUS_FRAME_SIZE);

        // Validasi header & footer
        if (buf[0] != SBUS_HEADER || buf[24] != SBUS_FOOTER) return false;

        // ── Decode 16 channel (masing-masing 11 bit) ─────────────────────────
        uint16_t raw[SBUS_CH_COUNT];
        raw[0]  = (buf[1]       | buf[2]  << 8)                 & 0x07FF;
        raw[1]  = (buf[2]  >> 3 | buf[3]  << 5)                 & 0x07FF;
        raw[2]  = (buf[3]  >> 6 | buf[4]  << 2  | buf[5] << 10) & 0x07FF;
        raw[3]  = (buf[5]  >> 1 | buf[6]  << 7)                 & 0x07FF;
        raw[4]  = (buf[6]  >> 4 | buf[7]  << 4)                 & 0x07FF;
        raw[5]  = (buf[7]  >> 7 | buf[8]  << 1  | buf[9] << 9)  & 0x07FF;
        raw[6]  = (buf[9]  >> 2 | buf[10] << 6)                 & 0x07FF;
        raw[7]  = (buf[10] >> 5 | buf[11] << 3)                 & 0x07FF;
        raw[8]  = (buf[12]      | buf[13] << 8)                 & 0x07FF;
        raw[9]  = (buf[13] >> 3 | buf[14] << 5)                 & 0x07FF;
        raw[10] = (buf[14] >> 6 | buf[15] << 2  | buf[16]<< 10) & 0x07FF;
        raw[11] = (buf[16] >> 1 | buf[17] << 7)                 & 0x07FF;
        raw[12] = (buf[17] >> 4 | buf[18] << 4)                 & 0x07FF;
        raw[13] = (buf[18] >> 7 | buf[19] << 1  | buf[20]<< 9)  & 0x07FF;
        raw[14] = (buf[20] >> 2 | buf[21] << 6)                 & 0x07FF;
        raw[15] = (buf[21] >> 5 | buf[22] << 3)                 & 0x07FF;

        // ── Flag byte (byte ke-23) ────────────────────────────────────────────
        signalLost = (buf[23] & (1 << 2)) != 0;   // bit2 = signal lost
        failsafe   = (buf[23] & (1 << 3)) != 0;   // bit3 = failsafe active

        // ── Map raw (172–1811) → PWM (1000–2000 µs) ──────────────────────────
        uint16_t* chArr[SBUS_CH_COUNT] = {
            &channels.ch1,  &channels.ch2,  &channels.ch3,  &channels.ch4,
            &channels.ch5,  &channels.ch6,  &channels.ch7,  &channels.ch8,
            &channels.ch9,  &channels.ch10, &channels.ch11, &channels.ch12,
            &channels.ch13, &channels.ch14, &channels.ch15, &channels.ch16
        };

        for (uint8_t i = 0; i < SBUS_CH_COUNT; i++) {
            *chArr[i] = mapChannel(raw[i]);
        }

        return true;
    }

private:
    // ── Map nilai SBUS mentah ke rentang PWM ─────────────────────────────────
    static uint16_t mapChannel(uint16_t raw) {
        uint16_t mapped = (uint16_t)map((long)raw,
                                        CH_MIN, CH_MAX,
                                        PWM_MIN, PWM_MAX);
        // Clamp agar tidak keluar batas
        return constrain(mapped, PWM_MIN, PWM_MAX);
    }
};

// ─── FreeRTOS Task ────────────────────────────────────────────────────────────
// Deklarasi objek global Radio — didefinisikan di .cpp / main agar tidak duplikat
Radio radio;

// Task untuk update radio secara periodik
// Jalankan dengan: xTaskCreatePinnedToCore(radioTask, "RadioTask", 2048, NULL, 5, NULL, 0);
inline void radioTask(void* pvParameters) {
    radio.begin();

    for (;;) {
        radio.update();          // parse frame SBUS terbaru
        vTaskDelay(pdMS_TO_TICKS(4)); // SBUS kirim frame tiap ~7ms, poll tiap 4ms
    }
}

#endif // RADIO_H