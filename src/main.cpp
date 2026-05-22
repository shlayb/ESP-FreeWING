#include <Arduino.h>
#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "../include/Sensor/imu.h"
#include "../include/Communication/radio.h"
#include "../include/Control/fixedwing.h"
#include "../include/Actuator/actuator.h"
#include "../include/Communication/TimingUtils.h"
#include "../include/Communication/params.h"
#include "../include/Communication/mavlink.h"

// ============================================================================
//  PIN ESP32
// ============================================================================

static constexpr int IMU_SDA_PIN = 21;
static constexpr int IMU_SCL_PIN = 22;

// ============================================================================
//  TASK HANDLE
// ============================================================================

TaskHandle_t taskImuHandle = nullptr;
TaskHandle_t taskPrintUsbHandle = nullptr;

// ============================================================================
//  HELPER STATUS
// ============================================================================

const char* statusToString(Mpu6050Status status) {
    switch (status) {
        case Mpu6050Status::OK:
            return "OK";

        case Mpu6050Status::I2C_ERROR:
            return "I2C_ERROR";

        case Mpu6050Status::WHO_AM_I_ERROR:
            return "WHO_AM_I_ERROR";

        case Mpu6050Status::NOT_INITIALIZED:
            return "NOT_INITIALIZED";

        default:
            return "UNKNOWN";
    }
}

// ============================================================================
//  TASK PRINT USB
// ============================================================================

void TaskPrintUsb(void* parameter) {
    Imu* imuPtr = static_cast<Imu*>(parameter);

    if (imuPtr == nullptr) {
        imuPtr = &imu;
    }

    const TickType_t printPeriod = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true) {
        ImuData data;

        if (imuPtr->readData(data)) {
            Serial.printf(
                "seq:%lu | t:%lu us | dt:%.6f s | "
                "acc[g] X:%.3f Y:%.3f Z:%.3f | "
                "gyro[dps] X:%.3f Y:%.3f Z:%.3f | "
                "angle[deg] roll:%.2f pitch:%.2f yaw:%.2f | "
                "temp:%.2f C | status:%s\n",

                static_cast<unsigned long>(data.sequence),
                static_cast<unsigned long>(data.timestamp_us),
                data.dt_s,

                data.ax_g,
                data.ay_g,
                data.az_g,

                data.gx_dps,
                data.gy_dps,
                data.gz_dps,

                data.roll_deg,
                data.pitch_deg,
                data.yaw_deg,

                data.temperature_c,
                statusToString(data.status)
            );
        } else {
            Serial.println("Gagal membaca data IMU: mutex timeout");
        }

        vTaskDelayUntil(&lastWakeTime, printPeriod);
    }
}

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("ESP32 MPU6050 RTOS START");
    Serial.println("Inisialisasi IMU...");

    Mpu6050Status imuStatus = imu.begin(
        Wire,
        IMU_SDA_PIN,
        IMU_SCL_PIN,
        MPU6050_FAST_I2C_CLOCK,
        true,
        1000,
        2
    );

    if (imuStatus != Mpu6050Status::OK) {
        Serial.print("IMU gagal inisialisasi. Status: ");
        Serial.println(statusToString(imuStatus));

        while (true) {
            delay(1000);
        }
    }

    servoController.init_actuators(ServoPinCofig());
    fixedWing.begin(ServoPinCofig());
    radio.begin();



    xTaskCreatePinnedToCore(
        TaskImu,
        "TaskImu",
        IMU_TASK_STACK_SIZE,
        &imu,
        IMU_TASK_PRIORITY,
        &taskImuHandle,
        1
    );

    // xTaskCreatePinnedToCore( TaskPrintUsb, "TaskPrintUsb", 4096, &imu, 1, &taskPrintUsbHandle, 0 );
    xTaskCreatePinnedToCore(
        fixedWingTask,
        "FixedWingTask",
        4096,
        &fixedWing,
        5,
        NULL,
        1
    );
    xTaskCreatePinnedToCore(
        radioTask,
        "RadioTask",
        4096,
        &radio,
        5,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        mavlinkTask,
        "MavTask",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    Serial.println("RTOS task aktif.");
}

// ============================================================================
//  LOOP
// ============================================================================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}