#pragma once
#ifndef ACTUATOR_H
#define ACTUATOR_H
#include <Arduino.h>
#include <ESP32Servo.h>

struct ServoPinCofig{
    uint8_t ServoPin1 = 25;
    uint8_t ServoPin2 = 26;
    uint8_t ServoPin3 = 27;
    uint8_t ServoPin4 = 14;
    uint8_t ServoPin5 = 13;
};

struct ServoPWM{
    uint16_t ServoPWM1 = 1000;
    uint16_t ServoPWM2 = 1000;
    uint16_t ServoPWM3 = 1000;
    uint16_t ServoPWM4 = 1000;
    uint16_t ServoPWM5 = 1000;
};

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

class ServoController{
public:
    void attachServos(ServoPinCofig config){
        servo1.attach(config.ServoPin1);
        servo2.attach(config.ServoPin2);
        servo3.attach(config.ServoPin3);
        servo4.attach(config.ServoPin4);
        servo5.attach(config.ServoPin5);
    }

    void writeServos(ServoPWM pwm){
        servo1.writeMicroseconds(pwm.ServoPWM1);
        servo2.writeMicroseconds(pwm.ServoPWM2);
        servo3.writeMicroseconds(pwm.ServoPWM3);
        servo4.writeMicroseconds(pwm.ServoPWM4);
        servo5.writeMicroseconds(pwm.ServoPWM5);
    }

    void init_actuators(ServoPinCofig config){
        attachServos(config);
        ServoPWM neutral_pwm;
        writeServos(neutral_pwm);
    }
};

ServoController servoController;
ServoPWM PWM_Servo;
ServoPinCofig       servoConfig;

#endif