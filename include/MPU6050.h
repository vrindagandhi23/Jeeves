#pragma once

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

class MPU6050Sensor {
public:
    MPU6050Sensor(int interruptPin);

    void begin();
    void update();

    float getYaw();   // radians
    float getPitch();
    float getRoll();

    bool isReady();

private:
    MPU6050 mpu;
    int interruptPin;

    bool dmpReady;
    uint16_t packetSize;
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    static void IRAM_ATTR dmpISR();
    static volatile bool interruptFlag;
};