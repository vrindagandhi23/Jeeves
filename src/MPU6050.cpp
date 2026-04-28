#include "MPU6050.h"

volatile bool MPU6050Sensor::interruptFlag = false;

void IRAM_ATTR MPU6050Sensor::dmpISR() {
    interruptFlag = true;
}

MPU6050Sensor::MPU6050Sensor(int interruptPin_)
    : interruptPin(interruptPin_), dmpReady(false) {}

void MPU6050Sensor::begin() {
    Wire.begin(21, 22);
    Wire.setClock(400000);

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        return;
    }

    uint8_t devStatus = mpu.dmpInitialize();

    // offsets (tune later)
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        mpu.setRate(99);
        mpu.setDMPEnabled(true);

        pinMode(interruptPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(interruptPin), dmpISR, RISING);

        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;

        Serial.println("MPU6050 DMP ready");
    } else {
        Serial.print("DMP init failed: ");
        Serial.println(devStatus);
    }
}

void MPU6050Sensor::update() {
    if (!dmpReady) return;

    // Only proceed if interrupt triggered OR data already waiting
    if (!interruptFlag && mpu.getFIFOCount() < packetSize) return;

    interruptFlag = false;

    uint16_t fifoCount = mpu.getFIFOCount();

    // Handle overflow
    if (fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
        return;
    }

    // Drain FIFO completely (THIS is the key fix)
    while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
    }

    // Now fifoBuffer contains the latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
}

float MPU6050Sensor::getYaw() {
    return ypr[0]; // radians
}

float MPU6050Sensor::getPitch() {
    return ypr[1];
}

float MPU6050Sensor::getRoll() {
    return ypr[2];
}

bool MPU6050Sensor::isReady() {
    return dmpReady;
}