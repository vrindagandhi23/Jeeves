#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

// struct to hold motor functions
struct Motors{
    int ENA, IN1, IN2, IN3, IN4, ENB, STBY;
    // ESP32 PWM (LEDC) configuration
    static constexpr uint32_t PWM_FREQ_HZ = 20000; // quiet-ish
    static constexpr uint8_t PWM_RES_BITS = 8;     // duty 0..255
    static constexpr int PWM_CH_A = 0;
    static constexpr int PWM_CH_B = 1;

    Motors(int ENA_, int IN1_, int IN2_, int IN3_, int IN4_, int ENB_, int STBY_)
      : ENA(ENA_), IN1(IN1_), IN2(IN2_), IN3(IN3_), IN4(IN4_), ENB(ENB_), STBY(STBY_) {
        pinMode(ENA, OUTPUT);
        pinMode(ENB, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(STBY, OUTPUT);

        // Explicitly attach PWM to ENA/ENB so duty changes actually affect the pins.
        ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttachPin(ENA, PWM_CH_A);
        ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_RES_BITS);
        ledcAttachPin(ENB, PWM_CH_B);

        // Start disabled until first motion command
        digitalWrite(STBY, LOW);
        ledcWrite(PWM_CH_A, 0);
        ledcWrite(PWM_CH_B, 0);
    }
    void forward(int spd);
    void backward(int spd);
    void leftTurn(int spd);
    void rightTurn(int spd);
    void stopMotors();
};

#endif