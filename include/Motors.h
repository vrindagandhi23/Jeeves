#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

// struct to hold motor functions
struct Motors{
    int ENA, IN1, IN2, IN3, IN4, ENB, STBY;
    Motors(int ENA_, int IN1_, int IN2_, int IN3_, int IN4_, int ENB_, int STBY_) : ENA(ENA_), IN1(IN1_), IN2(IN2_), IN3(IN3_), IN4(IN4_), ENB(ENB_), STBY(STBY_){  pinMode(ENA, OUTPUT);
        pinMode(ENB, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        pinMode(STBY, OUTPUT);
        digitalWrite(STBY, HIGH);
    }
    void forward(int spd);
    void backward(int spd);
    void leftTurn(int spd);
    void rightTurn(int spd);
    void stopMotors();
};

#endif