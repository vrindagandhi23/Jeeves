#ifndef TRIANGULATION_TEST_H
#define TRIANGULATION_TEST_H
#include <Arduino.h>

// 2048 step per rotation
const float stepAngle = 360.0 / 2048;

class Winch{
    private:
        int StepperPins[4];
        int step_number;
        void OneStep(bool dir);
    public:
        Winch(int IN1, int IN2, int IN3, int IN4);
        void turnDegrees(int angle);
        int getAngle();
};

#endif