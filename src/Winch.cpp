#include "Winch.h"

Winch::Winch(int IN1, int IN2, int IN3, int IN4){
    StepperPins[0] = IN1;
    StepperPins[1] = IN2;
    StepperPins[2] = IN3;
    StepperPins[3] = IN4;
}

void Winch::OneStep(bool dir){
    for(int i = 0; i < 4; i++){
        if(i == step_number){
            digitalWrite(StepperPins[i], HIGH);
        }
        else{
            digitalWrite(StepperPins[i], LOW);
        }
    }
    
    if(dir){
        step_number = (step_number + 1) % 4;
    }
    else{
        step_number = (step_number + 3) % 4;
    }
}

void Winch::turnDegrees(int angle){
    int steps = abs(angle) / stepAngle;
    if(angle > 0){
        for(int i = 0; i < steps; i++){
            OneStep(true);
            delay(2);
        }
    }
    else if(angle < 0){
        for(int i = 0; i < steps; i++){
            OneStep(false);
            delay(2);
        }
    }
}


