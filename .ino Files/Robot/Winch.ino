

// rn single stepping, if we need smoother/more torque we can half step

// 2048 step per rotation
const float stepAngle = 360.0 / 2048;

// pins need to be defined later
const byte StepperPins[] = {22, 1, 3, 21}; 
int step_number = 0;

void OneStep(bool dir){
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

// blocking code rn
void turnDegrees(int angle){
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

void setup(){
    Serial.begin(115200);
    for(int i = 0; i < 4; i++){
        pinMode(StepperPins[i], OUTPUT);
    }
}
void loop(){
    OneStep(true);
    Serial.println(step_number);
}