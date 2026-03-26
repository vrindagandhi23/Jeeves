#include "Motors.h"

void Motors::forward(int spd) {
  digitalWrite(STBY, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  int duty = constrain(spd, 0, 255);
  ledcWrite(PWM_CH_A, duty);
  ledcWrite(PWM_CH_B, duty);
}

void Motors::backward(int spd) {
  digitalWrite(STBY, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  int duty = constrain(spd, 0, 255);
  ledcWrite(PWM_CH_A, duty);
  ledcWrite(PWM_CH_B, duty);
}

void Motors::leftTurn(int spd) {
  digitalWrite(STBY, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  int duty = constrain(spd, 0, 255);
  ledcWrite(PWM_CH_A, duty);
  ledcWrite(PWM_CH_B, duty);
}

void Motors::rightTurn(int spd) {
  digitalWrite(STBY, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  int duty = constrain(spd, 0, 255);
  ledcWrite(PWM_CH_A, duty);
  ledcWrite(PWM_CH_B, duty);
}

void Motors::stopMotors() {
  // Hard disable driver and remove PWM to guarantee stop.
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(STBY, LOW);
}


