#include "Motors.h"

void Motors::forward(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void Motors::backward(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void Motors::leftTurn(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void Motors::rightTurn(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void Motors::stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


