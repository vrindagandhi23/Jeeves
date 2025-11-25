#include <ESP32Servo.h>
#include <math.h>

// --------------------- Motor Pin Mapping (TB6612FNG) --------------------------
#define ENA 14
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENB 12
#define STBY 32

// --------------------- Servo Pin Mapping -------------------------------------
Servo myservo;
int servoPin = 18;
int pos = 90;

// --------------------- Motor Functions ---------------------------------------
void forward(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void backward(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void leftTurn(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void rightTurn(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// --------------------- Demo Helpers ------------------------------------------
void demoMotors() {
  Serial.println("FORWARD");
  forward(200);
  delay(1000);
  stopMotors();
  delay(300);

  Serial.println("BACKWARD");
  backward(200);
  delay(1000);
  stopMotors();
  delay(300);

  Serial.println("RIGHT TURN");
  rightTurn(200);
  delay(1000);
  stopMotors();
  delay(300);

  Serial.println("LEFT TURN");
  leftTurn(200);
  delay(1000);
  stopMotors();
  delay(300);
}

void demoServo() {
  // ensure starting at 180 every cycle
  myservo.write(90);
  delay(300);

  Serial.println("Servo: 90 -> 0");
  for (pos = 90; pos >= 0; pos--) {
    myservo.write(pos);
    delay(15);
  }

  Serial.println("Servo: 0 -> 90");
  for (pos = 0; pos <= 90; pos++) {
    myservo.write(pos);
    delay(15);
  }

  delay(500);
}

// --------------------- Setup -------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Servo setup (your exact style)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400);

  // Start servo at 180
  myservo.write(90);
  delay(500);

  Serial.println("Combined motor + servo demo running (ESP32 core 3.x compatible).");
}

// --------------------- Main Loop --------------------------------------------
void loop() {
  demoMotors();
  demoServo();
  Serial.println("---- cycle complete ----\n");
  delay(1000);
}
