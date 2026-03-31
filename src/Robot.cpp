#include "Robot.h"

Robot::Robot(float x_, float y_, int ENA_, int IN1_, int IN2_, int IN3_, int IN4_, int ENB_, int STBY_) : x(x_), y(y_), goalX(100.0f), goalY(50.0f), motors(ENA_, IN1_, IN2_, IN3_, IN4_, ENB_, STBY_)
{

}

// ---------------------- Update Position API ------------------------
void Robot::updatePosition(float x_, float y_) {
  x = x_;
  y = y_;
}

// ---------------------- Control Loop -------------------------------
void Robot::pursueTarget() {
  float dx = goalX - x;
  float dy = goalY - y;
  float dist = sqrt(dx*dx + dy*dy);

  Serial.print("Robot: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("   Distance to target: ");
  Serial.println(dist);

  if (dist < 5.0) {
    motors.stopMotors();
    Serial.println("Reached target.");
    return;
  }

  // Angle from robot → target (radians)
  float targetAngle = atan2(dy, dx);

  // Assume robot always faces +X (0 radians)
  float robotHeading = 0;

  float angleError = targetAngle - robotHeading;

  // Normalize to [-PI, PI]
  while (angleError > PI)  angleError -= 2*PI;
  while (angleError < -PI) angleError += 2*PI;

  // --------------------- Turning Controller ------------------------
  float turnThreshold = 0.25;  // radians (~14°)
  // Scale all PWM speeds down (e.g. 0.1 = 10% speed)
  const float SPEED_SCALE = 0.05f; // ~5% speed (tune if needed)

  if (angleError > turnThreshold) {
    Serial.println("Turning right toward target");
    motors.rightTurn(200);
    return;
  }

  if (angleError < -turnThreshold) {
    Serial.println("Turning left toward target");
    motors.leftTurn(200);
    return;
  }

  // --------------------- Forward P-controller ----------------------
  //float Kp = 3.0;
  float spd = 0.05;

  Serial.print("Moving forward @ ");
  Serial.println(spd);

  motors.forward(spd);
}

  void Robot::GetPosition(float &x, float &y) const{
    x = this->x;
    y = this->y;
  }