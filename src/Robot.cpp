#include "Robot.h"
#include <math.h>

Robot::Robot(float x_, float y_, int mENA_, int mIN1_, int mIN2_, int mIN3_, int mIN4_, int mENB_, int mSTBY_, int wIN1_, int wIN2_, int wIN3_, int wIN4_)
  : x(x_), y(y_), goalX(0.0f), goalY(0.0f),
    motors(mENA_, mIN1_, mIN2_, mIN3_, mIN4_, mENB_, mSTBY_),
    winch(wIN1_, wIN2_, wIN3_, wIN4_),
    state(RobotState::CALIBRATING),
    heading(0.0f),
    calibStarted(false),
    calibStartX(0.0f), calibStartY(0.0f),
    calibStartTime(0)
{
}

// ---------------------- Update Position API ------------------------
void Robot::updatePosition(float x_, float y_) {
  x = x_;
  y = y_;
}

void Robot::GetPosition(float &x_, float &y_) const {
  x_ = this->x;
  y_ = this->y;
}

// ---------------------- Control Loop -------------------------------
void Robot::pursueTarget() {

  switch (state) {

    // ---------- Phase 1: drive forward briefly to derive heading ----------
    case RobotState::CALIBRATING: {
      if (!calibStarted) {
        calibStartX = x;
        calibStartY = y;
        calibStartTime = millis();
        calibStarted = true;
        Serial.println("Calibrating: driving forward to derive heading...");
      }

      motors.forward(0.05f);

      if (millis() - calibStartTime > 1000) {
        float dx = x - calibStartX;
        float dy = y - calibStartY;
        float moved = sqrt(dx*dx + dy*dy);

        if (moved > 1.0f) {
          heading = atan2(dy, dx);
          Serial.print("Heading derived: ");
          Serial.println(heading);
          state = RobotState::TURNING;
        } else {
          // Didn't move enough to trust — retry
          Serial.println("Calibration failed (not enough movement), retrying...");
          calibStarted = false;
        }
      }
      break;
    }

    // ---------- Phase 2: turn to face goal ----------
    case RobotState::TURNING: {
      float dx = goalX - x;
      float dy = goalY - y;
      float targetAngle = atan2(dy, dx);
      float angleError = targetAngle - heading;

      // Normalize to [-PI, PI]
      while (angleError >  PI) angleError -= 2 * PI;
      while (angleError < -PI) angleError += 2 * PI;

      Serial.print("Turning — angle error: ");
      Serial.println(angleError);

      if (abs(angleError) < 0.25f) {
        motors.stopMotors();
        Serial.println("Aligned. Pursuing target.");
        state = RobotState::PURSUING;
      } else if (angleError > 0) {
        motors.rightTurn(200);
      } else {
        motors.leftTurn(200);
      }
      break;
    }

    // ---------- Phase 3: drive straight toward goal ----------
    case RobotState::PURSUING: {
      float dx = goalX - x;
      float dy = goalY - y;
      float dist = sqrt(dx*dx + dy*dy);

      Serial.print("Robot: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print("   Distance to target: ");
      Serial.println(dist);

      if (dist < 5.0f) {
        motors.stopMotors();
        Serial.println("Reached target.");
        state = RobotState::DONE;
        return;
      }

      motors.forward(0.05f);
      break;
    }

    // ---------- Done ----------
    case RobotState::DONE:
      motors.stopMotors();
      break;
  }
}