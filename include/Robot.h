#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include "Motors.h"
#include "Winch.h"

enum class RobotState {
  CALIBRATING,
  TURNING,
  PURSUING,
  DONE
};

class Robot
{
private:
  float x;
  float y;
  float goalX;
  float goalY;

  // State machine
  RobotState state;
  float heading;
  bool calibStarted;
  float calibStartX, calibStartY;
  uint32_t calibStartTime;

  Motors motors;
  Winch winch;

public:
  Robot(float x, float y, int mENA_, int mIN1_, int mIN2_, int mIN3_, int mIN4_, int mENB_, int mSTBY_, int wIN1_, int wIN2_, int wIN3_, int wIN4_);

  void updatePosition(float x_, float y_);

  // Set goal position
  void setGoal(float gx, float gy);
  void getGoal(float &gx, float &gy) const;

  // Heading (world radians, same convention as atan2(dy,dx) to goal).
  void setHeading(float rad);
  float getHeading() const;
  void initHeadingFromBearingToGoal();
  float bearingToGoal() const;
  void adjustHeading(float deltaRad);

  void motorsStop();
  void motorsForward(int duty);
  void motorsLeftTurn(int duty);
  void motorsRightTurn(int duty);

  // Main control loop — call every tick
  void pursueTarget();

  // Returns robot position by reference
  void GetPosition(float &x, float &y) const;

  // Returns raw anchor distance
  float GetRawDistance() const;

  // Returns filtered anchor distance
  float GetDistance() const;

  // Returns whether a filtered distance has been initialized
  bool GetDistInitialize() const;
};

#endif