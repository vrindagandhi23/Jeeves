#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include "Motors.h"


class Robot
{
private:
  float x;
  float y;
  float goalX;
  float goalY;
  Motors motors;
public:
    // anchor initialization
  Robot(float x, float y, int ENA_, int IN1_, int IN2_, int IN3_, int IN4_, int ENB_, int STBY_);
  
  void updatePosition(float x_, float y_);
  void pursueTarget();
// returns robot position by reference
  void GetPosition(float &x, float &y) const;
// returns raw anchor distance
  float GetRawDistance() const;
// returns filtered anchor distance
  float GetDistance() const;
// returns whether a filtered distance has been initialized
  bool GetDistInitialize() const;
};

#endif