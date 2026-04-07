#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include "Motors.h"
#include "Winch.h"

class Robot
{
private:
  float x;
  float y;
  float goalX;
  float goalY;
  Motors motors;
  Winch winch;
public:
    // anchor initialization
  Robot(float x, float y, int mENA_, int mIN1_, int mIN2_, int mIN3_, int mIN4_, int mENB_, int mSTBY_, int wIN1_, int wIN2_, int wIN3_, int wIN4_);
  // 
  void updatePosition(float x_, float y_);
  // pursue goalX, goalY
  void pursueTarget();
  // pursue x, y
  void pursueTarget(float goalX, float goalY);
  
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