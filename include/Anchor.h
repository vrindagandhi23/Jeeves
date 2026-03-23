#ifndef ANCHOR_H
#define ANCHOR_H
#include <Arduino.h>
#include <vector>
#include "Filter.h"


class Anchor
{
private:
  float x;
  float y;
  String id;
  float distance;
  DistanceFilter filter;

public:
    // anchor initialization
  Anchor(float x, float y, String id);

  /*
  Takes in the serial port that the tag is connected to on the ESP32
  and polls the corresponding anchor for its distance. 
  True if successful
  False if fails
  Raw reading
  */
  bool PollDistance(HardwareSerial RYUW);
// returns anchor position by reference
  void GetPosition(float &x, float &y) const;
// returns raw anchor distance
  float GetRawDistance() const;
// returns filtered anchor distance
  float GetDistance() const;
// returns whether a filtered distance has been initialized
  bool GetDistInitialize() const;
};

#endif