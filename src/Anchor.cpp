#include "Anchor.h"

Anchor::Anchor(float x, float y, String id)
{
  this->x = x;
  this->y = y;
  this->id = id;
  distance = 0;
}

bool Anchor::PollDistance(HardwareSerial RYUW){
  // Send command to current TAG
  RYUW.println("AT+ANCHOR_SEND=" + id + ",1,A");

  // Wait up to 500 ms for a response for this tag
  bool gotDistance = false;
  TickType_t start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(500)){
    if (RYUW.available()) {
      String response = RYUW.readStringUntil('\n');
      response.trim();

      if (response.startsWith("+ANCHOR_RCV")) {
        int lastComma = response.lastIndexOf(',');
        if (lastComma != -1) {
          String distanceStr = response.substring(lastComma + 1);
          distanceStr.trim();  // "9 cm" → "9 cm"

          // Remove " cm" if it exists
          distanceStr.replace("cm", "");
          distanceStr.trim();  // now "9"

          // Convert to integer
          int d = distanceStr.toInt();
          distance = (float)d;
          filter.filter(distance);
          return true;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

  void Anchor::GetPosition(float &x, float &y) const{
    x = this->x;
    y = this->y;
  }

  float Anchor::GetRawDistance() const{
    return distance;
  }

  float Anchor::GetDistance() const{
    return filter.dFilt;
  }
  bool Anchor::GetDistInitialize() const{
    return filter.distInit;
  }
