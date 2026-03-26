#include "Anchor.h"
#include <cctype>
#include <cstring>

Anchor::Anchor(float x, float y, String id)
{
  this->x = x;
  this->y = y;
  this->id = id;
  distance = 0;
}

// Parse a line like "+ANCHOR_RCV=...,<distance>cm" and extract the distance.
static bool parseDistanceFromLine(const char* line, float& outDistanceCm) {
  if (strncmp(line, "+ANCHOR_RCV", 11) != 0) return false;
  const char* lastComma = strrchr(line, ',');
  if (!lastComma) return false;

  const char* p = lastComma + 1;
  while (*p == ' ' || *p == '\t') p++;

  int val = 0;
  bool any = false;
  while (*p && isdigit((unsigned char)*p)) {
    val = val * 10 + (*p - '0');
    p++;
    any = true;
  }

  if (!any) return false;
  outDistanceCm = (float)val;
  return true;
}

bool Anchor::PollDistance(HardwareSerial& RYUW){
  // Send command to current TAG (no dynamic String concatenation)
  char cmd[96];
  snprintf(cmd, sizeof(cmd), "AT+ANCHOR_SEND=%s,1,A", id.c_str());
  RYUW.println(cmd);

  // Wait up to 500 ms for a response for this tag
  TickType_t start = xTaskGetTickCount();
  char lineBuf[96];
  size_t lineLen = 0;
  lineBuf[0] = '\0';

  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(500)){
    while (RYUW.available()) {
      char c = (char)RYUW.read();

      if (c == '\r') continue;

      if (c == '\n') {
        lineBuf[lineLen] = '\0';
        float dRaw = 0.0f;
        if (parseDistanceFromLine(lineBuf, dRaw)) {
          distance = dRaw;
          filter.filter(distance);
          return true;
        }

        // reset for the next line
        lineLen = 0;
        lineBuf[0] = '\0';
      } else {
        if (lineLen < (sizeof(lineBuf) - 1)) {
          lineBuf[lineLen++] = c;
        } else {
          // Overflow; reset to keep parsing bounded.
          lineLen = 0;
          lineBuf[0] = '\0';
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
