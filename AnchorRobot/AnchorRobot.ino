// =============================================================================
// Bed-Making Robot: Anchor with UWB Triangulation + Motor Control
// FreeRTOS architecture: Task_UWB → Position Queue → Task_Motor
// Interrupts used only for timing/notifications; heavy work in tasks.
// =============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ----------------------------- UWB (Serial) ----------------------------------
#define TXD2 17
#define RXD2 16
#define RYUW_NRST 4
HardwareSerial RYUW(2);

// ----------------------------- Motor pins (TB6612FNG) ------------------------
#define ENA 10
#define IN1 9
#define IN2 13
#define IN3 12
#define IN4 14
#define ENB 27
#define STBY 32 // not used

// SERVO PINS 
/* wheels in the back
#define SERVO_PINL 7
#define SERVO_PINR 8

*/

// ----------------------------- Position queue (UWB → Motor) ------------------
typedef struct {
  float x;
  float y;
  uint8_t valid;  // 1 = fresh position, 0 = no fix yet
} PositionMessage_t;

#define POSITION_QUEUE_LEN  1
static QueueHandle_t positionQueue = NULL;

// ----------------------------- Robot state (motor task) ----------------------
static float robotX = 0;
static float robotY = 0;
static float goalX = 100.0f;
static float goalY = 50.0f;

// ----------------------------- Triangulation (from Anchor/Triangulation.ino) -
struct Anchor {
  float x;
  float y;
};

static Anchor anchors[4];
static float distances[4];
static const int NUM_TAGS = 4;

static bool triangulate(
    const Anchor* anchors,
    const float* distances,
    int n,
    float& outX,
    float& outY)
{
  if (n < 3) return false;

  float x1 = anchors[0].x;
  float y1 = anchors[0].y;
  float d1 = distances[0];

  float ATA[2][2] = {{0, 0}, {0, 0}};
  float ATb[2] = {0, 0};

  for (int i = 1; i < n; i++) {
    float xi = anchors[i].x;
    float yi = anchors[i].y;
    float di = distances[i];
    float ax = xi - x1;
    float ay = yi - y1;
    float bi = 0.5f * (
        (d1 * d1 - di * di) +
        (xi * xi - x1 * x1) +
        (yi * yi - y1 * y1));

    ATA[0][0] += ax * ax;
    ATA[0][1] += ax * ay;
    ATA[1][0] += ay * ax;
    ATA[1][1] += ay * ay;
    ATb[0] += ax * bi;
    ATb[1] += ay * bi;
  }

  float det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
  if (fabsf(det) < 1e-6f) return false;

  float inv00 =  ATA[1][1] / det;
  float inv01 = -ATA[0][1] / det;
  float inv10 = -ATA[1][0] / det;
  float inv11 =  ATA[0][0] / det;

  outX = inv00 * ATb[0] + inv01 * ATb[1];
  outY = inv10 * ATb[0] + inv11 * ATb[1];
  return true;
}

// ----------------------------- Motor API (from Robot/DriveRobot.ino) ---------
static void forward(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

static void backward(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

static void leftTurn(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

static void rightTurn(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

static void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

static void pursueTarget(void) {
  float dx = goalX - robotX;
  float dy = goalY - robotY;
  float dist = sqrtf(dx * dx + dy * dy);

  if (dist < 5.0f) {
    stopMotors();
    return;
  }

  float targetAngle = atan2f(dy, dx);
  float robotHeading = 0;
  float angleError = targetAngle - robotHeading;

  while (angleError > PI)  angleError -= 2.0f * PI;
  while (angleError < -PI) angleError += 2.0f * PI;

  const float turnThreshold = 0.25f;

  if (angleError > turnThreshold) {
    rightTurn(200);
    return;
  }
  if (angleError < -turnThreshold) {
    leftTurn(200);
    return;
  }

  const float Kp = 3.0f;
  int spd = (int)constrain(dist * Kp, 100.0f, 255.0f);
  forward(spd);
}

// ----------------------------- Task: UWB + Triangulation ---------------------
// Lower priority; runs triangulation and sends latest position to queue.
// Interrupts (if added later): use only to set a flag or give a semaphore;
// this task does all UWB read + triangulation so ISRs stay short.
static void taskUWB(void* pvParameters) {
  (void)pvParameters;
  PositionMessage_t msg = { .x = 0, .y = 0, .valid = 0 };

  for (;;) {
    for (int i = 0; i < NUM_TAGS; i++) {
      RYUW.println("AT+ANCHOR_SEND=TAG" + String(i + 1) + ",1,A");

      bool gotDistance = false;
      TickType_t start = xTaskGetTickCount();
      while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(500)) {
        if (RYUW.available()) {
          String response = RYUW.readStringUntil('\n');
          response.trim();

          if (response.startsWith("+ANCHOR_RCV")) {
            int lastComma = response.lastIndexOf(',');
            if (lastComma != -1) {
              String distanceStr = response.substring(lastComma + 1);
              distanceStr.replace("cm", "");
              distanceStr.trim();
              distances[i] = (float)distanceStr.toInt();
              gotDistance = true;
              break;
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      if (!gotDistance) {
        distances[i] = 0.0f;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (triangulate(anchors, distances, NUM_TAGS, msg.x, msg.y)) {
      msg.valid = 1;
      xQueueOverwrite(positionQueue, &msg);
    } else {
      msg.valid = 0;
      xQueueOverwrite(positionQueue, &msg);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ----------------------------- Task: Motor control ---------------------------
// Highest priority; fixed rate; reads latest position and runs pursuit.
static void taskMotor(void* pvParameters) {
  (void)pvParameters;
  const TickType_t period = pdMS_TO_TICKS(50);
  PositionMessage_t msg = { .x = 0, .y = 0, .valid = 0 };

  for (;;) {
    if (xQueueReceive(positionQueue, &msg, 0) == pdTRUE && msg.valid) {
      robotX = msg.x;
      robotY = msg.y;
    }

    pursueTarget();
    vTaskDelay(period);
  }
}

// ----------------------------- setup -----------------------------------------
void setup() {
  Serial.begin(115200);
  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, HIGH);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Tag positions (bed corners) — set to your layout; used as "anchors" for triangulation
  anchors[0] = { 137.0f,   0.0f };
  anchors[1] = {   0.0f,   0.0f };
  anchors[2] = {   0.0f, 127.0f };
  anchors[3] = { 137.0f, 127.0f };

  // Single-slot queue: overwrite so motor always gets latest position
  positionQueue = xQueueCreate(POSITION_QUEUE_LEN, sizeof(PositionMessage_t));
  if (positionQueue == NULL) {
    Serial.println("FATAL: position queue create failed");
    for (;;) delay(1000);
  }

  // Optional: set goal to a tag (e.g. first corner)
  goalX = anchors[0].x;
  goalY = anchors[0].y;

  BaseType_t ok;
  ok = xTaskCreate(taskUWB, "UWB", 4096, NULL, 1, NULL);
  if (ok != pdPASS) {
    Serial.println("FATAL: UWB task create failed");
    for (;;) delay(1000);
  }

  ok = xTaskCreate(taskMotor, "Motor", 3072, NULL, 2, NULL);  // higher priority
  if (ok != pdPASS) {
    Serial.println("FATAL: Motor task create failed");
    for (;;) delay(1000);
  }

  Serial.println("AnchorRobot FreeRTOS: UWB + Motor tasks running.");
}

// ----------------------------- loop ------------------------------------------
// FreeRTOS scheduler runs tasks; loop can be used for debug or low-priority work.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.print("Position: ");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.println(robotY);
}
