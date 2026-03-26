// =============================================================================
// Bed-Making Robot: Anchor with UWB Triangulation + Motor Control
// FreeRTOS architecture: Task_UWB → Position Queue → Task_Motor
// Interrupts used only for timing/notifications; heavy work in tasks.
// =============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "Robot.h"
#include "Anchor.h"
#include "Triangulation.h"
#include <vector>
#include <ESP32Servo.h>

// ----------------------------- UWB (Serial) ----------------------------------
#define TXD2 17
#define RXD2 16
#define RYUW_NRST 4
HardwareSerial RYUW(2);

// ----------------------------- Serial mutexes -------------------------------
// Guard UART access just in case more than one task ever touches these ports.
static SemaphoreHandle_t ryuwMutex = NULL;
static SemaphoreHandle_t serialMutex = NULL;


// ----------------------------- Motor pins (TB6612FNG) ------------------------
#define ENA 12
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENB 33
#define STBY 32 // not used

// SERVO PINS 
//wheels in the back
#define SERVO_PINL 18
#define SERVO_PINR 19

// ----------------------------- Position queue (UWB → Motor) ------------------
typedef struct {
  float x;
  float y;
  uint8_t valid;  // 1 = fresh position, 0 = no fix yet
} PositionMessage_t;

#define POSITION_QUEUE_LEN  1
static QueueHandle_t positionQueue = NULL;

// ----------------------------- Robot state (motor task) ----------------------
Robot robot(0.0f, 0.0f, ENA, IN1, IN2, IN3, IN4, ENB, STBY);

// ----------------------------- Triangulation (from Anchor/Triangulation.ino) -
std::vector<Anchor> anchors;


 
Servo servoL;  // create servo object to control a servo
Servo servoR;
 

int pos = 0;    // variable to store the servo position

// ----------------------------- Simple stabilization --------------------------
// Filter distances first (reduces jitter before triangulation), then filter (x,y),
// and reject physically-impossible jumps.
bool posInit = false;
static TickType_t lastPosTick = 0;

static constexpr float POS_ALPHA  = 0.20f;     // 0..1
static constexpr float MAX_SPEED_CM_S = 120.0f; // used for jump rejection
static constexpr float JUMP_MARGIN_CM = 15.0f;  // extra allowance on top of speed gate

// ----------------------------- Task: UWB + Triangulation ---------------------
// Lower priority; runs triangulation and sends latest position to queue.
// Interrupts (if added later): use only to set a flag or give a semaphore;
// this task does all UWB read + triangulation so ISRs stay short.
static void taskUWB(void* pvParameters) {
  (void)pvParameters;
  PositionMessage_t msg = { .x = 0, .y = 0, .valid = 0 };

  for (;;) {
    for (int i = 0; i < anchors.size(); i++) {
      bool ok = false;
      if (ryuwMutex) xSemaphoreTake(ryuwMutex, portMAX_DELAY);
      ok = anchors[i].PollDistance(RYUW);
      if (ryuwMutex) xSemaphoreGive(ryuwMutex);

      if(ok){
        if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.println(i);
        Serial.print("Distance Raw (float): ");
        Serial.println(anchors[i].GetRawDistance());

        Serial.print("Distance Filtered (float): ");
        Serial.println(anchors[i].GetDistance());
        if (serialMutex) xSemaphoreGive(serialMutex);
      }
      else{
        if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
        Serial.print("Distance Failed: ");
        Serial.println(i + 1);
        if (serialMutex) xSemaphoreGive(serialMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    // if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
    
    float xRaw = 0.0f;
    float yRaw = 0.0f;
    bool haveAllDistances = true;
    for (int i = 0; i < anchors.size(); i++) {
      if (!anchors[i].GetDistInitialize()) {
        haveAllDistances = false;
        break;
      }
    }
    
    // Serial.println("Has all distances");

    if (haveAllDistances && triangulate(anchors, xRaw, yRaw)) {
      TickType_t now = xTaskGetTickCount();
      float dtS = (lastPosTick == 0) ? 0.0f : ((float)(now - lastPosTick) / (float)configTICK_RATE_HZ);
      lastPosTick = now;

      float posXFilt, posYFilt;
      if (!posInit) {   
        posXFilt = xRaw;
        posYFilt = yRaw;
        robot.updatePosition(xRaw, yRaw);
        posInit = true;
      } else {
        robot.GetPosition(posXFilt, posYFilt);
        float dx = xRaw - posXFilt;
        float dy = yRaw - posYFilt;
        float step = sqrtf(dx * dx + dy * dy);

        // Serial.print("step ");
        // Serial.println(step);

        // Gate based on plausible max step given time elapsed
        float maxStep = JUMP_MARGIN_CM;
        if (dtS > 0.0f) {
          maxStep += MAX_SPEED_CM_S * dtS;
        } else {
          maxStep += 30.0f; // first step after init: be permissive
        }

        if (step <= maxStep) {
          posXFilt = ema(posXFilt, xRaw, POS_ALPHA);
          posYFilt = ema(posYFilt, yRaw, POS_ALPHA);
          robot.updatePosition(posXFilt, posYFilt);
        }
        // else: reject this jump; keep filtered position as-is
      }

      msg.x = posXFilt;
      msg.y = posYFilt;
      msg.valid = 1;
      xQueueOverwrite(positionQueue, &msg);
    } else {
      msg.valid = 0;
      xQueueOverwrite(positionQueue, &msg);
    }
    // if (serialMutex) xSemaphoreGive(serialMutex);
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
      robot.updatePosition(msg.x, msg.y);
    }

    robot.pursueTarget();
    vTaskDelay(period);
  }
}

// ----------------------------- setup -----------------------------------------
void setup() {
  Serial.begin(115200);
  // Give Serial Monitor time to connect; bootloader runs at 74880 so open monitor at 115200 then press Reset
  delay(1500);
  Serial.println("AnchorRobot starting...");

  // Create mutexes before tasks start.
  ryuwMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // anchors.push_back(Anchor(0, 0, "TAG1"));
  anchors.push_back(Anchor(0, 0, "TAG2"));
  anchors.push_back(Anchor(0, 1, "TAG3"));
  anchors.push_back(Anchor(1, 0, "TAG4"));

  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, HIGH);

  for (pos = 180; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
    servoL.write(pos);    // tell servo to go to position in variable 'pos'
    servoR.write(pos);
    delay(15);             // waits 15ms for the servo to reach the position
	}

  // Single-slot queue: overwrite so motor always gets latest position
  positionQueue = xQueueCreate(POSITION_QUEUE_LEN, sizeof(PositionMessage_t));
  if (positionQueue == NULL) {
    Serial.println("FATAL: position queue create failed");
    for (;;) delay(1000);
  }

  BaseType_t ok;
  ok = xTaskCreate(taskUWB, "UWB", 5120, NULL, 1, NULL);
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
  float robotX, robotY;
  robot.GetPosition(robotX, robotY);
  if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.print("Position: ");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.println(robotY);
  // Raw CSV line for TriangulationVisualizer.py ("x,y")
  Serial.print(robotX);
  Serial.print(",");
  Serial.println(robotY);
  if (serialMutex) xSemaphoreGive(serialMutex);
}
