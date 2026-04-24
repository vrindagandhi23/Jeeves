// =============================================================================
// Bed-Making Robot: Anchor with UWB triangulation + motors (no RTOS).
// State: average N fixes -> turn toward goal using tracked heading -> timed
// drive -> servo motion -> repeat until within arrival distance.
// =============================================================================

#include <Arduino.h>
#include <cmath>
#include "Robot.h"
#include "Anchor.h"
#include "Triangulation.h"
#include <vector>
#include <ESP32Servo.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ----------------------------- UWB (Serial) ----------------------------------
#define TXD2 17
#define RXD2 16
#define RYUW_NRST 4
HardwareSerial RYUW(2);

// ----------------------------- Motor pins (TB6612FNG) ------------------------
#define ENA 12
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENB 33
#define STBY 32

#define SERVO_PINL 18
#define SERVO_PINR 19

#define WIN1 22
#define WIN2 1
#define WIN3 3
#define WIN4 21

// ----------------------------- Navigation tuning ------------------------------
static constexpr int   TRIANGULATION_SAMPLES     = 10;
static constexpr int   TRIANGULATION_MIN_GOOD    = 5;
static constexpr uint32_t POLL_DELAY_MS        = 5;
static constexpr uint32_t BETWEEN_SAMPLE_MS    = 15;

static constexpr float ARRIVAL_THRESHOLD_CM      = 5.0f;

static constexpr uint32_t DRIVE_FORWARD_MS       = 2000;
static constexpr int    DRIVE_FORWARD_DUTY       = 55;

static constexpr int    TURN_PWM                 = 200;
static constexpr uint32_t TURN_SLICE_MS          = 35;
static constexpr float  TURN_HEADING_STEP_RAD    = 0.028f;
static constexpr float  TURN_ANGLE_TOL_RAD       = 0.22f;
static constexpr int    TURN_MAX_SLICES          = 450;

// ----------------------------- Robot & anchors --------------------------------
Robot robot(0.0f, 0.0f, ENA, IN1, IN2, IN3, IN4, ENB, STBY, WIN1, WIN2, WIN3, WIN4);
std::vector<Anchor> anchors;

Servo servoL;
Servo servoR;

static bool gSyncedHeadingFromFirstFix = false;

static float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

static void pollAllAnchorsOnce() {
  for (size_t i = 0; i < anchors.size(); i++) {
    anchors[i].PollDistance(RYUW);
    delay(POLL_DELAY_MS);
  }
}

// One full anchor poll + least-squares fix (raw triangulation, no EMA).
static bool oneTriangulationSample(float &outX, float &outY) {
  pollAllAnchorsOnce();
  return triangulate(anchors, outX, outY);
}

static bool averageTriangulatedPosition(float &outX, float &outY) {
  float sx = 0.0f;
  float sy = 0.0f;
  int n = 0;
  for (int k = 0; k < TRIANGULATION_SAMPLES; k++) {
    float xr = 0.0f;
    float yr = 0.0f;
    if (oneTriangulationSample(xr, yr)) {
      sx += xr;
      sy += yr;
      n++;
    }
    delay(BETWEEN_SAMPLE_MS);
  }
  if (n < TRIANGULATION_MIN_GOOD) {
    return false;
  }
  outX = sx / (float)n;
  outY = sy / (float)n;
  return true;
}

static float distanceToGoal(float x, float y, float gx, float gy) {
  float dx = gx - x;
  float dy = gy - y;
  return sqrtf(dx * dx + dy * dy);
}

static void alignHeadingToGoalBearing() {
  const float targetBearing = robot.bearingToGoal();

  for (int i = 0; i < TURN_MAX_SLICES; i++) {
    float err = wrapPi(targetBearing - robot.getHeading());
    if (fabsf(err) < TURN_ANGLE_TOL_RAD) {
      robot.motorsStop();
      return;
    }
    if (err > 0.0f) {
      robot.motorsRightTurn(TURN_PWM);
      delay(TURN_SLICE_MS);
      robot.motorsStop();
      robot.adjustHeading(TURN_HEADING_STEP_RAD);
    } else {
      robot.motorsLeftTurn(TURN_PWM);
      delay(TURN_SLICE_MS);
      robot.motorsStop();
      robot.adjustHeading(-TURN_HEADING_STEP_RAD);
    }
  }
  robot.motorsStop();
  Serial.println("WARN: turn align stopped after max slices (tune TURN_HEADING_STEP_RAD / TURN_SLICE_MS).");
}

static void servoWiggle() {
  for (int ang = 90; ang <= 120; ang++) {
    servoL.write(ang);
    servoR.write(ang);
    delay(12);
  }
  for (int ang = 120; ang >= 90; ang--) {
    servoL.write(ang);
    servoR.write(ang);
    delay(12);
  }
}

// ----------------------------- setup -----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("AnchorRobot starting (single-threaded nav)...");

  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);

  anchors.push_back(Anchor(0, 0, "TAG1"));
  anchors.push_back(Anchor(0, 114.3, "TAG2"));
  anchors.push_back(Anchor(144.78, 0, "TAG3"));
  anchors.push_back(Anchor(144.78, 114.3, "TAG4"));

  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, HIGH);

  servoL.attach(SERVO_PINL);
  servoR.attach(SERVO_PINR);
  servoL.write(90);
  servoR.write(90);

  float gx = 0.0f;
  float gy = 0.0f;
  anchors[0].GetPosition(gx, gy);
  robot.setGoal(gx, gy);

  Serial.print("Goal (cm): ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.println(gy);
}

// ----------------------------- loop ------------------------------------------
void loop() {
  float gx = 0.0f;
  float gy = 0.0f;
  robot.getGoal(gx, gy);

  float x = 0.0f;
  float y = 0.0f;
  if (!averageTriangulatedPosition(x, y)) {
    Serial.println("Triangulation sample batch failed (not enough good fixes).");
    delay(400);
    return;
  }

  robot.updatePosition(x, y);

  Serial.print("Avg position (cm): ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);

  float dist = distanceToGoal(x, y, gx, gy);
  if (dist < ARRIVAL_THRESHOLD_CM) {
    robot.motorsStop();
    Serial.println("Arrived within threshold. Holding.");
    delay(1000);
    return;
  }

  if (!gSyncedHeadingFromFirstFix) {
    robot.initHeadingFromBearingToGoal();
    gSyncedHeadingFromFirstFix = true;
    Serial.println("Heading initialized from first fix (robot assumed aimed toward goal).");
  }

  alignHeadingToGoalBearing();

  robot.motorsForward(DRIVE_FORWARD_DUTY);
  delay(DRIVE_FORWARD_MS);
  robot.motorsStop();

  servoWiggle();

  Serial.print("CSV ");
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
}
