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

// Pin 19 = R, pin 18 = L. R sweeps from rest down to 0 and back; L mirrored (180−r) so both wheels move together.
static constexpr int SERVO_WIGGLE_REST_R = 90;
static constexpr int SERVO_WIGGLE_REST_L = 180 - SERVO_WIGGLE_REST_R;
static constexpr int SERVO_LIFT_MIN_DEG  = 0;

#define WIN1 36
#define WIN2 39
#define WIN3 34
#define WIN4 35

#define IMU_INTERRUPT 23

// ----------------------------- Navigation tuning ------------------------------
static constexpr int   TRIANGULATION_SAMPLES     = 5;
static constexpr int   TRIANGULATION_MIN_GOOD    = 3;
static constexpr uint32_t POLL_DELAY_MS        = 5;
static constexpr uint32_t BETWEEN_SAMPLE_MS    = 15;

static constexpr float ARRIVAL_THRESHOLD_CM      = 30.0f;

// Timed straight segment after alignment (and bench test forward run).
static constexpr uint32_t DRIVE_FORWARD_MS       = 500;  // 1 seconds
static constexpr int    DRIVE_FORWARD_DUTY       = 200;
static constexpr uint32_t MOTOR_STOP_TO_WIGGLE_MS = 280;  // pause after drive before servos move

static constexpr int    TURN_PWM                 = 150;
static constexpr uint32_t TURN_SLICE_MS          = 10;
static constexpr float  TURN_HEADING_STEP_RAD    = 0.028f;
static constexpr float  TURN_ANGLE_TOL_RAD       = 0.22f;
static constexpr int    TURN_MAX_SLICES          = 450;

// Set true for bench test: no UWB, drive forward 2s once then idle. Set false for normal navigation.
static constexpr bool kMotorForwardBenchTest = false;

// Hard-coded goal: TAG2 is anchors[1] (must match anchors.push_back order in setup()).
static constexpr int GOAL_TAG_INDEX = 1;

// ----------------------------- Robot & anchors --------------------------------
Robot robot(0.0f, 0.0f, ENA, IN1, IN2, IN3, IN4, ENB, STBY, WIN1, WIN2, WIN3, WIN4, IMU_INTERRUPT);
std::vector<Anchor> anchors;

Servo servoL;
Servo servoR;

static bool gSyncedHeadingFromFirstFix = false;

static bool arrive = false;

static float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

static void pollAllAnchorsOnce() {
  for (size_t i = 0; i < anchors.size(); i++) {
    anchors[i].PollDistance(RYUW);
    // delay(POLL_DELAY_MS);
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
    Serial.println(k);
    float xr = 0.0f;
    float yr = 0.0f;
    if (oneTriangulationSample(xr, yr)) {
      sx += xr;
      sy += yr;
      n++;
    }
    // delay(BETWEEN_SAMPLE_MS);
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

  const float Kp = 200.0f;        // tune this
  const uint32_t LOOP_DT_MS = 10; // control loop period

  uint32_t lastUpdate = millis();

  while (true) {
    uint32_t now = millis();

    // Run control loop at fixed interval
    if (now - lastUpdate >= LOOP_DT_MS) {
      lastUpdate = now;

      float err = wrapPi(targetBearing - robot.getHeading());

      Serial.print("Heading: ");
      Serial.println(robot.getHeading());
      Serial.print("Err: ");
      Serial.println(err);

      // Stop when within tolerance
      if (fabsf(err) < TURN_ANGLE_TOL_RAD) {
        robot.motorsStop();
        return;
      }

      // Proportional control
      float turn = Kp * err;

      // Clamp PWM
      turn = constrain(turn, -255, 255);

      if (turn > 0) {
        robot.motorsRightTurn((int)fabs(turn));
      } else {
        robot.motorsLeftTurn((int)fabs(turn));
      }
    }

    // Yield CPU without blocking timing
    delay(1);
  }
}

static void driveStepTowardGoal() {
  float gx, gy;
  robot.getGoal(gx, gy);

  float x, y;
  robot.GetPosition(x, y);

  float dist = distanceToGoal(x, y, gx, gy);

  // Stop if already close
  if (dist < ARRIVAL_THRESHOLD_CM) {
    robot.motorsStop();
    return;
  }

  // 🔥 Proportional duration (this replaces fixed time)
  const float Kp_time = 8.0f;  // ms per cm (tune this)

  uint32_t driveTime = (uint32_t)(Kp_time * dist);

  // Clamp duration
  driveTime = constrain(driveTime, 100, 800);  // ms

  // Optional: slow down near goal
  int pwm = DRIVE_FORWARD_DUTY;
  if (dist < 2.0f * ARRIVAL_THRESHOLD_CM) {
    pwm *= 0.6f;
  }

  Serial.print("Drive step: dist=");
  Serial.print(dist);
  Serial.print(" time=");
  Serial.println(driveTime);

  robot.motorsForward(pwm);
  delay(driveTime);
  robot.motorsStop();
}

bool servoUp = false;
void moveServoUp(){
  if(!servoUp){
    for (int r = SERVO_WIGGLE_REST_R; r >= SERVO_LIFT_MIN_DEG; r--) {
      int l = 180 - r;
      servoR.write(constrain(r, 0, 180));
      servoL.write(constrain(l, 0, 180));
      delay(16);
    }
  }
  servoUp = true;
}

void moveServoDown(){
  if(servoUp){
    for (int r = SERVO_LIFT_MIN_DEG; r <= SERVO_WIGGLE_REST_R; r++) {
      int l = 180 - r;
      servoR.write(constrain(r, 0, 180));
      servoL.write(constrain(l, 0, 180));
      delay(16);
    }
  }
  servoUp = false;
}


// ----------------------------- setup -----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("AnchorRobot starting (single-threaded nav)...");

  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);

  anchors.push_back(Anchor(101.6, 0, "TAG1"));
  anchors.push_back(Anchor(116.8, 101.6, "TAG2"));
  anchors.push_back(Anchor(0, 0, "TAG3"));
  anchors.push_back(Anchor(0, 116.8, "TAG4"));

  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, HIGH);

  // Attach servos after motor LEDC init (Robot global ctor) so library picks low
  // channels; motors use channels 8/9 (see Motors.h) to avoid overlap.
  // attach() returns LEDC channel index; 0 is valid — do not treat as boolean.
  int chL = servoL.attach(SERVO_PINL, 500, 2400);
  int chR = servoR.attach(SERVO_PINR, 500, 2400);
  if (chL < 0 || chR < 0) {
    Serial.print("WARN: servo attach channel L=");
    Serial.print(chL);
    Serial.print(" R=");
    Serial.println(chR);
  }
  servoR.write(SERVO_WIGGLE_REST_R);
  servoL.write(SERVO_WIGGLE_REST_L);
  delay(300);

  float gx = 0.0f;
  float gy = 0.0f;
  anchors[GOAL_TAG_INDEX].GetPosition(gx, gy);
  robot.setGoal(gx, gy);

  robot.initializeIMU();

  Serial.print("Goal TAG2 (cm): ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.println(gy);
}

// ----------------------------- loop ------------------------------------------
void loop() {
  if (kMotorForwardBenchTest) {
    Serial.println("Bench: motor forward 2s (navigation disabled).");
    uint32_t t0 = millis();
    robot.motorsForward(DRIVE_FORWARD_DUTY);
    delay(DRIVE_FORWARD_MS);
    robot.motorsStop();
    Serial.print("Bench: stop. Actual run ms=");
    Serial.println(millis() - t0);
    delay(MOTOR_STOP_TO_WIGGLE_MS);
    delay(1000);
    return;
  }
  if(!arrive){

    moveServoUp();
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

    // float rx, ry;
    // robot.GetPosition(rx, ry);
    // float dx = x - rx;
    // float dy = y - ry;
    // float heading = atan2f(dy, dx);
    // robot.adjustHeading(heading);

    robot.updatePosition(x, y);

    Serial.print("Avg position (cm): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);

    float dist = distanceToGoal(x, y, gx, gy);
    Serial.print("dist: ");
    Serial.println(dist);
    if (dist < ARRIVAL_THRESHOLD_CM) {
      robot.motorsStop();
      Serial.println("Arrived within threshold. Holding.");
      delay(1000);
      arrive = true;
      return;
    }

    if (!gSyncedHeadingFromFirstFix) {
      robot.initHeadingFromBearingToGoal();
      gSyncedHeadingFromFirstFix = true;
      Serial.println("Heading initialized from first fix (robot assumed aimed toward goal).");
    }

    moveServoDown();

    alignHeadingToGoalBearing();

    Serial.print("Heading: ");
    Serial.println(robot.getHeading());

    Serial.println("Bearing to goal: ");
    Serial.println(robot.bearingToGoal());

    driveStepTowardGoal();


    delay(1000);

    // Serial.print("CSV ");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.println(y);
  }
}
