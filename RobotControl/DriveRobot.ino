// -----------------------------------------------------------------
// Robot Pursuit Controller (no triangulation inside)
// Uses TB6612FNG + ESP32
// Provide robotX, robotY externally by calling updateRobotPosition()
// Robot will drive toward a fixed target (goalX, goalY)
// -----------------------------------------------------------------

// --------------------- Motor Pin Mapping --------------------------
#define ENA 14
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENB 12
#define STBY 32

// --------------------- Robot State --------------------------------
float robotX = 0;
float robotY = 0;

// SET YOUR TARGET HERE:
float goalX = 100.0;   // cm (example)
float goalY =  50.0;   // cm

// --------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  Serial.println("Robot pursuit controller active.");
}

// --------------------- Motor Functions ----------------------------
void forward(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void backward(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void leftTurn(int spd) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void rightTurn(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------------------- Update Position API ------------------------
void updateRobotPosition(float x, float y) {
  robotX = x;
  robotY = y;
}

// ---------------------- Control Loop -------------------------------
void pursueTarget() {
  float dx = goalX - robotX;
  float dy = goalY - robotY;
  float dist = sqrt(dx*dx + dy*dy);

  Serial.print("Robot: ");
  Serial.print(robotX);
  Serial.print(", ");
  Serial.print(robotY);
  Serial.print("   Distance to target: ");
  Serial.println(dist);

  if (dist < 5.0) {
    stopMotors();
    Serial.println("Reached target.");
    return;
  }

  // Angle from robot → target (radians)
  float targetAngle = atan2(dy, dx);

  // Assume robot always faces +X (0 radians)
  float robotHeading = 0;

  float angleError = targetAngle - robotHeading;

  // Normalize to [-PI, PI]
  while (angleError > PI)  angleError -= 2*PI;
  while (angleError < -PI) angleError += 2*PI;

  // --------------------- Turning Controller ------------------------
  float turnThreshold = 0.25;  // radians (~14°)

  if (angleError > turnThreshold) {
    Serial.println("Turning right toward target");
    rightTurn(200);
    return;
  }

  if (angleError < -turnThreshold) {
    Serial.println("Turning left toward target");
    leftTurn(200);
    return;
  }

  // --------------------- Forward P-controller ----------------------
  float Kp = 3.0;
  int spd = constrain(dist * Kp, 100, 255);

  Serial.print("Moving forward @ ");
  Serial.println(spd);

  forward(spd);
}

// ---------------------- Main Loop ---------------------------------
void loop() {
  // At runtime you will call updateRobotPosition(x, y)
  // For now, fake motion (for debugging):
  // remove these two lines when integrating with triangulation
  robotX += 0.1;
  robotY += 0.05;

  pursueTarget();
  delay(50);
}
