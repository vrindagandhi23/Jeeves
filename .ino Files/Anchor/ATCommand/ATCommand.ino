#define TXD2 17
#define RXD2 16
#define RYUW_NRST 4
HardwareSerial RYUW(2);

void setup() {
  Serial.begin(115200);
  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);
  RYUW.flush();
  Serial.println("Resetting RYUW");
  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, LOW);
  delay(100);
  digitalWrite(RYUW_NRST, HIGH);
  Serial.println("ESP32 UART bridge started");
}

void loop() {
// Forward from USB Serial → UART2
  while (Serial.available()) {
    // Serial.print("[TX→RYU] ");
    char c = Serial.read();
    RYUW.write(c);
  }

  // Forward from UART2 → USB Serial
  while (RYUW.available()) {
    // Serial.print("[RX←RYU] ");
    char c = RYUW.read();
    Serial.write(c);
  }
}
