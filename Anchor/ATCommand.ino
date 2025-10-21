#define TXD0 17
#define RXD0 16

HardwareSerial mySerial(2);

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RXD0, TXD0);
  mySerial.flush();
  Serial.println("ESP32 UART bridge started");
}

void loop() {
// Forward from USB Serial → UART2
  while (Serial.available()) {
    char c = Serial.read();
    mySerial.write(c);
  }

  // Forward from UART2 → USB Serial
  while (mySerial.available()) {
    char c = mySerial.read();
    Serial.write(c);
  }
}
