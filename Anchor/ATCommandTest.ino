#include <HardwareSerial.h>
// #include <SoftwareSerial.h>

// Define the hardware serial port for the RYUW122 module
HardwareSerial rywSerial(2);
// SoftwareSerial rywSerial(16, 17);

void setup() {
  // Start the USB serial communication with the computer
  Serial.begin(115200);

delay(2000);

  // Start the hardware serial communication with the RYUW122 module
  // The default baud rate for the RYUW122 is 115200
    rywSerial.begin(115200, SERIAL_8N1, 16, 17);

  delay(2000);

  Serial.println("ESP32 ready. Type AT commands in the Serial Monitor.");
  Serial.println("Commands will be forwarded to the RYUW122 module.");
  Serial.println("Responses will be displayed here.");
}

void loop() {
  // // Read from the USB serial monitor and forward to the RYUW122
  // if (Serial.available()) {
  //   String command = Serial.readStringUntil('\n');
  //   Serial.println(command);
  //   rywSerial.println(command);
  //   Serial.println("Serial Available");
  // }

  // // Read from the RYUW122 and forward to the USB serial monitor
   if (rywSerial.available()) {
   Serial.println(rywSerial.read());
    Serial.println("UWB Serial Available");
  // }
  // // Serial.println("Looping");
  // delay(1000);

    while (Serial.available()) {
    rywSerial.write(Serial.read());
    Serial.println("Serial Available");
  }

  // Forward data from RYUW122 → Serial Monitor
  while (rywSerial.available()) {
    Serial.write(rywSerial.read());
    Serial.println("UWB Serial Available");
  }

  delay(1000);
}
}

