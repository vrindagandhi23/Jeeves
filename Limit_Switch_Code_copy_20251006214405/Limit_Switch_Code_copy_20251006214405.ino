// Simple Limit Switch Test for ESP32

const int limitSwitchPin = 34;  // You can use any GPIO (e.g., 18, 19, 21, 22)
const int ledPin = 2;           // Onboard LED for many ESP32 boards
//connect C to VCC
//connect NO to GPIO
int switchState = 0;

void setup() {
  pinMode(limitSwitchPin, INPUT_PULLUP);  // Use internal pull-up resistor
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  delay(1000); // Give time for serial to open
  Serial.println("ESP32 Limit Switch Test Started");
}

void loop() {
  switchState = digitalRead(limitSwitchPin);

  if (switchState == LOW) {  // LOW = pressed (because of INPUT_PULLUP)
    digitalWrite(ledPin, HIGH);
    Serial.println("Switch Pressed");
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("Switch Released");
  }

  delay(300);
}
