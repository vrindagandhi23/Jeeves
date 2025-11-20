##define TXD2 17
#define RXD2 16
#define RYUW_NRST 4

HardwareSerial RYUW(2);

// List of TAG names to poll
String tagList[] = { "TAG1", "TAG2", "TAG3" };
int totalTags = 3;
int currentTag = 0;

unsigned long lastPollTime = 0;
const unsigned long pollInterval = 800; // ms between polls

void setup() {
  Serial.begin(115200);
  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, HIGH);

  Serial.println("Starting alternating TAG1, TAG2, TAG3 poller...");
}

void loop() {

  unsigned long now = millis();

  // ------------------------
  // 1. Send a poll command every pollInterval
  // ------------------------
  if (now - lastPollTime >= pollInterval) {
    lastPollTime = now;

    String tag = tagList[currentTag];  // TAG1 or TAG2 or TAG3
    String cmd = "AT+ANCHOR_SEND=" + tag + ",1,A";

    RYUW.println(cmd);
    Serial.println("Sent: " + cmd);

    // Switch to next tag
    currentTag = (currentTag + 1) % totalTags;
  }

  // ------------------------
  // 2. Read RYUW responses continuously
  // ------------------------
  while (RYUW.available()) {

    String response = RYUW.readStringUntil('\n');
    response.trim();

    if (response.startsWith("+ANCHOR_RCV")) {

      Serial.println("Recv: " + response);

      // Extract distance (after last comma)
      int lastComma = response.lastIndexOf(',');
      if (lastComma != -1) {
        String distanceStr = response.substring(lastComma + 1);
        distanceStr.replace("cm", "");  // remove unit
        distanceStr.trim();

        int distance = distanceStr.toInt();

        Serial.print("Parsed distance: ");
        Serial.print(distance);
        Serial.println(" cm");
      }
    }
    else {
      // If you want to inspect other responses:
      // Serial.println("Other: " + response);
    }
  }
}
