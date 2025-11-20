#define TXD2 17
#define RXD2 16
#define RYUW_NRST 4

HardwareSerial RYUW(2);

#include <vector>
#include <array>
#include <math.h>

struct Anchor {
  float x;
  float y;
};

std::vector<Anchor> anchors;

int tags;

std::vector<float> distances;

// Performs 2D triangulation using least squares
bool triangulate(
    const std::vector<Anchor> &anchors,
    const std::vector<float> &distances,
    float &outX,
    float &outY
) {
    int n = anchors.size();
    if (n < 3) return false;

    if (distances.size() != n) return false;

    // Reference anchor
    float x1 = anchors[0].x;
    float y1 = anchors[0].y;
    float d1 = distances[0];

    // Build A (n-1 × 2) and b (n-1)
    std::vector<std::array<float, 2>> A;
    std::vector<float> b;
    A.reserve(n - 1);
    b.reserve(n - 1);

    for (int i = 1; i < n; i++) {
        float xi = anchors[i].x;
        float yi = anchors[i].y;
        float di = distances[i];

        A.push_back({ xi - x1, yi - y1 });

        float bi = 0.5f * (
            (d1 * d1 - di * di) +
            (xi * xi - x1 * x1) +
            (yi * yi - y1 * y1)
        );

        b.push_back(bi);
    }

    int m = n - 1; // number of rows

    // Compute ATA = AᵀA and ATb = Aᵀb
    float ATA[2][2] = {{0,0},{0,0}};
    float ATb[2] = {0, 0};

    for (int i = 0; i < m; i++) {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];

        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }

    // Invert 2×2 matrix
    float det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    if (fabs(det) < 1e-6) return false;

    float invATA[2][2];
    invATA[0][0] =  ATA[1][1] / det;
    invATA[0][1] = -ATA[0][1] / det;
    invATA[1][0] = -ATA[1][0] / det;
    invATA[1][1] =  ATA[0][0] / det;

    // Solve: position = inv(ATA) * (ATb)
    outX = invATA[0][0] * ATb[0] + invATA[0][1] * ATb[1];
    outY = invATA[1][0] * ATb[0] + invATA[1][1] * ATb[1];

    return true;
}


void setup() {
  Serial.begin(115200);
  RYUW.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(RYUW_NRST, OUTPUT);
  digitalWrite(RYUW_NRST, HIGH);

  anchors = {
    {137.0,   0.0},
    {0.0, 0.0},
    {0.0, 127.0}
  };

  distances.resize(3);

  Serial.println("Listening for RYUW responses...");
}

void loop() {

  for (int i = 0; i < 3; i++) {

    // Send command to current TAG
    RYUW.println("AT+ANCHOR_SEND=TAG" + String(i + 1) + ",1,A");

    // Wait up to 500 ms for a response for this tag
    bool gotDistance = false;
    unsigned long start = millis();
    while (millis() - start < 500) {
      if (RYUW.available()) {
        String response = RYUW.readStringUntil('\n');
        response.trim();

        if (response.startsWith("+ANCHOR_RCV")) {
          int lastComma = response.lastIndexOf(',');
          if (lastComma != -1) {
            String distanceStr = response.substring(lastComma + 1);
            distanceStr.trim();  // "9 cm" → "9 cm"

            // Remove " cm" if it exists
            distanceStr.replace("cm", "");
            distanceStr.trim();  // now "9"

            // Convert to integer
            int distance = distanceStr.toInt();
            distances[i] = distance;
            Serial.print("Distance (int): ");
            Serial.println(distance);
            gotDistance = true;
            break;  // stop waiting for this tag
          }
        }
      }
    }

    if (!gotDistance) {
      Serial.print("Distance Failed: ");
      Serial.println(i + 1);
    }

    delay(0); // keep your original pacing between tags
  }

  float x, y;
  if (triangulate(anchors, distances, x, y)) {
    Serial.println(String(x) + "," + String(y));
  } else {
    Serial.println("Triangulation failed (not enough data or singular matrix)");
  }
}
