#include "Triangulation.h"

// Performs 2D triangulation using least squares
bool triangulate(
    const std::vector<Anchor> &anchors,
    float &outX,
    float &outY
) {
    int n = anchors.size();
    if (n < 3) return false;

    // Serial.println("enough stuff");

    // Reference anchor
    float x1, y1;
    anchors[0].GetPosition(x1, y1);
    float d1 = anchors[0].GetDistance();

    // Compute ATA = AᵀA and ATb = Aᵀb
    float ATA00 = 0.0f, ATA01 = 0.0f;
    float ATA10 = 0.0f, ATA11 = 0.0f;
    float ATb0 = 0.0f, ATb1 = 0.0f;

    for (int i = 1; i < n; i++) {
        float xi, yi;
        anchors[i].GetPosition(xi, yi);
        float di = anchors[i].GetDistance();

        float ax = xi - x1;
        float ay = yi - y1;

        float bi = 0.5f * (
            (d1 * d1 - di * di) +
            (xi * xi - x1 * x1) +
            (yi * yi - y1 * y1)
        );

        ATA00 += ax * ax;
        ATA01 += ax * ay;
        ATA10 += ay * ax;
        ATA11 += ay * ay;

        ATb0 += ax * bi;
        ATb1 += ay * bi;
    }

    // Invert 2×2 matrix
    float det = ATA00 * ATA11 - ATA01 * ATA10;
    if (fabs(det) < 1e-6) return false;

    float invATA[2][2];
    invATA[0][0] =  ATA11 / det;
    invATA[0][1] = -ATA01 / det;
    invATA[1][0] = -ATA10 / det;
    invATA[1][1] =  ATA00 / det;

    // Solve: position = inv(ATA) * (ATb)
    outX = invATA[0][0] * ATb0 + invATA[0][1] * ATb1;
    outY = invATA[1][0] * ATb0 + invATA[1][1] * ATb1;

    return true;
}
