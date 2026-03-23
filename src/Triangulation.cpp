#include "Triangulation.h"

// Performs 2D triangulation using least squares
bool triangulate(
    const std::vector<Anchor> &anchors,
    float &outX,
    float &outY
) {
    int n = anchors.size();
    if (n < 3) return false;

    // Reference anchor
    float x1, y1;
    anchors[0].GetPosition(x1, y1);
    float d1 = anchors[0].GetDistance();

    // Build A (n-1 × 2) and b (n-1)
    std::vector<std::array<float, 2>> A;
    std::vector<float> b;
    A.reserve(n - 1);
    b.reserve(n - 1);

    for (int i = 1; i < n; i++) {
        float xi, yi;
        anchors[i].GetPosition(xi, yi);
        float di = anchors[i].GetDistance();

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
