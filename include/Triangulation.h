#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <Arduino.h>
#include <vector>
#include <math.h>
#include "Anchor.h"

// Performs 2D triangulation using least squares
bool triangulate(
    const std::vector<Anchor> &anchors,
    float &outX,
    float &outY
);

#endif