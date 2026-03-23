#ifndef TRIANGULATION_TEST_H
#define TRIANGULATION_TEST_H

#define TXD2 17
#define RXD2 16
#define RYUW_NRST 4

#include <Arduino.h>
#include <vector>
#include "Anchor.h"
#include "Triangulation.h"
#include "Filter.h"

static bool posInit = false;
static float posXFilt = 0.0f;
static float posYFilt = 0.0f;

static const float POS_ALPHA = 0.35f; // 0..1 (higher = more responsive)

extern HardwareSerial RYUW;

extern std::vector<Anchor> anchors;

void TriangulationTestSetup();
void TriangulationTestLoop();

#endif