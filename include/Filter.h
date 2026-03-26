#ifndef FILTER_H
#define FILTER_H
#include <Arduino.h>
#include <vector>

float ema(float prev, float next, float alpha);

// implements spike rejection, Median filter, and then EMA filter
struct DistanceFilter{
    float dFilt = 0.0f;
    bool distInit = false;
    float medianWindow[3];
    int windowIndex = 0;
    bool windowFilled = false;

    static constexpr float DIST_ALPHA = 0.35f; // 0..1 (higher = more responsive)
    static constexpr float SPIKE_REJECTION_DIST = 150.0f;

    static float median3(float a, float b, float c);
    bool spikeRejection(float dRaw);
    void updateMedianWindow(float dRaw);
    float MedianFilter(float dRaw);
    bool filter(float dRaw);
};

#endif