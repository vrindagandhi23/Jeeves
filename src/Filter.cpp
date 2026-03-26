#include "Filter.h"

float ema(float prev, float next, float alpha) {
    return prev + alpha * (next - prev);
}

float DistanceFilter::median3(float a, float b, float c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

// true for continue, false for reject
bool DistanceFilter::spikeRejection(float dRaw){
    return !distInit || fabs(dRaw - dFilt) < SPIKE_REJECTION_DIST;
}

void DistanceFilter::updateMedianWindow(float dRaw){
    // --- Median filter window update ---
    medianWindow[windowIndex] = dRaw;
    if(windowIndex== 2){
        windowFilled = true;
        // Serial.println("window filled");
    }
    windowIndex = (windowIndex + 1) % 3;
}

float DistanceFilter::MedianFilter(float dRaw){
// Only apply median after we have at least 3 samples
    if(windowFilled){
        return median3(
        medianWindow[0],
        medianWindow[1],
        medianWindow[2]
        );
    }
    else{
        return dRaw;
    }
}

bool DistanceFilter::filter(float dRaw){
    if(spikeRejection(dRaw)){
        updateMedianWindow(dRaw);
        float dMedian = MedianFilter(dRaw);
        // Serial.print("dMedian: ");
        // Serial.println(dMedian);
                // --- EMA smoothing ---
        if (!distInit) {
            distInit = true;
            dFilt = dMedian;
            return true;
        } else {
            dFilt = ema(dFilt, dMedian, DIST_ALPHA);
            return true;
        }
    }
    // Serial.println("failed spike rejection");
    return false;
}
