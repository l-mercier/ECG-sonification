#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

#include <Arduino.h>

class SignalProcessor {
public:
    SignalProcessor();
    
    // Preprocessing functions
    static void meanRemoval(float* signal, int len);
    static void amplitudeNormalization(float* signal, int len);
    static void movingAverageFilter(float* signal, int len, int windowSize);
    static int detectPeak(const float* signal, int len, float threshold);
    
    // Filter functions (stateful)
    float highpassFilter(float input, float dt, float cutoff);
    float lowpassFilter(float input, float dt, float cutoff);
    float bandpassFilter(float input, float dt);
    
    // Reset filter states
    void resetFilters();

private:
    // Filter state variables
    float hpPrevInput;
    float hpPrevOutput;
    float lpPrevOutput;
};

#endif // SIGNAL_PROCESSOR_H