#include "SignalProcessing.h"

SignalProcessor::SignalProcessor() {
    resetFilters();
}

void SignalProcessor::resetFilters() {
    hpPrevInput = 0.0;
    hpPrevOutput = 0.0;
    lpPrevOutput = 0.0;
}

void SignalProcessor::meanRemoval(float* signal, int len) {
    float sum = 0;
    for (int i = 0; i < len; i++) {
        sum += signal[i];
    }
    float mean = sum / len;
    for (int i = 0; i < len; i++) {
        signal[i] -= mean;
    }
}

void SignalProcessor::amplitudeNormalization(float* signal, int len) {
    float minVal = signal[0];
    float maxVal = signal[0];
    
    for (int i = 1; i < len; i++) {
        if (signal[i] < minVal) minVal = signal[i];
        if (signal[i] > maxVal) maxVal = signal[i];
    }
    
    float range = maxVal - minVal;
    if (range == 0) return;
    
    for (int i = 0; i < len; i++) {
        signal[i] = (signal[i] - minVal) / range;
    }
}

void SignalProcessor::movingAverageFilter(float* signal, int len, int windowSize) {
    float* temp = new float[len];
    
    for (int i = 0; i < len; i++) {
        float sum = 0;
        int count = 0;
        
        for (int j = i - windowSize; j <= i + windowSize; j++) {
            if (j >= 0 && j < len) {
                sum += signal[j];
                count++;
            }
        }
        temp[i] = sum / count;
    }
    
    for (int i = 0; i < len; i++) {
        signal[i] = temp[i];
    }
    
    delete[] temp;
}

int SignalProcessor::detectPeak(const float* signal, int len, float threshold) {
    for (int i = 1; i < len - 1; i++) {
        if (signal[i] > signal[i - 1] && 
            signal[i] > signal[i + 1] && 
            signal[i] > threshold) {
            return i;
        }
    }
    return -1;
}

float SignalProcessor::highpassFilter(float input, float dt, float cutoff) {
    float RC = 1.0 / (2.0 * PI * cutoff);
    float alpha = RC / (RC + dt);
    float output = alpha * (hpPrevOutput + input - hpPrevInput);
    
    hpPrevInput = input;
    hpPrevOutput = output;
    
    return output;
}

float SignalProcessor::lowpassFilter(float input, float dt, float cutoff) {
    float RC = 1.0 / (2.0 * PI * cutoff);
    float alpha = dt / (RC + dt);
    float output = lpPrevOutput + alpha * (input - lpPrevOutput);
    
    lpPrevOutput = output;
    
    return output;
}

float SignalProcessor::bandpassFilter(float input, float dt) {
    float hp = highpassFilter(input, dt, 0.5);    // Remove < 0.5 Hz
    float bp = lowpassFilter(hp, dt, 150.0);      // Remove > 150 Hz
    return bp;
}