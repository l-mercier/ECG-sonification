#include "MAX30102Handler.h"
#include <Wire.h>

MAX30102Handler::MAX30102Handler() {
}

bool MAX30102Handler::begin() {
    if (!sensor.begin()) {
        Serial.println("MAX30102 not found. Check wiring/power.");
        Wire.end();
        delay(100);
        return false;
    }
    Serial.println("MAX30102 initialized successfully.");
    return true;
}

void MAX30102Handler::configure(byte brightness, byte average, byte mode,
                                 int rate, int width, int range) {
    sensor.setup(brightness, average, mode, rate, width, range);
    Serial.println("MAX30102 configured.");
}

uint32_t MAX30102Handler::getIR() {
    return sensor.getIR();
}

float MAX30102Handler::getTemperature() {
    return sensor.readTemperature();
}

bool MAX30102Handler::checkForBeat(uint32_t irValue) {
    return checkForBeat(irValue);  // From heartRate.h library
}

MAX30105* MAX30102Handler::getSensor() {
    return &sensor;
}