#ifndef MAX30102_HANDLER_H
#define MAX30102_HANDLER_H

#include <Arduino.h>
#include "MAX30105.h"
#include "heartRate.h"

class MAX30102Handler {
public:
    MAX30102Handler();
    
    // Initialization
    bool begin();
    void configure(byte brightness, byte average, byte mode, 
                   int rate, int width, int range);
    
    // Data acquisition
    uint32_t getIR();
    float getTemperature();
    bool checkForBeat(uint32_t irValue);
    
    // Get the sensor object (if needed for advanced operations)
    MAX30105* getSensor();

private:
    MAX30105 sensor;
};

#endif // MAX30102_HANDLER_H