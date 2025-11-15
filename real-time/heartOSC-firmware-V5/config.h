#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <IPAddress.h>

// === Buffer Configuration ===
const int BUFFER_SIZE = 256;
const int PROCESS_EVERY_N_SAMPLES = 1;

// === Sensor Configuration ===
const byte LED_BRIGHTNESS = 60;        // 0=Off to 255=50mA
const byte SAMPLE_AVERAGE = 4;         // Options: 1, 2, 4, 8, 16, 32
const byte LED_MODE = 2;               // 1=Red only, 2=Red+IR, 3=Red+IR+Green
const int SAMPLE_RATE = 1000;          // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
const int PULSE_WIDTH = 118;           // Options: 69, 118, 215, 411
const int ADC_RANGE = 8192;            // Options: 2048, 4096, 8192, 16384

// === WiFi Configuration ===
const char* WIFI_SSID = "riot-leo";
const char* WIFI_PASSWORD = "12345678";

// Static IP configuration
const IPAddress LOCAL_IP(10, 10, 0, 150);
const IPAddress GATEWAY(10, 10, 0, 254);
const IPAddress SUBNET(255, 255, 0, 0);
const IPAddress PRIMARY_DNS(8, 8, 8, 8);
const IPAddress SECONDARY_DNS(1, 1, 1, 1);

// === OSC Configuration ===
const char* OSC_DEST_IP = "10.10.0.1";
const int OSC_DEST_PORT = 8000;
const int OSC_LOCAL_PORT = 12345;
const char* OSC_ADDRESS = "/ecg";

// === Filter Configuration ===
const float HIGHPASS_CUTOFF = 0.5;     // Hz
const float LOWPASS_CUTOFF = 150.0;    // Hz

#endif // CONFIG_H