#include "config.h"
#include "SignalProcessor.h"
#include "MAX30102Handler.h"
#include "NetworkManager.h"

// === Global Objects ===
MAX30102Handler sensorHandler;
SignalProcessor signalProcessor;
NetworkManager networkManager;

// === Data Buffer ===
float buffer[BUFFER_SIZE];
int sampleCounter = 0;
float lastFilteredIR = 0.0;

// === Setup Function ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize WiFi
  if (!networkManager.connectToWiFi(WIFI_SSID, WIFI_PASSWORD, 
                                     LOCAL_IP, GATEWAY, SUBNET,
                                     PRIMARY_DNS, SECONDARY_DNS)) {
    Serial.println("WiFi connection failed. Halting.");
    while (true);
  }
  
  // Configure OSC destination
  NetworkManagerConfigurator::setOSCDestination(networkManager, 
                                                 OSC_DEST_IP, OSC_DEST_PORT);
  networkManager.beginOSC(OSC_LOCAL_PORT);
  
  // Initialize sensor
  if (!sensorHandler.begin()) {
    Serial.println("Sensor initialization failed. Restarting...");
    ESP.restart();
  }
  
  // Configure sensor
  sensorHandler.configure(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE,
                          SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  
  // Initialize buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    buffer[i] = 0;
  }
  
  Serial.println("Setup complete. Starting main loop...");
}

// === Main Loop ===
void loop() {
  const float dt = 1.0 / SAMPLE_RATE;  // Sampling period
  
  // Shift buffer (drop oldest sample)
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
  
  // Acquire new data from sensor
  uint32_t irValue = sensorHandler.getIR();
  buffer[BUFFER_SIZE - 1] = irValue;
  
  // Beat detection
  bool beatDetected = sensorHandler.checkForBeat(irValue);
  
  // Read temperature
  float temperature = sensorHandler.getTemperature();
  
  sampleCounter++;
  
  // Process signal every N samples
  if (sampleCounter >= PROCESS_EVERY_N_SAMPLES) {
    sampleCounter = 0;
    
    // Optional: Apply additional preprocessing
    // float tempBuffer[BUFFER_SIZE];
    // memcpy(tempBuffer, buffer, sizeof(buffer));
    // SignalProcessor::meanRemoval(tempBuffer, BUFFER_SIZE);
    // SignalProcessor::amplitudeNormalization(tempBuffer, BUFFER_SIZE);
    
    // Apply bandpass filter
    lastFilteredIR = signalProcessor.bandpassFilter((float)irValue, dt);
    
    // Debug output
    Serial.print("IR: "); Serial.print(irValue);
    Serial.print(" | Filtered IR: "); Serial.print(lastFilteredIR);
    Serial.print(" | Temp: "); Serial.print(temperature);
    Serial.print(" | Beat: "); Serial.println(beatDetected);
  }
  
  // Send OSC message
  networkManager.sendOSC(OSC_ADDRESS, 
                         (int32_t)irValue,      // Raw IR
                         lastFilteredIR,         // Filtered IR
                         temperature,            // Temperature
                         (int32_t)beatDetected); // Beat flag
}