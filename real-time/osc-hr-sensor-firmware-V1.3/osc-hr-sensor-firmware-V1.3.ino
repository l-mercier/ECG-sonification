#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "heartRateAdvanced.h"

// Structure of the OSC message: IR value (i32), normalized value (f), temperature (f), beat detected (i32), BPM (f), confidence (f), presence (i32), sensor status (i32)

class AdaptiveNormalizer {
  private:
    float buffer[200];
    int index = 0;
    float baselineMin = 100000;
    float baselineMax = 110000;
    bool initialized = false;
    
    static int compareFloat(const void* a, const void* b) {
      float diff = (*(float*)a - *(float*)b);
      if (diff < 0) return -1;
      if (diff > 0) return 1;
      return 0;
    }
    
    float calculateMAD(float* sorted, float median) {
      float deviations[200];
      for(int i = 0; i < 200; i++) {
        deviations[i] = abs(sorted[i] - median);
      }
      qsort(deviations, 200, sizeof(float), compareFloat);
      return deviations[100];
    }
    
  public:
    AdaptiveNormalizer() {
      for(int i = 0; i < 200; i++) {
        buffer[i] = 105000;
      }
    }
    
    void addSample(float signal) {
      buffer[index] = signal;
      index = (index + 1) % 200;
      
      if(index == 0) initialized = true;
      
      if(index % 50 == 0 && initialized) {
        updateBaseline();
      }
    }
    
    void updateBaseline() {
      float sorted[200];
      memcpy(sorted, buffer, 200 * sizeof(float));
      qsort(sorted, 200, sizeof(float), compareFloat);
      
      float median = sorted[100];
      float mad = calculateMAD(sorted, median);
      
      if(mad < 50) mad = 50;
      
      baselineMin = median - 3 * mad;
      baselineMax = median + 3 * mad;
    }
    
    float normalize(float signal) {
      float center = (baselineMin + baselineMax) / 2.0;
      float range = (baselineMax - baselineMin) / 2.0;
      
      if(range < 10) range = 10;
      
      float normalized = (signal - center) / range;
      return constrain(normalized, -1.0, 1.0);
    }
    
    void reset() {
      index = 0;
      initialized = false;
      baselineMin = 100000;
      baselineMax = 110000;
      for(int i = 0; i < 200; i++) {
        buffer[i] = 105000;
      }
    }
};

// Sensor status enum
enum SensorStatus {
  SENSOR_NOT_CONNECTED = 0,
  SENSOR_INITIALIZING = 1,
  SENSOR_READY = 2,
  SENSOR_SLEEPING = 3,
  SENSOR_ERROR = 4
};

// Global objects
MAX30105 particleSensor;
AdaptiveNormalizer normalizer;
HeartRateDetector hrDetector;

// Sensor state
SensorStatus sensorStatus = SENSOR_NOT_CONNECTED;
unsigned long lastValidReading = 0;
const unsigned long SENSOR_TIMEOUT = 3000;
int consecutiveFailures = 0;
const int MAX_FAILURES = 5;

// Presence detection
long unblockedValue = 0;
const long PRESENCE_THRESHOLD = 5000;
bool fingerPresent = false;
bool sensorAwake = true;
unsigned long lastPresenceTime = 0;
const unsigned long SLEEP_DELAY = 5000;

// Retry mechanism
unsigned long lastRetryAttempt = 0;
const unsigned long RETRY_INTERVAL = 5000; // Retry every 5 seconds

// Sensor Configuration 
byte ledBrightness = 60;
byte sampleAverage = 4;
byte ledMode = 2;
byte sampleRate = 1000;
int pulseWidth = 118;
int adcRange = 8192;

// WiFi configuration
const char* ssid = "riot-leo";
const char* password = "12345678";

IPAddress local_IP(10, 10, 0, 150);
IPAddress gateway(10, 10, 0, 254);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(1, 1, 1, 1);

// OSC destination
const char* destIP = "10.10.0.1";
const int destPort = 8000;
const int localPort = 12345;

WiFiUDP Udp;

// === Sensor Management Functions ===

void calibrateBaseline() {
  Serial.println("Calibrating baseline (no finger)...");
  unblockedValue = 0;
  
  for (byte x = 0; x < 32; x++) {
    unblockedValue += particleSensor.getIR();
    delay(10);
  }
  unblockedValue /= 32;
  
  Serial.print("Baseline calibrated: ");
  Serial.println(unblockedValue);
}

bool detectPresence(uint32_t irValue) {
  if (sensorStatus != SENSOR_READY && sensorStatus != SENSOR_SLEEPING) {
    return false;
  }
  
  long delta = (long)irValue - unblockedValue;
  
  static bool lastPresenceState = false;
  long threshold = lastPresenceState ? PRESENCE_THRESHOLD * 0.7 : PRESENCE_THRESHOLD;
  
  bool presenceDetected = (delta > threshold);
  
  if (presenceDetected != lastPresenceState) {
    if (presenceDetected) {
      Serial.println(">>> Finger detected!");
      lastPresenceTime = millis();
    } else {
      Serial.println("<<< Finger removed");
    }
  }
  
  lastPresenceState = presenceDetected;
  
  if (presenceDetected) {
    lastPresenceTime = millis();
  }
  
  return presenceDetected;
}

void enterSleepMode() {
  if (!sensorAwake || sensorStatus != SENSOR_READY) return;
  
  Serial.println("Entering sleep mode...");
  particleSensor.shutDown();
  
  normalizer.reset();
  hrDetector.reset();
  
  sensorAwake = false;
  sensorStatus = SENSOR_SLEEPING;
  Serial.println("Sensor in sleep mode. Waiting for presence...");
}

void exitSleepMode() {
  if (sensorAwake || sensorStatus != SENSOR_SLEEPING) return;
  
  Serial.println("Waking up sensor...");
  particleSensor.wakeUp();
  delay(100);
  
  calibrateBaseline();
  
  sensorAwake = true;
  sensorStatus = SENSOR_READY;
  Serial.println("Sensor awake and ready!");
}

bool initializeSensor() {
  Serial.println("Initializing MAX30102...");
  sensorStatus = SENSOR_INITIALIZING;
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found on I2C bus");
    sensorStatus = SENSOR_NOT_CONNECTED;
    return false;
  }
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0);
  particleSensor.setPulseAmplitudeGreen(0);
  
  delay(100);
  uint32_t testValue = particleSensor.getIR();
  
  if (testValue == 0) {
    Serial.println("Sensor detected but not responding");
    sensorStatus = SENSOR_ERROR;
    return false;
  }
  
  Serial.println("MAX30102 initialized successfully");
  Serial.print("Test IR value: ");
  Serial.println(testValue);
  
  calibrateBaseline();
  
  lastValidReading = millis();
  lastPresenceTime = millis();
  consecutiveFailures = 0;
  sensorAwake = true;
  sensorStatus = SENSOR_READY;
  
  return true;
}

bool resetSensor() {
  Serial.println("Attempting sensor soft reset...");
  
  particleSensor.softReset();
  delay(100);
  
  normalizer.reset();
  hrDetector.reset();
  
  bool success = initializeSensor();
  
  if (success) {
    Serial.println("Sensor reset successful");
  } else {
    Serial.println("Sensor reset failed");
  }
  
  return success;
}

bool checkSensorHealth(uint32_t irValue) {
  if (sensorStatus != SENSOR_READY && sensorStatus != SENSOR_SLEEPING) {
    return false;
  }
  
  if (irValue == 0 || irValue > 200000) {
    consecutiveFailures++;
    
    if (consecutiveFailures >= MAX_FAILURES) {
      Serial.println("Too many consecutive failures, resetting sensor...");
      sensorStatus = SENSOR_ERROR;
      return resetSensor();
    }
    return false;
  }
  
  consecutiveFailures = 0;
  lastValidReading = millis();
  
  if (millis() - lastValidReading > SENSOR_TIMEOUT) {
    Serial.println("Sensor timeout detected, resetting...");
    sensorStatus = SENSOR_ERROR;
    return resetSensor();
  }
  
  return true;
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    Serial.println("Will retry in background...");
  }
}

void sendStatusMessage() {
  OSCMessage msg("/ecg");
  msg.add((int32_t)0);              // IR
  msg.add(0.0f);                    // normalized
  msg.add(0.0f);                    // temperature
  msg.add((int32_t)0);              // beat
  msg.add(0.0f);                    // bpm
  msg.add(0.0f);                    // confidence
  msg.add((int32_t)0);              // presence
  msg.add((int32_t)sensorStatus);   // sensor status
  
  Udp.beginPacket(destIP, destPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

// === Setup ===

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 Heart Rate Sensor V8 ===");
  Serial.println("Features: Graceful sensor handling");
  
  connectToWiFi();
  
  Wire.begin();
  Wire.setClock(400000);
  
  // Tentative unique d'initialisation
  if (!initializeSensor()) {
    Serial.println("\n*** SENSOR NOT CONNECTED ***");
    Serial.println("System will continue running.");
    Serial.println("Connect sensor and it will be detected automatically.");
    Serial.println("Checking every 5 seconds...\n");
  }

  Udp.begin(localPort);
  
  Serial.println("System ready!");
  Serial.println("==============================\n");
}

// === Main Loop ===

void loop() {
  const float dt = 1.0 / sampleRate;

  // === Gestion du capteur non connecté ===
  if (sensorStatus == SENSOR_NOT_CONNECTED || sensorStatus == SENSOR_ERROR) {
    // Retry périodique
    if (millis() - lastRetryAttempt > RETRY_INTERVAL) {
      Serial.println("Attempting to reconnect sensor...");
      if (initializeSensor()) {
        Serial.println("Sensor connected successfully!");
      } else {
        Serial.println("Sensor still not available. Will retry in 5s.");
      }
      lastRetryAttempt = millis();
    }
    
    // Envoyer message de status
    static unsigned long lastStatusMsg = 0;
    if (millis() - lastStatusMsg > 1000) {
      sendStatusMessage();
      lastStatusMsg = millis();
    }
    
    delay(100);
    return;
  }

  // === Capteur connecté - traitement normal ===
  
  uint32_t irValue = particleSensor.getIR();
  
  fingerPresent = detectPresence(irValue);
  
  // Gestion du mode sommeil
  if (fingerPresent && !sensorAwake) {
    exitSleepMode();
  } else if (!fingerPresent && sensorAwake) {
    if (millis() - lastPresenceTime > SLEEP_DELAY) {
      enterSleepMode();
    }
  }
  
  // Mode sommeil - envoi minimal
  if (!sensorAwake) {
    static unsigned long lastSleepMessage = 0;
    if (millis() - lastSleepMessage > 1000) {
      OSCMessage msg("/ecg");
      msg.add((int32_t)irValue);
      msg.add(0.0f);
      msg.add(0.0f);
      msg.add((int32_t)0);
      msg.add(0.0f);
      msg.add(0.0f);
      msg.add((int32_t)0);
      msg.add((int32_t)sensorStatus);
      
      Udp.beginPacket(destIP, destPort);
      msg.send(Udp);
      Udp.endPacket();
      msg.empty();
      
      lastSleepMessage = millis();
    }
    
    delay(100);
    return;
  }
  
  // === Mode actif ===
  
  if (!checkSensorHealth(irValue)) {
    if (consecutiveFailures >= MAX_FAILURES) {
      delay(1000);
      return;
    }
  }
  
  float signal = (float)irValue;
  
  normalizer.addSample(signal);
  float normalizedValue = normalizer.normalize(signal);
  
  bool beatDetected = hrDetector.detectBeat(normalizedValue);
  float bpm = hrDetector.getBPM();
  float confidence = hrDetector.getConfidence();

  static int tempCounter = 0;
  static float temperature = 0;
  if (tempCounter++ >= 100) {
    temperature = particleSensor.readTemperature();
    tempCounter = 0;
  }

  static int debugCounter = 0;
  if (debugCounter++ >= 500) {
    Serial.print("Status: "); Serial.print(sensorStatus);
    Serial.print(" | IR: "); Serial.print(irValue);
    Serial.print(" | Delta: "); Serial.print((long)irValue - unblockedValue);
    Serial.print(" | BPM: "); Serial.print(bpm, 1);
    Serial.print(" | Conf: "); Serial.print(confidence * 100, 1);
    Serial.print("% | Presence: "); Serial.println(fingerPresent ? "YES" : "NO");
    debugCounter = 0;
  }

  OSCMessage msg("/ecg");
  msg.add((int32_t)irValue);
  msg.add(normalizedValue);
  msg.add(temperature);
  msg.add((int32_t)beatDetected);
  msg.add(bpm);
  msg.add(confidence);
  msg.add((int32_t)fingerPresent);
  msg.add((int32_t)sensorStatus);

  Udp.beginPacket(destIP, destPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 10000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Reconnecting...");
      connectToWiFi();
    }
    lastWiFiCheck = millis();
  }
}