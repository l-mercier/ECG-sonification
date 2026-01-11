#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "heartRateAdvanced.h"

// Structure of the OSC message :   IR value (f), Filtered IR value (f), normalized value, (f) temperature (f), beat Detected (0, 1)

class AdaptiveNormalizer {
  private:
    float buffer[200];
    int index = 0;
    float baselineMin = 100000;
    float baselineMax = 110000;
    bool initialized = false;
    
    // Fonction de comparaison pour qsort
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
      return deviations[100]; // Médiane des déviations
    }
    
  public:
    AdaptiveNormalizer() {
      // Initialiser le buffer avec des valeurs moyennes
      for(int i = 0; i < 200; i++) {
        buffer[i] = 105000;
      }
    }
    
    void addSample(float signal) {
      buffer[index] = signal;
      index = (index + 1) % 200;
      
      // Marquer comme initialisé après le premier tour complet
      if(index == 0) initialized = true;
      
      // Recalculer baseline tous les 50 échantillons
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
      
      // Éviter une plage trop petite
      if(mad < 50) mad = 50;
      
      baselineMin = median - 3 * mad;
      baselineMax = median + 3 * mad;
    }
    
    float normalize(float signal) {
      float center = (baselineMin + baselineMax) / 2.0;
      float range = (baselineMax - baselineMin) / 2.0;
      
      if(range < 10) range = 10; // Sécurité
      
      float normalized = (signal - center) / range;
      return constrain(normalized, -1.0, 1.0);
    }
    
    // Fonction utilitaire pour voir les bornes actuelles
    void printBaseline() {
      Serial.print("Baseline Min: ");
      Serial.print(baselineMin);
      Serial.print(" | Max: ");
      Serial.println(baselineMax);
    }
};

MAX30105 particleSensor;
AdaptiveNormalizer normalizer;
HeartRateDetector hrDetector;


const int bufferSize = 256;
float buffer[bufferSize];
int sampleCounter = 0;
const int processEveryNSamples = 1; // Adjust to reduce compute frequency
float lastFilteredIR = 0.0;

// Sensor Configuration 

byte ledBrightness = 60; //Options: 0=Off to 255=50mA
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 118; //Options: 69, 118, 215, 411
int adcRange = 8192; //Options: 2048, 4096, 8192, 16384


// WiFi configuration
const char* ssid = "riot-leo";
const char* password = "12345678";

IPAddress local_IP(10, 10, 0, 150);     // Must be inside DHCP range
IPAddress gateway(10, 10, 0, 254);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(1, 1, 1, 1);

// OSC destination
const char* destIP = "10.10.0.1";       // Receiving device IP (adjust if needed)
const int destPort = 8000;              // Receiving port
const int localPort = 12345;            // Port to send from

WiFiUDP Udp;

// === Preprocessing functions ===

void meanRemoval(float* signal, int len) {
  float sum = 0;
  for (int i = 0; i < len; i++) sum += signal[i];
  float mean = sum / len;
  for (int i = 0; i < len; i++) signal[i] -= mean;
}

void amplitudeNormalization(float* signal, int len) {
  float minVal = signal[0], maxVal = signal[0];
  for (int i = 1; i < len; i++) {
    if (signal[i] < minVal) minVal = signal[i];
    if (signal[i] > maxVal) maxVal = signal[i];
  }
  float range = maxVal - minVal;
  if (range == 0) return;
  for (int i = 0; i < len; i++) signal[i] = (signal[i] - minVal) / range;
}

void movingAverageFilter(float* signal, int len, int windowSize) {
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
  for (int i = 0; i < len; i++) signal[i] = temp[i];
  delete[] temp;
}

int detectPeak(const float* signal, int len, float threshold) {
  for (int i = 1; i < len - 1; i++) {
    if (signal[i] > signal[i - 1] && signal[i] > signal[i + 1] && signal[i] > threshold) {
      return i;
    }
  }
  return -1;
}

float highpassFilter(float input, float dt, float cutoff) {
  static float prevInput = 0, prevOutput = 0;
  float RC = 1.0 / (2.0 * PI * cutoff);
  float alpha = RC / (RC + dt);
  float output = alpha * (prevOutput + input - prevInput);
  prevInput = input;
  prevOutput = output;
  return output;
}

float lowpassFilter(float input, float dt, float cutoff) {
  static float prevOutput = 0;
  float RC = 1.0 / (2.0 * PI * cutoff);
  float alpha = dt / (RC + dt);
  float output = prevOutput + alpha * (input - prevOutput);
  prevOutput = output;
  return output;
}

float bandpassFilter(float input, float dt) {
  float hp = highpassFilter(input, dt, 0.5);    // remove < 0.5 Hz
  float bp = lowpassFilter(hp, dt, 150.0);      // remove > 150 Hz
  return bp;
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
    while (true); // Stop execution
  }
}

// === Sensor & Processing Setup ===

void setup() {
  Serial.begin(115200);
  delay(1000); // Let peripherals initialize properly
  connectToWiFi();

  if (!particleSensor.begin()) {
    Serial.println("MAX30102 not found. Check wiring/power.");
    Wire.end();           // Gracefully shut down the I2C bus
    delay(100);           // Give it time to settle
    ESP.restart();        // Soft reset the ESP32
  }

  // Configure the sensor: brightness, averaging, mode, rate, pulse width, range
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  for (int i = 0; i < bufferSize; i++) buffer[i] = 0; // init buffer

  Udp.begin(localPort);
}

// Main loop //

void loop() {
  const float dt = 1.0 / sampleRate;  // 1000 Hz sampling rate

  // 1. UNE SEULE lecture du capteur
  uint32_t irValue = particleSensor.getIR();
  float signal = (float)irValue;
  
  // 2. Normalisation adaptative
  normalizer.addSample(signal);
  float normalizedValue = normalizer.normalize(signal);
  
  // 3. Filtrage bandpass
  float filteredIR = bandpassFilter(signal, dt);

  // 4. Beat detection
  // bool beatDetected = checkForBeat(irValue);

  bool beatDetected = hrDetector.detectBeat(normalizedValue);

  // Bonus: vous pouvez maintenant obtenir le BPM et la confiance
  float bpm = hrDetector.getBPM();
  float confidence = hrDetector.getConfidence();

  // 5. Température (MOINS SOUVENT - c'est lent!)
  static int tempCounter = 0;
  static float temperature = 0;
  if (tempCounter++ >= 100) { // Une fois toutes les 100 lectures
    temperature = particleSensor.readTemperature();
    tempCounter = 0;
  }

  // Debug
  static int debugCounter = 0;
  if (debugCounter++ >= 200) {
    Serial.print("IR: "); Serial.print(irValue);
    Serial.print(" | Normalized: "); Serial.print(normalizedValue);
    Serial.print(" | BPM: "); Serial.print(bpm);
    Serial.print(" | Confidence: "); Serial.print(confidence * 100);
    Serial.print("% | Threshold: "); Serial.println(hrDetector.getThreshold());
    debugCounter = 0;
  }

  // Envoi OSC
  OSCMessage msg("/ecg");
  msg.add((int32_t)irValue);
  msg.add(normalizedValue);
  msg.add(temperature);
  msg.add((int32_t)beatDetected);
  msg.add(bpm);
  msg.add(confidence);

  Udp.beginPacket(destIP, destPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}
