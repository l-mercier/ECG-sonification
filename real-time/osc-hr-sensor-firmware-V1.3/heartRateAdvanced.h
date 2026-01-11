/*
 * Advanced Heart Rate Detection
 * Utilise un algorithme adaptatif avec seuil dynamique
 * Plus robuste que l'algorithme PBA de base
 */

#ifndef HEARTRATE_ADVANCED_H
#define HEARTRATE_ADVANCED_H

#include <Arduino.h>

class HeartRateDetector {
  private:
    // Paramètres du détecteur
    static const int HISTORY_SIZE = 20;
    float peakHistory[HISTORY_SIZE];
    int historyIndex = 0;
    bool historyFull = false;
    
    // États de détection
    float lastValue = 0;
    float lastDerivative = 0;
    unsigned long lastBeatTime = 0;
    unsigned long lastValidBeatTime = 0;
    
    // Seuils adaptatifs
    float dynamicThreshold = 0.3;
    float noiseFloor = 0.1;
    
    // Validation physiologique
    const unsigned long MIN_BEAT_INTERVAL = 300;   // 200 BPM max
    const unsigned long MAX_BEAT_INTERVAL = 2000;  // 30 BPM min
    
    // Filtre dérivée pour détection de pente
    static const int DERIVATIVE_WINDOW = 5;
    float derivativeBuffer[DERIVATIVE_WINDOW];
    int derivativeIndex = 0;
    
    // Statistiques temps réel
    float meanPeakHeight = 0;
    float stdPeakHeight = 0;
    int validBeatsCount = 0;
    
    // BPM calculation
    static const int BPM_HISTORY = 5;
    float bpmHistory[BPM_HISTORY];
    int bpmIndex = 0;
    
  public:
    HeartRateDetector() {
      for(int i = 0; i < HISTORY_SIZE; i++) peakHistory[i] = 0;
      for(int i = 0; i < DERIVATIVE_WINDOW; i++) derivativeBuffer[i] = 0;
      for(int i = 0; i < BPM_HISTORY; i++) bpmHistory[i] = 0;
    }
    
    // Fonction principale de détection
    bool detectBeat(float normalizedSignal) {
      unsigned long currentTime = millis();
      bool beatDetected = false;
      
      // Calculer la dérivée (taux de changement)
      float derivative = normalizedSignal - lastValue;
      derivativeBuffer[derivativeIndex] = derivative;
      derivativeIndex = (derivativeIndex + 1) % DERIVATIVE_WINDOW;
      
      // Détection de pic: signal élevé + dérivée qui passe de positive à négative
      bool isPeak = (normalizedSignal > dynamicThreshold) &&
                    (lastDerivative > 0.01) && 
                    (derivative < 0.01) &&
                    (normalizedSignal > noiseFloor);
      
      if (isPeak) {
        unsigned long timeSinceLastBeat = currentTime - lastBeatTime;
        
        // Validation physiologique: intervalle réaliste
        if (timeSinceLastBeat >= MIN_BEAT_INTERVAL && 
            timeSinceLastBeat <= MAX_BEAT_INTERVAL) {
          
          // Vérification amplitude: éviter les faux positifs
          if (isValidPeakAmplitude(normalizedSignal)) {
            beatDetected = true;
            validBeatsCount++;
            lastValidBeatTime = currentTime;
            
            // Mise à jour BPM
            float currentBPM = 60000.0 / timeSinceLastBeat;
            updateBPM(currentBPM);
            
            // Mise à jour statistiques
            addPeakToHistory(normalizedSignal);
            updateDynamicThreshold();
          }
        }
        
        lastBeatTime = currentTime;
      }
      
      // Mise à jour états
      lastValue = normalizedSignal;
      lastDerivative = derivative;
      
      // Reset si pas de battement depuis longtemps
      if (currentTime - lastValidBeatTime > 5000) {
        resetDetector();
      }
      
      return beatDetected;
    }
    
    // Obtenir le BPM lissé
    float getBPM() {
      if (validBeatsCount < 2) return 0;
      
      float sum = 0;
      int count = 0;
      for(int i = 0; i < BPM_HISTORY; i++) {
        if (bpmHistory[i] > 0) {
          sum += bpmHistory[i];
          count++;
        }
      }
      return count > 0 ? sum / count : 0;
    }
    
    // Obtenir confiance de la mesure (0-1)
    float getConfidence() {
      if (validBeatsCount < 3) return 0;
      
      // Confiance basée sur:
      // 1. Régularité des intervalles (faible std = haute confiance)
      // 2. Nombre de battements détectés
      // 3. Qualité du signal (rapport signal/bruit)
      
      float regularityScore = 1.0 - constrain(stdPeakHeight / (meanPeakHeight + 0.01), 0, 1);
      float countScore = constrain(validBeatsCount / 10.0, 0, 1);
      float snrScore = constrain((meanPeakHeight - noiseFloor) / 0.5, 0, 1);
      
      return (regularityScore + countScore + snrScore) / 3.0;
    }
    
    // Réinitialiser le détecteur
    void reset() {
      resetDetector();
    }
    
    // Obtenir le seuil actuel (pour debug)
    float getThreshold() {
      return dynamicThreshold;
    }
    
  private:
    void addPeakToHistory(float peakValue) {
      peakHistory[historyIndex] = peakValue;
      historyIndex = (historyIndex + 1) % HISTORY_SIZE;
      if (historyIndex == 0) historyFull = true;
    }
    
    void updateDynamicThreshold() {
      // Calculer moyenne et écart-type des pics
      float sum = 0;
      float sumSquared = 0;
      int count = historyFull ? HISTORY_SIZE : historyIndex;
      
      if (count < 3) return; // Pas assez de données
      
      for(int i = 0; i < count; i++) {
        sum += peakHistory[i];
        sumSquared += peakHistory[i] * peakHistory[i];
      }
      
      meanPeakHeight = sum / count;
      float variance = (sumSquared / count) - (meanPeakHeight * meanPeakHeight);
      stdPeakHeight = sqrt(max(0.0f, variance));
      
      // Seuil adaptatif: moyenne - 1.5 * écart-type
      // Plus conservateur que moyenne simple
      float calculatedThreshold = meanPeakHeight - 1.5f * stdPeakHeight;
      dynamicThreshold = (calculatedThreshold > 0.2f) ? calculatedThreshold : 0.2f;
      
      // Mise à jour du plancher de bruit
      float calculatedFloor = meanPeakHeight - 3.0f * stdPeakHeight;
      noiseFloor = (calculatedFloor > 0.05f) ? calculatedFloor : 0.05f;
    }
    
    bool isValidPeakAmplitude(float amplitude) {
      // Vérifier que le pic n'est pas un outlier
      if (validBeatsCount < 3) return true; // Accepter les premiers pics
      
      // Rejeter si trop différent de la moyenne
      float deviation = abs(amplitude - meanPeakHeight);
      return deviation < (3.0 * stdPeakHeight + 0.1);
    }
    
    void updateBPM(float newBPM) {
      // Filtrer les valeurs aberrantes
      if (newBPM < 30 || newBPM > 200) return;
      
      bpmHistory[bpmIndex] = newBPM;
      bpmIndex = (bpmIndex + 1) % BPM_HISTORY;
    }
    
    void resetDetector() {
      validBeatsCount = 0;
      dynamicThreshold = 0.3;
      noiseFloor = 0.1;
      historyIndex = 0;
      historyFull = false;
      bpmIndex = 0;
      
      for(int i = 0; i < HISTORY_SIZE; i++) peakHistory[i] = 0;
      for(int i = 0; i < BPM_HISTORY; i++) bpmHistory[i] = 0;
    }
};

#endif // HEARTRATE_ADVANCED_H