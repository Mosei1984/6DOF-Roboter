#ifndef VL53L0X_SENSOR_H
#define VL53L0X_SENSOR_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <SD.h>
#include <SPI.h>
#include "TaskTiming.h"
#include "DebugTools.h"
#include "ConfigHandler.h"

// Measurement history for filtering and calibration
#define MEASUREMENT_HISTORY_SIZE 10

class VL53L0XSensor {
public:
    // Constructor with optional XSHUT pin
    VL53L0XSensor(int xshutPin = -1);
    
    // Initialization
    bool begin();
    
    // Main update method - call this regularly
    void update();
    
    // Get filtered distance in mm
    uint16_t getDistance() const;
    
    // Check if reading is valid
    bool isValid() const {
        // Überprüfen, ob der Sensor initialisiert ist und gültige Messwerte liefert
        return sensorOK && filteredDistance > 0 && filteredDistance < 2000 &&
               (millis() - lastReadTime) < 500; // Keine zu alten Messwerte
    }
    
    // Check if sensor is initialized properly
    bool isInitialized() const {
        return sensorOK;
    }
    
    // Load/save calibration from SD card
    bool loadCalibrationFromSD(const char* filename = "/lidar_cal.json");
    bool saveCalibrationToSD(const char* filename = "/lidar_cal.json");
    
    // Calibration methods
    void startCalibration(uint16_t knownDistance);
    void cancelCalibration();
    bool isCalibrating() const;
    bool calibrationComplete() const;
    
    // Reset to factory settings
    void resetCalibration();
    
    // Debug output
    String getDebug() const;
    
    // Manual control of XSHUT pin (power cycling)
    void shutdown();
    void powerUp();
    
    // Get raw (unfiltered) distance
    uint16_t getRawDistance() const;
    
    // Set and get long range mode
    void setLongRangeMode(bool enable);
    bool getLongRangeMode() const;
    
    // Set timing budget (measurement time)
    void setTimingBudget(uint16_t budget);

private:
    Adafruit_VL53L0X lox;
    int xshutPin;
    bool sensorOK;
    
    // Measurement data
    uint16_t rawDistance;
    uint16_t filteredDistance;
    unsigned long lastReadTime;
    uint16_t readingCount;
    
    // Measurement history for filtering
    uint16_t measurements[MEASUREMENT_HISTORY_SIZE];
    uint8_t historyIndex;
    bool historyFull;
    
    // Calibration variables
    bool calibrating;
    uint16_t calibrationTarget;
    uint16_t calibrationSamples;
    int32_t calibrationSum;
    int16_t offsetCorrection;
    float scaleCorrection;
    
    // Configuration
    bool longRangeMode;
    uint16_t timingBudget;
    
    // Error handling
    uint8_t errorCount;
    unsigned long lastErrorTime;
    
    // Internal methods
    void applySettings();
    uint16_t applyCalibration(uint16_t rawValue);
    void addMeasurementToHistory(uint16_t measurement);
    uint16_t calculateFilteredDistance();
    void handleMeasurementError();
};

#endif // VL53L0X_SENSOR_H
