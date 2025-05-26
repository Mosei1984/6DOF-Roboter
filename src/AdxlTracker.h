#ifndef ADXL_TRACKER_H
#define ADXL_TRACKER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <EEPROM.h>

// EEPROM addresses for storing configuration
#define EEPROM_ADDR_CALIBRATION_FLAG 0
#define EEPROM_ADDR_OFFSET_X 4
#define EEPROM_ADDR_OFFSET_Y 8
#define EEPROM_ADDR_OFFSET_Z 12

class AdxlTracker {
private:
    // ADXL345 sensor instance
    Adafruit_ADXL345_Unified accel;
    
    // Sensor ID
    uint8_t sensorID;
    
    // Tracking state
    bool isTracking;
    bool sensorInitialized;  // Status der Sensorinitialisierung
    bool dataValid;          // Gültigkeit der Sensordaten
    
    // Calibration values
    float offsetX;
    float offsetY;
    float offsetZ;
    
    // Raw acceleration values
    float rawX, rawY, rawZ;
    
    // Filtered acceleration values
    float filteredX, filteredY, filteredZ;
    
    // Orientation angles (in degrees)
    float roll, pitch, yaw;
    
    // Debug string
    String debugInfo;
    
    // Low-pass filter coefficient (0-1)
    float filterAlpha;
    
    // Calibration samples
    const int calibrationSamples = 100;
    bool isCalibrating;
    int calibrationCount;
    float calibSumX, calibSumY, calibSumZ;
    
    // Last update time
    unsigned long lastUpdateTime;
    unsigned long lastDataValidCheckTime;
    
    // Update frequency in Hz
    int updateFrequency;
    
    // Apply offsets to raw readings
    void applyOffsets();
    
    // Update filtered values
    void updateFiltered();
    
    // Calculate orientation angles
    void calculateOrientation();
    
    // Update debug info
    void updateDebugInfo();
    
    // Check data validity
    void checkDataValidity();

public:
    AdxlTracker(uint8_t sensor_id = 46);
    
    // Initialize the sensor
    bool begin();
    
    // Start tracking motion
    void startTracking();
    
    // Stop tracking motion
    void stopTracking();
    
    // Update sensor readings (call regularly in loop)
    void update();
    
    // Get debug information
    String getDebug();
    
    // Set manual calibration offsets
    void setManualOffsets(float x, float y, float z);
    
    // Save configuration to EEPROM
    void saveConfig();
    
    // Load configuration from EEPROM
    bool loadConfig();
    
    // Start calibration procedure
    void startCalibration();
    
    // Get current tracking status
    bool isCurrentlyTracking() { return isTracking; }
    
    // Get calibration status
    bool isCurrentlyCalibrating() { return isCalibrating; }
    
    // Neue Methode zur Prüfung der Datenvalidität
    bool isDataValid();
    
    // Get current orientation values
    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }
    
    // Get filtered acceleration values
    float getX() { return filteredX; }
    float getY() { return filteredY; }
    float getZ() { return filteredZ; }
    
    // Set update frequency
    void setUpdateFrequency(int freq) { updateFrequency = freq; }
    
    // Set filter alpha value
    void setFilterAlpha(float alpha) { 
        filterAlpha = constrain(alpha, 0.0, 1.0); 
    }
    
    // Get current offset values
    float getOffsetX() { return offsetX; }
    float getOffsetY() { return offsetY; }
    float getOffsetZ() { return offsetZ; }
};

#endif // ADXL_TRACKER_H
