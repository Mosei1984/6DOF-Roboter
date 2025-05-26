#ifndef LIDAR_CALIBRATION_H
#define LIDAR_CALIBRATION_H

#include <Arduino.h>
#include "VL53L0XSensor.h"
#include "ConfigHandler.h"

class LidarCalibration {
public:
    LidarCalibration(VL53L0XSensor* sensor, ConfigHandler* config) 
        : lidarSensor(sensor), configHandler(config) {
        // Default values
        mountHeight = 300.0;      // mm - distance from robot base to sensor
        correctionFactor = 1.0;    // scaling factor
        offsetValue = 0.0;         // mm - systematic offset
    }
    
    // Perform automatic calibration procedure
    bool calibrateAuto() {
        Serial.println("Starting automatic LIDAR calibration...");
        Serial.println("Please place the robot arm at several known heights and follow prompts.");
        
        // Arrays to store measurements
        const int numPoints = 5;
        float knownHeights[numPoints];
        float measuredDistances[numPoints];
        
        // Collect calibration points
        for (int i = 0; i < numPoints; i++) {
            Serial.print("Position the robot at a known height and enter the height in mm: ");
            while (!Serial.available()) {
                delay(100); // Wait for input
            }
            knownHeights[i] = Serial.parseFloat();
            Serial.println(knownHeights[i]);
            
            // Clear buffer
            while (Serial.available()) Serial.read();
            
            // Take multiple measurements and average them
            const int samples = 10;
            float sum = 0;
            for (int j = 0; j < samples; j++) {
                lidarSensor->update();
                sum += lidarSensor->getDistance();
                delay(50);
            }
            measuredDistances[i] = sum / samples;
            
            Serial.print("Measured distance: ");
            Serial.print(measuredDistances[i]);
            Serial.println(" mm");
        }
        
        // Calculate calibration parameters using linear regression
        // y = mx + b where y is actual height, x is measured distance
        // m is correction factor, b is mount height
        float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < numPoints; i++) {
            sumX += measuredDistances[i];
            sumY += knownHeights[i];
            sumXY += measuredDistances[i] * knownHeights[i];
            sumX2 += measuredDistances[i] * measuredDistances[i];
        }
        
        correctionFactor = (numPoints * sumXY - sumX * sumY) / (numPoints * sumX2 - sumX * sumX);
        mountHeight = (sumY - correctionFactor * sumX) / numPoints;
        
        Serial.println("Calibration complete!");
        Serial.print("Mount height: ");
        Serial.print(mountHeight);
        Serial.println(" mm");
        Serial.print("Correction factor: ");
        Serial.println(correctionFactor, 4);
        
        return true;
    }
    
    // Manual calibration with known values
    void calibrateManual(float newMountHeight, float newCorrectionFactor, float newOffset = 0.0) {
        mountHeight = newMountHeight;
        correctionFactor = newCorrectionFactor;
        offsetValue = newOffset;
        
        Serial.println("Manual calibration set:");
        Serial.print("Mount height: ");
        Serial.print(mountHeight);
        Serial.println(" mm");
        Serial.print("Correction factor: ");
        Serial.println(correctionFactor, 4);
        Serial.print("Offset: ");
        Serial.println(offsetValue);
    }
    
    // Test calibration with current settings
    void testCalibration() {
        Serial.println("Testing calibration...");
        Serial.println("Press any key to stop testing.");
        
        while (!Serial.available()) {
            lidarSensor->update();
            float rawDistance = lidarSensor->getDistance();
            float calculatedHeight = calculateHeight(rawDistance);
            
            Serial.print("Raw: ");
            Serial.print(rawDistance);
            Serial.print(" mm | Calculated height: ");
            Serial.print(calculatedHeight);
            Serial.println(" mm");
            
            delay(500);
        }
        
        // Clear buffer
        while (Serial.available()) Serial.read();
    }
    
    // Save calibration to EEPROM
    void saveCalibration() {
        // Create EEPROM addresses for these values
        const int EEPROM_MOUNT_HEIGHT = 100;
        const int EEPROM_CORRECTION_FACTOR = 104;
        const int EEPROM_OFFSET_VALUE = 108;
        
        EEPROM.put(EEPROM_MOUNT_HEIGHT, mountHeight);
        EEPROM.put(EEPROM_CORRECTION_FACTOR, correctionFactor);
        EEPROM.put(EEPROM_OFFSET_VALUE, offsetValue);
        
        Serial.println("Calibration saved to EEPROM");
    }
    
    // Load calibration from EEPROM
    void loadCalibration() {
        // Create EEPROM addresses for these values
        const int EEPROM_MOUNT_HEIGHT = 100;
        const int EEPROM_CORRECTION_FACTOR = 104;
        const int EEPROM_OFFSET_VALUE = 108;
        
        EEPROM.get(EEPROM_MOUNT_HEIGHT, mountHeight);
        EEPROM.get(EEPROM_CORRECTION_FACTOR, correctionFactor);
        EEPROM.get(EEPROM_OFFSET_VALUE, offsetValue);
        
        Serial.println("Calibration loaded from EEPROM:");
        Serial.print("Mount height: ");
        Serial.print(mountHeight);
        Serial.println(" mm");
        Serial.print("Correction factor: ");
        Serial.println(correctionFactor, 4);
        Serial.print("Offset: ");
        Serial.println(offsetValue);
    }
    
    // Calculate actual height from raw distance
    float calculateHeight(float rawDistance) {
        return mountHeight - (rawDistance * correctionFactor + offsetValue);
    }
    
    // Getters for calibration parameters
    float getMountHeight() const { return mountHeight; }
    float getCorrectionFactor() const { return correctionFactor; }
    float getOffsetValue() const { return offsetValue; }

private:
    VL53L0XSensor* lidarSensor;
    ConfigHandler* configHandler;
    float mountHeight;      // mm - distance from robot base to sensor
    float correctionFactor; // scaling factor
    float offsetValue;      // mm - systematic offset
};

#endif // LIDAR_CALIBRATION_H