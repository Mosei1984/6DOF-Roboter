#include "AdxlTracker.h"

AdxlTracker::AdxlTracker(uint8_t sensor_id) 
    : accel(sensor_id), sensorID(sensor_id), isTracking(false), sensorInitialized(false),
      offsetX(0.0), offsetY(0.0), offsetZ(0.0),
      rawX(0.0), rawY(0.0), rawZ(0.0),
      filteredX(0.0), filteredY(0.0), filteredZ(0.0),
      roll(0.0), pitch(0.0), yaw(0.0),
      filterAlpha(0.2), isCalibrating(false), calibrationCount(0),
      calibSumX(0.0), calibSumY(0.0), calibSumZ(0.0),
      lastUpdateTime(0), lastDataValidCheckTime(0), updateFrequency(50)
{
    dataValid = false;
}

bool AdxlTracker::begin() {
    // Initialize the ADXL345 sensor
    if (!accel.begin()) {
        debugInfo = "Failed to connect to ADXL345!";
        sensorInitialized = false;
        return false;
    }
    
    sensorInitialized = true;
    
    // Set range to ±4g (other options: ADXL345_RANGE_2_G, ADXL345_RANGE_8_G, ADXL345_RANGE_16_G)
    accel.setRange(ADXL345_RANGE_4_G);
    
    // Load saved configuration
    if (!loadConfig()) {
        // If no saved config, use default offsets
        offsetX = 0.0;
        offsetY = 0.0;
        offsetZ = 0.0;
    }
    
    debugInfo = "ADXL345 initialized successfully";
    return true;
}

void AdxlTracker::startTracking() {
    if (!sensorInitialized) {
        debugInfo = "Cannot start tracking - sensor not initialized";
        return;
    }
    
    isTracking = true;
    lastUpdateTime = millis();
    lastDataValidCheckTime = millis();
    debugInfo = "Tracking started";
}

void AdxlTracker::stopTracking() {
    isTracking = false;
    dataValid = false;
    debugInfo = "Tracking stopped";
}

void AdxlTracker::update() {
    if (!sensorInitialized) {
        return;
    }
    
    if (!isTracking && !isCalibrating) {
        return;
    }
    
    // Check if it's time to update based on frequency
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < static_cast<unsigned long>(1000 / updateFrequency)) {
        return;
    }
    lastUpdateTime = currentTime;
    
    // Read acceleration data
    sensors_event_t event;
    accel.getEvent(&event);
    
    // Store raw values
    rawX = event.acceleration.x;
    rawY = event.acceleration.y;
    rawZ = event.acceleration.z;
    
    if (isCalibrating) {
        // Add to calibration sums
        calibSumX += rawX;
        calibSumY += rawY;
        calibSumZ += rawZ;
        calibrationCount++;
        
        // Update debug with calibration progress
        debugInfo = "Calibrating: " + String(calibrationCount) + "/" + String(calibrationSamples);
        
        // Check if calibration is complete
        if (calibrationCount >= calibrationSamples) {
            // Calculate average offsets
            offsetX = -calibSumX / calibrationCount;
            offsetY = -calibSumY / calibrationCount;
            offsetZ = -calibSumZ / calibrationCount;
            
            // Adjust Z offset to account for gravity (1g = ~9.8 m/s²)
            offsetZ += 9.8;
            
            // End calibration
            isCalibrating = false;
            calibrationCount = 0;
            calibSumX = calibSumY = calibSumZ = 0.0;
            
            // Save the calibration data
            saveConfig();
            
            debugInfo = "Calibration complete! Offsets: X=" + String(offsetX) + 
                        " Y=" + String(offsetY) + " Z=" + String(offsetZ);
        }
    } else {
        // Apply offsets to raw values
        applyOffsets();
        
        // Update filtered values
        updateFiltered();
        
        // Calculate orientation
        calculateOrientation();
        
        // Check data validity
        checkDataValidity();
        
        // Update debug info
        updateDebugInfo();
    }
}

void AdxlTracker::applyOffsets() {
    // Apply calibration offsets
    rawX += offsetX;
    rawY += offsetY;
    rawZ += offsetZ;
}

void AdxlTracker::updateFiltered() {
    // Apply low-pass filter
    filteredX = filterAlpha * rawX + (1.0 - filterAlpha) * filteredX;
    filteredY = filterAlpha * rawY + (1.0 - filterAlpha) * filteredY;
    filteredZ = filterAlpha * rawZ + (1.0 - filterAlpha) * filteredZ;
}

void AdxlTracker::calculateOrientation() {
    // Calculate roll and pitch (in degrees)
    // Roll = atan2(y, z) * 180/PI
    // Pitch = atan2(-x, sqrt(y*y + z*z)) * 180/PI
    roll = atan2(filteredY, filteredZ) * 180.0 / PI;
    pitch = atan2(-filteredX, sqrt(filteredY * filteredY + filteredZ * filteredZ)) * 180.0 / PI;
    
    // Note: Yaw cannot be determined from accelerometer alone
    // It would require a magnetometer or gyroscope
}

void AdxlTracker::checkDataValidity() {
    unsigned long currentTime = millis();
    
    // Schwellenwerte für die Validierung
    const float DATA_VALIDITY_THRESHOLD = 30.0f; // Max zulässige Beschleunigung (in m/s²)
    const unsigned long DATA_VALIDITY_TIMEOUT = 1000; // 1 Sekunde Timeout
    
    // Check if data seems valid (no excessive acceleration)
    bool accelerationValid = 
        abs(filteredX) < DATA_VALIDITY_THRESHOLD &&
        abs(filteredY) < DATA_VALIDITY_THRESHOLD &&
        abs(filteredZ) < DATA_VALIDITY_THRESHOLD + 10.0f; // Extra threshold for Z due to gravity
    
    // Check if roll/pitch values are reasonable
    bool orientationValid = 
        abs(roll) < 90.0f &&
        abs(pitch) < 90.0f;
    
    // Update data validity flag
    dataValid = accelerationValid && orientationValid;
    
    // Timeout check - if no valid data received for too long, mark as invalid
    if (!dataValid) {
        if (currentTime - lastDataValidCheckTime > DATA_VALIDITY_TIMEOUT) {
            dataValid = false;
        }
    } else {
        // Reset timer when valid data is received
        lastDataValidCheckTime = currentTime;
    }
}

bool AdxlTracker::isDataValid() {
    // Überprüfen, ob der Sensor initialisiert ist und Daten valide sind
    return sensorInitialized && isTracking && dataValid;
}

void AdxlTracker::updateDebugInfo() {
    // Create debug string with current values
    debugInfo = "Accel: X=" + String(filteredX, 2) + 
                " Y=" + String(filteredY, 2) + 
                " Z=" + String(filteredZ, 2) +
                " | Roll=" + String(roll, 1) +
                " Pitch=" + String(pitch, 1) +
                " | Valid=" + String(dataValid ? "YES" : "NO");
}

String AdxlTracker::getDebug() {
    return debugInfo;
}

void AdxlTracker::setManualOffsets(float x, float y, float z) {
    offsetX = x;
    offsetY = y;
    offsetZ = z;
    
    debugInfo = "Manual offsets set: X=" + String(offsetX) + 
                " Y=" + String(offsetY) + " Z=" + String(offsetZ);
    
    // Save the configuration
    saveConfig();
}

void AdxlTracker::saveConfig() {
    // Write a flag to indicate valid calibration data (0xAA55 magic number)
    uint32_t calibrationFlag = 0xAA55;
    EEPROM.put(EEPROM_ADDR_CALIBRATION_FLAG, calibrationFlag);
    
    // Write offset values
    EEPROM.put(EEPROM_ADDR_OFFSET_X, offsetX);
    EEPROM.put(EEPROM_ADDR_OFFSET_Y, offsetY);
    EEPROM.put(EEPROM_ADDR_OFFSET_Z, offsetZ);
    
    debugInfo = "Configuration saved to EEPROM";
}

bool AdxlTracker::loadConfig() {
    // Read calibration flag
    uint32_t calibrationFlag;
    EEPROM.get(EEPROM_ADDR_CALIBRATION_FLAG, calibrationFlag);
    
    // Check if the flag indicates valid data
    if (calibrationFlag != 0xAA55) {
        debugInfo = "No valid configuration found in EEPROM";
        return false;
    }
    
    // Read offset values
    EEPROM.get(EEPROM_ADDR_OFFSET_X, offsetX);
    EEPROM.get(EEPROM_ADDR_OFFSET_Y, offsetY);
    EEPROM.get(EEPROM_ADDR_OFFSET_Z, offsetZ);
    
    debugInfo = "Configuration loaded: X=" + String(offsetX) + 
                " Y=" + String(offsetY) + " Z=" + String(offsetZ);
    return true;
}

void AdxlTracker::startCalibration() {
    // Reset calibration variables
    isCalibrating = true;
    calibrationCount = 0;
    calibSumX = 0.0;
    calibSumY = 0.0;
    calibSumZ = 0.0;
    
    debugInfo = "Calibration started. Keep sensor flat and still.";
}
