#include "VL53L0XSensor.h"
#include <ArduinoJson.h>

extern ConfigHandler configHandler;

VL53L0XSensor::VL53L0XSensor(int xshutPin) : 
    xshutPin(xshutPin),
    sensorOK(false),
    rawDistance(0),
    filteredDistance(0),
    lastReadTime(0),
    readingCount(0),
    historyIndex(0),
    historyFull(false),
    calibrating(false),
    calibrationTarget(0),
    calibrationSamples(0),
    calibrationSum(0),
    offsetCorrection(0),
    scaleCorrection(1.0f),
    longRangeMode(false),
    timingBudget(50), // Default 50ms
    errorCount(0),
    lastErrorTime(0)
{
    // Initialize measurement history
    for (int i = 0; i < MEASUREMENT_HISTORY_SIZE; i++) {
        measurements[i] = 0;
    }
}

bool VL53L0XSensor::begin() {
    // Configure XSHUT pin if provided
    if (xshutPin >= 0) {
        pinMode(xshutPin, OUTPUT);
        powerUp(); // Ensure sensor is powered up
        delay(10); // Wait for boot
    }
    
    // Initialize sensor
    if (!lox.begin()) {
        DebugTools::verbosePrintln(configHandler.config, "Failed to initialize VL53L0X!");
        sensorOK = false;
        return false;
    }
    
    sensorOK = true;
    
    // Try to load calibration data
    if (!loadCalibrationFromSD()) {
        DebugTools::verbosePrintln(configHandler.config, "No calibration found, using defaults");
        resetCalibration();
    } else {
        DebugTools::verbosePrintln(configHandler.config, "Loaded calibration: offset=" + String(offsetCorrection) + 
                                     ", scale=" + String(scaleCorrection, 3));
    }
    
    // Apply sensor settings
    applySettings();
    
    return true;
}

void VL53L0XSensor::update() {
    // Skip if sensor isn't initialized
    if (!sensorOK) return;
    
    // Get current time
    unsigned long now = millis();
    
    // Try to take a measurement
    VL53L0X_RangingMeasurementData_t measure;
    
    if (lox.rangingTest(&measure, false)) { // false = no debug output
        if (measure.RangeStatus != 4) {  // Phase failures are errors
            // Valid reading
            rawDistance = measure.RangeMilliMeter;
            
            // Add to history for filtering
            addMeasurementToHistory(rawDistance);
            
            // Update filtered distance
            filteredDistance = calculateFilteredDistance();
            
            // Update timestamp
            lastReadTime = now;
            readingCount++;
            
            // Reset error counter
            errorCount = 0;
            
            // Handle calibration if active
            if (calibrating) {
                calibrationSum += rawDistance;
                calibrationSamples++;
                
                // Check if we have enough samples
                if (calibrationSamples >= 20) { // 20 samples for calibration
                    // Calculate average reading
                    float avgReading = (float)calibrationSum / calibrationSamples;
                    
                    // Calculate new offset correction
                    offsetCorrection = calibrationTarget - avgReading;
                    
                    // Calculate new scale correction (advanced)
                    // This assumes scale = 1.0 - will be refined with multi-point calibration
                    
                    // Save calibration
                    saveCalibrationToSD();
                    
                    // End calibration mode
                    calibrating = false;
                    
                    DebugTools::verbosePrintln(configHandler.config, "Calibration complete: offset=" + 
                                               String(offsetCorrection) + ", scale=" + String(scaleCorrection, 3));
                }
            }
        } else {
            // "Out of range" error
            handleMeasurementError();
        }
    } else {
        // Sensor communication error
        handleMeasurementError();
    }
}

uint16_t VL53L0XSensor::getDistance() const {
    return filteredDistance;
}

uint16_t VL53L0XSensor::getRawDistance() const {
    return rawDistance;
}



bool VL53L0XSensor::loadCalibrationFromSD(const char* filename) {
    // Check if SD is available
    if (!SD.exists(filename)) {
        return false;
    }
    
    // Open file
    File file = SD.open(filename);
    if (!file) {
        return false;
    }
    
    // Parse JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        DebugTools::verbosePrintln(configHandler.config, "JSON parse error in calibration file");
        return false;
    }
    
    // Load values
    offsetCorrection = doc["offset"] | 0;
    scaleCorrection = doc["scale"] | 1.0f;
    longRangeMode = doc["longRange"] | false;
    timingBudget = doc["timingBudget"] | 50;
    
    // Apply settings
    applySettings();
    
    return true;
}

bool VL53L0XSensor::saveCalibrationToSD(const char* filename) {
    // Create JSON document
    JsonDocument doc;
    
    // Store calibration values
    doc["offset"] = offsetCorrection;
    doc["scale"] = scaleCorrection;
    doc["longRange"] = longRangeMode;
    doc["timingBudget"] = timingBudget;
    
    // Open file for writing
    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        DebugTools::verbosePrintln(configHandler.config, "Failed to open calibration file for writing");
        return false;
    }
    
    // Serialize JSON to file
    if (serializeJson(doc, file) == 0) {
        file.close();
        DebugTools::verbosePrintln(configHandler.config, "Failed to write to calibration file");
        return false;
    }
    
    file.close();
    return true;
}

void VL53L0XSensor::startCalibration(uint16_t knownDistance) {
    calibrating = true;
    calibrationTarget = knownDistance;
    calibrationSamples = 0;
    calibrationSum = 0;
    
    DebugTools::verbosePrintln(configHandler.config, "Starting calibration with target distance: " + 
                               String(knownDistance) + "mm");
}

void VL53L0XSensor::cancelCalibration() {
    calibrating = false;
    DebugTools::verbosePrintln(configHandler.config, "Calibration cancelled");
}

bool VL53L0XSensor::isCalibrating() const {
    return calibrating;
}

bool VL53L0XSensor::calibrationComplete() const {
    return (calibrationSamples >= 20);
}

void VL53L0XSensor::resetCalibration() {
    offsetCorrection = 0;
    scaleCorrection = 1.0f;
    DebugTools::verbosePrintln(configHandler.config, "Calibration reset to defaults");
}

String VL53L0XSensor::getDebug() const {
    char buffer[80];
    sprintf(buffer, "LIDAR: %umm (raw:%u) [offset:%d scale:%.2f] %s %s", 
            filteredDistance, 
            rawDistance,
            offsetCorrection,
            scaleCorrection,
            isValid() ? "VALID" : "INVALID",
            calibrating ? "CALIBRATING" : "");
    return String(buffer);
}

void VL53L0XSensor::shutdown() {
    if (xshutPin >= 0) {
        digitalWrite(xshutPin, LOW);
        DebugTools::verbosePrintln(configHandler.config, "VL53L0X shutdown");
    }
}

void VL53L0XSensor::powerUp() {
    if (xshutPin >= 0) {
        digitalWrite(xshutPin, HIGH);
        delay(10); // Wait for device to power up
        DebugTools::verbosePrintln(configHandler.config, "VL53L0X power up");
    }
}

void VL53L0XSensor::setLongRangeMode(bool enable) {
    longRangeMode = enable;
    applySettings();
    DebugTools::verbosePrintln(configHandler.config, "Long range mode: " + String(enable ? "ON" : "OFF"));
}

bool VL53L0XSensor::getLongRangeMode() const {
    return longRangeMode;
}

void VL53L0XSensor::setTimingBudget(uint16_t budget) {
    // Validate budget (must be >= 20ms)
    if (budget < 20) budget = 20;
    
    timingBudget = budget;
    applySettings();
    DebugTools::verbosePrintln(configHandler.config, "Timing budget set to: " + String(budget) + "ms");
}

void VL53L0XSensor::applySettings() {
    if (!sensorOK) return;
    
    if (longRangeMode) {
        
        lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
            
    } else {
        lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    }
}

uint16_t VL53L0XSensor::applyCalibration(uint16_t rawValue) {
    // Apply offset correction
    int32_t corrected = rawValue + offsetCorrection;
    
    // Apply scale correction
    corrected = corrected * scaleCorrection;
    
    // Ensure we don't return negative values
    if (corrected < 0) corrected = 0;
    
    return (uint16_t)corrected;
}

void VL53L0XSensor::addMeasurementToHistory(uint16_t measurement) {
    // Add to circular buffer
    measurements[historyIndex] = measurement;
    
    // Update index
    historyIndex = (historyIndex + 1) % MEASUREMENT_HISTORY_SIZE;
    
    // Check if buffer is full
    if (!historyFull && historyIndex == 0) {
        historyFull = true;
    }
}

uint16_t VL53L0XSensor::calculateFilteredDistance() {
    // No filtering if history isn't populated
    if (!historyFull && historyIndex < 3) {
        return applyCalibration(rawDistance);
    }
    
    // Determine how many samples to use
    int sampleCount = historyFull ? MEASUREMENT_HISTORY_SIZE : historyIndex;
    
    // Simple median filter for robust outlier rejection
    uint16_t sortedValues[MEASUREMENT_HISTORY_SIZE];
    
    // Copy values for sorting
    for (int i = 0; i < sampleCount; i++) {
        sortedValues[i] = measurements[i];
    }
    
    // Sort values (insertion sort is fine for small arrays)
    for (int i = 1; i < sampleCount; i++) {
        uint16_t key = sortedValues[i];
        int j = i - 1;
        
        while (j >= 0 && sortedValues[j] > key) {
            sortedValues[j + 1] = sortedValues[j];
            j--;
        }
        
        sortedValues[j + 1] = key;
    }
    
    // Get median value
    uint16_t medianValue;
    if (sampleCount % 2 == 0) {
        // Even number of samples - average the middle two
        medianValue = (sortedValues[sampleCount/2 - 1] + sortedValues[sampleCount/2]) / 2;
    } else {
        // Odd number of samples - use middle value
        medianValue = sortedValues[sampleCount/2];
    }
    
    // Apply calibration to median value
    return applyCalibration(medianValue);
}

void VL53L0XSensor::handleMeasurementError() {
    // Increment error counter
    errorCount++;
    
    // Log error if it's the first one in a sequence
    if (errorCount == 1) {
        lastErrorTime = millis();
        DebugTools::verbosePrintln(configHandler.config, "VL53L0X measurement error");
    }
    
    // Check if we have persistent errors
    if (errorCount > 10 && (millis() - lastErrorTime > 5000)) {
        // Try to recover by reinitializing
        DebugTools::verbosePrintln(configHandler.config, "VL53L0X persistent errors - attempting recovery");
        
        if (xshutPin >= 0) {
            shutdown();
            delay(100);
            powerUp();
            delay(100);
        }
        
        if (lox.begin()) {
            applySettings();
            DebugTools::verbosePrintln(configHandler.config, "VL53L0X recovered");
            errorCount = 0;
        } else {
            DebugTools::verbosePrintln(configHandler.config, "VL53L0X recovery failed");
        }
    }
}
