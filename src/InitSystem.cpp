// InitSystem.cpp
#include "InitSystem.h"
#include "Robo_Config_V1.h"
#include <AccelStepper.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_VL53L0X.h>
#include <U8g2lib.h>

U8G2* displayPtr = nullptr;
// Define sensor objects globally
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(46); // Unique ID
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Sensor calibration values
float accelOffsetX = 0.0, accelOffsetY = 0.0, accelOffsetZ = 0.0;
float distanceOffset = 0.0;

// Sensor position on the robot (in cm, relative to wrist center)
const float ACCEL_OFFSET_X = -6.5;
const float ACCEL_OFFSET_Y = 4.0;
const float ACCEL_OFFSET_Z = 0.0;

// Distance sensor position
const float DIST_OFFSET_X = -7.0;
const float DIST_OFFSET_Y = -4.0;
const float DIST_OFFSET_Z = 0.0;

// Kalman filter variables for accelerometer
float accelKalmanX[2] = {0, 0.01}; // [state, uncertainty]
float accelKalmanY[2] = {0, 0.01};
float accelKalmanZ[2] = {0, 0.01};
const float accelProcessNoise = 0.01;
const float accelMeasurementNoise = 0.1;

// Kalman filter for distance sensor
float distanceKalman[2] = {0, 0.01}; // [state, uncertainty]
const float distProcessNoise = 0.01;
const float distMeasurementNoise = 0.5;

// Direction configuration
bool MOTOR_DIRECTION[6] = {true, true, true, true, true, true}; // Default is positive
bool HOMING_DIRECTION[6] = {false, true, true, false, false, false}; // Default is negative

// Zugriff auf Motor-Array
extern AccelStepper motors[6];

// Kalman filter implementation
float updateKalmanFilter(float* kalman, float measurement, float processNoise, float measurementNoise) {
  // Prediction step
  float predictedUncertainty = kalman[1] + processNoise;
  
  // Update step
  float kalmanGain = predictedUncertainty / (predictedUncertainty + measurementNoise);
  float newState = kalman[0] + kalmanGain * (measurement - kalman[0]);
  float newUncertainty = (1 - kalmanGain) * predictedUncertainty;
  
  // Store updated values
  kalman[0] = newState;
  kalman[1] = newUncertainty;
  
  return newState;
}

// Function to calibrate the accelerometer
void calibrateAccelerometer() {
  Serial.println("Calibrating accelerometer... Keep robot stationary!");
  
  // Take multiple readings and average them
  const int numSamples = 50;
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t event;
    adxl.getEvent(&event);
    
    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
    
    delay(20); // Short delay between readings
  }
  
  // Calculate average offsets (removing gravity from Z)
  accelOffsetX = sumX / numSamples;
  accelOffsetY = sumY / numSamples;
  accelOffsetZ = (sumZ / numSamples) - 9.8; // Remove gravity
  
  Serial.println("Accelerometer calibration complete!");
  Serial.print("Offsets - X: "); Serial.print(accelOffsetX);
  Serial.print(" Y: "); Serial.print(accelOffsetY);
  Serial.print(" Z: "); Serial.println(accelOffsetZ);
}

// Function to calibrate distance sensor
void calibrateDistanceSensor() {
  Serial.println("Calibrating VL53L0X distance sensor...");
  Serial.println("Place a flat surface under the sensor");
  delay(3000); // Give user time to position
  
  // Take multiple readings and average them
  const int numSamples = 20;
  float sumDist = 0;
  int validReadings = 0;
  
  for (int i = 0; i < numSamples; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    if (lox.rangingTest(&measure, false) == VL53L0X_ERROR_NONE) {
      if (measure.RangeStatus != 4) { // Valid reading
        sumDist += measure.RangeMilliMeter;
        validReadings++;
      }
    }
    delay(50);
  }
  
  if (validReadings > 0) {
    float avgDist = sumDist / validReadings;
    distanceOffset = avgDist - 341.4; // Assuming 10cm (100mm) reference
    
    Serial.println("Distance sensor calibration complete!");
    Serial.print("Offset: "); Serial.print(distanceOffset); Serial.println(" mm");
  } else {
    Serial.println("Failed to calibrate distance sensor - no valid readings");
  }
}

// Get filtered acceleration readings from accelerometer
bool getFilteredAcceleration(float* accelX, float* accelY, float* accelZ) {
  // Read raw sensor data
  sensors_event_t event;
  if (!adxl.getEvent(&event)) {
    return false; // Failed to read sensor
  }
  
  // Apply exponential smoothing filter
  static float filteredX = 0, filteredY = 0, filteredZ = 0;
  const float ALPHA = 0.2; // Smoothing factor (0-1)
  
  filteredX = ALPHA * event.acceleration.x + (1-ALPHA) * filteredX;
  filteredY = ALPHA * event.acceleration.y + (1-ALPHA) * filteredY;
  filteredZ = ALPHA * event.acceleration.z + (1-ALPHA) * filteredZ;
  
  // Set output values
  *accelX = filteredX;
  *accelY = filteredY;
  *accelZ = filteredZ;
  
  return true;
}

// Get filtered distance reading from time-of-flight sensor
float getFilteredDistance() {
  // Read raw distance
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  // Check if measurement is valid
  if (measure.RangeStatus != 4) { // 4 = out of range
    // Apply smoothing filter
    static float filteredDistance = 350.0;
    const float ALPHA = 0.3;
    
    filteredDistance = ALPHA * measure.RangeMilliMeter + (1-ALPHA) * filteredDistance;
    return filteredDistance;
  }
  
  // Return last valid measurement if current one is invalid
  return 0;
}
 void updateGripperFromPotentiometer() {
  // Read potentiometer and map to servo range
  int potValue = analogRead(SERVO_POTTY_PIN);
  int servoPos = map(potValue, 0, 1023, SERVO_MIN_POS, SERVO_MAX_POS);
  
  // Debug output - print every 500ms to avoid flooding serial
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500) {
    Serial.print("[");
    Serial.print(millis());
    Serial.print("] Gripper Debug - Pot Value: ");
    Serial.print(potValue);
    Serial.print(", Mapped Servo Pos: ");
    Serial.print(servoPos);
    Serial.print(", Min/Max: ");
    Serial.print(SERVO_MIN_POS);
    Serial.print("/");
    Serial.println(SERVO_MAX_POS);
    lastDebugTime = millis();
  }
  
  // Apply to gripper - FIXED: use gripper instead of gripperServo
  gripper.write(servoPos);
}
void initializeSystem() {
  Serial.begin(115200);
  delay(500);
  Serial.println("🟢 Initialisierung startet...");

  // Initialize I2C for sensors
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock for faster readings
  gripper.attach(SERVO_PIN);
  // Stepper & Endstops
  for (int i = 0; i < 6; i++) {
    pinMode(ENABLE_PINS[i], OUTPUT);
    digitalWrite(ENABLE_PINS[i], LOW);
    
    // Apply direction configuration to motor
    motors[i].setPinsInverted(MOTOR_DIRECTION[i] ? false : true);
    motors[i].setMaxSpeed(DEFAULT_MAX_SPEED);
    motors[i].setAcceleration(DEFAULT_ACCELERATION);
    pinMode(ENDSTOP_PINS[i], INPUT_PULLUP);
  }

  // Buttons
  pinMode(BUTTON_MODE,    INPUT_PULLUP);
  pinMode(BUTTON_CONFIRM, INPUT_PULLUP);
  pinMode(SERVO_POTTY_PIN, INPUT);
  // Initialize sensors with proper error handling
  Serial.println("Initializing ADXL345 accelerometer...");
  bool adxlSuccess = adxl.begin();
  
  if (adxlSuccess) {
    Serial.println("✅ ADXL345 accelerometer initialized");
    adxl.setRange(ADXL345_RANGE_4_G); // Set range to 4G for better sensitivity
    adxl.setDataRate(ADXL345_DATARATE_100_HZ); // 100Hz data rate
    
    // Run calibration
    
  } else {
    Serial.println("❌ Failed to initialize ADXL345 accelerometer");
  }
  
  Serial.println("Initializing VL53L0X distance sensor...");
  bool loxSuccess = lox.begin();
  
  if (loxSuccess) {
    Serial.println("✅ VL53L0X distance sensor initialized");
    lox.startRangeContinuous();
    
    // Run calibration
    
  } else {
    Serial.println("❌ Failed to initialize VL53L0X sensor");
  }
  
  Serial.println("✅ Initialisierung abgeschlossen.");
}
