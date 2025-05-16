#include <Arduino.h>
#include "config.h"
#include "joystick.h"
#include "calibration.h"
#include "RobotKinematics.h"
#include "kalmanfilter.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Display variable
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Joystick objects
Joystick* leftJoystick = nullptr;
Joystick* rightJoystick = nullptr;

// Robot kinematics and selected joint
RobotKinematics robotKin;
int selectedJoint = 0;

// Display modes
enum DisplayMode {
  JOYSTICK_INFO,
  ROBOT_BITMAP,
  KINEMATICS_TEST
};

DisplayMode currentDisplayMode = JOYSTICK_INFO;

// Simple robot configuration
RobotConfig robotConfig;

// Initialize robot configuration with default values
void initRobotConfig() {
  // Set joint angle limits (in degrees)
  for (int i = 0; i < 6; i++) {
    robotConfig.minAngle[i] = -180.0;
    robotConfig.maxAngle[i] = 180.0;
  }
  
  // Set DH parameters - simple 6DOF arm
  // Format: {a, alpha, d, theta}
  robotConfig.dh[0] = {0.0, M_PI/2, 50.0, 0.0};     // Base
  robotConfig.dh[1] = {80.0, 0.0, 0.0, M_PI/2};     // Shoulder
  robotConfig.dh[2] = {80.0, 0.0, 0.0, 0.0};        // Elbow
  robotConfig.dh[3] = {0.0, M_PI/2, 80.0, 0.0};     // Wrist pitch
  robotConfig.dh[4] = {0.0, -M_PI/2, 0.0, 0.0};     // Wrist roll
  robotConfig.dh[5] = {0.0, 0.0, 40.0, 0.0};        // Gripper
  
  // Tool offset
  robotConfig.toolOffsetX = 0.0;
  robotConfig.toolOffsetY = 0.0;
  robotConfig.toolOffsetZ = 30.0;
}

// Display the robot bitmap
void displayRobotArm() {
  display.clearDisplay();
  display.drawBitmap(0, 0, robotArmBitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
}

// Display kinematics test
void displayKinematicsTest() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Kinematik Test:");
  
  // Get current joint angles
  JointAngles angles = robotKin.getCurrentJointAngles();
  
  // Display joint angles in degrees
  for (int i = 0; i < 6; i++) {
    display.print("J");
    display.print(i+1);
    display.print(":");
    display.print(angles.angles[i] * 180.0 / M_PI, 0);
    display.print(" ");
    if (i == 2) display.println();
  }
  display.println();
  
  // Get current end effector position
  CartesianPose pose = robotKin.getCurrentPose();
  
  // Display end effector position
  display.print("X:");
  display.print(pose.x, 1);
  display.print(" Y:");
  display.println(pose.y, 1);
  display.print("Z:");
  display.println(pose.z, 1);
  
  // Display selected joint
  display.print("Selected: J");
  display.println(selectedJoint + 1);
  
  display.display();
}

void setup() {
  Serial.begin(115200);
  
  // Load pin config
  loadDefaultPinConfig();
  pinMode(_pinConfig.errorLedPin, OUTPUT);
  
  // Create joysticks
  leftJoystick = new Joystick(_pinConfig.leftXPin, _pinConfig.leftYPin, _pinConfig.leftBtnPin);
  rightJoystick = new Joystick(_pinConfig.rightXPin, _pinConfig.rightYPin, _pinConfig.rightBtnPin);

  // Initialize I2C
  Wire.begin();

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    digitalWrite(_pinConfig.errorLedPin, HIGH);
    Serial.println("Display initialization failed");
    while(1);
  }
  
  // Initialize display settings
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Initialize robot config and kinematics
  initRobotConfig();
  robotKin.init(robotConfig);
  
  // Show startup message
  display.setCursor(0,0);
  display.println("6DOF Robot Start");
  display.println("Druecke L-Joystick");
  display.println("fuer Kalibrierung");
  display.display();

  leftJoystick->begin();
  rightJoystick->begin();

  // Basic calibration
  Calibration::calibrateJoysticks();
  
  delay(2000);
  
  // Test: Force kinematic mode
  currentDisplayMode = KINEMATICS_TEST;
  
  // Display it once to confirm it works
  displayKinematicsTest();
}

void loop() {
  // Check for calibration trigger
  Calibration::checkCalibrationTrigger();
  
  if (!Calibration::isCalibrating()) {
    leftJoystick->read();
    rightJoystick->read();

    // Serial output
    Serial.print("Left X: "); Serial.print(leftJoystick->getX());
    Serial.print(" Y: "); Serial.print(leftJoystick->getY());
    Serial.print(" Btn: "); Serial.print(leftJoystick->isPressed());
    Serial.print(" | Right X: "); Serial.print(rightJoystick->getX());
    Serial.print(" Y: "); Serial.print(rightJoystick->getY());
    Serial.print(" Btn: "); Serial.println(rightJoystick->isPressed());

    // Mode toggling with both buttons
    static unsigned long lastToggleTime = 0;
    
    if (leftJoystick->isPressed() && rightJoystick->isPressed()) {
      if (millis() - lastToggleTime > 500) {
        // Cycle through modes
        int newMode = (int)currentDisplayMode + 1;
        if (newMode > 2) newMode = 0;
        currentDisplayMode = (DisplayMode)newMode;
        
        // Debug output
        Serial.print("Switching to mode: ");
        Serial.println(newMode);
        
        lastToggleTime = millis();
      }
    }
    
    // Kinematics control when in kinematics test mode
    if (currentDisplayMode == KINEMATICS_TEST) {
      // Joint selection with left joystick Y
      if (abs(leftJoystick->getY()) > 0.5 && millis() - lastToggleTime > 300) {
        if (leftJoystick->getY() > 0 && selectedJoint < 5) {
          selectedJoint++;
          lastToggleTime = millis();
          Serial.print("Selected joint: ");
          Serial.println(selectedJoint + 1);
        } else if (leftJoystick->getY() < 0 && selectedJoint > 0) {
          selectedJoint--;
          lastToggleTime = millis();
          Serial.print("Selected joint: ");
          Serial.println(selectedJoint + 1);
        }
      }
      
      // Joint angle control with right joystick Y
      float jointChange = rightJoystick->getY() * -0.03;  // Scale and invert
      
      if (abs(jointChange) > 0.001) {
        JointAngles angles = robotKin.getCurrentJointAngles();
        angles.angles[selectedJoint] += jointChange;
        
        // Check if joint angles are valid
        if (robotKin.isValidJointAngles(angles)) {
          robotKin.setCurrentJointAngles(angles);
        }
      }
    }
    
    // Update display based on current mode
    switch (currentDisplayMode) {
      case JOYSTICK_INFO:
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("L X: "); display.print(leftJoystick->getX());
        display.print(" Y: "); display.print(leftJoystick->getY());
        display.print(" B: "); display.println(leftJoystick->isPressed() ? "P" : "-");
        display.print("R X: "); display.print(rightJoystick->getX());
        display.print(" Y: "); display.print(rightJoystick->getY());
        display.print(" B: "); display.println(rightJoystick->isPressed() ? "P" : "-");
        display.display();
        break;
        
      case ROBOT_BITMAP:
        displayRobotArm();
        break;
        
      case KINEMATICS_TEST:
        displayKinematicsTest();
        break;
    }
  }

  delay(50);
}
