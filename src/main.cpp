// Include necessary libraries and files
#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_SSD1306.h>
#include "StateManager.h"
#include "MainMenuState.h"
#include "DriveModeSelectState.h"
#include "ConfigHandler.h"
#include "VL53L0XSensor.h"
#include "TrajectoryPlanner.h"
#include "AdxlTracker.h"
#include "KinematicsHandler.h"
#include "TaskTiming.h"
#include "DebugTools.h"
#include "robot_bitmaps.h"

// Define pin connections
#define JOYSTICKL_X_PIN 40
#define JOYSTICKL_Y_PIN 41
#define JOYSTICKR_X_PIN 38
#define JOYSTICKR_Y_PIN 39
#define BUTTONL_PIN 27
#define BUTTONR_PIN 26

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Global instances that will be used by all menu states
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ConfigHandler configHandler;
KinematicsHandler kinematics;
VL53L0XSensor* lidarSensor = nullptr;
TrajectoryPlanner* trajectoryPlanner = nullptr;
AdxlTracker* adxlTracker = nullptr;

// Input states
int joyLX = 512, joyLY = 512;
int joyRX = 512, joyRY = 512;
bool btnL = false, btnR = false;
bool prevBtnL = false, prevBtnR = false;

// Task timers
TaskTimer displayUpdateTimer;
TaskTimer inputPollTimer;
TaskTimer debugTimer;
TaskTimer debugTimer1;
TaskTimer debugTimer2;
TaskTimer debugTimer3;

// Function prototypes
void pollInputs();
void updateDisplay();

// Setup function
void setup() {
  Serial.begin(115200);
  Serial.println(F("6DOF Robot Arm Control System Starting..."));
  
  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();
  
  // Initialize SD card
  if(!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD card initialization failed!"));
    display.println(F("SD card failed!"));
    display.display();
    delay(1000);
  } else {
    display.println(F("SD card OK"));
    display.display();
  }
  
  // Initialize ConfigHandler
  if (!configHandler.loadFromSD()) {
    Serial.println(F("Failed to load config from SD, using defaults"));
    configHandler.resetToDefault();
    display.println(F("Using default config"));
  } else {
    display.println(F("Config loaded"));
  }
  display.display();
  
  // Set up debug pins if enabled
  if (configHandler.config.useDebugmode) {
    DebugTools::setupDebugPins(configHandler.config);
    display.println(F("Debug mode ON"));
    display.display();
  }
  
  // Initialize sensors - create objects first
  lidarSensor = new VL53L0XSensor(/* XSHUT_PIN = */ 36); // Specify your XSHUT pin here
  if (lidarSensor) {
    if (lidarSensor->begin()) {
      display.println(F("LIDAR OK"));
    } else {
      display.println(F("LIDAR failed!"));
    }
    display.display();
  }
  
  adxlTracker = new AdxlTracker();
  if (adxlTracker) {
    if (adxlTracker->begin()) {
      display.println(F("ADXL OK"));
    } else {
      display.println(F("ADXL failed!"));
    }
    display.display();
  }
  
  // Initialize trajectory planner
  trajectoryPlanner = new TrajectoryPlanner(&kinematics, &configHandler);
  
  // Initialize timers
  displayUpdateTimer.interval = 100;  // Update display every 100ms
  inputPollTimer.interval = 50;       // Poll input every 50ms
  debugTimer.interval = 1000;         // Debug output every 1000ms
  debugTimer1.interval = 250;         // Fast blink
  debugTimer2.interval = 500;         // Medium blink
  debugTimer3.interval = 1000;        // Slow blink
  
  // Input pin setup
  pinMode(BUTTONL_PIN, INPUT_PULLUP);
  pinMode(BUTTONR_PIN, INPUT_PULLUP);
  
  // Set initial state
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Starting menu..."));
  display.display();
  StateManager::setState(new MainMenuState());
  
  Serial.println(F("Setup complete!"));
}

// Function to poll inputs with deadzone
void pollInputs() {
  unsigned long now = millis();
  
  if (inputPollTimer.isDue(now)) {
    // Read inputs
    int newJoyLX = analogRead(JOYSTICKL_X_PIN);
    int newJoyLY = analogRead(JOYSTICKL_Y_PIN);
    int newJoyRX = analogRead(JOYSTICKR_X_PIN);
    int newJoyRY = analogRead(JOYSTICKR_Y_PIN);
    bool newBtnL = !digitalRead(BUTTONL_PIN);
    bool newBtnR = !digitalRead(BUTTONR_PIN);
    
    // Apply deadzone to joystick inputs
    const int DEADZONE = 50;
    
    // Check for significant joystick movement
    if (abs(newJoyLX - joyLX) > DEADZONE || abs(newJoyLY - joyLY) > DEADZONE) {
      // Convert to -512 to 511 range for easier directional logic
      StateManager::joystickLMoved(newJoyLX - 512, newJoyLY - 512);
      joyLX = newJoyLX;
      joyLY = newJoyLY;
    }
    
    if (abs(newJoyRX - joyRX) > DEADZONE || abs(newJoyRY - joyRY) > DEADZONE) {
      StateManager::joystickRMoved(newJoyRX - 512, newJoyRY - 512);
      joyRX = newJoyRX;
      joyRY = newJoyRY;
    }
    
    // Check for button press events (detect rising edge)
    if (newBtnL && !prevBtnL) {
      StateManager::buttonLPressed();
      if (configHandler.config.useDebugmode) {
        Serial.println(F("Left button pressed"));
      }
    }
    
    if (newBtnR && !prevBtnR) {
      StateManager::buttonRPressed();
      if (configHandler.config.useDebugmode) {
        Serial.println(F("Right button pressed"));
      }
    }
    
    // Update previous button states
    prevBtnL = newBtnL;
    prevBtnR = newBtnR;
    
    inputPollTimer.reset(now);
  }
}

// Main loop
void loop() {
  unsigned long now = millis();
  
  // Poll inputs
  pollInputs();
  
  // Update state manager
  if (displayUpdateTimer.isDue(now)) {
    StateManager::update();
    displayUpdateTimer.reset(now);
  }
  
  // Debug output
  if (debugTimer.isDue(now) && configHandler.config.useDebugmode) {
    Serial.printf("Current Mode: %s\n", configHandler.getDriveModeString());
    Serial.printf("JoyL: X=%d, Y=%d | JoyR: X=%d, Y=%d\n", 
                 joyLX - 512, joyLY - 512, joyRX - 512, joyRY - 512);
    debugTimer.reset(now);
  }
  
  // Handle debug pins if enabled
  if (configHandler.config.useDebugmode) {
    DebugTools::handleDebugPins(configHandler.config, 
                               debugTimer1, debugTimer2, debugTimer3, now);
  }
  
  // Delay to prevent excessive CPU usage
  delay(5);
}
