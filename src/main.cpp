// main.cpp
#include <AccelStepper.h>
#include <IntervalTimer.h>
#include <U8g2lib.h>
#include <Arduino.h>
#include <MultiStepper.h>
#include "Robo_Config_V1.h"
#include "StepperControl.h"
#include "ForwardKinematics.h"
#include "InverseKinematics.h"
#include "Homing.h"
#include "InitSystem.h"
#include "JointMode.h"

// --- 1) AccelStepper-Instanzen ---
AccelStepper motors[6] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[4], DIR_PINS[4]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[5], DIR_PINS[5])
};

// Global variables initialization (for those not defined in InitSystem.cpp)
Servo gripper;
unsigned long lastJoystickCheck = 0;

AccelStepper* steppers = motors;
MultiStepper  multiStepperGroup;
long          multiTargetSteps[6];

// --- 2) Timer & Status ---
IntervalTimer stepTimer;
volatile uint8_t activeMotor = 0;


Mode    currentMode         = MODE_INDIVIDUAL_MOTOR;
bool    buttonModePressed   = false;
bool    buttonConfirmPressed= false;
bool    homing              = false;
bool    motorsEnabled       = true;
float   currentSpeed        = 0;

// --- 3) Joystick-Kalibrierung ---
int16_t joyLXCenter, joyLYCenter, joyRZCenter, joyRYawCenter;
int   joystick_lx_samples[JOYSTICK_SMOOTHING];
int   joystick_ly_samples[JOYSTICK_SMOOTHING];
int   joystick_rz_samples[JOYSTICK_SMOOTHING];
int   joystick_ryaw_samples[JOYSTICK_SMOOTHING];
int   sample_index = 0;
int   joystick_lx, joystick_ly, joystick_rz, joystick_ryaw;
// Make sure this is defined just once
extern unsigned long lastJoystickCheck;
// --- 4) Servo & Display ---

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);


// --- ISR für jitter-freie STEP-Impulse ---
void stepISR() {
  updateSteppers();
}

// Add this function to start a coordinated movement for IK/FK modes
void startCoordinatedMove(double targetJointAngles[6], unsigned long duration) {
  int servoPos = map(analogRead(SERVO_POTTY_PIN), 0, 1023, SERVO_MIN_POS, SERVO_MAX_POS);
  gripper.write(servoPos);
  // Convert joint angles to motor steps
  for (int i = 0; i < 6; i++) {
    double targetDegrees = targetJointAngles[i] * 180.0 / M_PI;
    
    // Calculate target steps using steps-per-degree conversion
    float stepsPerDegree = (BASE_STEPS * gearRatios[i]) / 360.0;
    long targetSteps = targetDegrees * stepsPerDegree;
    
    // Store current and target positions
    coordMoveStartSteps[i] = motors[i].currentPosition();
    coordMoveTargetSteps[i] = targetSteps;
    
    // Debug info
  }
  
  // Set up timing
  coordMoveStartTime = millis();
  coordMoveDuration = duration;
  
  // Activate coordinated movement
  coordMoveActive = true;
}

// Function prototypes
void readInputs();
void handleMotorControl();
void handleJointControl();
void handleHoming();
void handleServoTest();
void enableMotors(bool enable);
void resetMotorSettings();
float calculateSpeedFromJoystick(int joystickValue, int centerValue);
void readJoystickValues();


void setup() {
  Serial.begin(115200);
  delay(200);
  displayPtr = &u8g2;
  // --- Joystick kalibrieren (Mittelwert aus 20 Messungen) ---
  long sumLX = 0, sumLY = 0, sumRZ = 0, sumRYaw = 0;
  for (int i = 0; i < 20; i++) {
    sumLX += analogRead(JOYSTICK_L_X);
    sumLY += analogRead(JOYSTICK_L_Y);
    sumRZ += analogRead(JOYSTICK_R_Z);
    sumRYaw += analogRead(JOYSTICK_R_YAW);
    delay(5);
  }
  
  joyLXCenter = sumLX / 20;
  joyLYCenter = sumLY / 20;
  joyRZCenter = sumRZ / 20;
  joyRYawCenter = sumRYaw / 20;
  
  
  // Initialize smoothing arrays
  for (int i = 0; i < JOYSTICK_SMOOTHING; i++) {
    joystick_lx_samples[i] = joyLXCenter;
    joystick_ly_samples[i] = joyLYCenter;
    joystick_rz_samples[i] = joyRZCenter;
    joystick_ryaw_samples[i] = joyRYawCenter;
  }
  
  // --- Pins initialisieren ---
  for (int i = 0; i < 6; i++) {
    // Enable-Pins aktiv (LOW = enabled)
    pinMode(ENABLE_PINS[i], OUTPUT);
    digitalWrite(ENABLE_PINS[i], LOW);
    
    // Endstop-Pins
    pinMode(ENDSTOP_PINS[i], INPUT_PULLUP);
    
    // STEP/DIR-Pins
    pinMode(STEP_PINS[i], OUTPUT);
    pinMode(DIR_PINS[i],  OUTPUT);
  }
  
  pinMode(BUTTON_MODE,    INPUT_PULLUP);
  pinMode(BUTTON_CONFIRM, INPUT_PULLUP);
  
  // Initialize motors
  for (int i = 0; i < 6; i++) {
    motors[i].setMaxSpeed(MAX_SPEED);
    motors[i].setAcceleration(MAX_ACCEL);
  }
  
  // Initialize MultiStepper for coordinated movement
  multiStepperGroup.addStepper(motors[0]);
  multiStepperGroup.addStepper(motors[1]);
  multiStepperGroup.addStepper(motors[2]);
  multiStepperGroup.addStepper(motors[3]);
  multiStepperGroup.addStepper(motors[4]);
  multiStepperGroup.addStepper(motors[5]);
  
  
  // Initialize servo
  gripper.attach(SERVO_PIN);
  gripper.write(SERVO_DEFAULT_POS);
  
  // --- OLED starten ---
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0, 12);
  u8g2.print("6DOF Robot Controller");
  u8g2.setCursor(0, 28);
  u8g2.print("Initializing...");
  u8g2.sendBuffer();
  displayPtr = &u8g2;
  // Initialize sensors
  initializeSystem();
  
  // --- Timer starten: alle 20 µs → bis 50 kHz Flankenrate ---
  stepTimer.begin(stepISR, 20);
  
  delay(500);
}

void loop() {
  // Read joystick values with smoothing
  readJoystickValues();
  
  // Read button inputs
  readInputs();
   // Update gripper position from potentiometer

  
  // Process mode selection
  if (buttonModePressed) {
    currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % MODE_COUNT);
    buttonModePressed = false;
    resetMotorSettings();
  }
  
  // Process current mode logic
  unsigned long ikStartTime = 0;
  
  switch (currentMode) {
    case MODE_INDIVIDUAL_MOTOR:
      handleMotorControl();
      updateDisplay();
      break;
    case MODE_JOINT_CONTROL:
      // Call joint mode handler from JointMode.cpp
      handleJointMode();
      break;
    case MODE_HOMING:
      handleHoming();
      // Remove any calls to updateDisplay() or updateHomingDisplay() here
      break;
    case MODE_SERVO_TEST:
      handleServoTest();
      updateDisplay();
      break;
    case MODE_FORWARD_KINEMATICS:
      // Call the function from ForwardKinematics.cpp
      handleForwardKinematicsMode();
      break;
    case MODE_INVERSE_KINEMATICS:
      // Record start time for timeout monitoring
      ikStartTime = millis();
      
      // Call the function from InverseKinematics.cpp
      handleInverseKinematicsMode();
      
      // Check if it took too long
      if (millis() - ikStartTime > 500) {
      }
      break;
    case MODE_COUNT:
      // Handle this case to avoid warnings
      currentMode = MODE_INDIVIDUAL_MOTOR;
      break;
  }
}

void readJoystickValues() {
  // Read and store new samples
  joystick_lx_samples[sample_index] = analogRead(JOYSTICK_L_X);
  joystick_ly_samples[sample_index] = analogRead(JOYSTICK_L_Y);
  joystick_rz_samples[sample_index] = analogRead(JOYSTICK_R_Z);
  joystick_ryaw_samples[sample_index] = analogRead(JOYSTICK_R_YAW);
  
  // Update sample index
  sample_index = (sample_index + 1) % JOYSTICK_SMOOTHING;
  
  // Calculate smoothed values
  long lx_sum = 0, ly_sum = 0, rz_sum = 0, ryaw_sum = 0;
  for (int i = 0; i < JOYSTICK_SMOOTHING; i++) {
    lx_sum += joystick_lx_samples[i];
    ly_sum += joystick_ly_samples[i];
    rz_sum += joystick_rz_samples[i];
    ryaw_sum += joystick_ryaw_samples[i];
  }
  
  joystick_lx = lx_sum / JOYSTICK_SMOOTHING;
  joystick_ly = ly_sum / JOYSTICK_SMOOTHING;
  joystick_rz = rz_sum / JOYSTICK_SMOOTHING;
  joystick_ryaw = ryaw_sum / JOYSTICK_SMOOTHING;
}

void readInputs() {
  static unsigned long lastModeButtonTime = 0;
  static unsigned long lastDoubleClickTime = 0;
  static bool confirmHeld = false;
  static int clickCount = 0;
  
  // Check if confirm button is held down (for combination)
  confirmHeld = !digitalRead(BUTTON_CONFIRM);
  
  // Handle mode button with debounce
  if (!digitalRead(BUTTON_MODE) && millis() - lastModeButtonTime > 300) {
    // Register the click
    clickCount++;
    lastModeButtonTime = millis();
    
    // If this is the first click, start the double click timer
    if (clickCount == 1) {
      lastDoubleClickTime = millis();
    }
    
    // If second click within 500ms, it's a double click
    if (clickCount == 2 && millis() - lastDoubleClickTime < 500) {
      // Double click detected - change modes
      buttonModePressed = true;
      clickCount = 0; // Reset click count after handling
    }
    
    // If confirm is held while pressing mode, change modes (works in any mode)
    else if (confirmHeld) {
      buttonModePressed = true;
      clickCount = 0; // Reset click count after handling
    }
    // Otherwise, if in individual motor mode, cycle motors
    else if (currentMode == MODE_INDIVIDUAL_MOTOR && clickCount == 1) {
      activeMotor = (activeMotor + 1) % 6;
    }
  }
  
  // Reset click count if too much time has passed since first click (500ms)
  if (clickCount == 1 && millis() - lastDoubleClickTime > 500) {
    clickCount = 0;
  }
  
  // Handle confirm button for regular functions (with debounce)
  static unsigned long lastConfirmButtonTime = 0;
  if (confirmHeld && millis() - lastConfirmButtonTime > 300 && digitalRead(BUTTON_MODE)) {
    // Only trigger confirm actions when mode button is NOT pressed
    buttonConfirmPressed = true;
    lastConfirmButtonTime = millis();
  } else {
    buttonConfirmPressed = false;
  }
}
// Update gripper position from potentiometer

float calculateSpeedFromJoystick(int joystickValue, int centerValue) {
  // Calculate distance from center
  int16_t distance = joystickValue - centerValue;
  
  // Apply deadzone
  if (abs(distance) <= DEADZONE) {
    return 0;
  }
  
  // Normalize to -1.0 to 1.0 range, accounting for deadzone
  float norm = (distance > 0)
    ? float(distance - DEADZONE) / (1023.0f - DEADZONE - centerValue)
    : float(distance + DEADZONE) / (centerValue - DEADZONE);
  
  // Map to speed range with minimum practical speed
  float speed = norm * MAX_SPEED;
  
  // Apply minimum practical speed if needed
  if (abs(speed) < MIN_PRACTICAL_SPEED && abs(speed) > 0.1) {
    speed = (speed > 0) ? MIN_PRACTICAL_SPEED : -MIN_PRACTICAL_SPEED;
  }
  
  return speed;
}

void handleMotorControl() {
  // Individual motor control mode
  if (buttonConfirmPressed) {
    motors[activeMotor].setCurrentPosition(0);
    buttonConfirmPressed = false;
  }
  
  // Calculate speed from joystick position
  float targetSpeed = calculateSpeedFromJoystick(joystick_ly, joyLYCenter);
  
  // Apply to active motor
  motors[activeMotor].setSpeed(targetSpeed);
  currentSpeed = targetSpeed;
}

void handleJointControl() {
  int servoPos = map(analogRead(SERVO_POTTY_PIN), 0, 1023, SERVO_MIN_POS, SERVO_MAX_POS);
  gripper.write(servoPos);
  // Joint control mode - multiple motors at once
  // Map joysticks to different motors
  float speedX = calculateSpeedFromJoystick(joystick_lx, joyLXCenter);
  float speedY = calculateSpeedFromJoystick(joystick_ly, joyLYCenter);
  float speedZ = calculateSpeedFromJoystick(joystick_rz, joyRZCenter);
  float speedYaw = calculateSpeedFromJoystick(joystick_ryaw, joyRYawCenter);
  
  // Apply speeds to motors
  motors[0].setSpeed(speedX);
  motors[1].setSpeed(speedY);
  motors[2].setSpeed(speedZ);
  motors[3].setSpeed(speedYaw);
  motors[4].setSpeed(speedX);
  motors[5].setSpeed(speedY);
  
  // Reset position if confirm button pressed
  if (buttonConfirmPressed) {
    for (int i = 0; i < 6; i++) {
      motors[i].setCurrentPosition(0);
    }
    buttonConfirmPressed = false;
  }
  
  currentSpeed = max(abs(speedX), max(abs(speedY), max(abs(speedZ), abs(speedYaw))));
}

void handleHoming() {
  static unsigned long lastDisplayUpdate = 0;
  static bool displayInitialized = false;
  
  // Homing mode - press confirm to start/stop homing
  if (buttonConfirmPressed) {
    homing = !homing; // Toggle homing state
    buttonConfirmPressed = false;
    
    if (homing) {
      startHoming();
    } else {
      stopHoming();
    }
  }
  
  // Process homing if active
  if (homing) {
    processHoming();
    
    // Check if all homing is complete
    if (isHomingComplete()) {
      homing = false;
      moveToHomePose();  

    }
  }
  
  // DISPLAY MANAGEMENT - Only update every 1 second
  if (millis() - lastDisplayUpdate > 1000 || !displayInitialized) {
    lastDisplayUpdate = millis();
    displayInitialized = true;
    
    // Start display update
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tr); // Smaller font to fit more text
    
    // Header
    u8g2.setCursor(0, 7);
    u8g2.print("HOMING");
    u8g2.setCursor(50, 7);
    u8g2.print(homing ? "ACTIVE" : "READY");
    
    // Progress bar
    int progress = getHomingProgress();
    u8g2.setCursor(100, 7);
    u8g2.print(progress);
    u8g2.print("%");
    
    u8g2.drawFrame(0, 9, 128, 4);
    u8g2.drawBox(1, 10, (progress * 126) / 100, 2);
    
    // Helper function for state text - fixed missing semicolon
    auto getShortStateText = [](HomingState state) -> const char* {
      switch (state) {
        case HomingState::IDLE: return "IDLE";
        case HomingState::FAST_APPROACH: return "FAST";
        case HomingState::BACKING_OFF: return "BACK";
        case HomingState::SLOW_APPROACH: return "SLOW";
        case HomingState::COMPLETED: return "DONE";
        case HomingState::ERROR: return "ERR!";
        default: return "????";
      }
    };
    
    // First row - axes 1-3
    for (int i = 0; i < 3; i++) {
      u8g2.setCursor(i*42, 20);
      u8g2.print(i+1);
      u8g2.print(":");
      u8g2.print(getShortStateText(axisHomingState[i]));
      
      // Show endstop indicator
      if (digitalRead(ENDSTOP_PINS[i]) == HIGH) {
        u8g2.print("*");
      }
    }
    
    // Second row - axes 4-6
    for (int i = 3; i < 6; i++) {
      u8g2.setCursor((i-3)*42, 30);
      u8g2.print(i+1);
      u8g2.print(":");
      u8g2.print(getShortStateText(axisHomingState[i]));
      
      // Show endstop indicator
      if (digitalRead(ENDSTOP_PINS[i]) == HIGH) {
        u8g2.print("*");
      }
    }
    
    // Fixed: Define showingDetail before using it
    bool showingDetail = false;
    
    // Show details for any axis in SLOW_APPROACH
    for (int i = 0; i < 6; i++) {
      if (axisHomingState[i] == HomingState::SLOW_APPROACH) {
        u8g2.setCursor(0, 40);
        u8g2.print("Axis ");
        u8g2.print(i+1);
        u8g2.print(": SLOW APPROACH");
        showingDetail = true;
        break;  // Only show one to avoid clutter
      }
    }
    
    // If no axis in SLOW_APPROACH, show any in FAST_APPROACH
    if (!showingDetail) {
      for (int i = 0; i < 6; i++) {
        if (axisHomingState[i] == HomingState::FAST_APPROACH) {
          u8g2.setCursor(0, 40);
          u8g2.print("Axis ");
          u8g2.print(i+1);
          u8g2.print(": FAST APPROACH");
          showingDetail = true;
          break;
        }
      }
    }
    
    // If no axis in APPROACH phases, show any in BACKING_OFF
    if (!showingDetail) {
      for (int i = 0; i < 6; i++) {
        if (axisHomingState[i] == HomingState::BACKING_OFF) {
          u8g2.setCursor(0, 40);
          u8g2.print("Axis ");
          u8g2.print(i+1);
          u8g2.print(": BACKING OFF");
          showingDetail = true;
          break;
        }
      }
    }
    
    // Instructions
    u8g2.setCursor(0, 55);
    u8g2.print(homing ? "Press to STOP" : "Press to START");
    
    // Finalize
    u8g2.sendBuffer();
  }
}

void handleServoTest() {
  // Servo test mode - use joystick to control servo
  int servoPos = map(analogRead(SERVO_POTTY_PIN), 0, 1023, SERVO_MIN_POS, SERVO_MAX_POS);
  gripper.write(servoPos);
  
  // Toggle motors enable/disable with confirm button
  if (buttonConfirmPressed) {
    motorsEnabled = !motorsEnabled;
    enableMotors(motorsEnabled);
    buttonConfirmPressed = false;
  }
}

void enableMotors(bool enable) {
  for (int i = 0; i < 6; i++) {
    digitalWrite(ENABLE_PINS[i], enable ? LOW : HIGH); // LOW active
  }
}

void resetMotorSettings() {
  // Stop all motors
  for (int i = 0; i < 6; i++) {
    motors[i].setSpeed(0);
  }
  
  // Reset mode-specific variables
  if (currentMode == MODE_INDIVIDUAL_MOTOR) {
    // In motor control mode, default to first motor
    activeMotor = 0;
  } else if (currentMode == MODE_JOINT_CONTROL) {
    // Initialize Joint mode
    initJointMode(&u8g2);
  } else if (currentMode == MODE_HOMING) {
    // In homing mode, make sure homing is stopped
    homing = false;
    initHomingMode();
  } else if (currentMode == MODE_FORWARD_KINEMATICS) {
    // Initialize Forward Kinematics mode
    initForwardKinematicsMode(&u8g2);
  } else if (currentMode == MODE_INVERSE_KINEMATICS) {
    // Initialize Inverse Kinematics mode
    initInverseKinematicsMode(&u8g2);
  }
  
  // Reset speed-related variables
  currentSpeed = 0;
}

void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // First line: Mode
  u8g2.setCursor(0, 12);
  switch (currentMode) {
    case MODE_INDIVIDUAL_MOTOR:
      u8g2.print("Mode: Motor Control");
      break;
    case MODE_JOINT_CONTROL:
      u8g2.print("Mode: Joint Control");
      break;
    case MODE_HOMING:
      u8g2.print("Mode: Homing");
      break;
    case MODE_SERVO_TEST:
      u8g2.print("Mode: Servo Test");
      break;
    case MODE_FORWARD_KINEMATICS:
      u8g2.print("Mode: FK");
      break;
    case MODE_INVERSE_KINEMATICS:
      u8g2.print("Mode: IK");
      break;
    case MODE_COUNT:
      break;
  }

  // Second line: Motor info or status
  u8g2.setCursor(0, 24);
  if (currentMode == MODE_INDIVIDUAL_MOTOR) {
    u8g2.print("Motor: ");
    u8g2.print(activeMotor + 1);
  } else if (currentMode == MODE_HOMING) {
    u8g2.print("Status: ");
    u8g2.print(homing ? "HOMING" : "READY");
  } else if (currentMode == MODE_SERVO_TEST) {
    u8g2.print("Servo Pos: ");
    u8g2.print(gripper.read());
  }

  // Third line: Speed info
  u8g2.setCursor(0, 36);
  u8g2.print("Speed: ");
  u8g2.print(int(currentSpeed));

  // Fourth line: Position (only in motor control mode)
  if (currentMode == MODE_INDIVIDUAL_MOTOR) {
    u8g2.setCursor(0, 48);
    u8g2.print("Pos: ");
    u8g2.print(motors[activeMotor].currentPosition());
  } else if (currentMode == MODE_JOINT_CONTROL) {
    // Show abbreviated positions for multiple motors
    u8g2.setCursor(0, 48);
    u8g2.print("X:");
    u8g2.print(motors[0].currentPosition());
    u8g2.print(" Y:");
    u8g2.print(motors[1].currentPosition());
  }

  // Fifth line: Motor status
  u8g2.setCursor(0, 60);
  u8g2.print("Motors: ");
  u8g2.print(motorsEnabled ? "ON" : "OFF");

  u8g2.sendBuffer();
}

// Super stable homing display with no screen changes
void updateHomingDisplay() {
  // Static variables for timing control - use 1 full second between updates
  static unsigned long lastUpdate = 0;
  
  // Only update every 1000ms (1 second) to prevent any jumpiness
  if (millis() - lastUpdate < 1000) {
    return;
  }
  lastUpdate = millis();
  
  // Start drawing
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  
  // Fixed header with status
  u8g2.setCursor(0, 10);
  u8g2.print("HOMING");
  u8g2.setCursor(50, 10);
  u8g2.print(homing ? "ACTIVE" : "READY");
  
  // Show progress
  int progress = getHomingProgress();
  u8g2.setCursor(100, 10);
  u8g2.print(progress);
  u8g2.print("%");
  
  // Simple progress bar
  u8g2.drawFrame(0, 13, 128, 5);
  u8g2.drawBox(1, 14, (progress * 126) / 100, 3);
  
  // Show status of each axis on ONE screen
  // First row - axes 1-3
  u8g2.setCursor(0, 25);
  u8g2.print("1:");
  u8g2.print(axisHomingState[0] == HomingState::COMPLETED ? "OK" : 
             (axisHomingState[0] == HomingState::ERROR ? "ERR" : 
             (axisHomingState[0] == HomingState::IDLE ? "---" : "RUN")));
  
  u8g2.setCursor(40, 25);
  u8g2.print("2:");
  u8g2.print(axisHomingState[1] == HomingState::COMPLETED ? "OK" : 
             (axisHomingState[1] == HomingState::ERROR ? "ERR" : 
             (axisHomingState[1] == HomingState::IDLE ? "---" : "RUN")));
  
  u8g2.setCursor(80, 25);
  u8g2.print("3:");
  u8g2.print(axisHomingState[2] == HomingState::COMPLETED ? "OK" : 
             (axisHomingState[2] == HomingState::ERROR ? "ERR" : 
             (axisHomingState[2] == HomingState::IDLE ? "---" : "RUN")));
  
  // Second row - axes 4-6
  u8g2.setCursor(0, 35);
  u8g2.print("4:");
  u8g2.print(axisHomingState[3] == HomingState::COMPLETED ? "OK" : 
             (axisHomingState[3] == HomingState::ERROR ? "ERR" : 
             (axisHomingState[3] == HomingState::IDLE ? "---" : "RUN")));
  
  u8g2.setCursor(40, 35);
  u8g2.print("5:");
  u8g2.print(axisHomingState[4] == HomingState::COMPLETED ? "OK" : 
             (axisHomingState[4] == HomingState::ERROR ? "ERR" : 
             (axisHomingState[4] == HomingState::IDLE ? "---" : "RUN")));
  
  u8g2.setCursor(80, 35);
  u8g2.print("6:");
  u8g2.print(axisHomingState[5] == HomingState::COMPLETED ? "OK" : 
             (axisHomingState[5] == HomingState::ERROR ? "ERR" : 
             (axisHomingState[5] == HomingState::IDLE ? "---" : "RUN")));
  
  // Endstop status
  u8g2.setCursor(0, 45);
  u8g2.print("Endstops: ");
  for (int i = 0; i < 6; i++) {
    if (digitalRead(ENDSTOP_PINS[i]) == HIGH) {
      u8g2.print(i+1);
      u8g2.print(" ");
    }
  }
  
  // Instructions
  u8g2.setCursor(0, 55);
  u8g2.print(homing ? "Press to STOP" : "Press to START");
  
  // Finalize
  u8g2.sendBuffer();
}

// Helper function to convert steps to degrees for each joint
float stepsToDegree(int jointIndex, long steps) {
  // Use gear ratios from Robo_Config_V1.h
  const float STEPS_PER_DEGREE[6] = {
    static_cast<float>((BASE_STEPS * gearRatios[0]) / 360.0),  // Joint 1 (base)
    static_cast<float>((BASE_STEPS * gearRatios[1]) / 360.0),  // Joint 2 (shoulder) 
    static_cast<float>((BASE_STEPS * gearRatios[2]) / 360.0),  // Joint 3 (elbow)
    static_cast<float>((BASE_STEPS * gearRatios[3]) / 360.0),  // Joint 4 (wrist pitch)
    static_cast<float>((BASE_STEPS * gearRatios[4]) / 360.0),  // Joint 5 (wrist roll)
    static_cast<float>((BASE_STEPS * gearRatios[5]) / 360.0)   // Joint 6 (tool roll)
  };
  
    
    // Parse commands in format M[motor] [value]
    // Example: "M1 1000" to move motor 1 to position 1000
    if (command.startsWith("M") && command.length() >= 3) {
      int motorIndex = command.charAt(1) - '0' - 1; // Convert to 0-based index
      
      if (motorIndex >= 0 && motorIndex < 6) {
        String params = command.substring(3);
        processMotorCommand(motorIndex, params);
      } else {
      }
    }
    // Parse IK commands in format IK X Y Z R P Y
    // Example: "IK 0.5 0.1 0.3 0 0 0" for position XYZ and rotation RPY
    else if (command.startsWith("IK ")) {
      double values[6] = {0.0};
      int valueIndex = 0;
      String valueStr = "";
      
      // Parse 6 space-separated values
      for (int i = 0; i <= static_cast<int>(command.length()) && valueIndex < 6; i++) {
        if (i == static_cast<int>(command.length()) || command.charAt(i) == ' ') {
          if (valueStr.length() > 0) {
            values[valueIndex++] = valueStr.toFloat();
            valueStr = "";
          }
        } else if (i >= 3) { // Skip "IK "
          valueStr += command.charAt(i);
        }
      }
      
      if (valueIndex == 6) {
        // Create target position and rotation arrays
        double targetPos[3] = {values[0], values[1], values[2]};
        double targetRot[3] = {
          values[3] * M_PI / 180.0, // Convert from degrees to radians
          values[4] * M_PI / 180.0,
          values[5] * M_PI / 180.0
        };
        
        // Call the moveToCartesianTarget function (defined in InverseKinematics.cpp)
          bool success = moveToCartesianTarget(targetPos, targetRot, 2000);

          if (success) {
            Serial.println("IK solution found, movement started");
          } else {
            Serial.println("No valid IK solution for this target");
          }
        } else {
          Serial.println("Error: IK command requires 6 values (X Y Z Roll Pitch Yaw)");
        }
    }
    // Parse FK command to request current position
    else if (command == "FK") {
      // Calculate current position
      double jointAnglesRad[6];
      for (int i = 0; i < 6; i++) {
        float degrees = stepsToDegree(i, motors[i].currentPosition());
        jointAnglesRad[i] = degrees * M_PI / 180.0;
      }
      
      double currentPos[3], currentRot[3];
        calculateForwardKinematics(jointAnglesRad, currentPos, currentRot);

        // Return current position and orientation
        Serial.print("Position: X=");
        Serial.print(currentPos[0], 4);
        Serial.print(" Y=");
        Serial.print(currentPos[1], 4);
        Serial.print(" Z=");
        Serial.println(currentPos[2], 4);

        Serial.print("Orientation: Roll=");
        Serial.print(currentRot[0] * 180.0 / M_PI, 1);
        Serial.print(" Pitch=");
        Serial.print(currentRot[1] * 180.0 / M_PI, 1);
        Serial.print(" Yaw=");
        Serial.println(currentRot[2] * 180.0 / M_PI, 1);

      }
    // Parse HOME command to start homing
    else if (command == "HOME") {
      if (currentMode != MODE_HOMING) {
        currentMode = MODE_HOMING;
        resetMotorSettings();
      }
      homing = true;
      startHoming();
    }
    // Parse STOP command to stop all motors
    else if (command == "STOP") {
      for (int i = 0; i < 6; i++) {
        motors[i].setSpeed(0);
        motors[i].stop();
      }
      coordMoveActive = false;
    }
    else {
    }
  }
}

void processMotorCommand(int motorIndex, String params) {
  // Parse the parameters
  params.trim();
  
  // Move to absolute position: M1 1000
  if (params.length() > 0 && (isdigit(params[0]) || params[0] == '-')) {
    long targetPosition = params.toInt();
    
    motors[motorIndex].moveTo(targetPosition);
    motors[motorIndex].setSpeed(DEFAULT_MAX_SPEED * (motors[motorIndex].distanceToGo() > 0 ? 1 : -1));
  }
  // Reset position: M1 R
  else if (params.startsWith("R")) {
    motors[motorIndex].setCurrentPosition(0);
  }
  // Set speed: M1 S500
    // Set speed: M1 S500
  else if (params.startsWith("S")) {
    float speed = params.substring(1).toFloat();
    motors[motorIndex].setSpeed(speed);
  }
  // Get position: M1 ?
  else if (params == "?") {
  }
  else {
  }
}
 
