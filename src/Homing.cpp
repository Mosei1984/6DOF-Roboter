  // Homing.cpp
  #include "Homing.h"
  


  // External references
  extern AccelStepper motors[6];

  // Track homing state for each axis
  HomingState axisHomingState[6] = {HomingState::IDLE};
  unsigned long homingStartTime[6] = {0};
  unsigned long homingTimeout = 1200000; // 120-second timeout per axis

  // Homing speeds (steps/sec)
  const float HOMING_FAST_SPEED = 200.0;
  const float HOMING_SLOW_SPEED = 50.0;
  const long HOMING_BACKOFF_STEPS = 200; // Steps to back off after hitting endstop

  // Indicator for whether all homing is complete
  bool allHomingComplete = false;

  void initHomingMode() {
    Serial.println("Initializing Homing Mode");
  
    // Reset all homing states
    for (int i = 0; i < 6; i++) {
      axisHomingState[i] = HomingState::IDLE;
      homingStartTime[i] = 0;
    
      // Stop motors
      motors[i].setSpeed(0);
      motors[i].setMaxSpeed(HOMING_FAST_SPEED);
      motors[i].setAcceleration(HOME_ACCEL);
    }
  
    allHomingComplete = false;
  }

  void startHoming() {
    Serial.println("Starting homing sequence for all enabled axes");
  
    // Initialize homing for all enabled axes
    for (int i = 0; i < 6; i++) {
      if (HOMING_ENABLED[i]) {
        axisHomingState[i] = HomingState::FAST_APPROACH;
        homingStartTime[i] = millis();
        Serial.print("Starting homing for axis ");
        Serial.println(i + 1);
      } else {
        axisHomingState[i] = HomingState::COMPLETED; // Skip disabled axes
        Serial.print("Skipping homing for disabled axis ");
        Serial.println(i + 1);
      }
    }
  
    allHomingComplete = false;
  }

  bool isHomingComplete() {
    // Check if all axes are either COMPLETED or ERROR state
    for (int i = 0; i < 6; i++) {
      if (HOMING_ENABLED[i] && 
          axisHomingState[i] != HomingState::COMPLETED && 
          axisHomingState[i] != HomingState::ERROR) {
        return false;
      }
    }
  
    return true;
  }

  void processHoming() {
    bool anyAxisActive = false;
  
    // Process each axis
    for (int i = 0; i < 6; i++) {
      // Skip if this axis isn't enabled for homing
      if (!HOMING_ENABLED[i]) continue;
    
      // Check for timeout
      if (axisHomingState[i] != HomingState::IDLE && 
          axisHomingState[i] != HomingState::COMPLETED && 
          axisHomingState[i] != HomingState::ERROR) {
      
        if (millis() - homingStartTime[i] > homingTimeout) {
          Serial.print("Homing timeout for axis ");
          Serial.println(i + 1);
          axisHomingState[i] = HomingState::ERROR;
          motors[i].setSpeed(0);
          continue;
        }
      }
    
      // Process according to current state
      switch (axisHomingState[i]) {
        case HomingState::IDLE:
          // Do nothing
          break;
        
        case HomingState::FAST_APPROACH:
          anyAxisActive = true;
        
          // Set fast homing speed (direction based on configuration)
          motors[i].setSpeed(HOMING_DIRECTION[i] ? HOMING_FAST_SPEED : -HOMING_FAST_SPEED);
          motors[i].runSpeed();
        
          // Check if endstop is hit - FIXED for NC wiring
          if (digitalRead(ENDSTOP_PINS[i]) == HIGH) { // Endstop hit
            Serial.print("Fast approach completed for axis ");
            Serial.println(i + 1);
          
            // Stop motor
            motors[i].setSpeed(0);
          
            // Move to backing off state
            axisHomingState[i] = HomingState::BACKING_OFF;
          
            // Set position for backing off (opposite direction)
            motors[i].move(HOMING_DIRECTION[i] ? -HOMING_BACKOFF_STEPS : HOMING_BACKOFF_STEPS);
            motors[i].setSpeed(HOMING_DIRECTION[i] ? -HOMING_FAST_SPEED : HOMING_FAST_SPEED);
          }
          break;
        
        case HomingState::BACKING_OFF:
          anyAxisActive = true;
        
          // Run backing off movement
          if ((HOMING_DIRECTION[i] && motors[i].distanceToGo() < 0) || 
              (!HOMING_DIRECTION[i] && motors[i].distanceToGo() > 0)) {
            motors[i].runSpeed();
          } else {
            // Backing off complete, start slow approach
            Serial.print("Backing off completed for axis ");
            Serial.println(i + 1);
          
            axisHomingState[i] = HomingState::SLOW_APPROACH;
          }
          break;
        
        case HomingState::SLOW_APPROACH:
          anyAxisActive = true;
        
          // Set slow homing speed (direction based on configuration)
          motors[i].setSpeed(HOMING_DIRECTION[i] ? HOMING_SLOW_SPEED : -HOMING_SLOW_SPEED);
          motors[i].runSpeed();
        
          // Check if endstop is hit - FIXED for NC wiring
                  if (digitalRead(ENDSTOP_PINS[i]) == HIGH) { // Endstop hit
            Serial.print("Slow approach completed for axis ");
            Serial.println(i + 1);
          
            // Stop motor
            motors[i].setSpeed(0);
          
            // Set position to 0 (homed position)
            motors[i].setCurrentPosition(0);
          
            // Homing complete for this axis
            axisHomingState[i] = HomingState::COMPLETED;
          }
          break;
        
        case HomingState::COMPLETED:
          // Already complete
          break;
        
        case HomingState::ERROR:
          // Error state, do nothing
          break;
      }
    }
  
    // If all axes are complete or in error, set allHomingComplete flag
    if (!anyAxisActive && !allHomingComplete) {
      allHomingComplete = true;
    
      // Report homing results
      Serial.println("Homing sequence completed");
      for (int i = 0; i < 6; i++) {
        if (HOMING_ENABLED[i]) {
          Serial.print("Axis ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.println(axisHomingState[i] == HomingState::COMPLETED ? "SUCCESS" : "FAILED");
        }
      }
    }
  }

  void stopHoming() {
    // Stop all motors
    for (int i = 0; i < 6; i++) {
      motors[i].setSpeed(0);
      axisHomingState[i] = HomingState::IDLE;
    }
  
    Serial.println("Homing stopped");
  }

  bool homeSingleAxis(int index) {
    if (!HOMING_ENABLED[index]) {
      Serial.print("Homing not enabled for axis ");
      Serial.println(index + 1);
      return false;
    }
  
    // Reset state for this axis
    axisHomingState[index] = HomingState::FAST_APPROACH;
    homingStartTime[index] = millis();
  
    Serial.print("Starting homing for axis ");
    Serial.println(index + 1);
  
    // Continue until complete or error
    while (axisHomingState[index] != HomingState::COMPLETED && 
         axisHomingState[index] != HomingState::ERROR) {
    
      // Check for timeout
      if (millis() - homingStartTime[index] > homingTimeout) {
        Serial.print("Homing timeout for axis ");
        Serial.println(index + 1);
        axisHomingState[index] = HomingState::ERROR;
        motors[index].setSpeed(0);
        return false;
      }
    
      // Process according to current state
      switch (axisHomingState[index]) {
        case HomingState::FAST_APPROACH:
          // Set fast homing speed (negative direction toward endstop)
           motors[index].setSpeed(HOMING_DIRECTION[index] ? HOMING_FAST_SPEED : -HOMING_FAST_SPEED);
           motors[index].runSpeed();
        
          // Check if endstop is hit
          if (digitalRead(ENDSTOP_PINS[index]) == HIGH) { // Endstop hit
            Serial.print("Fast approach completed for axis ");
            Serial.println(index + 1);
          
            // Stop motor
            motors[index].setSpeed(0);
          
            // Move to backing off state
            axisHomingState[index] = HomingState::BACKING_OFF;
          
            // Set position for backing off
            motors[index].move(HOMING_DIRECTION[index] ? -HOMING_BACKOFF_STEPS : HOMING_BACKOFF_STEPS);
            motors[index].setSpeed(HOMING_DIRECTION[index] ? -HOMING_FAST_SPEED : HOMING_FAST_SPEED);
          }
          break;
        
        case HomingState::BACKING_OFF:
          // Run backing off movement
          if (motors[index].distanceToGo() > 0) {
            motors[index].runSpeed();
          } else {
            // Backing off complete, start slow approach
            Serial.print("Backing off completed for axis ");
            Serial.println(index + 1);
          
            axisHomingState[index] = HomingState::SLOW_APPROACH;
          }
          break;
        
        case HomingState::SLOW_APPROACH:
          // Set slow homing speed (negative direction toward endstop)
          motors[index].setSpeed(HOMING_DIRECTION[index] ? HOMING_SLOW_SPEED : -HOMING_SLOW_SPEED);
          motors[index].runSpeed();
        
          // Check if endstop is hit
          if (digitalRead(ENDSTOP_PINS[index]) == HIGH) { // Endstop hit
            Serial.print("Slow approach completed for axis ");
            Serial.println(index + 1);
          
            // Stop motor
            motors[index].setSpeed(0);
          
            // Set position to 0 (homed position)
            motors[index].setCurrentPosition(0);
          
            // Homing complete for this axis
            axisHomingState[index] = HomingState::COMPLETED;
          }
          break;
        
        default:
          // Other states shouldn't occur here
          Serial.print("Unexpected state for axis ");
          Serial.println(index + 1);
          return false;
      }
    
      // Small delay to prevent CPU hogging
      delay(1);
    }
  
    return axisHomingState[index] == HomingState::COMPLETED;
  }

  // Return the current homing state as a string
  const char* getHomingStateString(int axisIndex) {
    switch (axisHomingState[axisIndex]) {
      case HomingState::IDLE: return "IDLE";
      case HomingState::FAST_APPROACH: return "FAST APPROACH";
      case HomingState::BACKING_OFF: return "BACKING OFF";
      case HomingState::SLOW_APPROACH: return "SLOW APPROACH";
      case HomingState::COMPLETED: return "COMPLETED";
      case HomingState::ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }
  void moveToHomePose() {
  Serial.println("🔁 Bewegung zur definierten Home-Pose...");

  for (int i = 0; i < 6; i++) {
    float deg = HOME_ANGLES[i];
    float stepsPerDeg = (BASE_STEPS * gearRatios[i]) / 360.0;
    long target = deg * stepsPerDeg;
    motors[i].moveTo(target);
    motors[i].setSpeed((target > motors[i].currentPosition() ? 1 : -1) * MAX_SPEED);
  }

  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < 6; i++) {
      motors[i].run();
      if (motors[i].distanceToGo() != 0) {
        moving = true;
      }
    }
  }

  // Update aktuelle Winkel in Radiant
  for (int i = 0; i < 6; i++) {
    currentJointAngles[i] = HOME_ANGLES[i] * M_PI / 180.0;
  }
  calibrateAccelerometer();
  calibrateDistanceSensor();
  Serial.println("✅ Home-Pose erreicht und als Startwert gesetzt, Gyro und Lidar kalibriert.");
}

  // Return homing progress percentage (0-100)
  int getHomingProgress() {
    int totalAxes = 0;
    int completedAxes = 0;
  
    for (int i = 0; i < 6; i++) {
      if (HOMING_ENABLED[i]) {
        totalAxes++;
      
        if (axisHomingState[i] == HomingState::COMPLETED) {
          completedAxes++;
        } else if (axisHomingState[i] == HomingState::SLOW_APPROACH) {
          completedAxes += 0.75; // 75% complete when in slow approach
        } else if (axisHomingState[i] == HomingState::BACKING_OFF) {
          completedAxes += 0.5; // 50% complete when backing off
        } else if (axisHomingState[i] == HomingState::FAST_APPROACH) {
          completedAxes += 0.25; // 25% complete when in fast approach
        }
      }
    }
  
    if (totalAxes == 0) return 100; // No axes to home
  
    return (completedAxes * 100) / totalAxes;
  }

