#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "Robo_Config_V1.h"
#include "Dh_config.h"
#include "InitSystem.h"

// Enthält currentJointAngles[]
// Homing constants
#define HOMING_SPEED 200
#define HOME_ACCEL 200
#define HOMING_OFFSET 200
#define MAX_HOMING_TIME 1200000 // 120 seconds timeout

// Enum for homing states - use enum class to avoid naming conflicts
enum class HomingState {
  IDLE,
  FAST_APPROACH,
  BACKING_OFF,
  SLOW_APPROACH,
  COMPLETED,
  ERROR
};

// Function declarations
void initHomingMode();
void startHoming();
void processHoming();
void stopHoming();
bool isHomingComplete();
void moveToHomePose();
void calibrateDistanceSensor();
void calibrateAccelerometer();
bool homeSingleAxis(int index);
const char* getHomingStateString(int axisIndex);
int getHomingProgress();

// External state variables
extern HomingState axisHomingState[6];
extern bool allHomingComplete;

#endif // HOMING_H