// StepperControl.cpp
#include "StepperControl.h"
#include "Robo_Config_V1.h"
#include <AccelStepper.h>
// Zugriff auf die globalen Objekte aus main.cpp
extern AccelStepper    motors[6];


void stopAllSteppers() {
  for (int i = 0; i < 6; i++) {
    motors[i].setSpeed(0);
  }
}

void configureSteppers() {
  for (int i = 0; i < 6; i++) {
    motors[i].setMaxSpeed(DEFAULT_MAX_SPEED);
    motors[i].setAcceleration(DEFAULT_ACCELERATION);
  }
}

void moveMotorsToPositions(long positions[6], float speed) {
  for (int i = 0; i < 6; i++) {
    motors[i].moveTo(positions[i]);
  }
  bool moving = true;
  while (moving) {
    moving = false;
    for (int i = 0; i < 6; i++) {
      if (motors[i].distanceToGo() != 0) {
        motors[i].run();
        moving = true;
      }
    }
  }
}
