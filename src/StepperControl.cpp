// StepperControl.cpp
#include "StepperControl.h"
#include "Robo_Config_V1.h"
#include <AccelStepper.h>
#include <Arduino.h>
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
    float dir = (positions[i] > motors[i].currentPosition()) ? 1.0f : -1.0f;
    motors[i].setSpeed(dir * speed);
  }

  bool moving = true;
  while (moving) {
    moving = false;
    updateSteppers();
    for (int i = 0; i < 6; i++) {
      if (motors[i].distanceToGo() != 0) {
        moving = true;
      }
    }
  }
}

// Centralized stepper update used by both the ISR and blocking loops
void updateSteppers() {
  extern Mode currentMode;
  extern volatile uint8_t activeMotor;
  extern bool homing;
  extern bool coordMoveActive;
  extern unsigned long coordMoveStartTime;
  extern unsigned long coordMoveDuration;
  extern long coordMoveStartSteps[6];
  extern long coordMoveTargetSteps[6];

  if (currentMode == MODE_INDIVIDUAL_MOTOR) {
    motors[activeMotor].runSpeed();
  } else if (currentMode == MODE_JOINT_CONTROL) {
    for (int i = 0; i < 6; i++) {
      if (abs(motors[i].speed()) > 0.01f) motors[i].runSpeed();
    }
  } else if (currentMode == MODE_HOMING && homing) {
    for (int i = 0; i < 6; i++) {
      if (digitalRead(ENDSTOP_PINS[i]) == HIGH && abs(motors[i].speed()) > 0.01f)
        motors[i].runSpeed();
    }
  } else if (currentMode == MODE_FORWARD_KINEMATICS || currentMode == MODE_INVERSE_KINEMATICS) {
    if (coordMoveActive) {

      unsigned long elapsedTime = millis() - coordMoveStartTime;

      if (elapsedTime >= coordMoveDuration) {
        coordMoveActive = false;
        for (int i = 0; i < 6; i++) {
          motors[i].setSpeed(0);
        }
      } else {
        float progress = (float)elapsedTime / coordMoveDuration;
        float smoothProgress = sin(progress * PI - PI/2) * 0.5 + 0.5;

        bool anyMotorMoving = false;

        for (int i = 0; i < 6; i++) {
          long currentTarget = coordMoveStartSteps[i] + (coordMoveTargetSteps[i] - coordMoveStartSteps[i]) * smoothProgress;
          long stepsToGo = currentTarget - motors[i].currentPosition();

          if (abs(stepsToGo) > 0) {
            anyMotorMoving = true;

            float requiredSpeed = constrain(stepsToGo * 50.0f, -MAX_SPEED, MAX_SPEED);

            if (abs(requiredSpeed) < MIN_PRACTICAL_SPEED && abs(requiredSpeed) > 0.1f) {
              requiredSpeed = (requiredSpeed > 0) ? MIN_PRACTICAL_SPEED : -MIN_PRACTICAL_SPEED;
            }

            motors[i].setSpeed(requiredSpeed);
            motors[i].runSpeed();
          }
        }

        if (!anyMotorMoving) {
          coordMoveActive = false;
        }
      }
    } else {
      for (int i = 0; i < 6; i++) {
        if (abs(motors[i].speed()) > 0.01f) motors[i].runSpeed();
      }
    }
  }
}
