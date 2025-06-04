#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <AccelStepper.h>
#include "Robo_Config_V1.h"

// Enum definition for robot modes
enum Mode {
  MODE_INDIVIDUAL_MOTOR,
  MODE_JOINT_CONTROL,
  MODE_HOMING,
  MODE_SERVO_TEST,
  MODE_FORWARD_KINEMATICS,
  MODE_INVERSE_KINEMATICS,
  MODE_COUNT
};

// ONLY declarations, no implementations
void stopAllSteppers();
void configureSteppers();
void moveMotorsToPositions(long positions[6], float speed);
void updateSteppers();

#endif // STEPPER_CONTROL_H