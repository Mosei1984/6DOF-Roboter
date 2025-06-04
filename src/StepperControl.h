#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <AccelStepper.h>
#include "Robo_Config_V1.h"

// ONLY declarations, no implementations
void stopAllSteppers();
void configureSteppers();
void moveMotorsToPositions(long positions[6], float speed);

#endif // STEPPER_CONTROL_H