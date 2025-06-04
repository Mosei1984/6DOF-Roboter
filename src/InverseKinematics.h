#pragma once
#include "Dh_config.h"
#include <U8g2lib.h>
#include "ForwardKinematics.h"
#include "Robo_Config_V1.h"
#include <array>
#include <cmath>
#include <AccelStepper.h>
#include <Servo.h>

// Add this line to declare the external pointer
extern U8G2* displayPtr;

void initializeDisplay(U8G2* display);
bool calculateInverseKinematics(
    const double targetPosition[3],
    const double targetRotationZYX[3],
    const double currentJointAngles[6]
);
void executeInverseKinematicsMotion(double currentJointAngles[6]);
void onSetTargetButton(
    double desiredPos[3],
    double desiredRotZYX[3],
    double currentJointAngles[6]
);
void onExecuteButton(double currentJointAngles[6]);
void initInverseKinematicsMode(U8G2* display);
void handleInverseKinematicsMode();
void updateIKDisplay(U8G2* display);
extern double currentTargetPos[3];
extern double currentTargetRot[3];
extern bool ikSolutionReady;

// Add these new function declarations
void startCoordinatedMove(double targetJointAngles[6], unsigned long duration);
bool moveToCartesianTarget(double targetPos[3], double targetRot[3], unsigned long duration);

// External variable declarations - these are defined in InverseKinematics.cpp
extern bool coordMoveActive;
extern unsigned long coordMoveStartTime;
extern unsigned long coordMoveDuration;
extern long coordMoveStartSteps[6];
extern long coordMoveTargetSteps[6];
extern double cartesianStartPos[3];
extern double cartesianTargetPos[3];
extern double cartesianStartRot[3];
extern double cartesianTargetRot[3];
void startCoordinatedMove(double targetJointAngles[6], unsigned long duration);