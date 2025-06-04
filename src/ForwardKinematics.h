#pragma once
#include "DHUtilities.h"
#include <U8g2lib.h>
#include "Dh_config.h"    // Enthält robotDHParams[]
#include "InverseKinematics.h"
#include "InitSystem.h"
#include "Robo_Config_V1.h"  // Include the central config file
#include <array>
#include <AccelStepper.h>
#include <Servo.h>
// -----------------------------------------------------------------------------
// ForwardKinematics.h
// Deklaration der Forward-Kinematik-Funktionen
// -----------------------------------------------------------------------------

// Berechnet Position (X, Y, Z) und Orientierung (Roll, Pitch, Yaw) des Endeffektors
//   currentJointAngles: Array der aktuellen 6 Gelenkwinkel (in Radiant)
//   endPosition[3]: Rückgabe der X/Y/Z-Koordinaten (in Metern)
//   endRotation[3]: Rückgabe der Euler-Winkel (Roll, Pitch, Yaw in Radiant)
void calculateForwardKinematics(
    const double currentJointAngles[6],
    double endPosition[3],
    double endRotation[3]
);

// Anzeige-Update der FK-Ergebnisse (Position + Orientierung)
void updateForwardKinematicsDisplay(
    const double endPosition[3],
    const double endRotation[3]
);

// Forward Kinematics Mode functions (for main UI)
void initForwardKinematicsMode(U8G2* display);
void handleForwardKinematicsMode();
