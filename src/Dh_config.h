#pragma once
#include "DHUtilities.h"

// -----------------------------------------------------------------------------
// Configuration.h
// Zentrale Konfigurationsdatei: DH-Parameter, Gelenk-Limits, aktuelle Winkel
// -----------------------------------------------------------------------------

// This part is crucial - define variables only in one compilation unit
#ifdef DEFINE_DH_CONFIG_VARIABLES
// Variables are defined here
DHParams robotDHParams[6] = {
    // a,           alpha,           d,      theta (initial)
    { 0.0,         M_PI/2.0,        0.1,    0.0 },  // Gelenk 1 (Basis)
    { 0.5,         0.0,             0.0,    0.0 },  // Gelenk 2 (Schulter)
    { 0.4,         0.0,             0.0,    0.0 },  // Gelenk 3 (Ellenbogen)
    { 0.0,        -M_PI/2.0,        0.0,    0.0 },  // Gelenk 4 (Wrist Pitch)
    { 0.0,         M_PI/2.0,        0.0,    0.0 },  // Gelenk 5 (Wrist Roll)
    { 0.0,         0.0,             0.1,    0.0 }   // Gelenk 6 (Tool-Offset)
};

JointLimits jointLimits[6] = {
    { -2.967,   2.967 },  // θ1: -170° bis +170°
    { -1.396,   1.396 },  // θ2: -80° bis +80°
    { -2.094,   2.094 },  // θ3: -120° bis +120°
    { -M_PI,    M_PI   },  // θ4: -180° bis +180°
    { -M_PI,    M_PI   },  // θ5: -180° bis +180°
    { -M_PI,    M_PI   }   // θ6: -180° bis +180°
};

double currentJointAngles[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const double toolLength = 0.01; // in Metern

#else
// Only declare variables for other files
extern DHParams robotDHParams[6];
extern JointLimits jointLimits[6];
extern double currentJointAngles[6];
extern const double toolLength;
#endif
