#ifndef INITSYSTEM_H
#define INITSYSTEM_H

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_VL53L0X.h>
#include <SD.h>
#include "Robo_Config_V1.h"
#include <U8g2lib.h>
#include <Servo.h>
//--------------------------------------------------
// 🌐 Globale Sensorobjekte (I²C)
//--------------------------------------------------
extern Adafruit_ADXL345_Unified adxl;
extern Adafruit_VL53L0X lox;
extern Servo gripper;
//--------------------------------------------------
// 🔧 Initialisierungsfunktion
//--------------------------------------------------
void initializeSystem();

// Sensor reading functions
bool getFilteredAcceleration(float* accelX, float* accelY, float* accelZ);
float getFilteredDistance();
void updateGripperFromPotentiometer();
// UI initialization
void initializeDisplay(U8G2* display);

#endif // INITSYSTEM_H