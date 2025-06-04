#ifndef JOINT_MODE_H
#define JOINT_MODE_H

#include <U8g2lib.h>
#include <AccelStepper.h>
#include "Robo_Config_V1.h"
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include "InitSystem.h"
#include "Robo_Config_V1.h"
#include "StepperControl.h"

// Constants for NeoPixel
#define NUM_PIXELS 16
#define NEOPIXEL_PIN 6
#define PIXEL_BRIGHTNESS 50
extern unsigned long lastJoystickCheck;
// Constants for Joint Mode
#define JOINT_MODE_MAX_SPEED 2000
#define JOINT_MODE_MAX_ACCELERATION 1000

// Function declarations
void initJointMode(U8G2* display);
void handleJointMode();
void updateJointModeDisplay(U8G2_SSD1306_128X64_NONAME_F_HW_I2C* display);

// External references
extern AccelStepper motors[6];
extern bool buttonConfirmPressed;
extern float stepsToDegree(int jointIndex, long steps);
void initializeDisplay(U8G2* display);

// Declare this as extern so it's not defined in multiple places
extern unsigned long lastJoystickCheck;

#endif // JOINT_MODE_H
