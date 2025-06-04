// Robo_Config_V1.h
#ifndef ROBO_CONFIG_V1_H
#define ROBO_CONFIG_V1_H
#include <U8g2lib.h>
//----------------------------------------
// 📍 Globale Einstellungen
//----------------------------------------
#define MICROSTEPPING          32
#define STEPS_PER_REVOLUTION   200 
#define BASE_STEPS  (STEPS_PER_REVOLUTION * MICROSTEPPING)

#define DEFAULT_MAX_SPEED      1000.0
#define DEFAULT_ACCELERATION   2000.0

//----------------------------------------
// ⚙️ Übersetzungsverhältnisse der 6 Achsen
//----------------------------------------
static const float gearRatios[6] = {
  3.0, 5.0, 2.0, 2.0, 1.0, 1.0
};

//----------------------------------------
// 🧠 Motor-Pinbelegung (pro Achse)
//----------------------------------------
static const int STEP_PINS[6]    = {2,  5,  8,  11, 14, 17};
static const int DIR_PINS[6]     = {3,  6,  9,  12, 15, 20};
static const int ENABLE_PINS[6]  = {4,  7,  10, 13, 16, 21};
static const int ENDSTOP_PINS[6] = {22, 23, 24, 25, 29, 28};

//----------------------------------------
// 🔄 Motor Direction Configuration
//----------------------------------------
// Set to true for positive direction, false for negative
extern bool MOTOR_DIRECTION[6];  // Standard movement direction
extern bool HOMING_DIRECTION[6]; // Direction for homing movement

//----------------------------------------
// 🎮 Joystick-Pinbelegung (analog)
//----------------------------------------
#define JOYSTICK_L_X    41
#define JOYSTICK_L_Y    40
#define JOYSTICK_R_Z    39
#define JOYSTICK_R_YAW  38

extern int16_t joyLXCenter, joyLYCenter, joyRZCenter, joyRYawCenter;
#define JOYSTICK_SMOOTHING  8
#define DEADZONE            20

//----------------------------------------
// 🔘 Buttons (digital, mit Pullup)
//----------------------------------------
#define BUTTON_MODE      26
#define BUTTON_CONFIRM   34

//----------------------------------------
// 🎛️ Servo-Konfiguration
//----------------------------------------
#define SERVO_PIN           33
#define SERVO_DEFAULT_POS    165
#define SERVO_MIN_POS         40
#define SERVO_MAX_POS       180
#define SERVO_POTTY_PIN      27
//----------------------------------------
// ⚙️ Speed-Parameter
//----------------------------------------
#define MAX_SPEED           2000.0
#define MIN_PRACTICAL_SPEED   20.0
#define MAX_ACCEL           1000.0
#define HOME_SPEED           500.0

//----------------------------------------
// ⚙️ Homing aktivieren/deaktivieren pro Achse
//----------------------------------------
static const bool HOMING_ENABLED[6] = {
  true, true, true, true, false, false
};

//----------------------------------------
// 🌐 Shared global variables
//----------------------------------------
// Display pointer for UI operations
extern U8G2* displayPtr;

// Timing variables
extern unsigned long lastJoystickCheck;
static const float HOME_ANGLES[6] = {
  100.0,    // Base
  -30.0,   // Schulter (Max-oben)
  -110.0,  // Ellenbogen
  125.0,  // Wrist Roll
  0.0,  // Wrist Pitch
  0.0     // Tool Roll
};


#endif // ROBO_CONFIG_V1_H
