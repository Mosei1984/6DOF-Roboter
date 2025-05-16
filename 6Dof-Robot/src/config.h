#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Struktur für die Pin-Konfiguration
struct PinConfig {
  int leftXPin;
  int leftYPin;
  int leftBtnPin;
  int rightXPin;
  int rightYPin;
  int rightBtnPin;
  int errorLedPin;
  
  // Stepper motor pins als 2D Array (6 Motoren, 4 Pins pro Motor)
  int stepperPins[6][4];  // [motor][pin_type] wobei pin_type: 0=STEP, 1=DIR, 2=ENABLE, 3=ENDSTOP
  
  // OLED Display Pins
  int oledSdaPin;
  int oledSclPin;
};

// Globale Variable für die Pin-Konfiguration
extern PinConfig _pinConfig;

// Bitmap für den Roboterarm - wird in config.cpp definiert
extern const unsigned char PROGMEM robotArmBitmap[];

// Funktion zum Laden der Standard-Konfiguration
void loadDefaultPinConfig();

#endif