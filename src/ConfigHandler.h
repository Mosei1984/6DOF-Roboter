#ifndef CONFIG_HANDLER_H
#define CONFIG_HANDLER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SD.h>
#include <Adafruit_SSD1306.h>

// Define enum DriveMode only once
enum DriveMode {
    MODE_FORWARD,
    MODE_INVERSE,
    MODE_IK_ADXL,
    MODE_JOINT,
    MODE_REFERENCE,
    MODE_TEACH,
    MODE_PLAY
};

// Struktur für Hauptkonfiguration
struct SystemConfig {
    char config_version[6];     // "v1.00"
    float gearRatios[6];        // Übersetzungsverhältnis pro Motor
    float maxAngle[6];          // Maximale Winkel von Home
    float maxSpeed[6];          // Maximalgeschwindigkeit (mm/s)
    float accel[6];             // Beschleunigung (mm/s²)
    uint8_t microstepping[6];   // Mikroschritte pro Motor
    bool hasEndstop[6];         // Endschalter vorhanden?
    bool useSensorlessHoming[6];// Sensorloses Homing erlaubt?
    bool invertDirection[6];    // Richtungsumkehr
    bool useGripperMode;
    bool enableDebugPin1;
    bool enableDebugPin2;
    bool enableDebugPin3;
    bool useDebugmode;
    bool useVerboseMode;
    uint8_t debugPin1;
    uint8_t debugPin2;  
    uint8_t debugPin3;
    uint8_t Gripper_Pin;
    bool inverseMode;           // Inverse Kinematik aktiviert?
    bool jointMode;             // Direkter Gelenkmodus?
    bool toolMotorEnabled;      // Werkzeugmotor erlaubt?
    bool wristControlEnabled;   // Handgelenksteuerung erlaubt?
    DriveMode currentDriveMode; // Current drive mode
};

class ConfigHandler {
public:
    SystemConfig config;
    DriveMode currentDriveMode;
    uint8_t jointSelected;
    
    void loadFromEEPROM();            // EEPROM-Daten lesen
    void saveToEEPROM();              // EEPROM-Daten schreiben
    void resetToDefault();            // Rücksetzen auf Standard
    bool loadFromSD(const char *path = "/config.json");
    bool saveToSD(const char *path = "/config.json");
    void printConfig();               // Serieller Dump
    void useDebugmode();
    void useVerboseMode();
    
    // Mode switching method
    bool setDriveMode(DriveMode newMode);
    const char* getDriveModeString(); // Returns current mode as string

private:
    const int EEPROM_ADDR = 0;
};

#endif
