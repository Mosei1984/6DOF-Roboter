#include "ConfigHandler.h"

bool ConfigHandler::setDriveMode(DriveMode newMode) {
    // Check if the requested mode is valid
    switch(newMode) {
        case MODE_FORWARD:
            config.inverseMode = false;
            config.jointMode = false;
            break;
        case MODE_INVERSE:
            config.inverseMode = true;
            config.jointMode = false;
            break;
        case MODE_IK_ADXL:
            config.inverseMode = true;
            config.jointMode = false;
            // Additional ADXL-specific setup could go here
            break;
        case MODE_JOINT:
            config.inverseMode = false;
            config.jointMode = true;
            break;
        case MODE_REFERENCE:
            // Reference mode setup
            break;
        case MODE_TEACH:
            // Teaching mode setup
            break;
        case MODE_PLAY:
            // Playback mode setup
            break;
        default:
            return false; // Invalid mode
    }
    
    // Update both current drive mode values
    currentDriveMode = newMode;
    config.currentDriveMode = newMode;
    
    if (config.useVerboseMode) {
        Serial.print("Drive mode changed to: ");
        Serial.println(getDriveModeString());
    }
    
    return true;
}

const char* ConfigHandler::getDriveModeString() {
    switch(currentDriveMode) {
        case MODE_FORWARD:
            return "Forward Kinematics";
        case MODE_INVERSE:
            return "Inverse Kinematics";
        case MODE_IK_ADXL:
            return "IK with ADXL";
        case MODE_JOINT:
            return "Joint Mode";
        case MODE_REFERENCE:
            return "Reference Mode";
        case MODE_TEACH:
            return "Teach Mode";
        case MODE_PLAY:
            return "Play Mode";
        default:
            return "Unknown Mode";
    }
}

void ConfigHandler::resetToDefault() {
    // Standardwerte setzen
    strcpy(config.config_version, "v1.00");

    for (int i = 0; i < 6; i++) {
        config.gearRatios[i] = 1.0;
        config.maxAngle[i] = 180.0;
        config.maxSpeed[i] = 50.0;
        config.accel[i] = 100.0;
        config.microstepping[i] = 16;
        config.hasEndstop[i] = (i != 5); // Motor 6 hat standardmäßig keinen Endschalter
        config.useSensorlessHoming[i] = false;
        config.invertDirection[i] = false;
        config.enableDebugPin1 = false;
        config.enableDebugPin2= false;
        config.enableDebugPin3 = false;
        config.Gripper_Pin = 33;
        config.debugPin1 = 30;
        config.debugPin2 = 31;
        config.debugPin3 = 32;
    }
    config.useDebugmode = false;
    config.useGripperMode = false;
    config.useVerboseMode = false;
    config.inverseMode = false;
    config.jointMode = false;
    config.toolMotorEnabled = false;
    config.wristControlEnabled = false;
    config.currentDriveMode = MODE_FORWARD; // Default mode
    currentDriveMode = MODE_FORWARD; // Update class variable too
}

void ConfigHandler::useVerboseMode() {
    config.useVerboseMode = true;
}


void ConfigHandler::saveToEEPROM() {
    EEPROM.put(EEPROM_ADDR, config);
    
}
void ConfigHandler::useDebugmode(){
    config.enableDebugPin1 = true;
    config.enableDebugPin2 = true;
    config.enableDebugPin3 = true;
    config.useVerboseMode = true;
};
void ConfigHandler::loadFromEEPROM() {
    EEPROM.get(EEPROM_ADDR, config);

    // Falls keine gültige Version vorhanden, auf Default setzen
    if (strncmp(config.config_version, "v1.00", 5) != 0) {
        resetToDefault();
        saveToEEPROM();
    }
}

bool ConfigHandler::loadFromSD(const char *path) {
    File file = SD.open(path);
    if (!file) {
        Serial.println("⚠️ Konnte Konfigurationsdatei nicht öffnen.");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.print("⚠️ JSON-Fehler: ");
        Serial.println(error.c_str());
        return false;
    }

    for (int i = 0; i < 6; i++) {
        config.gearRatios[i]         = doc["gearRatios"][i] | 1.0;
        config.maxAngle[i]           = doc["maxAngle"][i] | 180.0;
        config.maxSpeed[i]           = doc["maxSpeed"][i] | 50.0;
        config.accel[i]              = doc["accel"][i] | 100.0;
        config.microstepping[i]      = doc["microstepping"][i] | 16;
        config.hasEndstop[i]         = doc["hasEndstop"][i] | (i != 5);
        config.useSensorlessHoming[i]= doc["useSensorlessHoming"][i] | false;
        config.invertDirection[i]    = doc["invertDirection"][i] | false;
        config.enableDebugPin1 = doc["enableDebugPin1"] | false;
        config.enableDebugPin2 = doc["enableDebugPin2"] | false;
        config.enableDebugPin3 = doc["enableDebugPin3"] | false;
        config.useDebugmode = doc["useDebugmode"] | false;
        config.useVerboseMode = doc["useVerboseMode"] | false;
        config.Gripper_Pin = doc["gripper_pin"] | 33;
        config.debugPin1 = doc["debugPin1"] | 30;
        config.debugPin2 = doc["debugPin2"] | 31;
        config.debugPin3 = doc["debugPin3"] | 32;
        config.inverseMode = doc["inverseMode"] | false;
        config.jointMode = doc["jointMode"] | false;
        config.toolMotorEnabled = doc["toolMotorEnabled"] | false;
        config.wristControlEnabled = doc["wristControlEnabled"] | false;
    }

    config.useGripperMode = doc["useGripperMode"] | false;
    strlcpy(config.config_version, doc["config_version"] | "v1.00", sizeof(config.config_version));

    return true;
}

bool ConfigHandler::saveToSD(const char *path) {
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("⚠️ Konnte Datei zum Speichern nicht öffnen.");
        return false;
    }

    JsonDocument doc;

    doc["config_version"] = config.config_version;
    for (int i = 0; i < 6; i++) {
        doc["gearRatios"][i]          = config.gearRatios[i];
        doc["maxAngle"][i]            = config.maxAngle[i];
        doc["maxSpeed"][i]            = config.maxSpeed[i];
        doc["accel"][i]               = config.accel[i];
        doc["microstepping"][i]       = config.microstepping[i];
        doc["hasEndstop"][i]          = config.hasEndstop[i];
        doc["useSensorlessHoming"][i] = config.useSensorlessHoming[i];
        doc["invertDirection"][i]     = config.invertDirection[i];
        doc["enableDebugPin1"] = config.enableDebugPin1;
        doc["enableDebugPin2"] = config.enableDebugPin2;
        doc["enableDebugPin3"] = config.enableDebugPin3;
        doc["debugPin1"] = config.debugPin1;
        doc["debugPin2"] = config.debugPin2;
        doc["debugPin3"] = config.debugPin3;
        doc["gripper_pin"] = config.Gripper_Pin;
        doc["inverseMode"] = config.inverseMode;
        doc["jointMode"] = config.jointMode;
        doc["toolMotorEnabled"] = config.toolMotorEnabled;
        doc["wristControlEnabled"] = config.wristControlEnabled;

    }
    doc["useGripperMode"] = config.useGripperMode;
    doc["useDebugmode"] = config.useDebugmode;
    doc["useVerboseMode"] = config.useVerboseMode;
    serializeJsonPretty(doc, file);
    file.close();
    return true;
}

void ConfigHandler::printConfig() {
    Serial.println("=== AKTUELLE SYSTEMKONFIGURATION ===");
    for (int i = 0; i < 6; i++) {
        Serial.printf("Motor %d: Ratio=%.2f, MaxAngle=%.1f°, µStep=%d, Speed=%.1f mm/s, Accel=%.1f mm/s², Endstop=%s, Sensorless=%s, Invert=%s\n", 
                      i + 1,
                      config.gearRatios[i],
                      config.maxAngle[i],
                      config.microstepping[i],
                      config.maxSpeed[i],
                      config.accel[i],
                      config.hasEndstop[i] ? "JA" : "NEIN",
                      config.useSensorlessHoming[i] ? "JA" : "NEIN",
                      config.invertDirection[i] ? "JA" : "NEIN");
                      
    }
    Serial.printf("Gripper-Modus): %s\n", config.useGripperMode ? "AKTIV" : "INAKTIV");
    Serial.printf("DebugPin1 aktiv: %s (Pin %d)\n", config.enableDebugPin1 ? "JA" : "NEIN", config.debugPin1);
    Serial.printf("DebugPin2 aktiv: %s (Pin %d)\n", config.enableDebugPin2 ? "JA" : "NEIN", config.debugPin2);
    Serial.printf("DebugPin3 aktiv: %s (Pin %d)\n", config.enableDebugPin3 ? "JA" : "NEIN", config.debugPin3);
    Serial.printf("Inverse-Kinematik: %s\n", config.inverseMode ? "JA" : "NEIN");
    Serial.printf("Joint-Modus:       %s\n", config.jointMode ? "JA" : "NEIN");
    Serial.printf("Toolmotor:         %s\n", config.toolMotorEnabled ? "JA" : "NEIN");
    Serial.printf("Wrist-Steuerung:   %s\n", config.wristControlEnabled ? "JA" : "NEIN");
    Serial.printf("Debugmode aktiv: %s\n", config.useDebugmode ? "JA" : "NEIN");
    Serial.printf("Verbose Mode aktiv: %s\n", config.useVerboseMode ? "JA" : "NEIN");
    Serial.printf("Gripper-Pin: %d\n", config.Gripper_Pin);
    Serial.println("=====================================");
}
