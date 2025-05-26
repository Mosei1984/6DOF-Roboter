#include "DebugTools.h"

// Debugpins setzen
void DebugTools::setupDebugPins(const SystemConfig& config) {
    if (config.enableDebugPin1) pinMode(config.debugPin1, OUTPUT);
    if (config.enableDebugPin2) pinMode(config.debugPin2, OUTPUT);
    if (config.enableDebugPin3) pinMode(config.debugPin3, OUTPUT);
}

// Blinker-Logik
void DebugTools::handleDebugPins(SystemConfig& config, TaskTimer& t1, TaskTimer& t2, TaskTimer& t3, unsigned long now) {
    if (config.enableDebugPin1 && t1.isDue(now)) {
        digitalWrite(config.debugPin1, !digitalRead(config.debugPin1));
        t1.reset(now);
    }
    if (config.enableDebugPin2 && t2.isDue(now)) {
        digitalWrite(config.debugPin2, !digitalRead(config.debugPin2));
        t2.reset(now);
    }
    if (config.enableDebugPin3 && t3.isDue(now)) {
        digitalWrite(config.debugPin3, !digitalRead(config.debugPin3));
        t3.reset(now);
    }
}

// Impuls für Oszi / Analyzer: HIGH → kurze Pause → LOW
void DebugTools::pulsePin(uint8_t pin, uint16_t durationMicros) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(durationMicros);
    digitalWrite(pin, LOW);
}

// Setzt Pin je nach gewünschtem Zustand (eventuelle Statuswechsel sichtbar machen)
void DebugTools::scopeEvent(uint8_t pin, bool value) {
    digitalWrite(pin, value ? HIGH : LOW);
}

// Verbose-Ausgaben bei aktivem Modus
void DebugTools::verbosePrintln(const SystemConfig& config, const String& message) {
    if (config.useVerboseMode) {
        Serial.println(message);
    }
}

void DebugTools::verbosePrint(const SystemConfig& config, const String& message) {
    if (config.useVerboseMode) {
        Serial.print(message);
    }
}

void DebugTools::verbosePrintf(const SystemConfig& config, const char* format, ...) {
    if (config.useVerboseMode) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        Serial.print(buffer);
    }
}
