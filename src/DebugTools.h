#ifndef DEBUG_TOOLS_H
#define DEBUG_TOOLS_H

#include <Arduino.h>
#include "ConfigHandler.h"
#include "TaskTiming.h"

class DebugTools {
public:
    // Setup: Debugpins konfigurieren
    static void setupDebugPins(const SystemConfig& config);

    // Regelmäßiges Togglen der Pins (Blinker)
    static void handleDebugPins(SystemConfig& config, TaskTimer& t1, TaskTimer& t2, TaskTimer& t3, unsigned long now);

    // Oszilloskop- oder Logic-Analyzer-Trigger
    static void pulsePin(uint8_t pin, uint16_t durationMicros = 10);

    // Zustandswechsel sichtbar machen (z. B. rising edge)
    static void scopeEvent(uint8_t pin, bool value);

    // Serielle Debug-Ausgaben nur bei aktivem verbose
    static void verbosePrintln(const SystemConfig& config, const String& message);
    static void verbosePrint(const SystemConfig& config, const String& message);
    static void verbosePrintf(const SystemConfig& config, const char* format, ...);
};

#endif
