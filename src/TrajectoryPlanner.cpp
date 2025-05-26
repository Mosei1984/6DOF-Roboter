#include "TrajectoryPlanner.h"
#include "DebugTools.h"
#include <algorithm>
#include <cmath>

// Konstruktor
TrajectoryPlanner::TrajectoryPlanner(KinematicsHandler* kinematics, ConfigHandler* config)
     : kinematicsHandler(kinematics),
      configHandler(config),
      adxlTracker(nullptr),
      lidarSensor(nullptr),
      isMoving(false),
      moveStartTime(0),
      moveDuration(1000),
      lastUpdateTime(millis()),
      currentStep(0),
      totalSteps(0),
      trajectoryType(TRAJECTORY_LINEAR),
      targetDistance(300.0f)
{
    // Initialisierung der Positionen und Orientierungen
    for (int i = 0; i < 3; i++) {
        currentPosition[i] = 0.0f;
        targetPosition[i] = 0.0f;
        startPosition[i] = 0.0f;
        currentOrientation[i] = 0.0f;
        targetOrientation[i] = 0.0f;
        startOrientation[i] = 0.0f;
    }
    
    // Standardwerte für die Bewegung
    moveDuration = 1000; // 1 Sekunde
    trajectoryType = TRAJECTORY_LINEAR;
    
    lastUpdateTime = millis();
}

// Sensoren setzen
void TrajectoryPlanner::setAdxlTracker(AdxlTracker* tracker) {
    adxlTracker = tracker;
}

void TrajectoryPlanner::setLidarSensor(VL53L0XSensor* lidar) {
    lidarSensor = lidar;
}

// Eine neue Bewegung starten
void TrajectoryPlanner::startMove(float x, float y, float z, float roll, float pitch, float yaw, unsigned long duration) {
    // Aktuelle Position als Startposition speichern
    for (int i = 0; i < 3; i++) {
        startPosition[i] = currentPosition[i];
        startOrientation[i] = currentOrientation[i];
    }
    
    // Zielposition setzen
    targetPosition[0] = x;
    targetPosition[1] = y;
    targetPosition[2] = z;
    targetOrientation[0] = roll;
    targetOrientation[1] = pitch;
    targetOrientation[2] = yaw;
    
    // Bewegungsparameter initialisieren
    moveDuration = duration;
    currentStep = 0;
    totalSteps = duration / UPDATE_INTERVAL;
    isMoving = true;
    
    // Startzeit merken
    moveStartTime = millis();
    
    if (configHandler && configHandler->config.useVerboseMode) {
        Serial.printf("Starte Bewegung: Von (%.1f, %.1f, %.1f) nach (%.1f, %.1f, %.1f) in %lu ms\n",
                      startPosition[0], startPosition[1], startPosition[2],
                      targetPosition[0], targetPosition[1], targetPosition[2],
                      duration);
    }
}

// Bewegung zu einer relativen Position
void TrajectoryPlanner::moveRelative(float dx, float dy, float dz, float droll, float dpitch, float dyaw, unsigned long duration) {
    startMove(
        currentPosition[0] + dx,
        currentPosition[1] + dy,
        currentPosition[2] + dz,
        currentOrientation[0] + droll,
        currentOrientation[1] + dpitch,
        currentOrientation[2] + dyaw,
        duration
    );
}

// Bewegung zu einem vordefinierten Zielpunkt
void TrajectoryPlanner::moveToPoint(uint8_t pointIndex, unsigned long duration) {
    if (pointIndex >= MAX_WAYPOINTS || !waypoints[pointIndex].valid) {
        if (configHandler && configHandler->config.useVerboseMode) {
            Serial.printf("Ungültiger Zielpunkt: %d\n", pointIndex);
        }
        return;
    }
    
    startMove(
        waypoints[pointIndex].position[0],
        waypoints[pointIndex].position[1],
        waypoints[pointIndex].position[2],
        waypoints[pointIndex].orientation[0],
        waypoints[pointIndex].orientation[1],
        waypoints[pointIndex].orientation[2],
        duration
    );
}

// Zielpunkt speichern
bool TrajectoryPlanner::saveWaypoint(uint8_t index) {
    if (index >= MAX_WAYPOINTS) return false;
    
    waypoints[index].valid = true;
    for (int i = 0; i < 3; i++) {
        waypoints[index].position[i] = currentPosition[i];
        waypoints[index].orientation[i] = currentOrientation[i];
    }
    
    if (configHandler && configHandler->config.useVerboseMode) {
        Serial.printf("Waypoint %d gespeichert: (%.1f, %.1f, %.1f)\n", 
                     index, currentPosition[0], currentPosition[1], currentPosition[2]);
    }
    
    return true;
}

// Regelmäßiges Update der Bewegung
void TrajectoryPlanner::update() {
    unsigned long currentTime = millis();
    
    // Zeitdifferenz seit dem letzten Update
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0f; // in Sekunden
    lastUpdateTime = currentTime;
    
    // Wenn keine Bewegung aktiv ist
    if (!isMoving) {
        // Trotzdem Sensor-Feedback anwenden, falls aktiviert
        if (configHandler && configHandler->config.currentDriveMode == MODE_IK_ADXL) {
            applyFeedbackCorrection(deltaTime);
        }
        return;
    }
    
    // Berechne die verstrichene Zeit seit Bewegungsbeginn
    unsigned long elapsedTime = currentTime - moveStartTime;
    
    // Prüfe ob die Bewegung abgeschlossen ist
    if (elapsedTime >= moveDuration) {
        // Bewegung abschließen
        for (int i = 0; i < 3; i++) {
            currentPosition[i] = targetPosition[i];
            currentOrientation[i] = targetOrientation[i];
        }
        
        isMoving = false;
        
        if (configHandler && configHandler->config.useVerboseMode) {
            Serial.println("Bewegung abgeschlossen");
        }
        
        // Position an die Kinematik übertragen
        updateRobotPosition();
        return;
    }
    
    // Fortschritt berechnen (0.0 bis 1.0)
    float progress = static_cast<float>(elapsedTime) / moveDuration;
    
    // Je nach Trajektorientyp unterschiedliche Interpolation
    float factor;
    switch (trajectoryType) {
        case TRAJECTORY_LINEAR:
            factor = progress;
            break;
            
        case TRAJECTORY_SMOOTH:
            // Smooth-Step-Funktion für sanfte Beschleunigung/Verzögerung
            factor = progress * progress * (3.0f - 2.0f * progress);
            break;
            
        case TRAJECTORY_ACCELERATE:
            // Quadratische Beschleunigung
            factor = progress * progress;
            break;
            
        case TRAJECTORY_DECELERATE:
            // Quadratische Verzögerung
            factor = 1.0f - (1.0f - progress) * (1.0f - progress);
            break;
            
        default:
            factor = progress;
    }
    
    // Position und Orientierung interpolieren
    for (int i = 0; i < 3; i++) {
        currentPosition[i] = startPosition[i] + factor * (targetPosition[i] - startPosition[i]);
        currentOrientation[i] = startOrientation[i] + factor * (targetOrientation[i] - startOrientation[i]);
    }
    
    // Sensor-Feedback anwenden, falls aktiviert
    if (configHandler && configHandler->config.currentDriveMode == MODE_IK_ADXL) {
        applyFeedbackCorrection(deltaTime);
    }
    
    // Position an die Kinematik übertragen
    updateRobotPosition();
}

// Anwenden von Sensor-Feedback zur Korrektur
void TrajectoryPlanner::applyFeedbackCorrection(float deltaTime) {
    if (!adxlTracker || !lidarSensor) {
        return; // Keine Sensoren verfügbar
    }

    // Nur korrigieren, wenn beide Sensoren gültige Daten liefern
    if (!adxlTracker->isDataValid() || !lidarSensor->isValid()) {
        return;
    }

    // Beschleunigungsdaten abrufen
    float xAccel = adxlTracker->getX();
    float yAccel = adxlTracker->getY();
    float zAccel = adxlTracker->getZ();
    
    // Neigungswinkel des Roboters berechnen
    float roll = adxlTracker->getRoll();
    float pitch = adxlTracker->getPitch();
    
    // Aktuelle Abstandsmessung vom LIDAR
    float currentDistance = lidarSensor->getDistance();
    
    // Schwellenwerte für Korrekturen
    const float ANGLE_THRESHOLD = 5.0f;     // in Grad
    const float ACCEL_THRESHOLD = 0.2f;     // in g
    const float DISTANCE_THRESHOLD = 10.0f; // in mm
    
    // Korrekturfaktoren
    float heightCorrection = 0.0f;
    
    // 1. LIDAR-basierte Höhenkorrektur
    if (fabsf(currentDistance - targetDistance) > DISTANCE_THRESHOLD) {
        // Berechne Höhenkorrektur basierend auf LIDAR-Abstandsmessung
        // Negativer Wert, wenn zu weit weg, positiver Wert, wenn zu nah
        heightCorrection = (targetDistance - currentDistance) * 0.05f; // Dämpfungsfaktor
        
        // Begrenzen der maximalen Korrektur pro Zeitschritt
        if (heightCorrection > MAX_CORRECTION_STEP) heightCorrection = MAX_CORRECTION_STEP;
        if (heightCorrection < -MAX_CORRECTION_STEP) heightCorrection = -MAX_CORRECTION_STEP;
        
        // Z-Position aktualisieren
        currentPosition[2] += heightCorrection;
        
        if (configHandler && configHandler->config.useVerboseMode) {
            Serial.printf("LIDAR Korrektur: %.2f mm (Ziel: %.1f, Ist: %.1f)\n", 
                         heightCorrection, targetDistance, currentDistance);
        }
    }
    
    // 2. ADXL-basierte Winkelkorrektur
    if (fabsf(roll) > ANGLE_THRESHOLD || fabsf(pitch) > ANGLE_THRESHOLD) {
        // Berechne eine Korrektur, um die Neigung zu kompensieren
        float rollCorrection = -roll * 0.02f;  // Dämpfungsfaktor anpassen
        float pitchCorrection = -pitch * 0.02f;
        
        // Begrenzen der maximalen Winkelkorrektur
        if (rollCorrection > MAX_ANGLE_CORRECTION) rollCorrection = MAX_ANGLE_CORRECTION;
        if (rollCorrection < -MAX_ANGLE_CORRECTION) rollCorrection = -MAX_ANGLE_CORRECTION;
        if (pitchCorrection > MAX_ANGLE_CORRECTION) pitchCorrection = MAX_ANGLE_CORRECTION;
        if (pitchCorrection < -MAX_ANGLE_CORRECTION) pitchCorrection = -MAX_ANGLE_CORRECTION;
        
        // Korrektur auf die Orientierung anwenden
        currentOrientation[0] += rollCorrection;
        currentOrientation[1] += pitchCorrection;
        
        if (configHandler && configHandler->config.useVerboseMode) {
            Serial.printf("ADXL Winkelkorrektur: Roll %.2f°, Pitch %.2f°\n", 
                         rollCorrection, pitchCorrection);
        }
    }
    
    // 3. Verwende zAccel für vertikale Beschleunigungserkennung
    if (fabsf(zAccel - 1.0f) > ACCEL_THRESHOLD) {
        // zAccel sollte normalerweise etwa 1g sein (durch Schwerkraft)
        // Werte > 1 bedeuten Aufwärtsbeschleunigung, < 1 bedeuten Abwärtsbeschleunigung
        
        // Berechne eine sanfte Korrektur basierend auf der Z-Beschleunigung
        float zCorrection = (1.0f - zAccel) * 0.5f; // Dämpfungsfaktor
        
        // Begrenzen der maximalen Beschleunigungskorrektur
        if (zCorrection > MAX_CORRECTION_STEP) zCorrection = MAX_CORRECTION_STEP;
        if (zCorrection < -MAX_CORRECTION_STEP) zCorrection = -MAX_CORRECTION_STEP;
        
        // Z-Position aktualisieren
        currentPosition[2] += zCorrection;
        
        if (configHandler && configHandler->config.useVerboseMode) {
            Serial.printf("Z-Beschl. Korrektur: %.2f mm (Z-Accel: %.2f g)\n", 
                         zCorrection, zAccel);
        }
    }
    
    // Korrigierte Position/Orientierung an die Kinematik senden
    updateRobotPosition();
}

// Aktuelle Position an die Kinematik übergeben
void TrajectoryPlanner::updateRobotPosition() {
    if (!kinematicsHandler) return;
    
    // Inverse Kinematik aufrufen, um die Motorwinkel zu berechnen
    // Übergebe die Arrays direkt statt einzelner Werte
    bool success = kinematicsHandler->calculateInverseKinematics(
        currentPosition, currentOrientation
    );
    
    if (!success && configHandler && configHandler->config.useVerboseMode) {
        Serial.println("Fehler bei der inversen Kinematik!");
    }
}

String TrajectoryPlanner::getDebug() const {
    String debugInfo = "";
    
    // Position und Orientierung
    debugInfo += "Pos: [";
    debugInfo += String(currentPosition[0], 1) + ", ";
    debugInfo += String(currentPosition[1], 1) + ", ";
    debugInfo += String(currentPosition[2], 1) + "] ";
    
    debugInfo += "Ori: [";
    debugInfo += String(currentOrientation[0], 1) + ", ";
    debugInfo += String(currentOrientation[1], 1) + ", ";
    debugInfo += String(currentOrientation[2], 1) + "] ";
    
    // Bewegungsinformationen
    if (isMoving) {
        unsigned long elapsedTime = millis() - moveStartTime;
        int progressPercent = min(100, static_cast<int>((elapsedTime * 100) / moveDuration));
        
        debugInfo += "Moving: " + String(progressPercent) + "% ";
        debugInfo += "(" + String(elapsedTime) + "/" + String(moveDuration) + "ms)";
    } else {
        debugInfo += "Stopped";
    }
    
    // Sensordaten hinzufügen, falls verfügbar
    if (lidarSensor && lidarSensor->isValid()) {
        debugInfo += " | LIDAR: " + String(lidarSensor->getDistance()) + "mm";
    }
    
    if (adxlTracker && adxlTracker->isDataValid()) {
        debugInfo += " | Roll: " + String(adxlTracker->getRoll(), 1);
        debugInfo += " Pitch: " + String(adxlTracker->getPitch(), 1);
    }
    
    return debugInfo;
}

// Aktuelle Position und Orientierung abrufen
void TrajectoryPlanner::getCurrentPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw) {
    x = currentPosition[0];
    y = currentPosition[1];
    z = currentPosition[2];
    roll = currentOrientation[0];
    pitch = currentOrientation[1];
    yaw = currentOrientation[2];
}

// Ziel-Distanz für LIDAR-Korrekturen setzen
void TrajectoryPlanner::setTargetDistance(float distance) {
    targetDistance = distance;
    
    if (configHandler && configHandler->config.useVerboseMode) {
        Serial.printf("LIDAR Zieldistanz: %.1f mm\n", targetDistance);
    }
}

// Art der Trajektorie setzen
void TrajectoryPlanner::setTrajectoryType(TrajectoryType type) {
    trajectoryType = type;
}

// Trajektorie pausieren
void TrajectoryPlanner::pauseTrajectory() {
    if (!isMoving) return;
    
    // Merken der aktuellen Position als neue Startposition
    for (int i = 0; i < 3; i++) {
        startPosition[i] = currentPosition[i];
        startOrientation[i] = currentOrientation[i];
    }
    
    // Bewegung pausieren
    isMoving = false;
    
    if (configHandler && configHandler->config.useVerboseMode) {
        Serial.println("Trajektorie pausiert");
    }
}

// Trajektorie fortsetzen
void TrajectoryPlanner::resumeTrajectory() {
    if (isMoving) return;
    
    // Neue Bewegung mit restlicher Strecke
    startMove(
        targetPosition[0],
        targetPosition[1],
        targetPosition[2],
        targetOrientation[0],
        targetOrientation[1],
        targetOrientation[2],
        moveDuration * (1.0f - static_cast<float>(currentStep) / totalSteps)
    );
    
    if (configHandler && configHandler->config.useVerboseMode) {
        Serial.println("Trajektorie fortgesetzt");
    }
}

// Notfall-Stop
void TrajectoryPlanner::emergencyStop() {
    isMoving = false;
    
    if (configHandler && configHandler->config.useVerboseMode) {
        Serial.println("NOTFALL-STOP aktiviert!");
    }
}

// Ist eine Bewegung im Gange?
bool TrajectoryPlanner::isTrajectoryActive() {
    return isMoving;
}

// Alle gespeicherten Wegpunkte laden
bool TrajectoryPlanner::loadWaypoints(const char* filename) {
    // Implementierung zum Laden von Wegpunkten von SD-Karte
    // ...
    
    return true;
}

// Alle gespeicherten Wegpunkte speichern
bool TrajectoryPlanner::saveWaypoints(const char* filename) {
    // Implementierung zum Speichern von Wegpunkten auf SD-Karte
    // ...
    
    return true;
}
