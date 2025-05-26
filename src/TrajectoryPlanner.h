#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "KinematicsHandler.h"
#include "ConfigHandler.h"
#include "AdxlTracker.h"
#include "VL53L0XSensor.h"

// Verschiedene Arten von Trajektorien
enum TrajectoryType {
    TRAJECTORY_LINEAR,    // Lineare Interpolation
    TRAJECTORY_SMOOTH,    // Sanfter Start und Stopp
    TRAJECTORY_ACCELERATE,// Beschleunigen
    TRAJECTORY_DECELERATE // Abbremsen
};

// Struktur für einen Wegpunkt
struct Waypoint {
    float position[3];    // X, Y, Z
    float orientation[3]; // Roll, Pitch, Yaw
    bool valid;           // Ist der Wegpunkt gültig?
};

class TrajectoryPlanner {
public:
    // Konstruktor
    TrajectoryPlanner(KinematicsHandler* kinematics, ConfigHandler* config);
    String getDebug() const;
    // Sensoren setzen
    void setAdxlTracker(AdxlTracker* tracker);
    void setLidarSensor(VL53L0XSensor* lidar);
    
    // Bewegungsfunktionen
    void startMove(float x, float y, float z, float roll, float pitch, float yaw, unsigned long duration);
    void moveRelative(float dx, float dy, float dz, float droll, float dpitch, float dyaw, unsigned long duration);
    void moveToPoint(uint8_t pointIndex, unsigned long duration);
    
    // Wegpunkte verwalten
    bool saveWaypoint(uint8_t index);
    bool loadWaypoints(const char* filename);
    bool saveWaypoints(const char* filename);
    
    // Status und Steuerung
    void update();
    void pauseTrajectory();
    void resumeTrajectory();
    void emergencyStop();
    bool isTrajectoryActive();
    
    // Einstellungen
    void setTrajectoryType(TrajectoryType type);
    void setTargetDistance(float distance);
    
    // Aktuelle Position und Orientierung
    void getCurrentPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw);
    
    // Sensor-Feedback
    void applyFeedbackCorrection(float deltaTime);

private:
    // Referenzen auf andere Komponenten
    KinematicsHandler* kinematicsHandler;
    ConfigHandler* configHandler;
    AdxlTracker* adxlTracker;
    VL53L0XSensor* lidarSensor;
    
    // Aktuelle Position und Orientierung
    float currentPosition[3];    // X, Y, Z
    float currentOrientation[3]; // Roll, Pitch, Yaw
    
    // Zielposition und Orientierung
    float targetPosition[3];
    float targetOrientation[3];
    
    // Startposition und Orientierung (für Interpolation)
    float startPosition[3];
    float startOrientation[3];
    
    // Bewegungsparameter
    bool isMoving;
    unsigned long moveStartTime;
    unsigned long moveDuration;
    unsigned long lastUpdateTime;
    unsigned int currentStep;
    unsigned int totalSteps;
    TrajectoryType trajectoryType;
    
    // Konstanten für Sensorkorrekturen
    static constexpr float MAX_CORRECTION_STEP = 5.0f;   // max mm pro Update
    static constexpr float MAX_ANGLE_CORRECTION = 2.0f;  // max Grad pro Update
    float targetDistance;                                // Zielabstand in mm
     // Wegpunkte
    static const int MAX_WAYPOINTS = 50;
    Waypoint waypoints[MAX_WAYPOINTS];
    
    // Aktualisierungsintervall in ms
    static const unsigned long UPDATE_INTERVAL = 20;
    // Hilfsfunktionen
    void updateRobotPosition();
};

#endif // TRAJECTORY_PLANNER_H
