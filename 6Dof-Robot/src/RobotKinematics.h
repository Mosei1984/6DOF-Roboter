#pragma once
#include <Arduino.h>

// Struktur für kartesische Position und Orientierung
struct CartesianPose {
  float x, y, z;        // Position in mm
  float roll, pitch, yaw; // Orientierung in Radiant
};

// Struktur für die Gelenkwinkel
struct JointAngles {
  float angles[6];      // Gelenkwinkel in Radiant
};

// Denavit-Hartenberg-Parameter für ein Gelenk
struct DHParams {
  float a;      // Länge des Gliedes
  float alpha;  // Verdrehung des Gliedes
  float d;      // Offset entlang der z-Achse
  float theta;  // Gemeinsamer Offset-Winkel
};

// Konfiguration des Roboters
struct RobotConfig {
  float minAngle[6];    // Minimaler Winkel pro Gelenk in Grad
  float maxAngle[6];    // Maximaler Winkel pro Gelenk in Grad
  DHParams dh[6];       // DH-Parameter für jedes Gelenk
  float toolOffsetX;    // Werkzeug-Offset in X-Richtung
  float toolOffsetY;    // Werkzeug-Offset in Y-Richtung
  float toolOffsetZ;    // Werkzeug-Offset in Z-Richtung
};

class RobotKinematics {
public:
  RobotKinematics();
  
  // Initialisierung mit Roboterkonfiguration
  void init(const RobotConfig& robot_config);
  
  // Vorwärtskinematik: Berechne Position aus Gelenkwinkeln
  CartesianPose forwardKinematics(const JointAngles& angles);
  
  // Inverse Kinematik: Berechne Gelenkwinkel aus Position
  bool inverseKinematics(const CartesianPose& targetPose, JointAngles& angles);
  
  // Setzen und Abrufen des aktuellen Roboterzustands
  void setCurrentJointAngles(const JointAngles& angles);
  JointAngles getCurrentJointAngles() const;
  CartesianPose getCurrentPose() const;
  
  // Hilfsfunktionen
  bool isValidJointAngles(const JointAngles& angles);
  bool isPoseReachable(const CartesianPose& pose);
  float normalizeAngle(float angle);

private:
  RobotConfig _config;
  JointAngles _currentAngles;
  CartesianPose _currentPose;
};