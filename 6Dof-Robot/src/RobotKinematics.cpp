#include "RobotKinematics.h"
#include <math.h>

// Konstruktor
RobotKinematics::RobotKinematics() {
  for (int i = 0; i < 6; i++) {
    _currentAngles.angles[i] = 0.0;
    _config.minAngle[i] = -180.0;
    _config.maxAngle[i] = 180.0;
    _config.dh[i] = {0.0, 0.0, 0.0, 0.0};
  }
  _config.toolOffsetX = _config.toolOffsetY = _config.toolOffsetZ = 0.0;
  _currentPose = forwardKinematics(_currentAngles);
}

void RobotKinematics::init(const RobotConfig& robot_config) {
    _config = robot_config;
    for (int i = 0; i < 6; i++) {
        _currentAngles.angles[i] = 0.0;
    }
   _currentPose = forwardKinematics(_currentAngles);
}

// Matrixmultiplikation 4x4 * 4x4
void multiplyMatrix(const float A[4][4], const float B[4][4], float result[4][4]) {
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      result[r][c] = 0;
      for (int k = 0; k < 4; k++) {
        result[r][c] += A[r][k] * B[k][c];
      }
    }
  }
}

CartesianPose RobotKinematics::forwardKinematics(const JointAngles& angles) {
  float T[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };
  
  for (int i = 0; i < 6; i++) {
    float a = _config.dh[i].a;
    float alpha = _config.dh[i].alpha;
    float d = _config.dh[i].d;
    float theta = _config.dh[i].theta + angles.angles[i];
    
    float ct = cos(theta), st = sin(theta);
    float ca = cos(alpha), sa = sin(alpha);
    
    float A[4][4] = {
      {ct, -st*ca, st*sa, a*ct},
      {st, ct*ca, -ct*sa, a*st},
      {0, sa, ca, d},
      {0, 0, 0, 1}
    };
    
    float temp[4][4];
    multiplyMatrix(T, A, temp);
    memcpy(T, temp, sizeof(temp));
  }
  
  CartesianPose pose;
  pose.x = T[0][3] + _config.toolOffsetX;
  pose.y = T[1][3] + _config.toolOffsetY;
  pose.z = T[2][3] + _config.toolOffsetZ;
  
  // Berechne Euler-Winkel aus der Rotationsmatrix
  pose.pitch = atan2(-T[2][0], sqrt(T[0][0]*T[0][0] + T[1][0]*T[1][0]));
  
  // Behandle Singularitäten (Gimbal Lock)
  if (fabs(pose.pitch - M_PI/2) < 0.001) {
    pose.yaw = 0;
    pose.roll = atan2(T[0][1], T[1][1]);
  } else if (fabs(pose.pitch + M_PI/2) < 0.001) {
    pose.yaw = 0;
    pose.roll = -atan2(T[0][1], T[1][1]);
  } else {
    pose.yaw = atan2(T[1][0]/cos(pose.pitch), T[0][0]/cos(pose.pitch));
    pose.roll = atan2(T[2][1]/cos(pose.pitch), T[2][2]/cos(pose.pitch));
  }
  
  return pose;
}

bool RobotKinematics::inverseKinematics(const CartesianPose& targetPose, JointAngles& angles) {
    // Add this debug code near the start of the function:
    Serial.print("IK Target: X=");
    Serial.print(targetPose.x);
    Serial.print(" Y=");
    Serial.print(targetPose.y);
    Serial.print(" Z=");
    Serial.println(targetPose.z);
    
    // Calculate wrist position
    float wx = targetPose.x - (_config.dh[5].a + _config.toolOffsetX) * cos(targetPose.yaw) * cos(targetPose.pitch);
    float wy = targetPose.y - (_config.dh[5].a + _config.toolOffsetY) * sin(targetPose.yaw) * cos(targetPose.pitch);
    float wz = targetPose.z - (_config.dh[5].a + _config.toolOffsetZ) * sin(targetPose.pitch) + _config.dh[5].d;
    
    Serial.print("Wrist position: X=");
    Serial.print(wx);
    Serial.print(" Y=");
    Serial.print(wy);
    Serial.print(" Z=");
    Serial.println(wz);
    
    // Berechne Gelenkwinkel 1 (Basis)
    float theta1 = atan2(wy, wx);
    
    // Überprüfe, ob der Winkel im gültigen Bereich liegt
    float theta1Deg = theta1 * 180.0 / M_PI;
    if (theta1Deg < _config.minAngle[0] || theta1Deg > _config.maxAngle[0]) {
      Serial.println("Gelenkwinkel 1 außerhalb des gültigen Bereichs");
      return false;
    }
    
    // Berechne die Distanz vom Ursprung zum Handgelenk in der XY-Ebene
    float r = sqrt(wx*wx + wy*wy);
    
    // Berücksichtige den Offset der ersten Achse
    r -= _config.dh[0].a;
    
    // Höhe des Handgelenks relativ zur Basis
    float z = wz - _config.dh[0].d;
    
    // Berechne die Länge der Glieder 2 und 3
    float a2 = _config.dh[1].a;
    float a3 = _config.dh[2].a;
    
    // Berechne die Distanz vom Schultergelenk zum Handgelenk
    float d = sqrt(r*r + z*z);
    
    // Überprüfe, ob die Position erreichbar ist
    if (d > (a2 + a3) || d < abs(a2 - a3)) {
      Serial.print("Position außerhalb der Reichweite: d=");
      Serial.print(d);
      Serial.print(", a2+a3=");
      Serial.print(a2 + a3);
      Serial.print(", |a2-a3|=");
      Serial.println(abs(a2 - a3));
      return false;
    }
    
    // Berechne Gelenkwinkel 3 (Ellbogen) mit Kosinussatz
    float cos_theta3 = (d*d - a2*a2 - a3*a3) / (2 * a2 * a3);
    
    // Begrenze cos_theta3 auf gültigen Bereich [-1, 1]
    cos_theta3 = constrain(cos_theta3, -1.0, 1.0);
    
    float theta3 = acos(cos_theta3);
    
    // Überprüfe, ob der Winkel im gültigen Bereich liegt
    float theta3Deg = theta3 * 180.0 / M_PI;
    if (theta3Deg < _config.minAngle[2] || theta3Deg > _config.maxAngle[2]) {
      Serial.println("Gelenkwinkel 3 außerhalb des gültigen Bereichs");
      return false;
    }
    
    // Berechne Gelenkwinkel 2 (Schulter)
    float beta = atan2(z, r);
    float alpha = atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
    float theta2 = beta - alpha;
    
    // Überprüfe, ob der Winkel im gültigen Bereich liegt
    float theta2Deg = theta2 * 180.0 / M_PI;
    if (theta2Deg < _config.minAngle[1] || theta2Deg > _config.maxAngle[1]) {
      Serial.println("Gelenkwinkel 2 außerhalb des gültigen Bereichs");
      return false;
    }
    
    // Berechne die Rotationsmatrix der ersten drei Gelenke
    float c1 = cos(theta1), s1 = sin(theta1);
    float c23 = cos(theta2 + theta3), s23 = sin(theta2 + theta3);
    
    float R03[3][3] = {
      {c1 * c23, -c1 * s23, s1},
      {s1 * c23, -s1 * s23, -c1},
      {s23, c23, 0}
    };
    
    // Berechne die gewünschte Orientierung des Endeffektors
    float R06[3][3] = {
      {cos(targetPose.yaw) * cos(targetPose.pitch), -sin(targetPose.yaw), cos(targetPose.yaw) * sin(targetPose.pitch)},
      {sin(targetPose.yaw) * cos(targetPose.pitch), cos(targetPose.yaw), sin(targetPose.yaw) * sin(targetPose.pitch)},
      {-sin(targetPose.pitch), 0, cos(targetPose.pitch)}
    };
    
    // Berechne die Rotationsmatrix für die letzten drei Gelenke
    float R36[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R36[i][j] = 0;
        for (int k = 0; k < 3; k++) {
          // Transponierte von R03 multipliziert mit R06
          R36[i][j] += R03[k][i] * R06[k][j];
        }
      }
    }
    
    // Berechne die Euler-Winkel aus R36
    float theta4 = atan2(R36[1][2], R36[0][2]);
    float theta5 = atan2(sqrt(R36[0][2]*R36[0][2] + R36[1][2]*R36[1][2]), R36[2][2]);
    float theta6 = atan2(R36[2][1], -R36[2][0]);
    
    // Überprüfe, ob die Winkel im gültigen Bereich liegen
    float theta4Deg = theta4 * 180.0 / M_PI;
    float theta5Deg = theta5 * 180.0 / M_PI;
    float theta6Deg = theta6 * 180.0 / M_PI;
    
    if (theta4Deg < _config.minAngle[3] || theta4Deg > _config.maxAngle[3] ||
        theta5Deg < _config.minAngle[4] || theta5Deg > _config.maxAngle[4] ||
        theta6Deg < _config.minAngle[5] || theta6Deg > _config.maxAngle[5]) {
      Serial.println("Gelenkwinkel 4, 5 oder 6 außerhalb des gültigen Bereichs");
      return false;
    }
    
    // Setze die berechneten Gelenkwinkel
    angles.angles[0] = theta1;
    angles.angles[1] = theta2;
    angles.angles[2] = theta3;
    angles.angles[3] = theta4;
    angles.angles[4] = theta5;
    angles.angles[5] = theta6;
    
    // Überprüfe die Lösung mit Vorwärtskinematik
    CartesianPose checkPose = forwardKinematics(angles);
    float posError = sqrt(
      pow(checkPose.x - targetPose.x, 2) +
      pow(checkPose.y - targetPose.y, 2) +
      pow(checkPose.z - targetPose.z, 2)
    );
    
    Serial.print("IK Lösung gefunden. Positionsfehler: ");
    Serial.print(posError);
    Serial.println(" mm");
    
    // Gib die berechneten Winkel aus
    Serial.println("Berechnete Gelenkwinkel (Grad):");
    for (int i = 0; i < 6; i++) {
      Serial.print("  Gelenk ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.println(angles.angles[i] * 180.0 / M_PI);
    }
    
    return true;
}

// Hilfsfunktion: Normalisiere Winkel auf Bereich [-PI, PI]
float RobotKinematics::normalizeAngle(float angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

void RobotKinematics::setCurrentJointAngles(const JointAngles& angles) {
  _currentAngles = angles;
  _currentPose = forwardKinematics(_currentAngles);
}

JointAngles RobotKinematics::getCurrentJointAngles() const {
  return _currentAngles;
}

CartesianPose RobotKinematics::getCurrentPose() const {
  return _currentPose;
}

bool RobotKinematics::isValidJointAngles(const JointAngles& angles) {
  for (int i = 0; i < 6; i++) {
    float deg = angles.angles[i] * 180.0 / M_PI;
    if (deg < _config.minAngle[i] || deg > _config.maxAngle[i]) return false;
  }
  return true;
}

bool RobotKinematics::isPoseReachable(const CartesianPose& pose) {
  JointAngles test;
  return inverseKinematics(pose, test);
}