#include "InverseKinematics.h"
extern AccelStepper motors[6];
extern Servo gripper;
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern int16_t joyLXCenter, joyLYCenter, joyRZCenter, joyRYawCenter;
extern int joystick_lx, joystick_ly, joystick_rz, joystick_ryaw;
extern bool buttonConfirmPressed;
extern float stepsToDegree(int jointIndex, long steps);

// Initialize with proper Cartesian coordinates for the home position
// These values should be calculated based on the forward kinematics of HOME_ANGLES
double currentTargetPos[3] = {0.225, 0.0, 0.225}; // Example values - update with actual FK result
double currentTargetRot[3] = {0.0, 0.0, 0.0}; // Zero orientation at home position
bool ikSolutionReady = false;

// Internal storage for IK result
double resultJointAngles[6];
bool ikSolutionFound = false;

// Define these global variables ONLY in InverseKinematics.cpp
bool coordMoveActive = false;
unsigned long coordMoveStartTime = 0;
unsigned long coordMoveDuration = 0;
long coordMoveStartSteps[6] = {0, 0, 0, 0, 0, 0};
long coordMoveTargetSteps[6] = {0, 0, 0, 0, 0, 0};
double cartesianStartPos[3] = {0.0, 0.0, 0.0};
double cartesianTargetPos[3] = {0.0, 0.0, 0.0};
double cartesianStartRot[3] = {0.0, 0.0, 0.0};
double cartesianTargetRot[3] = {0.0, 0.0, 0.0};

// Hilfsfunktion: Begrenze einen Winkel auf die zulässigen Gelenk-Limits
static double constrainAngle(int jointIndex, double angle) {
    if (angle < jointLimits[jointIndex].minAngle) {
        return jointLimits[jointIndex].minAngle;
    }
    if (angle > jointLimits[jointIndex].maxAngle) {
        return jointLimits[jointIndex].maxAngle;
    }
    return angle;
}

// Hilfsfunktion: Signum-Funktion
static int sgn(double x) {
    return (x > 0) - (x < 0);
}

// Complete inverse kinematics calculation implementation
bool calculateInverseKinematics(
    const double targetPosition[3],
    const double targetRotationZYX[3],
    const double currentJointAngles[6]
) {
    // 1. Ziel-Rotationsmatrix R06 aus Euler-ZYX (Roll, Pitch, Yaw) erzeugen
    double roll  = targetRotationZYX[0];
    double pitch = targetRotationZYX[1];
    double yaw   = targetRotationZYX[2];
    
    // Rotationsmatrizen für Z, Y, X einzeln:
    // Rz(yaw)
    std::array<std::array<double,4>,4> Rz = {{
        {{ cos(yaw), -sin(yaw), 0.0, 0.0 }},
        {{ sin(yaw),  cos(yaw), 0.0, 0.0 }},
        {{ 0.0,       0.0,      1.0, 0.0 }},
        {{ 0.0,       0.0,      0.0, 1.0 }}
    }};
    
    // Ry(pitch)
    std::array<std::array<double,4>,4> Ry = {{
        {{  cos(pitch), 0.0, sin(pitch), 0.0 }},
        {{  0.0,        1.0, 0.0,        0.0 }},
        {{ -sin(pitch), 0.0, cos(pitch), 0.0 }},
        {{  0.0,        0.0, 0.0,        1.0 }}
    }};
    
    // Rx(roll)
    std::array<std::array<double,4>,4> Rx = {{
        {{ 1.0, 0.0,        0.0,       0.0 }},
        {{ 0.0, cos(roll), -sin(roll), 0.0 }},
        {{ 0.0, sin(roll),  cos(roll), 0.0 }},
        {{ 0.0, 0.0,        0.0,       1.0 }}
    }};
    
    // R06 = Rz * Ry * Rx
    auto RzRy = matMul4x4(Rz, Ry);
    auto R06  = matMul4x4(RzRy, Rx);
    
    // 2. Handgelenkszentrum (Wrist Center) berechnen
    //    Verwende das Werkzeug-Offset als d6 (robotDHParams[5].d) oder separate toolLength
    double toolLength = robotDHParams[5].d;
    
    // Die z-Achse des Endeffektors (3. Spalte von R06)
    std::array<double,3> zAxisEnd = { R06[0][2], R06[1][2], R06[2][2] };
    std::array<double,3> wristCenter;
    for (int i = 0; i < 3; ++i) {
        wristCenter[i] = targetPosition[i] - toolLength * zAxisEnd[i];
    }
    
    // 3. Gelenkwinkel θ1, θ2, θ3 berechnen (Planare Dreieckslösung)
    double xw = wristCenter[0];
    double yw = wristCenter[1];
    double zw = wristCenter[2];
    
    // θ1 = atan2(yw, xw)
    double theta1 = atan2(yw, xw);
    theta1 = constrainAngle(0, theta1);
    
    // Parameter für Schulter/Ellenbogen
    double d0 = robotDHParams[0].d; // z-Achsen-Offset der Basis
    double r = sqrt(xw*xw + yw*yw) - robotDHParams[0].a; 
    double s = zw - d0;
    double a1 = robotDHParams[1].a;
    double a2 = robotDHParams[2].a;
    double D = (r*r + s*s - a1*a1 - a2*a2) / (2 * a1 * a2);
    
    if (fabs(D) > 1.0) {
        ikSolutionFound = false;
        return false;  // kein gültiger Bereich für Ellbogen
    }
    
    // Wähle Ellbogen-„unten"-Konfiguration (positive Wurzel); alternativ:
    //  double theta3_alt = atan2(-sqrt(1 - D*D), D);
    double theta3 = atan2(+sqrt(1 - D*D), D);
    theta3 = constrainAngle(2, theta3);
    
    // θ2 aus Planardreieck
    double k1 = a1 + a2 * cos(theta3);
    double k2 = a2 * sin(theta3);
    double gamma = atan2(k2, k1);
    double phi   = atan2(s, r);
    double theta2 = phi - gamma;
    theta2 = constrainAngle(1, theta2);
    
    // 4. Berechne T0_3 und Rotationsmatrix R0_3
    // DH-Matrizen für Gelenke 1..3
    std::array<std::array<double,4>,4> T0_1 = dhToMatrix({ robotDHParams[0].a, robotDHParams[0].alpha, robotDHParams[0].d, theta1 });
    std::array<std::array<double,4>,4> T1_2 = dhToMatrix({ robotDHParams[1].a, robotDHParams[1].alpha, robotDHParams[1].d, theta2 });
    std::array<std::array<double,4>,4> T2_3 = dhToMatrix({ robotDHParams[2].a, robotDHParams[2].alpha, robotDHParams[2].d, theta3 });
    
    auto T0_2 = matMul4x4(T0_1, T1_2);
    auto T0_3 = matMul4x4(T0_2, T2_3);
    
    // Rotationsanteil R0_3 = obere 3×3 von T0_3
    // Transponiere R0_3 für spätere Multiplikation mit R06
    std::array<std::array<double,4>,4> R03T{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R03T[i][j] = T0_3[j][i];
        }
    }
    
    // Setze Homogenität in R03T
    R03T[0][3] = 0.0;  R03T[1][3] = 0.0;  R03T[2][3] = 0.0;
    R03T[3][0] = 0.0;  R03T[3][1] = 0.0;  R03T[3][2] = 0.0;  R03T[3][3] = 1.0;
    
    // 5. Berechne R3_6 = R0_3ᵀ * R06
    auto R36_hom = matMul4x4(R03T, R06);
    
    // 6. Extrahiere θ4, θ5, θ6 aus R36
    double r11 = R36_hom[0][0], r12 = R36_hom[0][1], r13 = R36_hom[0][2];
    double r21 = R36_hom[1][0], r22 = R36_hom[1][1], r23 = R36_hom[1][2];
    double r31 = R36_hom[2][0], r32 = R36_hom[2][1], r33 = R36_hom[2][2];
    
    // θ5 = atan2(√(r13² + r23²), r33)
    double theta5 = atan2(sqrt(r13*r13 + r23*r23), r33);
    double theta4, theta6;
    
    if (fabs(sin(theta5)) < 1e-6) {
        // Singulärer Fall: θ5 ≈ 0 oder π
        theta4 = 0.0;
        theta6 = atan2(-r12, r11);
    } else {
        theta4 = atan2(r23, r13);
        theta6 = atan2(r32, -r31);
    }
    
    theta4 = constrainAngle(3, theta4);
    theta5 = constrainAngle(4, theta5);
    theta6 = constrainAngle(5, theta6);
    
    // 7. Speichere alle sechs Winkel in resultJointAngles[]
    resultJointAngles[0] = theta1;
    resultJointAngles[1] = theta2;
    resultJointAngles[2] = theta3;
    resultJointAngles[3] = theta4;
    resultJointAngles[4] = theta5;
    resultJointAngles[5] = theta6;
    
    // 8. Validierung: Vorwärtskinematik prüfen (Positionstoleranz z.B. 1 cm = 0.01 m)
    double checkPos[3], checkRot[3];
    calculateForwardKinematics(resultJointAngles, checkPos, checkRot);
    
    double posError = sqrt(
        pow(checkPos[0] - targetPosition[0], 2) +
        pow(checkPos[1] - targetPosition[1], 2) +
        pow(checkPos[2] - targetPosition[2], 2)
    );
    
    if (posError > 0.01) {
        ikSolutionFound = false;
        return false;
    }
    
    ikSolutionFound = true;
    return true;
}

void executeInverseKinematicsMotion(double currentJointAngles[6]) {
    if (!ikSolutionFound) {
        // Keine gültige Lösung – zeige z.B. Fehlermeldung
        Serial.println("IK-Lösung ungültig. Ziel außerhalb Arbeitsraum!");
        return;
    }
    
    // IMPORTANT: Make a copy of resultJointAngles for the coordinated move
    double targetAngles[6];
    for (int i = 0; i < 6; i++) {
        targetAngles[i] = resultJointAngles[i];
        // Also update the current joint angles as requested by the function signature
        currentJointAngles[i] = resultJointAngles[i];
    }
    
    // Use the coordinated move function for smooth motion
    // Calculate appropriate duration based on largest angle change
    double maxAngleChange = 0;
    for (int i = 0; i < 6; i++) {
        double angleChange = abs(targetAngles[i] - currentJointAngles[i]);
        if (angleChange > maxAngleChange) {
            maxAngleChange = angleChange;
        }
    }
    
    // Set duration based on angle change (larger changes take longer)
    // Ensure minimum duration of 1000ms and scale based on angle
    unsigned long moveDuration = 1000 + (unsigned long)(maxAngleChange * 5000 / M_PI);
    
    // Cap at reasonable maximum duration
    moveDuration = min(moveDuration, 5000UL);
    
    // Start the coordinated move
    startCoordinatedMove(targetAngles, moveDuration);
    
    Serial.println("Bewegung an Zielpose ausgeführt!");
}

void onSetTargetButton(
    double desiredPos[3],
    double desiredRotZYX[3],
    double currentJointAngles[6]
) {
    bool ok = calculateInverseKinematics(desiredPos, desiredRotZYX, currentJointAngles);
    
    if (ok) {
        Serial.println("IK solution found. Ready to execute.");
        
        // Display the calculated joint angles
        for (int i = 0; i < 6; ++i) {
            Serial.print("Joint ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(resultJointAngles[i] * 180.0 / M_PI);
            Serial.println(" degrees");
        }
        
        ikSolutionReady = true;
    } else {
        Serial.println("Keine gültige IK-Lösung für diese Pose!");
        ikSolutionReady = false;
    }
}

void onExecuteButton(double currentJointAngles[6]) {
    executeInverseKinematicsMotion(currentJointAngles);
}

// Inverse Kinematics Mode implementation
void initInverseKinematicsMode(U8G2* display) {
    // Store display pointer for later use
    displayPtr = display;
    Serial.println("Inverse Kinematics mode initialized");
    
    // Calculate the actual Cartesian position from current joint angles
    double jointAnglesRad[6];
    for (int i = 0; i < 6; i++) {
        float degrees = stepsToDegree(i, motors[i].currentPosition());
        jointAnglesRad[i] = degrees * M_PI / 180.0;
    }
    
    // Calculate forward kinematics to get current Cartesian position
    calculateForwardKinematics(jointAnglesRad, currentTargetPos, currentTargetRot);
    
    Serial.print("Initial IK target position: X=");
    Serial.print(currentTargetPos[0]);
    Serial.print(", Y=");
    Serial.print(currentTargetPos[1]);
    Serial.print(", Z=");
    Serial.println(currentTargetPos[2]);
    
    // Display initial information
    display->clearBuffer();
    display->setFont(u8g2_font_ncenB08_tr);
    display->drawStr(0, 12, "Inverse Kinematics");
    display->drawStr(0, 24, "Use joystick to set");
    display->drawStr(0, 36, "target position");
    display->sendBuffer();
}

void handleInverseKinematicsMode() {
  
  // Update gripper from potentiometer (centralized control)
  updateGripperFromPotentiometer();
  
  // No manual control mode – joystick always modifies cartesian target
  
  // Only process joystick input if not in a coordinated move
  if (!coordMoveActive) {
    // Map joystick values to position/orientation changes
    float xChange = (joystick_lx - joyLXCenter) / 500.0;
    float yChange = (joystick_ly - joyLYCenter) / 500.0;
    float zChange = (joystick_rz - joyRZCenter) / 500.0;
    float yawChange = (joystick_ryaw - joyRYawCenter) / 200.0 * (M_PI / 180.0);
    
    // Apply deadzone
    if (abs(xChange) < 0.005) xChange = 0;
    if (abs(yChange) < 0.005) yChange = 0;
    if (abs(zChange) < 0.005) zChange = 0;
    if (abs(yawChange) < 0.005) yawChange = 0;
    
    // Update target position and orientation
    currentTargetPos[0] += xChange;
    currentTargetPos[1] += yChange;
    currentTargetPos[2] += zChange;
    currentTargetRot[2] += yawChange; // Yaw rotation
    
    // Limit to reasonable workspace
    for (int i = 0; i < 3; i++) {
      currentTargetPos[i] = constrain(currentTargetPos[i], -0.4, 0.4);
      currentTargetRot[i] = constrain(currentTargetRot[i], -M_PI, M_PI);
    }
    
    // If confirm button is pressed, calculate IK and move
    if (buttonConfirmPressed) {
      
      // Start a direct cartesian movement to target
      bool success = moveToCartesianTarget(currentTargetPos, currentTargetRot, 2000);
      
      if (success) {
        displayPtr->clearBuffer();
        displayPtr->setFont(u8g2_font_ncenB08_tr);
        displayPtr->drawStr(0, 12, "IK solution found");
        displayPtr->drawStr(0, 24, "Executing motion...");
        displayPtr->sendBuffer();
      } else {
        displayPtr->clearBuffer();
        displayPtr->setFont(u8g2_font_ncenB08_tr);
        displayPtr->drawStr(0, 12, "No valid IK solution");
        displayPtr->drawStr(0, 24, "Try another position");
        displayPtr->sendBuffer();
        delay(1000);
      }
      
      // Reset button state
      buttonConfirmPressed = false;
    }
  }
  
  // Regularly update display with current target information
  updateIKDisplay(displayPtr);
}

// Display function for IK mode
void updateIKDisplay(U8G2* display) {
    display->clearBuffer();
    display->setFont(u8g2_font_6x10_tf);
  
    // Title
    display->drawFrame(0, 0, 128, 12);
    display->drawStr(4, 10, "IK Mode");
  
    // Show current target position
    display->setCursor(0, 24);
    display->print("X:");
    display->print(currentTargetPos[0], 2);
  
    display->setCursor(64, 24);
    display->print("Y:");
    display->print(currentTargetPos[1], 2);
  
    display->setCursor(0, 36);
    display->print("Z:");
    display->print(currentTargetPos[2], 2);
  
    display->setCursor(64, 36);
    display->print("Yaw:");
    display->print(currentTargetRot[2] * 180.0 / M_PI, 0);
  
    display->setCursor(0, 48);
    display->print("Roll:");
    display->print(currentTargetRot[0] * 180.0 / M_PI, 0);
  
    display->setCursor(64, 48);
    display->print("Pitch:");
    display->print(currentTargetRot[1] * 180.0 / M_PI, 0);
  
    // Status bar at the bottom
    display->drawFrame(0, 52, 128, 12);
  
    // Only recheck IK solution every 500ms to avoid display flicker
    static unsigned long lastIKCheckTime = 0;
    static bool ikSolutionFound = false;
  
    if (millis() - lastIKCheckTime > 500) {
        lastIKCheckTime = millis();
    
        double jointAnglesRad[6];
        for (int i = 0; i < 6; i++) {
            float degrees = stepsToDegree(i, motors[i].currentPosition());
            jointAnglesRad[i] = degrees * M_PI / 180.0;
        }
    
        ikSolutionFound = calculateInverseKinematics(
            currentTargetPos,
            currentTargetRot,
            jointAnglesRad
        );
    
        ikSolutionReady = ikSolutionFound;
    }
  
    if (ikSolutionFound) {
        display->drawStr(4, 62, "Solution OK - CONFIRM to move");
    } else {
        display->drawStr(4, 62, "NO SOLUTION - Adjust target");
    }
  
    display->sendBuffer();
}

bool moveToCartesianTarget(double targetPos[3], double targetRot[3], unsigned long duration) {
  Serial.println("moveToCartesianTarget called");
  Serial.print("Target position: X=");
  Serial.print(targetPos[0]);
  Serial.print(", Y=");
  Serial.print(targetPos[1]);
  Serial.print(", Z=");
  Serial.println(targetPos[2]);
  
  // Get current joint angles
  double currentJointAngles[6];
  for (int i = 0; i < 6; i++) {
    float degrees = stepsToDegree(i, motors[i].currentPosition());
    currentJointAngles[i] = degrees * M_PI / 180.0;
    Serial.print("Current joint ");
    Serial.print(i+1);
    Serial.print(" angle: ");
    Serial.println(degrees);
  }
  
  // Calculate current position from forward kinematics
  double currentPosFK[3], currentRotFK[3];
  calculateForwardKinematics(currentJointAngles, currentPosFK, currentRotFK);
  
  // Store target position and current position
  for (int i = 0; i < 3; i++) {
    cartesianStartPos[i] = currentPosFK[i]; // Use the calculated FK position as start
    cartesianTargetPos[i] = targetPos[i];
    cartesianStartRot[i] = currentRotFK[i]; // Use the calculated FK orientation as start
    cartesianTargetRot[i] = targetRot[i];
  }
  
  // Print current position from forward kinematics
  Serial.print("Current position: X=");
  Serial.print(currentPosFK[0]);
  Serial.print(", Y=");
  Serial.print(currentPosFK[1]);
  Serial.print(", Z=");
  Serial.println(currentPosFK[2]);
  
  // Calculate IK for target position
  Serial.println("Calculating IK solution...");
  bool success = calculateInverseKinematics(targetPos, targetRot, currentJointAngles);
  
  if (success) {
    Serial.println("IK solution found, starting movement");
    
    // Copy resultJointAngles to pass to the coordinated move
    double targetJointAngles[6];
    for (int i = 0; i < 6; i++) {
      targetJointAngles[i] = resultJointAngles[i]; 
      Serial.print("Target joint ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.println(targetJointAngles[i] * 180.0 / M_PI);
    }
    
    // Start coordinated movement to target joint angles
    startCoordinatedMove(targetJointAngles, duration);
    Serial.println("IK movement started");
    return true;
  } else {
    Serial.println("IK solution not found for target position");
    return false;
  }
}
