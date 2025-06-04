#include "ForwardKinematics.h"


// External variables from main.cpp
extern AccelStepper motors[6];
extern Servo gripper;
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern int16_t joyLXCenter, joyLYCenter, joyRZCenter, joyRYawCenter;
extern int joystick_lx, joystick_ly, joystick_rz, joystick_ryaw;
extern unsigned long lastJoystickCheck;
extern bool buttonConfirmPressed;
extern float stepsToDegree(int jointIndex, long steps);

// Global variables for FK mode
static double currentPos[3] = {0.0, 0.0, 0.0};
static double currentRot[3] = {0.0, 0.0, 0.0};

// Original FK calculations (unchanged)
void calculateForwardKinematics(
    const double currentJointAngles[6],
    double endPosition[3],
    double endRotation[3]
) {
    // 1. Initialisiere T als 4×4-Identitätsmatrix
    std::array<std::array<double,4>,4> T = {{
        {{1.0, 0.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0, 0.0}},
        {{0.0, 0.0, 1.0, 0.0}},
        {{0.0, 0.0, 0.0, 1.0}}
    }};

    // 2. Für jedes Gelenk i: Erzeuge DH-Matrix und multipliziere
    for (int i = 0; i < 6; ++i) {
        DHParams p;
        p.a     = robotDHParams[i].a;
        p.alpha = robotDHParams[i].alpha;
        p.d     = robotDHParams[i].d;
        p.theta = currentJointAngles[i]; // aktueller Winkel in Radiant

        auto Ti = dhToMatrix(p);
        T = matMul4x4(T, Ti);
    }

    // 3. Extrahiere Position (X, Y, Z)
    endPosition[0] = T[0][3];
    endPosition[1] = T[1][3];
    endPosition[2] = T[2][3];

    // 4. Extrahiere Euler-Winkel (Roll, Pitch, Yaw)
    double roll, pitch, yaw;
    rotationMatrixToEulerZYX(T, roll, pitch, yaw);

    endRotation[0] = roll;
    endRotation[1] = pitch;
    endRotation[2] = yaw;
}

// Placeholder for the display function
void updateForwardKinematicsDisplay(
    const double endPosition[3],
    const double endRotation[3]
) {
    // Convert Radiant → Grad:
    double roll_deg  = endRotation[0] * 180.0 / M_PI;
    double pitch_deg = endRotation[1] * 180.0 / M_PI;
    double yaw_deg   = endRotation[2] * 180.0 / M_PI;

    // Use these variables to avoid warnings 
    (void)roll_deg;
    (void)pitch_deg;
    (void)yaw_deg;
}

// Forward Kinematics Mode implementation
void initForwardKinematicsMode(U8G2* display) {
    // Initialize display pointer
  
    // Reset variables
    for (int i = 0; i < 3; i++) {
        currentPos[i] = 0.0;
        currentRot[i] = 0.0;
    }
  
    // Initial FK calculation
    double jointAnglesRad[6];
    for (int i = 0; i < 6; i++) {
        float degrees = stepsToDegree(i, motors[i].currentPosition());
        jointAnglesRad[i] = degrees * M_PI / 180.0;
        currentJointAngles[i] = jointAnglesRad[i]; // Update global joint angles
    }
  
    calculateForwardKinematics(jointAnglesRad, currentPos, currentRot);
  
    // Also update the target position to match current position
    for (int i = 0; i < 3; i++) {
        currentTargetPos[i] = currentPos[i];
        currentTargetRot[i] = currentRot[i];
    }
}

void handleForwardKinematicsMode() {
    // Calculate joint angles in radians from motor positions
    double jointAnglesRad[6];
    for (int i = 0; i < 6; i++) {
        float degrees = stepsToDegree(i, motors[i].currentPosition());
        jointAnglesRad[i] = degrees * M_PI / 180.0;
        currentJointAngles[i] = jointAnglesRad[i]; // Update global joint angles
    }
  
    // Calculate forward kinematics
    calculateForwardKinematics(jointAnglesRad, currentPos, currentRot);
  
    // Get accelerometer data for level correction
    float accelX, accelY, accelZ;
    bool accelValid = getFilteredAcceleration(&accelX, &accelY, &accelZ);
  
    // Get distance sensor data
    float distance = getFilteredDistance();
  
    // Use joystick to control selected joint
    static int activeMotor = 0;
  
    // Joystick for joint selection (RIGHT joystick)
    if (abs(joystick_ryaw - joyRYawCenter) > 200) {
        if (millis() - lastJoystickCheck > 300) { // Debounce
            if (joystick_ryaw > joyRYawCenter + 200) {
                activeMotor = (activeMotor + 1) % 6;
            } else if (joystick_ryaw < joyRYawCenter - 200) {
                activeMotor = (activeMotor + 5) % 6; // +5 = -1 mod 6
            }
            lastJoystickCheck = millis();
        }
    }
  
    // Joystick for joint movement (LEFT joystick Y-axis)
    float speedY = 0;
    if (abs(joystick_ly - joyLYCenter) > 100) {
        speedY = (joystick_ly - joyLYCenter) * 2.0; // Scale for appropriate speed
    }
  
    // Apply speed to active motor
    motors[activeMotor].setSpeed(speedY);
  
    // Update gripper from potentiometer (centralized control)
    updateGripperFromPotentiometer();
  
    // Apply automatic level correction if accelerometer data is valid
    if (accelValid && buttonConfirmPressed) {
        buttonConfirmPressed = false;
    
        // Calculate pitch and roll from accelerometer
        // Simplified calculation - you might want to use proper filtering
        float roll = atan2(accelY, accelZ) * 180.0 / M_PI;
        float pitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / M_PI;
    
        Serial.print("Measured Roll: "); Serial.print(roll);
        Serial.print(", Pitch: "); Serial.println(pitch);
    
        // Update orientation to maintain level tool
        currentTargetRot[0] = -roll * M_PI / 180.0; // Negate to counteract
        currentTargetRot[1] = -pitch * M_PI / 180.0; // Negate to counteract
    
        Serial.println("Tool level corrected with accelerometer data!");
    
        // Optional: Apply IK to maintain level tool
        double currentAngles[6];
        for (int i = 0; i < 6; i++) {
            float degrees = stepsToDegree(i, motors[i].currentPosition());
            currentAngles[i] = degrees * M_PI / 180.0;
        }
    
        bool ikSuccess = calculateInverseKinematics(
            currentTargetPos,
            currentTargetRot,
            currentAngles
        );
    
        if (ikSuccess) {
            Serial.println("Level correction IK successful!");
            executeInverseKinematicsMotion(currentJointAngles);
        } else {
            Serial.println("Level correction IK failed!");
        }
    }
  
    // Update display with current FK information
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
  
    // Title
    u8g2.drawFrame(0, 0, 128, 12);
    u8g2.drawStr(4, 10, "Forward Kinematics");
  
    // Position info
    char buffer[30];
    sprintf(buffer, "X:%.2f Y:%.2f", currentPos[0], currentPos[1]);
    u8g2.drawStr(0, 24, buffer);
  
    sprintf(buffer, "Z:%.2f", currentPos[2]);
    u8g2.drawStr(0, 34, buffer);
  
    // Orientation info (in degrees)
    sprintf(buffer, "R:%.1f P:%.1f Y:%.1f", 
            currentRot[0] * 180.0/M_PI, 
            currentRot[1] * 180.0/M_PI,
            currentRot[2] * 180.0/M_PI);
    u8g2.drawStr(0, 44, buffer);
  
    // Active joint and sensor data
    sprintf(buffer, "J%d:%.1f D:%.2f", 
            activeMotor + 1, 
            stepsToDegree(activeMotor, motors[activeMotor].currentPosition()),
            distance);
    u8g2.drawStr(0, 54, buffer);
  
    u8g2.sendBuffer();
}
