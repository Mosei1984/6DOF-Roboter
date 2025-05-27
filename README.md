<think>I need to create a comprehensive README.md file for the RoboterArm project. I'll include details about:

1. Overview of the project
2. Key components and their relationships
3. Main classes and their purposes
4. Important variables and functions
5. How to use the system
6. Technical details

From the code I've seen, the project appears to be a 6-DoF robot arm control system with:
- Kinematics handling (forward and inverse kinematics)
- Trajectory planning
- Multiple drive modes
- Sensor integration (ADXL accelerometer, VL53L0X distance sensor)
- Configuration management
- Menu-based user interface

I'll organize the README to cover all these aspects in a clear, structured way.</think>
# RoboterArm - 6DOF Robot Control System

## Übersicht / Overview

Dieses Projekt implementiert ein umfassendes Steuerungssystem für einen 6-Achsen-Roboterarm mit fortschrittlicher Kinematik, Trajektorienplanung und Sensorintegration. Das System unterstützt verschiedene Betriebsmodi, darunter direkte und inverse Kinematik, sowie Sensorfeedback für präzise Positionierung.

*This project implements a comprehensive control system for a 6-axis robot arm with advanced kinematics, trajectory planning, and sensor integration. The system supports various operating modes including direct and inverse kinematics, as well as sensor feedback for precise positioning.*

## Hauptkomponenten / Main Components

### Klassen / Classes

#### 1. KinematicsHandler
Verantwortlich für alle kinematischen Berechnungen des Roboters.

```cpp
class KinematicsHandler {
public:
    // Konstruktor und Basisfunktionen
    KinematicsHandler();
    void resetToDefault();
    
    // Sensorkonfiguration
    void setAdxlTracker(AdxlTracker* adxl);
    void setLidarSensor(VL53L0XSensor* lidar);
    
    // Kinematikfunktionen
    void calculateForwardKinematics(float angles[6], float position[6]);
    bool calculateInverseKinematics(float position[6], float angles[6]);
    bool calculateInverseKinematicsDLS(float target[6], float solution[6], bool useZCorrection = false);
    
    // Konfiguration und Parameter
    bool loadDHFromSD(const char* filename = "/dh_params.json");
    bool loadWorkspaceFromSD(const char* filename = "/workspace.json");
    bool saveWorkspaceToSD(const char* filename = "/workspace.json");
    
    // Einstellungen
    bool filterEnabled() const;
    void setFilterEnabled(bool enabled);
    bool debugEnabled() const;
    void setDebugEnabled(bool enabled);
    
private:
    // Denavit-Hartenberg Parameter
    float _dh_table[6][4];       // a, d, alpha, theta_offset
    float _tool_rpy[3];          // Werkzeugorientierung (Roll, Pitch, Yaw)
    float _workspace_limits[6][2]; // Arbeitsraumlimits (X,Y,Z,R,P,Y min/max)
    float _joint_limits[6][2];   // Gelenkwinkelgrenzen (min/max)
    
    // Einstellungen
    bool _filterEnabled;         // Filterung aktiv?
    bool _debugEnabled;          // Debug-Ausgaben aktiv?
    
    // Inverse-Kinematik-Parameter
    float _ikDamping;            // Dämpfungsfaktor für DLS
    int _ikMaxIterations;        // Maximale Iterationen
    float _ikErrorThreshold;     // Fehlertoleranz
    float _ikSingularityThreshold; // Singularitätsschwelle
    
    // Erweiterte IK-Optionen
    bool _ikAdaptiveDamping;     // Adaptive Dämpfung
    bool _ikNullSpaceOptimization; // Nullraum-Optimierung
    bool _ikTaskPrioritization;  // Aufgabenprioritisierung
    bool _ikUseKalmanFilter;     // Kalman-Filter verwenden
    bool _ikCollisionAvoidance;  // Kollisionsvermeidung
    
    // Sensorreferenzen
    AdxlTracker* _adxl;          // Beschleunigungssensor
    VL53L0XSensor* _lidar;       // Abstandssensor
    
    // Kalman-Filter-Status
    Eigen::VectorXf _kalmanState;
    Eigen::MatrixXf _kalmanP;
    Eigen::MatrixXf _kalmanQ;
    Eigen::MatrixXf _kalmanR;
    
    // Hilfsmethoden für IK
    void calculateJacobian(float angles[6], Eigen::MatrixXf& J);
    void calculateSVD(const Eigen::MatrixXf& J, Eigen::MatrixXf& U, Eigen::VectorXf& S, Eigen::MatrixXf& V);
    float calculateAdaptiveDamping(const Eigen::VectorXf& S);
    Eigen::MatrixXf calculateDampedPseudoInverse(const Eigen::MatrixXf& J, float lambda);
    Eigen::MatrixXf calculateNullSpaceProjection(const Eigen::MatrixXf& J, const Eigen::MatrixXf& J_pinv);
    Eigen::VectorXf calculateNullSpaceGradient(const float angles[6]);
    void updateKalmanFilter(float measurements[6], float deltaTime);
    void initializeKalmanFilter();
    
    // Hilfsmethoden für Vorwärtskinematik
    Eigen::Matrix4f DHTransform(float a, float d, float alpha, float theta);
    void rotationMatrixToEulerAngles(const Eigen::Matrix3f& R, float& roll, float& pitch, float& yaw);
};
```

#### 2. TrajectoryPlanner
Plant und steuert Bewegungstrajektorien des Roboters.

```cpp
class TrajectoryPlanner {
public:
    TrajectoryPlanner(KinematicsHandler* kinematics, ConfigHandler* config);
    
    // Bewegungssteuerung
    void startMove(float x, float y, float z, float roll, float pitch, float yaw, unsigned long duration);
    void moveRelative(float dx, float dy, float dz, float droll, float dpitch, float dyaw, unsigned long duration);
    void moveToPoint(uint8_t pointIndex, unsigned long duration);
    void pauseTrajectory();
    void resumeTrajectory();
    void emergencyStop();
    
    // Wegpunkt-Management
    bool saveWaypoint(uint8_t index);
    bool loadWaypoints(const char* filename);
    bool saveWaypoints(const char* filename);
    
    // Update und Status
    void update();
    bool isTrajectoryActive();
    String getDebug() const;
    void getCurrentPose(float& x, float& y, float& z, float& roll, float& pitch, float& yaw);
    
    // Einstellungen
    void setTrajectoryType(TrajectoryType type);
    
private:
    // Referenzen
    KinematicsHandler* kinematicsHandler;
    ConfigHandler* configHandler;
    
    // Status
    bool isMoving;
    unsigned long moveStartTime;
    unsigned long moveDuration;
    unsigned long lastUpdateTime;
    unsigned long currentStep;
    unsigned long totalSteps;
    
    // Positions- und Orientierungsdaten
    float currentPosition[3];    // X, Y, Z
    float targetPosition[3];     // X, Y, Z
    float startPosition[3];      // X, Y, Z
    float currentOrientation[3]; // Roll, Pitch, Yaw
    float targetOrientation[3];  // Roll, Pitch, Yaw
    float startOrientation[3];   // Roll, Pitch, Yaw
    
    // Wegpunkte und Einstellungen
    Waypoint waypoints[MAX_WAYPOINTS];
    TrajectoryType trajectoryType;
    
    // Hilfsfunktionen
    void updateRobotPosition();
};
```

#### 3. ConfigHandler
Verwaltet die Konfiguration und Einstellungen des Roboters.

```cpp
class ConfigHandler {
public:
    // Hauptkonfigurationsstruktur
    SystemConfig config;
    DriveMode currentDriveMode;
    uint8_t jointSelected;
    
    // Konfigurationsfunktionen
    void loadFromEEPROM();
    void saveToEEPROM();
    void resetToDefault();
    bool loadFromSD(const char *path = "/config.json");
    bool saveToSD(const char *path = "/config.json");
    void printConfig();
    
    // Debugfunktionen
    void useDebugmode();
    void useVerboseMode();
    
    // Modus-Umschaltung
    bool setDriveMode(DriveMode newMode);
    const char* getDriveModeString();
};
```

#### 4. AdxlTracker
Verarbeitet Daten vom ADXL345-Beschleunigungssensor.

```cpp
class AdxlTracker {
public:
    AdxlTracker(uint8_t sensor_id = 46);
    
    // Initialisierung und Kontrolle
    bool begin();
    void startTracking();
    void stopTracking();
    void update();
    
    // Kalibration
    void startCalibration();
    void setManualOffsets(float x, float y, float z);
    void saveConfig();
    bool loadConfig();
    
    // Status und Daten
    bool isCurrentlyTracking();
    bool isCurrentlyCalibrating();
    bool isDataValid();
    String getDebug();
    
    // Sensordaten abrufen
    float getRoll();
    float getPitch();
    float getYaw();
    float getX();
    float getY();
    float getZ();
    
    // Einstellungen
    void setUpdateFrequency(int freq);
    void setFilterAlpha(float alpha);
    
    // Offsetwerte
    float getOffsetX();
    float getOffsetY();
    float getOffsetZ();
};
```

#### 5. VL53L0XSensor
Verarbeitet Daten vom VL53L0X-Abstandssensor.

```cpp
class VL53L0XSensor {
public:
    VL53L0XSensor(int xshutPin = -1);
    
    // Initialisierung und Update
    bool begin();
    void update();
    
    // Datenzugriff
    uint16_t getDistance() const;
    uint16_t getRawDistance() const;
    bool isValid() const;
    bool isInitialized() const;
    
    // Kalibration
    bool loadCalibrationFromSD(const char* filename = "/lidar_cal.json");
    bool saveCalibrationToSD(const char* filename = "/lidar_cal.json");
    void startCalibration(uint16_t knownDistance);
    void cancelCalibration();
    bool isCalibrating() const;
    bool calibrationComplete() const;
    void resetCalibration();
    
    // Steuerung und Debug
    void shutdown();
    void powerUp();
    String getDebug() const;
    
    // Einstellungen
    void setLongRangeMode(bool enable);
    bool getLongRangeMode() const;
    void setTimingBudget(uint16_t budget);
};
```

#### 6. Menu-Zustände
Verwalten die Benutzeroberfläche und Interaktion.

```cpp
// Basisklasse
class MenuState {
public:
    virtual void update() = 0;
    virtual void render() = 0;
    virtual void onButtonLPress() = 0;
    virtual void onButtonRPress() = 0;
    virtual void onJoystickLMove(int xL, int yL) = 0;
    virtual void onJoystickRMove(int xR, int yR) = 0;
};

// Verschiedene Menüzustände
class MainMenuState : public MenuState { ... };
class DriveModeMenuState : public MenuState { ... };
class DriveModeSelectState : public MenuState { ... };
class KinematicsMenuState : public MenuState { ... };
class SdCardMenuState : public MenuState { ... };
class AdxlMenuState : public MenuState { ... };
class JoystickMenuState : public MenuState { ... };
class StepperMenuState : public MenuState { ... };
```

### Enumerationen / Enumerations

```cpp
// Fahrmodi
enum DriveMode {
    MODE_FORWARD,     // Vorwärtskinematik
    MODE_INVERSE,     // Inverse Kinematik
    MODE_IK_ADXL,     // Inverse Kinematik mit ADXL-Sensorunterstützung
    MODE_JOINT,       // Direkter Gelenkmodus
    MODE_REFERENCE,   // Referenzmodus
    MODE_TEACH,       // Teachmodus
    MODE_PLAY         // Abspielmodus
};

// Trajektorientypen
enum TrajectoryType {
    TRAJECTORY_LINEAR,     // Lineare Interpolation
    TRAJECTORY_SMOOTH,     // Sanfte Beschleunigung und Verzögerung
    TRAJECTORY_ACCELERATE, // Quadratische Beschleunigung
    TRAJECTORY_DECELERATE  // Quadratische Verzögerung
};
```

### Strukturen / Structures

```cpp
// Hauptkonfigurationsstruktur
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
    bool useGripperMode;        // Greifer verwenden?
    bool enableDebugPin1;       // Debug-Pin 1 aktivieren
    bool enableDebugPin2;       // Debug-Pin 2 aktivieren
    bool enableDebugPin3;       // Debug-Pin 3 aktivieren
    bool useDebugmode;          // Debug-Modus aktivieren
    bool useVerboseMode;        // Ausführliche Ausgaben aktivieren
    uint8_t debugPin1;          // Debug-Pin 1 Nummer
    uint8_t debugPin2;          // Debug-Pin 2 Nummer
    uint8_t debugPin3;          // Debug-Pin 3 Nummer
    uint8_t Gripper_Pin;        // Greifer-Pin Nummer
    bool inverseMode;           // Inverse Kinematik aktiviert?
    bool jointMode;             // Direkter Gelenkmodus?
    bool toolMotorEnabled;      // Werkzeugmotor erlaubt?
    bool wristControlEnabled;   // Handgelenksteuerung erlaubt?
    DriveMode currentDriveMode; // Aktueller Fahrmodus
};

// Wegpunkt-Struktur für Trajektorienplanung
struct Waypoint {
    float position[3];      // X, Y, Z
    float orientation[3];   // Roll, Pitch, Yaw
    bool valid;             // Gültiger Wegpunkt?
};
```

## Wichtige Funktionen und Parameter / Important Functions and Parameters

### Kinematik / Kinematics

- **Vorwärtskinematik (Forward Kinematics)**:
  ```cpp
  void calculateForwardKinematics(float angles[6], float position[6]);
  ```
  Berechnet die Endeffektorposition und -orientierung aus den Gelenkwinkeln unter Verwendung der Denavit-Hartenberg-Parameter.

- **Inverse Kinematik (Inverse Kinematics)**:
  ```cpp
  bool calculateInverseKinematics(float position[6], float angles[6]);
  bool calculateInverseKinematicsDLS(float target[6], float solution[6], bool useZCorrection);
  ```
  Berechnet die Gelenkwinkel für eine gewünschte Endeffektorposition und -orientierung mit der Damped Least Squares (DLS) Methode.

- **DH-Parameter**:
  ```cpp
  float _dh_table[6][4];  // a, d, alpha, theta_offset
  ```
  Denavit-Hartenberg-Parameter, die die Kinematik des Roboters definieren.

### Trajektorienplanung / Trajectory Planning

- **Bewegungssteuerung**:
  ```cpp
  void startMove(float x, float y, float z, float roll, float pitch, float yaw, unsigned long duration);
  void moveRelative(float dx, float dy, float dz, float droll, float dpitch, float dyaw, unsigned long duration);
  ```
  Startet Bewegungen zu absoluten oder relativen Positionen mit definierter Dauer.

- **Wegpunkte**:
  ```cpp
  bool saveWaypoint(uint8_t index);
  void moveToPoint(uint8_t pointIndex, unsigned long duration);
  ```
  Speichert aktuelle Positionen als Wegpunkte oder bewegt den Roboter zu gespeicherten Wegpunkten.

- **Trajektorientypen**:
  ```cpp
  void setTrajectoryType(TrajectoryType type);
  ```
  Legt die Art der Interpolation zwischen Start- und Zielpunkt fest (linear, smooth, beschleunigend, verzögernd).

### Sensoren / Sensors

- **ADXL-Beschleunigungssensor**:
  ```cpp
  bool isDataValid();
  float getRoll(), getPitch(), getYaw();
  ```
  Stellt Lage- und Beschleunigungsdaten zur Verfügung für Orientierungsfeedback.

- **VL53L0X-Abstandssensor**:
  ```cpp
  uint16_t getDistance();
  bool isValid();
  ```
  Liefert präzise Abstandsmessungen für Höhenkorrekturen.

### Konfiguration / Configuration

- **Konfigurationsmanagement**:
  ```cpp
  void resetToDefault();
  bool loadFromSD(const char *path);
  bool saveToSD(const char *path);
  ```
  Verwaltet Roboterkonfigurationen, die im EEPROM oder auf SD-Karte gespeichert werden können.

- **Fahrmodi**:
  ```cpp
  bool setDriveMode(DriveMode newMode);
  const char* getDriveModeString();
  ```
  Schaltet zwischen verschiedenen Betriebsmodi des Roboters um.

## Betriebsmodi / Operating Modes

1. **Vorwärtskinematik (MODE_FORWARD)**:
   - Direkte Steuerung der Gelenkwinkel
   - Berechnung der resultierenden Endeffektorposition

2. **Inverse Kinematik (MODE_INVERSE)**:
   - Steuerung der Endeffektorposition und -orientierung
   - Automatische Berechnung der notwendigen Gelenkwinkel

3. **IK mit ADXL-Unterstützung (MODE_IK_ADXL)**:
   - Inverse Kinematik mit Sensorkorrektur
   - Verwendet ADXL-Daten für Orientierungskorrektur
   - Verwendet LIDAR-Daten für Höhenkorrektur

4. **Gelenkmodus (MODE_JOINT)**:
   - Direkte Steuerung einzelner Gelenke
   - Keine kinematische Umrechnung

5. **Referenzmodus (MODE_REFERENCE)**:
   - Zurückfahren in eine definierte Referenzposition

6. **Teach-Modus (MODE_TEACH)**:
   - Aufzeichnung von Bewegungen als Wegpunkte
   - Speichern von Bewegungsabläufen

7. **Abspielmodus (MODE_PLAY)**:
   - Automatisches Abspielen gespeicherter Bewegungen
   - Durchlaufen von Wegpunkten

## Fortschrittliche Funktionen / Advanced Features

1. **Damped Least Squares IK**:
   - Robuste Lösung für inverse Kinematik
   - Vermeidung von Singularitäten
   - Nullraum-Optimierung für verbesserte Bewegungen

2. **Adaptive Dämpfung**:
   - Dynamische Anpassung der Dämpfung basierend auf Nähe zu Singularitäten
   - Verbesserte Stabilität in schwierigen Posen

3. **Kalman-Filter**:
   - Sensorfusion für verbesserte Positionsverfolgung
   - Filterung von Sensorrauschen

4. **Trajektorieninterpolation**:
   - Verschiedene Interpolationstypen für optimierte Bewegungen
   - Lineare, kubische und beschleunigende/verzögernde Pfade

5. **Sensorbasierte Korrekturen**:
   - Echtzeit-Anpassung der Bewegung basierend auf Sensordaten
   - Kompensation von externen Störungen

## Wichtige Konstanten / Important Constants

```cpp
// Trajektorienplanung
#define MAX_WAYPOINTS 10        // Maximale Anzahl an Wegpunkten
#define UPDATE_INTERVAL 20      // Aktualisierungsintervall in ms

// Roboterkinematik
const float DEFAULT_DH_TABLE[6][4] = {...};  // Standard DH-Parameter
const float DEFAULT_WORKSPACE_LIMITS[6][2] = {...};  // Arbeitsraumgrenzen
const float DEFAULT_JOINT_LIMITS[6][2] = {...};  // Gelenkwinkelgrenzen
```

## Benutzeroberfläche / User Interface

Das System bietet eine menübasierte Benutzeroberfläche, die über einen OLED-Display und Joysticks/Buttons gesteuert wird:

1. **Hauptmenü**: Zugriff auf alle Untermenüs und Funktionen
2. **Fahrmodus-Menü**: Auswahl und Konfiguration des Betriebsmodus
3. **Kinematik-Menü**: Einstellungen für kinematische Parameter
4. **Sensor-Menüs**: Konfiguration und Kalibrierung der Sensoren
5. **SD-Karten-Menü**: Speichern und Laden von Konfigurationen

## Beispielverwendung / Example Usage

```cpp
// Initialisierung der Hauptkomponenten
ConfigHandler configHandler;
KinematicsHandler kinematicsHandler;
AdxlTracker adxlTracker;
VL53L0XSensor lidarSensor;
TrajectoryPlanner trajectoryPlanner(&kinematicsHandler, &configHandler);

void setup() {
    // Sensoren initialisieren
    adxlTracker.begin();
    lidarSensor.begin();
    
    // Sensoren mit der Kinematik verbinden
    kinematicsHandler.setAdxlTracker(&adxlTracker);
    kinematicsHandler.setLidarSensor(&lidarSensor);
    
    // Konfiguration laden
    configHandler.loadFromEEPROM();
    kinematicsHandler.loadDHFromSD();
    
    // Fahrmodus einstellen
    configHandler.setDriveMode(MODE_INVERSE);
}

void loop() {
    // Sensoren aktualisieren
    adxlTracker.update();
    lidarSensor.update();
    
    // Trajektorien aktualisieren
    trajectoryPlanner.update();
    
    // Bewegung starten (Beispiel)
    if (startButtonPressed) {
        trajectoryPlanner.startMove(100, 200, 150, 0, 0, 0, 2000);
    }
}
```

## Technische Details / Technical Details

- **Kinematikberechnung**: Verwendet die Eigen-Bibliothek für effiziente Matrixoperationen
- **Speicherung**: Konfigurationen werden im EEPROM und auf SD-Karte gespeichert
- **Kommunikation**: Serielles Debugging und Statusberichte
- **Display**: OLED-Display für Menüdarstellung und Statusinformationen
- **Eingabegeräte**: Zwei Joysticks und zwei Buttons für Navigation und Steuerung

---

*Dieses Dokument bietet einen umfassenden Überblick über das 6DOF RoboterArm-Steuerungssystem. Für detailliertere Informationen zu bestimmten Funktionen, bitte den Quellcode konsultieren.*

*This document provides a comprehensive overview of the 6DOF RoboterArm control system. For more detailed information on specific functions, please consult the source code.*
