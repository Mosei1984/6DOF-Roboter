#ifndef LIDAR_CALIBRATION_STATE_H
#define LIDAR_CALIBRATION_STATE_H

#include "MenuState.h"
#include "VL53L0XSensor.h"
#include "TaskTiming.h"
#include "ConfigHandler.h"
#include "TrajectoryPlanner.h"
#include <Adafruit_SSD1306.h>

class LidarCalibrationState : public MenuState {
public:
    // Hauptkonstruktor mit allen Parametern
    LidarCalibrationState(Adafruit_SSD1306& display, 
                         VL53L0XSensor* lidarSensor,
                         TrajectoryPlanner* trajectoryPlanner, 
                         ConfigHandler* configHandler);
    
    // Alternativer Konstruktor für einfachen Testmodus
    LidarCalibrationState(VL53L0XSensor* sensor);
    
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;
    
private:
    // Die Reihenfolge hier definiert die korrekte Initialisierungsreihenfolge im Konstruktor
    Adafruit_SSD1306& display;
    VL53L0XSensor* lidarSensor;
    TrajectoryPlanner* trajectoryPlanner;
    ConfigHandler* configHandler;
    
    int selection;
    String statusMessage;
    int calibrationTarget;
    bool isCalibrating;
    TaskTimer updateTimer;

    void render();
    void startCalibration();
    void cancelCalibration();
    void resetCalibration();
    void adjustCalibrationTarget(int amount);
};

#endif // LIDAR_CALIBRATION_STATE_H
