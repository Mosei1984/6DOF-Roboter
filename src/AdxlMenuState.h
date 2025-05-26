#ifndef ADXL_MENU_STATE_H
#define ADXL_MENU_STATE_H

#include "MenuState.h"
#include "AdxlTracker.h"
#include "VL53L0XSensor.h"
#include "TrajectoryPlanner.h"
#include "ConfigHandler.h"
#include <Adafruit_SSD1306.h>

class AdxlMenuState : public MenuState {
private:
    Adafruit_SSD1306& display;
    AdxlTracker* adxl;
    VL53L0XSensor* lidarSensor;
    TrajectoryPlanner* trajectoryPlanner;
    ConfigHandler* configHandler;
    
    int selection = 0;
    
    // Current offset values
    float currentOffsetX;
    float currentOffsetY;
    float currentOffsetZ;
    
    void render();
    void startCalibration();
    void resetCalibration();
    void toggleTracking();
    void enterLidarMenu();
    
public:
    // Constructor with all required parameters
    AdxlMenuState(Adafruit_SSD1306& display, 
                 VL53L0XSensor* lidarSensor, 
                 TrajectoryPlanner* trajectoryPlanner, 
                 ConfigHandler* configHandler,
                 AdxlTracker* adxl = nullptr)
        : display(display), 
          adxl(adxl), 
          lidarSensor(lidarSensor),
          trajectoryPlanner(trajectoryPlanner), 
          configHandler(configHandler) {}
    
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;
};

#endif // ADXL_MENU_STATE_H
