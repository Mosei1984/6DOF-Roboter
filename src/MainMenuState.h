#ifndef MAIN_MENU_STATE_H
#define MAIN_MENU_STATE_H

#include "MenuState.h"
#include <Adafruit_SSD1306.h>

#include "TrajectoryPlanner.h"
#include "AdxlTracker.h"
#include "ConfigHandler.h"

// Forward declarations for external globals
extern Adafruit_SSD1306 display;
extern VL53L0XSensor* lidarSensor;
extern TrajectoryPlanner* trajectoryPlanner;
extern ConfigHandler* config;
extern AdxlTracker* adxlTracker;

class MainMenuState : public MenuState {
private:
    int selection = 0;
    
    void render();
    
public:
    // Default constructor that uses global objects
    MainMenuState() {}
    
    // Full constructor with all parameters
    MainMenuState(Adafruit_SSD1306& disp, 
                  VL53L0XSensor* lidar,
                  TrajectoryPlanner* trajPlanner, 
                  ConfigHandler* config,
                  AdxlTracker* adxl = nullptr) {}
    
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;
};

#endif
