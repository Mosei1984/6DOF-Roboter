#ifndef DRIVE_MODE_SELECT_STATE_H
#define DRIVE_MODE_SELECT_STATE_H

#include "MenuState.h"
#include "ConfigHandler.h"
#include <Adafruit_SSD1306.h>

// External object declarations
extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

class DriveModeSelectState : public MenuState {
private:
    int selection = 0;
    
    // Define drive mode names
    const char* driveModes[7] = {
        "Forward",
        "Inverse",
        "IK + ADXL",
        "Joint",
        "Referenz",
        "Teach",
        "Play"
    };

public:
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;

private:
    void render();
};

#endif // DRIVE_MODE_SELECT_STATE_H
