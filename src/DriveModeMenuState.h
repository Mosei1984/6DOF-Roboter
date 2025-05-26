#ifndef DRIVEMODE_MENU_STATE_H
#define DRIVEMODE_MENU_STATE_H

#include "MenuState.h"
#include "ConfigHandler.h"

class DriveModeMenuState : public MenuState {
public:
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;
    
    // Drive mode functions
    void setDriveMode(DriveMode mode);

private:
    int selection = 0;
    void render();
    void toggleInverseMode();
    void toggleJointMode();
    void toggleGripperControl();
    void toggleToolControl();
    void toggleWristControl();
    void toggleDebug();
};

#endif
