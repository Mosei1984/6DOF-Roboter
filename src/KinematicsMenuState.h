#ifndef KINEMATICS_MENU_STATE_H
#define KINEMATICS_MENU_STATE_H

#include "MenuState.h"
#include "ConfigHandler.h"

class KinematicsMenuState : public MenuState {
public:
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;

private:
    int selection = 0;
    void render();
    
    // Funktionen zur Änderung der Kinematics-Einstellungen
    void toggleInverseMode();
    void toggleJointMode();
    void toggleGripperControl();
    void toggleToolControl();
    void toggleWristControl();
    void toggleDebug();
    void setDriveMode(DriveMode mode);
};

#endif
