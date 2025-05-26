#ifndef JOYSTICK_MENU_STATE_H
#define JOYSTICK_MENU_STATE_H

#include "MenuState.h"

class JoystickMenuState : public MenuState {
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
    void startCalibrationL();
    void startCalibrationR();
    void saveCalibration();
    void resetCalibration();
    void showLiveDebug();
};

#endif
