#ifndef STEPPER_MENU_STATE_H
#define STEPPER_MENU_STATE_H

#include "MenuState.h"

class StepperMenuState : public MenuState {
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
    void renderHomingProgress(uint8_t jointIndex);
    void startHomingForJoint(uint8_t jointIndex);
};

#endif
