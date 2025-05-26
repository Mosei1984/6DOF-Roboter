#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "MenuState.h"

class StateManager {
public:
    static void setState(MenuState* newState);
    static void update();
    static void buttonRPressed();
    static void buttonLPressed();
    static void joystickLMoved(int xL, int yL);
    static void joystickRMoved(int xR, int yR);

private:
    static MenuState* currentState;
};

#endif
