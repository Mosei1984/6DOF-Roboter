#include "StateManager.h"

MenuState* StateManager::currentState = nullptr;

void StateManager::setState(MenuState* newState) {
    if (currentState) currentState->onExit();
    currentState = newState;
    if (currentState) currentState->onEnter();
}

void StateManager::update() {
    if (currentState) currentState->update();
}

void StateManager::buttonRPressed() {
    if (currentState) currentState->onButtonRPress();
}

void StateManager::joystickLMoved(int xL    , int yL) {
    if (currentState) currentState->onJoystickLMove(xL, yL);
}
void StateManager::buttonLPressed() {
    if (currentState) currentState->onButtonLPress();
}

void StateManager::joystickRMoved(int xR, int yR) {
    if (currentState) currentState->onJoystickRMove(xR, yR);
}