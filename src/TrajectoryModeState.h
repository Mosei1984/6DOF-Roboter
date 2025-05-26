#pragma once

#include "MenuState.h"
#include "TrajectoryPlanner.h"

class TrajectoryModeState : public MenuState {
public:
    TrajectoryModeState(TrajectoryPlanner* planner);
    
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;

private:
    TrajectoryPlanner* planner;
    void render();
};
