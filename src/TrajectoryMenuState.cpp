#include "TrajectoryModeState.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

TrajectoryModeState::TrajectoryModeState(TrajectoryPlanner* planner)
    : planner(planner) {}

void TrajectoryModeState::onEnter() {
    render();
}

void TrajectoryModeState::onExit() {
    display.clearDisplay();
    display.display();
}

void TrajectoryModeState::update() {
    render();
}

void TrajectoryModeState::onButtonLPress() {
    // Use the default constructor which will use global instances
    StateManager::setState(new MainMenuState());
}

void TrajectoryModeState::onButtonRPress() {
    // Kein Effekt – kann z. B. später zum Speichern verwendet werden
}

void TrajectoryModeState::onJoystickLMove(int xL, int yL) {}
void TrajectoryModeState::onJoystickRMove(int xR, int yR) {}

void TrajectoryModeState::render() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Trajectory Debug");

    String dbg = planner->getDebug();
    size_t pos = 0;
    int line = 0;

    while (pos < dbg.length() && line < 6) {
        int next = dbg.indexOf('\n', pos);
        if (next == -1) next = dbg.length();
        display.setCursor(0, 10 + line * 9);
        display.println(dbg.substring(pos, next));
        pos = next + 1;
        line++;
    }

    display.display();
}
