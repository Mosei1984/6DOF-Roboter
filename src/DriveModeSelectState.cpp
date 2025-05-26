#include "DriveModeSelectState.h"
#include "DisplayHandler.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "DriveModeMenuState.h"
#include "robot_bitmaps.h"

// External objects
extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

void DriveModeSelectState::onEnter() {
    // Start with current drive mode as selection
    selection = static_cast<int>(configHandler.currentDriveMode);
    render();
}

void DriveModeSelectState::onExit() {
    display.clearDisplay();
    display.display();
}

void DriveModeSelectState::update() {
    // Nothing needed here, updates happen on input events
}

void DriveModeSelectState::onButtonLPress() {
    StateManager::setState(new MainMenuState());
}

void DriveModeSelectState::onButtonRPress() {
    // Save the selected drive mode
    configHandler.currentDriveMode = static_cast<DriveMode>(selection);
    configHandler.setDriveMode(configHandler.currentDriveMode);
    configHandler.saveToSD();
    Serial.printf("🚦 Active Drive Mode: %s\n", driveModes[selection]);
    
    // Go to drive mode menu for further configuration
    StateManager::setState(new DriveModeMenuState());
}

void DriveModeSelectState::onJoystickLMove(int xL, int yL) {
    // Navigate up/down through drive modes
    if (yL < -200) {
        selection = (selection + 1) % 7;
        render();
    }
    if (yL > 200) {
        selection = (selection - 1 + 7) % 7;
        render();
    }
}

void DriveModeSelectState::onJoystickRMove(int xR, int yR) {
    // Not used in this menu
}

void DriveModeSelectState::render() {
    display.clearDisplay();
    display.drawBitmap(0, 0, robot_icon4_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.print("Select Drive Mode");

    for (int i = 0; i < 7; i++) {
        display.setCursor(10, 18 + i * 10);
        if (i == selection) display.print("> ");
        else display.print("  ");
        display.println(driveModes[i]);
    }

    display.display();
}