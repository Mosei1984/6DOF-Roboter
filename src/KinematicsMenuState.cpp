#include "KinematicsMenuState.h"
#include "DisplayHandler.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "DriveModeSelectState.h"
#include "robot_bitmaps.h"

extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

void KinematicsMenuState::onEnter() {
    selection = 0;
    render();
}

void KinematicsMenuState::onExit() {
    display.clearDisplay();
    display.display();
}

void KinematicsMenuState::update() {
    // Nothing needed here, updates happen on input events
}

void KinematicsMenuState::onButtonLPress() {
    // Go back to drive mode selection
    StateManager::setState(new DriveModeSelectState());
}

void KinematicsMenuState::onButtonRPress() {
    // Handle selection based on menu item
    switch(selection) {
        case 0: toggleInverseMode(); break;
        case 1: toggleJointMode(); break;
        case 2: toggleGripperControl(); break;
        case 3: toggleToolControl(); break;
        case 4: toggleWristControl(); break;
        case 5: toggleDebug(); break;
    }
    render();
}

void KinematicsMenuState::onJoystickLMove(int xL, int yL) {
    // Navigate up/down through options
    if (yL < -200) {
        selection = (selection + 1) % 6;
        render();
    }
    if (yL > 200) {
        selection = (selection - 1 + 6) % 6;
        render();
    }
}

void KinematicsMenuState::onJoystickRMove(int xR, int yR) {
    // Not used in this menu
}

void KinematicsMenuState::render() {
    display.clearDisplay();
    display.drawBitmap(0, 0, robot_icon4_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.print("Kinematik: ");
    display.println(configHandler.getDriveModeString());

    // Menu options
    const char* options[] = {
        "Inverse Mode",
        "Joint Mode",
        "Gripper Control",
        "Tool Control",
        "Wrist Control",
        "Debug Mode"
    };

    // Display status for each option
    for (int i = 0; i < 6; i++) {
        display.setCursor(10, 18 + i * 10);
        if (i == selection) display.print("> ");
        else display.print("  ");
        display.print(options[i]);
        display.setCursor(110, 18 + i * 10);
        
        // Show current status
        bool isEnabled = false;
        switch(i) {
            case 0: isEnabled = configHandler.config.inverseMode; break;
            case 1: isEnabled = configHandler.config.jointMode; break;
            case 2: isEnabled = configHandler.config.useGripperMode; break;
            case 3: isEnabled = configHandler.config.toolMotorEnabled; break;
            case 4: isEnabled = configHandler.config.wristControlEnabled; break;
            case 5: isEnabled = configHandler.config.useDebugmode; break;
        }
        display.print(isEnabled ? "ON" : "OFF");
    }

    display.display();
}

void KinematicsMenuState::setDriveMode(DriveMode mode) {
    configHandler.setDriveMode(mode);
}

void KinematicsMenuState::toggleInverseMode() {
    configHandler.config.inverseMode = !configHandler.config.inverseMode;
    configHandler.saveToSD();
    Serial.printf("Inverse Mode: %s\n", configHandler.config.inverseMode ? "ON" : "OFF");
}

void KinematicsMenuState::toggleJointMode() {
    configHandler.config.jointMode = !configHandler.config.jointMode;
    configHandler.saveToSD();
    Serial.printf("Joint Mode: %s\n", configHandler.config.jointMode ? "ON" : "OFF");
}

void KinematicsMenuState::toggleGripperControl() {
    configHandler.config.useGripperMode = !configHandler.config.useGripperMode;
    configHandler.saveToSD();
    Serial.printf("Gripper Control: %s\n", configHandler.config.useGripperMode ? "ON" : "OFF");
}

void KinematicsMenuState::toggleToolControl() {
    configHandler.config.toolMotorEnabled = !configHandler.config.toolMotorEnabled;
    configHandler.saveToSD();
    Serial.printf("Tool Control: %s\n", configHandler.config.toolMotorEnabled ? "ON" : "OFF");
}

void KinematicsMenuState::toggleWristControl() {
    configHandler.config.wristControlEnabled = !configHandler.config.wristControlEnabled;
    configHandler.saveToSD();
    Serial.printf("Wrist Control: %s\n", configHandler.config.wristControlEnabled ? "ON" : "OFF");
}

void KinematicsMenuState::toggleDebug() {
    configHandler.config.useDebugmode = !configHandler.config.useDebugmode;
    if (configHandler.config.useDebugmode) {
        configHandler.useDebugmode();
    }
    configHandler.saveToSD();
    Serial.printf("Debug Mode: %s\n", configHandler.config.useDebugmode ? "ON" : "OFF");
}
