#include "MainMenuState.h"
#include "StateManager.h"
#include "DriveModeSelectState.h"
#include "robot_bitmaps.h"

// External objects
extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

void MainMenuState::onEnter() {
    selection = 0;
    render();
}

void MainMenuState::onExit() {
    display.clearDisplay();
    display.display();
}

void MainMenuState::update() {
    // Updates happen on input events
}

void MainMenuState::onButtonLPress() {
    // Back button could go to a welcome screen or do nothing in main menu
    // For now, we'll do nothing as this is the top-level menu
}

void MainMenuState::onButtonRPress() {
    // Handle main menu selection
    switch(selection) {
        case 0: // Drive Mode
            StateManager::setState(new DriveModeSelectState());
            break;
        case 1: // Stepper Settings
            // StateManager::setState(new StepperSettingsState());
            Serial.println("Stepper Settings - Not yet implemented");
            break;
        case 2: // SD Card
            // StateManager::setState(new SDCardState());
            Serial.println("SD Card - Not yet implemented");
            break;
        case 3: // Kinematics
            // StateManager::setState(new KinematicsState());
            Serial.println("Kinematics - Not yet implemented");
            break;
        case 4: // Back/Exit
            // Could go to a welcome screen or keep in main menu
            Serial.println("Exiting menu - Not yet implemented");
            break;
    }
}

void MainMenuState::onJoystickLMove(int xL, int yL) {
    // Navigate through menu items
    if (yL < -200) {
        selection = (selection + 1) % 5; // 5 menu items
        render();
    }
    if (yL > 200) {
        selection = (selection - 1 + 5) % 5;
        render();
    }
}

void MainMenuState::onJoystickRMove(int xR, int yR) {
    // Not used in main menu
}

void MainMenuState::render() {
    display.clearDisplay();
    display.drawBitmap(0, 0, robot_icon4_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.println("Main Menu");
    
    // Main menu options
    const char* options[] = {
        "Drive Mode",
        "Stepper Settings",
        "SD Card",
        "Kinematics",
        "Back"
    };

    for (int i = 0; i < 5; i++) {
        display.setCursor(10, 18 + i * 10);
        if (i == selection) display.print("> ");
        else display.print("  ");
        display.println(options[i]);
    }

    // Show current drive mode at bottom
    display.setCursor(0, 54);
    display.print("Mode: ");
    display.print(configHandler.getDriveModeString());

    display.display();
}