#include "SdCardMenuState.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "robot_bitmaps.h"

// External objects - use the same approach as in other files
extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

void SdCardMenuState::onEnter() {
    selection = 0;
    message = "";
    render();
}

void SdCardMenuState::onExit() {
    display.clearDisplay();
    display.display();
}

void SdCardMenuState::update() {
    // Nothing needed here for this menu
}

void SdCardMenuState::onButtonLPress() {
    // Back button - return to main menu
    StateManager::setState(new MainMenuState());
}

void SdCardMenuState::onButtonRPress() {
    switch(selection) {
        case 0: // Load from SD
            if (configHandler.loadFromSD()) {
                message = "Config loaded";
            } else {
                message = "Load failed!";
            }
            break;
            
        case 1: // Save to SD
            if (configHandler.saveToSD()) {
                message = "Config saved";
            } else {
                message = "Save failed!";
            }
            break;
            
        case 2: // Reset defaults
            configHandler.resetToDefault();
            configHandler.saveToSD();
            message = "Defaults restored";
            break;
            
        case 3: // Back
            StateManager::setState(new MainMenuState());
            return;
    }
    
    render();
}

void SdCardMenuState::onJoystickLMove(int xL, int yL) {
    // Navigate up/down through menu options
    if (yL < -200) {
        selection = (selection + 1) % 4;
        render();
    }
    if (yL > 200) {
        selection = (selection - 1 + 4) % 4;
        render();
    }
}

void SdCardMenuState::onJoystickRMove(int xR, int yR) {
    // Not used in this menu
}

void SdCardMenuState::render() {
    display.clearDisplay();
    
    // Draw header
    display.drawBitmap(0, 0, robot_icon5_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("SD Card Config");
    display.drawLine(0, 9, display.width(), 9, SSD1306_WHITE);
    
    // Draw message if any
    if (message.length() > 0) {
        display.setCursor(0, 12);
        display.println(message);
    }
    
    // Draw menu items
    for (int i = 0; i < 4; i++) {
        if (i == selection) {
            display.fillRect(0, 22 + i*10, display.width(), 10, WHITE);
            display.setTextColor(BLACK);
        } else {
            display.setTextColor(WHITE);
        }
        display.setCursor(10, 24 + i*10);
        display.println(sdItems[i]);
    }
    
    display.display();
}
