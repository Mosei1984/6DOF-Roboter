#include "AdxlMenuState.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "LidarCalibrationState.h"
#include "robot_bitmaps.h"

extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

void AdxlMenuState::onEnter() {
    // If ADXL sensor is available, get current offsets
    if (adxl && adxl->begin()) {
        // Load saved configuration
        adxl->loadConfig();
        
        // Get current offsets
        currentOffsetX = adxl->getOffsetX();
        currentOffsetY = adxl->getOffsetY();
        currentOffsetZ = adxl->getOffsetZ();
        
        // Display initialization success
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("ADXL345 Ready!");
        display.display();
        delay(1000);
    } else {
        // Display error message if sensor not available
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("ADXL345 not found!");
        display.println("Some features disabled.");
        display.display();
        delay(2000);
        
        // Set default values
        currentOffsetX = 0;
        currentOffsetY = 0;
        currentOffsetZ = 0;
    }
    
    render();
}

void AdxlMenuState::onExit() {
    // Stop tracking if active
    if (adxl && adxl->isCurrentlyTracking()) {
        adxl->stopTracking();
    }
    
    display.clearDisplay();
    display.display();
}

void AdxlMenuState::update() {
    // Update the tracker
    if (adxl) {
        adxl->update();
    }
    
    // Update the LIDAR sensor if available
    if (lidarSensor) {
        lidarSensor->update();
    }
}

void AdxlMenuState::onButtonLPress() {
    // Make sure to pass all the required parameters
    StateManager::setState(new MainMenuState());
}

void AdxlMenuState::onButtonRPress() {
    switch (selection) {
        case 0:
            startCalibration();
            break;
        case 1:
            resetCalibration();
            break;
        case 2:
            toggleTracking();
            break;
        case 3:
            enterLidarMenu();
            break;
    }
}

void AdxlMenuState::onJoystickLMove(int xL, int yL) {
    // Navigate menu items with vertical movement
    if (yL < -200) {
        selection = (selection + 1) % 4;
        render();
    }
    if (yL > 200) {
        selection = (selection - 1 + 4) % 4;
        render();
    }
}

void AdxlMenuState::onJoystickRMove(int xR, int yR) {
    // Not used in this menu
}

void AdxlMenuState::render() {
    display.clearDisplay();
    display.drawBitmap(0, 0, robot_icon3_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("Sensor Settings");
    display.drawLine(0, 10, display.width(), 10, WHITE);
    
    // Menu items
    const char* menuItems[] = {
        "Calibrate ADXL",
        "Reset ADXL",
        "Toggle Tracking",
        "LIDAR Calibration"
    };
    
    for (int i = 0; i < 4; i++) {
        if (i == selection) {
            display.fillRect(0, 12 + i*10, display.width(), 10, WHITE);
            display.setTextColor(BLACK);
        } else {
            display.setTextColor(WHITE);
        }
        display.setCursor(2, 14 + i*10);
        display.println(menuItems[i]);
    }
    
    // Show sensor values at bottom
    display.setTextColor(WHITE);
    
    // Show ADXL values
    display.setCursor(0, 54);
    if (adxl) {
        if (adxl->isCurrentlyTracking()) {
            display.print("R:");
            display.print(adxl->getRoll(), 1);
            display.print(" P:");
            display.print(adxl->getPitch(), 1);
        } else {
            display.print("X:");
            display.print(currentOffsetX, 1);
            display.print(" Y:");
            display.print(currentOffsetY, 1);
            display.print(" Z:");
            display.print(currentOffsetZ, 1);
        }
    } else {
        display.print("ADXL not available");
    }
    
    // Show LIDAR data if available (on second status row)
    if (lidarSensor && lidarSensor->isValid()) {
        display.setCursor(0, 44);
        display.print("LIDAR: ");
        display.print(lidarSensor->getDistance());
        display.print("mm");
    }
    
    display.display();
}

void AdxlMenuState::startCalibration() {
    if (!adxl) {
        // Show error if sensor not available
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.println("ADXL sensor not");
        display.println("available!");
        display.display();
        delay(1500);
        render();
        return;
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println("Calibrating...");
    display.println("Keep sensor still!");
    display.println("Place on level surface.");
    display.display();
    
    adxl->startCalibration();
    
    // Wait for calibration to complete
    while (adxl->isCurrentlyCalibrating()) {
        adxl->update();
        delay(10);
    }
    
    // Update display with new offsets
    currentOffsetX = adxl->getOffsetX();
    currentOffsetY = adxl->getOffsetY();
    currentOffsetZ = adxl->getOffsetZ();
    
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("Calibration Done!");
    display.println("X: " + String(currentOffsetX, 1));
    display.println("Y: " + String(currentOffsetY, 1));
    display.println("Z: " + String(currentOffsetZ, 1));
    display.display();
    delay(2000);
    
    render();
}

void AdxlMenuState::resetCalibration() {
    if (!adxl) {
        // Show error if sensor not available
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.println("ADXL sensor not");
        display.println("available!");
        display.display();
        delay(1500);
        render();
        return;
    }
    
    adxl->setManualOffsets(0, 0, 0);
    adxl->saveConfig();
    
    // Update display with new offsets
    currentOffsetX = 0;
    currentOffsetY = 0;
    currentOffsetZ = 0;
    
    display.clearDisplay();
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println("Calibration Reset!");
    display.display();
    delay(1500);
    
    render();
}

void AdxlMenuState::toggleTracking() {
    if (!adxl) {
        // Show error if sensor not available
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
        display.println("ADXL sensor not");
        display.println("available!");
        display.display();
        delay(1500);
        render();
        return;
    }
    
    if (adxl->isCurrentlyTracking()) {
        adxl->stopTracking();
        
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("Tracking Stopped!");
        display.display();
        delay(1000);
    } else {
        adxl->startTracking();
        
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("Tracking Started!");
        display.println("Angles will be shown");
        display.println("at bottom of screen.");
        display.display();
        delay(1500);
    }
    
    render();
}

void AdxlMenuState::enterLidarMenu() {
    // Create a new LidarCalibrationState with the lidarSensor
    if (lidarSensor) {
        StateManager::setState(new LidarCalibrationState(lidarSensor));
    } else {
        // Show error message if LIDAR is not available
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("LIDAR sensor not");
        display.println("available!");
        display.display();
        delay(1500);
        render();
    }
}
