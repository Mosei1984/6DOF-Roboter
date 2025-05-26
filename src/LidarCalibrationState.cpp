#include "LidarCalibrationState.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "robot_bitmaps.h"

// External objects
extern Adafruit_SSD1306 display;
extern ConfigHandler configHandler;

LidarCalibrationState::LidarCalibrationState(Adafruit_SSD1306& disp, 
                                           VL53L0XSensor* sensor,
                                           TrajectoryPlanner* trajPlanner, 
                                           ConfigHandler* config)
    : display(disp),
      lidarSensor(sensor),
      trajectoryPlanner(trajPlanner),
      configHandler(config),
      selection(0),
      statusMessage("Ready"),
      calibrationTarget(300),
      isCalibrating(false)
{
    updateTimer.interval = 100;  // Update display every 100ms
}

// Konstruktor mit nur dem Sensor (für einfache Tests)
LidarCalibrationState::LidarCalibrationState(VL53L0XSensor* sensor)
    : display(::display),  // Verwende globales Display-Objekt
      lidarSensor(sensor),
      trajectoryPlanner(nullptr),
      configHandler(nullptr),
      selection(0),
      statusMessage("Ready"),
      calibrationTarget(300),
      isCalibrating(false)
{
    updateTimer.interval = 100;  // Update display every 100ms
}

void LidarCalibrationState::onEnter() {
    selection = 0;
    render();
}

void LidarCalibrationState::onExit() {
    // Cancel any ongoing calibration
    if (isCalibrating && lidarSensor) {
        lidarSensor->cancelCalibration();
    }
    
    display.clearDisplay();
    display.display();
}

void LidarCalibrationState::update() {
    unsigned long now = millis();
    
    // Update sensor if available
    if (lidarSensor) {
        lidarSensor->update();
        
        // Check if calibration completed
        if (isCalibrating && lidarSensor->calibrationComplete()) {
            isCalibrating = false;
            statusMessage = "Calibration complete!";
        }
    }
    
    // Update display periodically
    if (updateTimer.isDue(now)) {
        render();
        updateTimer.reset(now);
    }
}

void LidarCalibrationState::onButtonLPress() {
    // Back button - return to main menu
    StateManager::setState(new MainMenuState());
}

void LidarCalibrationState::onButtonRPress() {
    if (!lidarSensor) {
        // If no sensor, only allow exit
        if (selection == 5) {
            StateManager::setState(new MainMenuState());
        }
        return;
    }
    
    // Action depends on selection
    switch (selection) {
        case 0:  // Start/Cancel Calibration
            if (isCalibrating) {
                cancelCalibration();
            } else {
                startCalibration();
            }
            break;
            
        case 1:  // Reset Calibration
            resetCalibration();
            break;
            
        case 2:  // Save to SD
            if (lidarSensor->saveCalibrationToSD()) {
                statusMessage = "Saved to SD";
            } else {
                statusMessage = "Save failed!";
            }
            break;
            
        case 3:  // Load from SD
            if (lidarSensor->loadCalibrationFromSD()) {
                statusMessage = "Loaded from SD";
            } else {
                statusMessage = "Load failed!";
            }
            break;
            
        case 4:  // Toggle Long Range
            lidarSensor->setLongRangeMode(!lidarSensor->getLongRangeMode());
            statusMessage = "Long range: " + String(lidarSensor->getLongRangeMode() ? "ON" : "OFF");
            break;
            
        case 5:  // Back
            StateManager::setState(new MainMenuState());
            break;
    }
    
    render();
}

void LidarCalibrationState::onJoystickLMove(int xL, int yL) {
    // Navigate up/down through menu options
    if (yL < -200) {
        selection = (selection + 1) % 6;
        render();
    }
    if (yL > 200) {
        selection = (selection - 1 + 6) % 6;
        render();
    }
    
    // If on calibration target, allow adjustment with left/right
    if (selection == 0 && !isCalibrating && lidarSensor) {
        if (xL > 200) {
            adjustCalibrationTarget(10);
            render();
        }
        if (xL < -200) {
            adjustCalibrationTarget(-10);
            render();
        }
    }
}

void LidarCalibrationState::onJoystickRMove(int xR, int yR) {
    // Not used in this menu
}

void LidarCalibrationState::render() {
    display.clearDisplay();
    
    // Title
    display.drawBitmap(0, 0, robot_icon4_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.println("LIDAR Calibration");
    display.drawLine(0, 9, display.width(), 9, SSD1306_WHITE);
    
    // Current reading
    display.setCursor(0, 12);
    display.print("Current: ");
    if (lidarSensor && lidarSensor->isValid()) {
        display.print(lidarSensor->getDistance());
        display.print("mm");
    } else {
        display.print("Invalid");
    }
    
    // Status message
    display.setCursor(0, 22);
    display.print("Status: ");
    display.print(statusMessage);
    
    // Menu options
    const char* options[] = {
        isCalibrating ? "Cancel Calibration" : "Calibrate at ",
        "Reset Calibration",
        "Save to SD",
        "Load from SD",
        "Long Range Mode",
        "Back"
    };
    
    for (int i = 0; i < 6; i++) {
        display.setCursor(10, 32 + i * 8);
        if (i == selection) display.print("> ");
        else display.print("  ");
        
        display.print(options[i]);
        
        // Show calibration target if on first option
        if (i == 0 && !isCalibrating) {
            display.print(calibrationTarget);
            display.print("mm");
        }
        
        // Show status for long range mode
        if (i == 4 && lidarSensor) {
            display.setCursor(110, 32 + i * 8);
            display.print(lidarSensor->getLongRangeMode() ? "ON" : "OFF");
        }
    }
    
    display.display();
}

void LidarCalibrationState::startCalibration() {
    if (!lidarSensor) return;
    
    lidarSensor->startCalibration(calibrationTarget);
    isCalibrating = true;
    statusMessage = "Calibrating...";
}

void LidarCalibrationState::cancelCalibration() {
    if (!lidarSensor) return;
    
    lidarSensor->cancelCalibration();
    isCalibrating = false;
    statusMessage = "Calibration cancelled";
}

void LidarCalibrationState::resetCalibration() {
    if (!lidarSensor) return;
    
    lidarSensor->resetCalibration();
    statusMessage = "Calibration reset";
}

void LidarCalibrationState::adjustCalibrationTarget(int amount) {
    calibrationTarget += amount;
    
    // Constrain to reasonable values
    if (calibrationTarget < 50) calibrationTarget = 50;
    if (calibrationTarget > 2000) calibrationTarget = 2000;
}
