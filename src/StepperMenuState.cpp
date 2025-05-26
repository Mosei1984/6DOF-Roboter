#include "StepperMenuState.h"
#include "DisplayHandler.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "robot_bitmaps.h"
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

const char* stepperItems[] = {
    "Homing: Alle Motoren",
    "Homing: Einzelmotor",
    "Konfiguration",
    "Debugger"
};

void StepperMenuState::onEnter() {
    render();
}

void StepperMenuState::onExit() {
    display.clearDisplay();
    display.display();
}

void StepperMenuState::update() {
    // Optional: Animation oder Hintergrundprozesse
}

void StepperMenuState::onButtonLPress() {
    StateManager::setState(new MainMenuState());
}

void StepperMenuState::onButtonRPress() {
    switch (selection) {
        case 0:
            for (uint8_t i = 0; i < 6; i++) {
                renderHomingProgress(i);
                startHomingForJoint(i);
            }
            break;
        case 1:
            renderHomingProgress(selection);
            startHomingForJoint(selection);
            break;
        case 2:
            // Konfigurationsmenü (später)
            break;
        case 3:
            // Debug aktivieren (später)
            break;
    }
}

void StepperMenuState::onJoystickLMove(int xL, int yL) {
    if (yL < -200) selection = (selection + 1) % 4;
    if (yL >  200) selection = (selection - 1 + 4) % 4;
    render();
}

void StepperMenuState::onJoystickRMove(int xR, int yR) {
    // Reserviert für Feineinstellungen später
}

void StepperMenuState::render() {
    display.clearDisplay();
    display.drawBitmap(0, 0, Gear_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.println("Stepper-Menü");

    for (int i = 0; i < 4; i++) {
        display.setCursor(10, 18 + i * 10);
        if (i == selection) display.print("> ");
        else display.print("  ");
        display.println(stepperItems[i]);
    }

    display.display();
}

void StepperMenuState::renderHomingProgress(uint8_t jointIndex) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.printf("Homing Motor %d...\n", jointIndex + 1);

    for (int p = 0; p <= 100; p += 5) {
        display.fillRect(0, 20, map(p, 0, 100, 0, 128), 10, WHITE);
        display.drawRect(0, 20, 128, 10, WHITE);
        display.display();
        delay(20); // Dummy-Fortschritt
    }

    display.setCursor(0, 35);
    display.println("Abgeschlossen.");
    display.display();
    delay(500);
}

void StepperMenuState::startHomingForJoint(uint8_t jointIndex) {
    // Platzhalter für spätere Stepper-Ansteuerung
    Serial.printf("Starte Homing für Motor %d...\n", jointIndex + 1);
}
