#include "JoystickMenuState.h"
#include "DisplayHandler.h"
#include "StateManager.h"
#include "MainMenuState.h"
#include "robot_bitmaps.h"
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

const char* joystickItems[] = {
    "Kalibrierung starten",
    "Kalibrierung speichern",
    "Kalibrierung zurücksetzen",
    "Live-Debug anzeigen"
};

void JoystickMenuState::onEnter() {
    render();
}

void JoystickMenuState::onExit() {
    display.clearDisplay();
    display.display();
}

void JoystickMenuState::update() {
    // Optional: z. B. Debuganzeige zyklisch aktualisieren
}

void JoystickMenuState::onButtonLPress() {
    StateManager::setState(new MainMenuState());
}

void JoystickMenuState::onButtonRPress() {
    switch (selection) {
        case 0:
            startCalibrationL();
            break;
        case 1:
            startCalibrationR();
            break;
        case 2:
            resetCalibration();
            break;
        case 3:
            showLiveDebug();
            break;
        case 4:
            saveCalibration();
            break;
    }
}

void JoystickMenuState::onJoystickLMove(int xL, int yL) {
    if (yL < -200) selection = (selection + 1) % 4;
    if (yL >  200) selection = (selection - 1 + 4) % 4;
    render();
}

void JoystickMenuState::onJoystickRMove(int xR, int yR) {
    // Optional: Empfindlichkeit oder Filter in Echtzeit anpassen
}

void JoystickMenuState::render() {
    display.clearDisplay();
    display.drawBitmap(0, 0, Joystickicon_small, 16, 16, WHITE);
    display.setCursor(20, 0);
    display.setTextSize(1);
    display.println("Joystick-Menü");

    for (int i = 0; i < 4; i++) {
        display.setCursor(10, 18 + i * 10);
        if (i == selection) display.print("> ");
        else display.print("  ");
        display.println(joystickItems[i]);
    }

    display.display();
}

void JoystickMenuState::startCalibrationL() {
    Serial.println("🕹️ Kalibrierung gestartet...");
    // Platzhalter für Kalibrierfunktion
}
void JoystickMenuState::startCalibrationR() {
    Serial.println("🕹️ Kalibrierung gestartet...");
    // Platzhalter für Kalibrierfunktion
}
void JoystickMenuState::saveCalibration() {
    Serial.println("💾 Kalibrierung gespeichert.");
    // Platzhalter für Speicherung
}

void JoystickMenuState::resetCalibration() {
    Serial.println("↩️ Kalibrierung zurückgesetzt.");
    // Platzhalter für Reset
}

void JoystickMenuState::showLiveDebug() {
    Serial.println("📊 Joystick-Debug aktiv.");
    // Platzhalter für Anzeige
}
