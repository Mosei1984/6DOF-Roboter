#include "calibration.h"
#include "joystick.h"

extern Joystick* leftJoystick;
extern Joystick* rightJoystick;

bool Calibration::_isCalibrating = false;

void Calibration::calibrateJoysticks() {
    // Grundkalibrierung für Zentrum
    leftJoystick->calibrate();
    rightJoystick->calibrate();
}

void Calibration::checkCalibrationTrigger() {
    // Wenn linker Joystick gedrückt wird, starte Kalibrierung
    if (leftJoystick->isPressed() && !_isCalibrating) {
        _isCalibrating = true;
        
        // Warte bis der Knopf losgelassen wird
        while (leftJoystick->isPressed()) {
            delay(10);
        }
        
        // Starte die erweiterte Kalibrierung für beide Joysticks
        leftJoystick->startCalibration();
        rightJoystick->startCalibration();
        
        _isCalibrating = false;
    }
}

bool Calibration::isCalibrating() {
    return _isCalibrating;
}