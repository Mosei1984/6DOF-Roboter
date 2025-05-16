#pragma once

class Calibration {
public:
    static void calibrateJoysticks();
    static void checkCalibrationTrigger();
    static bool isCalibrating();
    
private:
    static bool _isCalibrating;
};
