#ifndef MENU_STATE_H
#define MENU_STATE_H

#include <Arduino.h>

class MenuState {
public:
    virtual void onEnter() = 0;          // Wird einmalig beim Eintritt aufgerufen
    virtual void onExit() = 0;           // Beim Verlassen
    virtual void update() = 0;           // Zyklisch im Hauptloop
    virtual void onButtonLPress() = 0;    // Taste gedrückt
    virtual void onJoystickRMove(int xR, int yR) = 0; // Joystick-Eingabe
    virtual void onButtonRPress() = 0;    // Taste gedrückt
    virtual void onJoystickLMove(int xL, int yL) = 0; // Joystick-Eingabe
    virtual ~MenuState() {}
};

#endif
