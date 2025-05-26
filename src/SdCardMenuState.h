#ifndef SD_CARD_MENU_STATE_H
#define SD_CARD_MENU_STATE_H

#include "MenuState.h"
#include "ConfigHandler.h"
#include <Adafruit_SSD1306.h>

class SdCardMenuState : public MenuState {
private:
    int selection = 0;
    String message = "";
    const char* sdItems[4] = {
        "Load Config",
        "Save Config",
        "Reset to Default",
        "Back"
    };
    
    void render();
    
public:
    void onEnter() override;
    void onExit() override;
    void update() override;
    void onButtonLPress() override;
    void onButtonRPress() override;
    void onJoystickLMove(int xL, int yL) override;
    void onJoystickRMove(int xR, int yR) override;
};

#endif // SD_CARD_MENU_STATE_H
