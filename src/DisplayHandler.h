#include <Adafruit_SSD1306.h>
#ifndef DISPLAY_HANDLER_H
#define DISPLAY_HANDLER_H

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <vector>
#include <functional>

// Display constants
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

// Menu constants
#define MAX_MENU_ITEMS 6
#define MENU_ITEM_HEIGHT 10

class MenuItem {
public:
    String title;
    std::function<void()> action;
    std::vector<MenuItem*> children;
    
    MenuItem(String _title, std::function<void()> _action = nullptr) 
        : title(_title), action(_action) {}
    
    ~MenuItem() {
        for (auto child : children) {
            delete child;
        }
    }
    
    void addChild(MenuItem* child) {
        children.push_back(child);
    }
};

class DisplayHandler {
private:
    Adafruit_SSD1306 display;
    MenuItem* rootMenu;
    MenuItem* currentMenu;
    size_t selectedIndex = 0;
    size_t scrollOffset = 0;
    size_t visibleItems = 0;
    
    // Joystick pins
    int joystick1X;
    int joystick1Y;
    int joystick2X;
    int joystick2Y;
    
    // Button pins
    int button1;
    int button2;
    
    // Button states
    bool button1Pressed = false;
    bool button2Pressed = false;
    
    // Joystick states
    int lastJoystick1X = 512;
    int lastJoystick1Y = 512;
    int lastJoystick2X = 512;
    int lastJoystick2Y = 512;
    
    // Navigation timeout
    unsigned long lastNavTime = 0;
    const unsigned long NAV_TIMEOUT = 300; // ms
    
    void drawMenu();
    void processJoystick();
    void processButtons();
    void executeMenuItem();
    void returnToPreviousMenu();
    
    // Menu state history for navigation
    std::vector<MenuItem*> menuHistory;
    std::vector<int> selectionHistory;

public:
    DisplayHandler(int js1X, int js1Y, int js2X, int js2Y, int btn1, int btn2);
    ~DisplayHandler();
    
    void begin();
    void update();
    void setupMenus();
    
    // Robot arm specific functions
    void manualControl();
    void runProgram();
    void teachMode();
    void settings();
    void calibration();
    void about();
};

#endif // DISPLAY_HANDLER_H
