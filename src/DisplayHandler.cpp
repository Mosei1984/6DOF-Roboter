#include "DisplayHandler.h"

DisplayHandler::DisplayHandler(int js1X, int js1Y, int js2X, int js2Y, int btn1, int btn2)
    : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
      joystick1X(js1X), joystick1Y(js1Y), joystick2X(js2X), joystick2Y(js2Y),
      button1(btn1), button2(btn2)
{
    // Initialize menu system
    rootMenu = nullptr;
    currentMenu = nullptr;
}

DisplayHandler::~DisplayHandler() {
    if (rootMenu) {
        delete rootMenu;
    }
}

void DisplayHandler::begin() {
    // Initialize display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    
    // Setup display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.cp437(true);
    
    // Setup input pins
    pinMode(button1, INPUT_PULLUP);
    pinMode(button2, INPUT_PULLUP);
    
    // Initialize menu system
    setupMenus();
    
    // Show initial menu
    drawMenu();
}

void DisplayHandler::setupMenus() {
    // Create root menu
    rootMenu = new MenuItem("Main Menu");
    
    // Add main menu items
    rootMenu->addChild(new MenuItem("Manual Control", [this]() { manualControl(); }));
    rootMenu->addChild(new MenuItem("Run Program", [this]() { runProgram(); }));
    rootMenu->addChild(new MenuItem("Teach Mode", [this]() { teachMode(); }));
    rootMenu->addChild(new MenuItem("Settings", [this]() { settings(); }));
    rootMenu->addChild(new MenuItem("Calibration", [this]() { calibration(); }));
    rootMenu->addChild(new MenuItem("About", [this]() { about(); }));
    
    // Add submenu for Settings
    MenuItem* settingsMenu = rootMenu->children[3];
    settingsMenu->addChild(new MenuItem("Speed"));
    settingsMenu->addChild(new MenuItem("Acceleration"));
    settingsMenu->addChild(new MenuItem("Save Config"));
    settingsMenu->addChild(new MenuItem("Load Config"));
    settingsMenu->addChild(new MenuItem("Back", [this]() { returnToPreviousMenu(); }));
    
    // Add submenu for Calibration
    MenuItem* calibMenu = rootMenu->children[4];
    calibMenu->addChild(new MenuItem("Home All Axes"));
    calibMenu->addChild(new MenuItem("Calibrate Axis 1"));
    calibMenu->addChild(new MenuItem("Calibrate Axis 2"));
    calibMenu->addChild(new MenuItem("Calibrate Axis 3"));
    calibMenu->addChild(new MenuItem("Calibrate Axis 4"));
    calibMenu->addChild(new MenuItem("Calibrate Axis 5"));
    calibMenu->addChild(new MenuItem("Back", [this]() { returnToPreviousMenu(); }));
    
    // Set current menu to root
    currentMenu = rootMenu;
}

void DisplayHandler::update() {
    // Process joystick and button inputs
    processJoystick();
    processButtons();
    
    // Redraw the menu if needed
    drawMenu();
}

void DisplayHandler::drawMenu() {
    display.clearDisplay();
    display.setCursor(0, 0);
    
    // Draw title
    display.setTextSize(1);
    display.println(currentMenu->title);
    display.drawLine(0, 9, SCREEN_WIDTH-1, 9, SSD1306_WHITE);
    
    // Draw menu items
    if (currentMenu->children.size() > 0) {
        // Berechne die maximale Anzahl sichtbarer Elemente
        int maxVisibleItems = min((int)currentMenu->children.size(), MAX_MENU_ITEMS);
        int totalItems = currentMenu->children.size();
        
        // Stelle sicher, dass der ausgewählte Index immer sichtbar ist
        if (selectedIndex < scrollOffset) {
            scrollOffset = selectedIndex;
        } else if (selectedIndex >= scrollOffset + maxVisibleItems) {
            scrollOffset = selectedIndex - maxVisibleItems + 1;
        }
        
        // Anzahl der tatsächlich sichtbaren Elemente
        int visibleItems = min(totalItems - scrollOffset, maxVisibleItems);
        
        // Draw visible items
        for (size_t i = 0; i < static_cast<size_t>(visibleItems); i++) {
            size_t itemIndex = i + scrollOffset;
            if (itemIndex < static_cast<size_t>(currentMenu->children.size())) {
                if (itemIndex == selectedIndex) {
                    // Draw selection indicator
                    display.fillRect(0, 11 + i*MENU_ITEM_HEIGHT, SCREEN_WIDTH, MENU_ITEM_HEIGHT, SSD1306_WHITE);
                    display.setTextColor(SSD1306_BLACK);
                } else {
                    display.setTextColor(SSD1306_WHITE);
                }
                
                display.setCursor(2, 13 + i*MENU_ITEM_HEIGHT);
                display.println(currentMenu->children[itemIndex]->title);
                
                // Draw submenu indicator if needed
                if (currentMenu->children[itemIndex]->children.size() > 0) {
                    display.drawTriangle(
                        SCREEN_WIDTH-5, 13 + i*MENU_ITEM_HEIGHT + MENU_ITEM_HEIGHT/2 - 3,
                        SCREEN_WIDTH-5, 13 + i*MENU_ITEM_HEIGHT + MENU_ITEM_HEIGHT/2 + 3,
                        SCREEN_WIDTH-1, 13 + i*MENU_ITEM_HEIGHT + MENU_ITEM_HEIGHT/2,
                        itemIndex == selectedIndex ? SSD1306_BLACK : SSD1306_WHITE);
                }
                
                // Reset text color
                display.setTextColor(SSD1306_WHITE);
            }
        }
        
        // Draw scroll indicators when needed
        if (scrollOffset > 0) {
            // Up arrow
            display.fillTriangle(
                SCREEN_WIDTH-10, 11, 
                SCREEN_WIDTH-6, 11, 
                SCREEN_WIDTH-8, 9, 
                SSD1306_WHITE);
        }
        
        if (scrollOffset + visibleItems < totalItems) {
            // Down arrow
            int y = 11 + visibleItems*MENU_ITEM_HEIGHT;
            display.fillTriangle(
                SCREEN_WIDTH-10, y-2, 
                SCREEN_WIDTH-6, y-2, 
                SCREEN_WIDTH-8, y, 
                SSD1306_WHITE);
        }
        
        // Draw scrollbar if needed
        if (totalItems > maxVisibleItems) {
            int scrollbarHeight = (visibleItems * (SCREEN_HEIGHT - 11)) / totalItems;
            int scrollbarPos = 11 + (scrollOffset * (SCREEN_HEIGHT - 11 - scrollbarHeight)) / (totalItems - visibleItems);
            
            display.drawRect(SCREEN_WIDTH-2, scrollbarPos, 2, scrollbarHeight, SSD1306_WHITE);
        }
    } else {
        display.setCursor(0, 20);
        display.println("No menu items");
    }
    
    display.display();
}

void DisplayHandler::processJoystick() {
    // Read joystick values
    int js1X = analogRead(joystick1X);
    int js1Y = analogRead(joystick1Y);
    
    // Check if enough time has passed since last navigation
    unsigned long currentTime = millis();
    unsigned long timeSinceLastNav = currentTime - lastNavTime;
    
    // Dynamische Timeout-Anpassung für schnelleres Scrollen bei längerem Halten
    unsigned long dynamicTimeout = NAV_TIMEOUT;
    if (timeSinceLastNav > 1000) { // Nach 1 Sekunde Halten
        // Reduziere Timeout für schnelleres Scrollen (min. 100ms)
        dynamicTimeout = max(NAV_TIMEOUT / 3, 100UL);
    }
    
    if (timeSinceLastNav < dynamicTimeout) {
        return;
    }
    
    bool navigationChanged = false;
    
    // Check for vertical navigation (menu selection)
    if (js1Y < 300 && lastJoystick1Y >= 300) {  // Up
        if (selectedIndex > 0) {
            selectedIndex--;
            navigationChanged = true;
        }
    } else if (js1Y > 700 && lastJoystick1Y <= 700) {  // Down
        if (selectedIndex < currentMenu->children.size() - 1) {
            selectedIndex++;
            navigationChanged = true;
        }
    }
    
    // Schnelles Scrollen bei längerem Halten
    if ((js1Y < 300 || js1Y > 700) && timeSinceLastNav > 1500) {  // Nach 1.5 Sekunden
        // Berechne Scroll-Menge basierend auf der Haltedauer
        int scrollAmount = 1 + (timeSinceLastNav - 1500) / 500;  // Zusätzlich +1 alle 500ms
        scrollAmount = min(scrollAmount, 3);  // Maximal 3 auf einmal
        
        if (js1Y < 300) {  // Nach oben
            selectedIndex = max((int)selectedIndex - scrollAmount, 0);
            navigationChanged = true;
        } else if (js1Y > 700) {  // Nach unten
            selectedIndex = min(selectedIndex + scrollAmount, currentMenu->children.size() - 1);
            navigationChanged = true;
        }
    }
    
    // Check for horizontal navigation (menu navigation)
    if (js1X > 700 && lastJoystick1X <= 700) {  // Right - enter submenu
        if (selectedIndex < currentMenu->children.size() && 
            currentMenu->children[selectedIndex]->children.size() > 0) {
            
            menuHistory.push_back(currentMenu);
            selectionHistory.push_back(selectedIndex);
            
            currentMenu = currentMenu->children[selectedIndex];
            selectedIndex = 0;
            scrollOffset = 0;
            navigationChanged = true;
        }
    } else if (js1X < 300 && lastJoystick1X >= 300) {  // Left - go back
        returnToPreviousMenu();
        navigationChanged = true;
    }
    
    // Update navigation time and redraw if needed
    if (navigationChanged) {
        lastNavTime = currentTime;
    }
    
    // Save current joystick values
    lastJoystick1X = js1X;
    lastJoystick1Y = js1Y;
}

void DisplayHandler::processButtons() {
    // Read button states (inverted because of INPUT_PULLUP)
    bool btn1State = !digitalRead(button1);
    bool btn2State = !digitalRead(button2);
    
    // Button 1: Select/Execute
    if (btn1State && !button1Pressed) {
        executeMenuItem();
    }
    
    // Button 2: Back/Cancel
    if (btn2State && !button2Pressed) {
        returnToPreviousMenu();
    }
    
    // Update button states
    button1Pressed = btn1State;
    button2Pressed = btn2State;
}

void DisplayHandler::executeMenuItem() {
    if (selectedIndex < currentMenu->children.size()) {
        MenuItem* item = currentMenu->children[selectedIndex];
        
        // If item has an action, execute it
        if (item->action) {
            item->action();
        }
        // If item has children but no action, navigate to submenu
        else if (item->children.size() > 0) {
            menuHistory.push_back(currentMenu);
            selectionHistory.push_back(selectedIndex);
            
            currentMenu = item;
            selectedIndex = 0;
            scrollOffset = 0;
        }
    }
}

void DisplayHandler::returnToPreviousMenu() {
    if (!menuHistory.empty()) {
        currentMenu = menuHistory.back();
        menuHistory.pop_back();
        
        selectedIndex = selectionHistory.back();
        selectionHistory.pop_back();
        
        // Stellt sicher, dass die ausgewählte Option sichtbar ist
        scrollOffset = max(0, min(selectedIndex, (int)currentMenu->children.size() - MAX_MENU_ITEMS));
        
        lastNavTime = millis();
    }
}

// Robot arm specific functions
void DisplayHandler::manualControl() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Manual Control");
    display.println("Use joysticks to move");
    display.println("JS1: X/Y axes");
    display.println("JS2: Z/rotation");
    display.println("BTN1: Toggle gripper");
    display.println("BTN2: Back to menu");
    display.display();
    
    // Wait for button 2 to return
    while (digitalRead(button2) == HIGH) {
        delay(10);
    }
    
    // Wait for button release
    while (digitalRead(button2) == LOW) {
        delay(10);
    }
}

void DisplayHandler::runProgram() {
    // Program execution screen
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Run Program");
    display.println("Program running...");
    display.println("BTN2: Stop and return");
    display.display();
    
    // Wait for button 2 to return
    while (digitalRead(button2) == HIGH) {
        delay(10);
    }
    
    // Wait for button release
    while (digitalRead(button2) == LOW) {
        delay(10);
    }
}

void DisplayHandler::teachMode() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Teach Mode");
    display.println("Move robot to position");
    display.println("BTN1: Record point");
    display.println("BTN2: Back to menu");
    display.display();
    
    // Wait for button 2 to return
    while (digitalRead(button2) == HIGH) {
        delay(10);
    }
    
    // Wait for button release
    while (digitalRead(button2) == LOW) {
        delay(10);
    }
}

void DisplayHandler::settings() {
    // This function will be called when Settings is selected
    // The menu navigation will handle the submenu
}

void DisplayHandler::calibration() {
    // This function will be called when Calibration is selected
    // The menu navigation will handle the submenu
}

void DisplayHandler::about() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("6DOF Robot Arm");
    display.println("Version 1.0");
    display.println("Created by:");
    display.println("Mosei1984");
    display.println("\nBTN2: Back to menu");
    display.display();
    
    // Wait for button 2 to return
    while (digitalRead(button2) == HIGH) {
        delay(10);
    }
    
    // Wait for button release
    while (digitalRead(button2) == LOW) {
        delay(10);
    }
}
