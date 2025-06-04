#include "JointMode.h"


// Forward declarations for functions used within JointMode.cpp
void handleAxisSelection();
void handleAxisMovement();
void updateJointMode();
void readDirectJoysticks();
void readDirectButtons();
void updateLEDs();
void stopAllSteppers();

// Whenever you use motors in this file, use the indexing like this:
//motors[0].setSpeed(500);  // Correct way to use the alias

// Include the shared UI header


// Servo für den Greifer
extern Servo gripper;
int servoPosition = SERVO_DEFAULT_POS; // Startposition aus Konfiguration

// NeoPixel-Setup
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LED Farben definieren
#define COLOR_OFF     pixels.Color(0, 0, 0)
#define COLOR_RED     pixels.Color(255, 0, 0)
#define COLOR_GREEN   pixels.Color(0, 255, 0)
#define COLOR_BLUE    pixels.Color(0, 0, 255)
#define COLOR_YELLOW  pixels.Color(255, 255, 0)
#define COLOR_MAGENTA pixels.Color(255, 0, 255)
#define COLOR_CYAN    pixels.Color(0, 255, 255)
#define COLOR_WHITE   pixels.Color(255, 255, 255)

// Variablen für die Achsenauswahl und -steuerung
int selectedAxis = 0;        // Aktuell ausgewählte Achse (0-6)
bool axisConfirmed = false;  // Ob eine Achse bestätigt wurde
extern unsigned long lastJoystickCheck; // Für Joystick-Aktualisierung
unsigned long lastLedUpdate = 0;     // Für LED-Animation
unsigned long lastServoUpdate = 0;   // Für Servo-Updates

// Joystick-Werte direkt einlesen
int joystickLeftX = 0;
int joystickLeftY = 0;
int joystickRightZ = 0;
int joystickRightYaw = 0;

// REMOVED: bool buttonConfirmPressed = false; - now using global from main.cpp
bool prevButtonConfirmState = HIGH;
unsigned long lastButtonDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 5;

// Motor-Steuerung
float currentmotorspeeds[7] = {0}; // Auf 7 erweitert für alle Motoren + Greifer

// Achsennamen für alle 7 Elemente
const char* axisNames[7] = {
  "Basis",
  "Schulter",
  "Ellenbogen",
  "Wrist Pitch",
  "Wrist Roll",
  "Tool Roll",  // Stepper 6
  "Greifer"     // Servo (jetzt getrennt als 7. Element)
};

// Geschwindigkeits- und Beschleunigungsvariablen
const float JOINT_MAX_SPEED = JOINT_MODE_MAX_SPEED;
const float MAX_ACCELERATION = JOINT_MODE_MAX_ACCELERATION;

// Glättungsvariablen für Joystick
float smoothedJoystickY = 512.0; // Startwert (Mitte)
const float SMOOTHING_FACTOR = 0.15; // Anpassbar (0.05-0.3)

// Variablen für non-blocking Timing
unsigned long buttonFeedbackStartTime = 0;
bool inButtonFeedbackMode = false;
unsigned long axisSelectionFeedbackTime = 0;
bool inAxisFeedbackMode = false;
unsigned long initLedSequenceTime = 0;
bool inInitSequence = false;
unsigned long lastStepperUpdateTime = 0;

// Getter-Funktionen
int getSelectedAxis() {
  return selectedAxis;
}

bool isAxisConfirmed() {
  return axisConfirmed;
}

const char* getAxisName(int index) {
  if (index >= 0 && index < 7) {
    return axisNames[index];
  }
  return "Unbekannt";
}

int getServoPosition() {
  return servoPosition;
}

// LED-Funktionen
void setAllPixels(uint32_t color) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void setAxisLED(int axis, uint32_t color) {
  // Sicherheitscheck für LED-Index
  if (axis >= 0 && axis < NUM_PIXELS) {
    pixels.setPixelColor(axis, color);
    pixels.show();
  }
}

void updateLEDs() {
  // Update LEDs based on current status
  if (!axisConfirmed) {
    // Im Auswahlmodus: Rot für die ausgewählte Achse, aus für die anderen
    for (int i = 0; i < NUM_PIXELS; i++) {
      // Sicherheitscheck: Nur verfügbare LEDs aktualisieren
      if (i < selectedAxis && i < NUM_PIXELS) {
        pixels.setPixelColor(i, COLOR_OFF);
      } 
      else if (i == selectedAxis && i < NUM_PIXELS) {
        pixels.setPixelColor(i, COLOR_RED);
      }
      else if (i < NUM_PIXELS) {
        pixels.setPixelColor(i, COLOR_OFF);
      }
    }
  } else {
    // Im Bewegungsmodus: Grün mit Helligkeit basierend auf Geschwindigkeit
    for (int i = 0; i < NUM_PIXELS; i++) {
      if (i == selectedAxis && i < NUM_PIXELS) {
        // Spezialfall für Servo (immer Grün wenn aktiv)
        if (i == 6 && i < NUM_PIXELS) {
          pixels.setPixelColor(i < NUM_PIXELS ? i : NUM_PIXELS-1, COLOR_GREEN);
        } else if (i < NUM_PIXELS) {
          // Intensität basierend auf aktueller Geschwindigkeit
          float speed = abs(currentmotorspeeds[i]);
          int brightness = map(speed, 0, MAX_SPEED, 0, 255);
          pixels.setPixelColor(i, pixels.Color(0, brightness, 0));
        }
      } else if (i < NUM_PIXELS) {
        pixels.setPixelColor(i, COLOR_OFF);
      }
    }
  }
  pixels.show();
}

// Direkte Joystick-Funktionen
void readDirectJoysticks() {
  // Joystick-Werte direkt lesen
  int rawLeftY = analogRead(JOYSTICK_L_Y);
  joystickLeftX = analogRead(JOYSTICK_L_X);
  joystickRightZ = analogRead(JOYSTICK_R_Z);
  joystickRightYaw = analogRead(JOYSTICK_R_YAW);
  
  // Exponentielles Glätten des Y-Werts für die Motorsteuerung
  smoothedJoystickY = SMOOTHING_FACTOR * rawLeftY + (1-SMOOTHING_FACTOR) * smoothedJoystickY;
  
  // Gerundeten geglätteten Wert für die weitere Verarbeitung verwenden
  joystickLeftY = round(smoothedJoystickY);
  
}

void readDirectButtons() {
  // Button-Status lesen (LOW = gedrückt, HIGH = nicht gedrückt)
  bool currentButtonState = digitalRead(BUTTON_CONFIRM);
  
  // Entprellung
  if ((millis() - lastButtonDebounceTime) > DEBOUNCE_DELAY) {
    // Button-Änderung erkennen (fallende Flanke = Drücken)
    if (currentButtonState == LOW && prevButtonConfirmState == HIGH) {
      buttonConfirmPressed = true;
      lastButtonDebounceTime = millis();
      
      // Modus umschalten
      axisConfirmed = !axisConfirmed;
      
      // LED-Feedback starten (non-blocking)
      inButtonFeedbackMode = true;
      buttonFeedbackStartTime = millis();
      setAllPixels(COLOR_BLUE);
      
      // Alle Motoren stoppen, wenn Auswahl aufgehoben
      if (!axisConfirmed) {
        stopAllSteppers();
      }
    } else {
      buttonConfirmPressed = false;
    }
    
    prevButtonConfirmState = currentButtonState;
  }
  
  // Non-blocking LED-Feedback bei Tastendruck verarbeiten
  if (inButtonFeedbackMode && millis() - buttonFeedbackStartTime > 100) {
    inButtonFeedbackMode = false;
    updateLEDs(); // Zurück zu normalen LEDs
  }
}

void initJointMode(U8G2* display) {
  displayPtr = display;
  selectedAxis = 0;
  axisConfirmed = false;
  
  // Initialisiere den Servo
  gripper.attach(SERVO_PIN);
  servoPosition = SERVO_DEFAULT_POS;
  gripper.write(servoPosition);
  
  // NeoPixel initialisieren
  pixels.begin();
  pixels.setBrightness(PIXEL_BRIGHTNESS);
  
  // Initiale LED-Sequenz starten (non-blocking)
  setAllPixels(COLOR_CYAN);
  inInitSequence = true;
  initLedSequenceTime = millis();
  
  // Motoren vorbereiten - erhöhte Geschwindigkeit für JointMode
  for (int i = 0; i < 6; i++) {
    // Maximale Geschwindigkeit und Beschleunigung setzen
    motors[i].setMaxSpeed(JOINT_MAX_SPEED);
    motors[i].setAcceleration(MAX_ACCELERATION);
    
    // Motor aktivieren
    digitalWrite(ENABLE_PINS[i], LOW);
    
    // Geschwindigkeit auf 0 setzen
    currentmotorspeeds[i] = 0;
  }
  
  // Auch für den 7. Index initialisieren (Greifer)
  currentmotorspeeds[6] = 0;
  
  // Joystick-Pins direkt konfigurieren
  pinMode(JOYSTICK_L_X, INPUT);
  pinMode(JOYSTICK_L_Y, INPUT);
  pinMode(JOYSTICK_R_Z, INPUT);
  pinMode(JOYSTICK_R_YAW, INPUT);
  
  // Button-Pin konfigurieren
  pinMode(BUTTON_CONFIRM, INPUT_PULLUP);
  prevButtonConfirmState = digitalRead(BUTTON_CONFIRM);
  
  
  // Anfangs-LED-Zustand wird in handleJointMode gesetzt, sobald die Initialsequenz abgeschlossen ist
  lastStepperUpdateTime = micros();
}

void handleJointMode() {
  // Non-blocking init LED sequence
  if (inInitSequence) {
    if (millis() - initLedSequenceTime > 500) {
      inInitSequence = false;
      setAllPixels(COLOR_OFF);
      updateLEDs(); // Normaler LED-Status
    }
    // Während der Initialisierungssequenz trotzdem weitermachen
  }

  // Direkte Joystick- und Button-Werte einlesen
  readDirectJoysticks();
  readDirectButtons();
  
  // Always update gripper from potentiometer unless in gripper-specific control
  if (!(axisConfirmed && selectedAxis == 6)) {
    updateGripperFromPotentiometer();
  }
  
  // Achsenauswahl mit rechtem Joystick, wenn keine Achse bestätigt ist
  if (!axisConfirmed) {
    handleAxisSelection();
    
    // Alle Motoren anhalten wenn im Auswahlmodus
    for (int i = 0; i < 6; i++) {
      currentmotorspeeds[i] = 0;
      motors[i].setSpeed(0);
    }
  } 
  // Achsensteuerung mit linkem Joystick, wenn eine Achse bestätigt ist
  else {
    handleAxisMovement();
  }
  
  // OPTIMIERUNG: Motoren mehrmals pro Loop aktualisieren für flüssigere Bewegung
  // Non-blocking mit micros() timing
  const int MOTOR_UPDATES_PER_LOOP = 700;
  const unsigned long UPDATE_INTERVAL_MICROS = 5; // 50 Mikrosekunden zwischen Updates
  
  unsigned long currentMicros = micros();
  // Wenn genug Zeit seit dem letzten Update vergangen ist
  if (currentMicros - lastStepperUpdateTime >= UPDATE_INTERVAL_MICROS) {
    lastStepperUpdateTime = currentMicros;
    
    // Eine Iteration von Stepper-Updates
    static int updateCounter = 0;
    
    // Nur eine Iteration pro Aufruf (non-blocking)
    if (updateCounter < MOTOR_UPDATES_PER_LOOP) {
      updateSteppers();
      updateCounter++;
    } else {
      updateCounter = 0; // Reset für nächsten Loop
    }
  }
  
  // LED Feedback für Achsenauswahl verarbeiten
  if (inAxisFeedbackMode && millis() - axisSelectionFeedbackTime > 50) {
    inAxisFeedbackMode = false;
    updateLEDs(); // Zurück zu normalen LEDs
  }
  
  // LEDs regelmäßig aktualisieren
  if (millis() - lastLedUpdate > 200) { // Alle 100ms
    updateLEDs();
    lastLedUpdate = millis();
  }
  
  // Display aktualisieren
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 200) { // Alle 100ms
    updateJointMode ();
    lastDisplayUpdate = millis();
  }
}

void handleAxisSelection() {
  // Wert von rechtem Joystick (YAW) für Achsenauswahl verwenden
  int yawValue = joystickRightYaw;
  
  // Achse nur ändern, wenn Joystick weit genug ausgelenkt ist
  if (millis() - lastJoystickCheck > 200) {
    if (yawValue > 700) { // nach oben
      if (selectedAxis < 6) { // Maximal Achse 7 (Index 6)
        selectedAxis++;
        
        // LED-Feedback (non-blocking)
        if (selectedAxis < NUM_PIXELS) {
          inAxisFeedbackMode = true;
          axisSelectionFeedbackTime = millis();
          setAxisLED(selectedAxis, COLOR_MAGENTA);
        }
      }
      lastJoystickCheck = millis();
    } 
    else if (yawValue < 300) { // nach unten
      if (selectedAxis > 0) {
        selectedAxis--;
        
        // LED-Feedback (non-blocking)
        if (selectedAxis < NUM_PIXELS) {
          inAxisFeedbackMode = true;
          axisSelectionFeedbackTime = millis();
          setAxisLED(selectedAxis, COLOR_MAGENTA);
        }
      }
      lastJoystickCheck = millis();
    }
  }
}

void handleAxisMovement() {
  // SICHERHEITSABFRAGE
  if (!axisConfirmed) {
    return;
  }
  
  // Wenn Greifer ausgewählt ist (jetzt Index 6), steuere den Servo
  if (selectedAxis == 6) {
    // Special case: When gripper is specifically selected, provide direct control
    // Use potentiometer control when joystick is in neutral position
    int potValue = analogRead(SERVO_POTTY_PIN);
    servoPosition = map(potValue, 0, 1023, SERVO_MIN_POS, SERVO_MAX_POS);
    
    // Den Servo aktualisieren
    gripper.write(servoPosition);
    
    // LED-Feedback (wenn verfügbar)
    int brightness = map(servoPosition, 0, 180, 20, 255);
    if (selectedAxis < NUM_PIXELS) {
      pixels.setPixelColor(selectedAxis, pixels.Color(brightness, brightness, 0));
      pixels.show();
    }
  }
  // Sonst normale Stepper-Achsen steuern
  else if (selectedAxis < 6) { // Sicherheitscheck für die Stepper-Indizes
    // Joystick-Wert in Geschwindigkeit umwandeln
    int yValue = joystickLeftY;
    
    // Totzone in der Mitte (470-550)
    if (yValue > 470 && yValue < 550) {
      // Stepper anhalten
      currentmotorspeeds[selectedAxis] = 0;
    } else {
      // Joystick-Wert in Geschwindigkeit umrechnen
      // Mitte (512) = 0, unten (0) = MAX_SPEED negativ, oben (1023) = MAX_SPEED positiv
      float speedFactor = map(yValue, 0, 1023, -MAX_SPEED, MAX_SPEED);
      
      // Kleine Totzone um die Mitte
      if (abs(speedFactor) < (MAX_SPEED * 0.1)) {
        speedFactor = 0;
      }
      
      // Erhöhte Glättung: Nur ein kleinerer Teil der Differenz pro Update ändern
      float speedDifference = speedFactor - currentmotorspeeds[selectedAxis];
      currentmotorspeeds[selectedAxis] += speedDifference * 0.15; // 15% Annäherung (sanfter)
      
      // Bei sehr kleiner Geschwindigkeit komplett stoppen
      if (abs(currentmotorspeeds[selectedAxis]) < 50) {
        currentmotorspeeds[selectedAxis] = 0;
      }
      
    }
    
    // Geschwindigkeit direkt auf den Stepper anwenden
    motors[selectedAxis].setSpeed(currentmotorspeeds[selectedAxis]);
  }
}

void updateJointMode() {
  if (!displayPtr) return;
  
  displayPtr->clearBuffer();
  displayPtr->setFont(u8g2_font_6x10_tf);
  
  // Titelleiste mit Rahmen
  displayPtr->drawFrame(0, 0, 128, 14);
  displayPtr->drawStr(4, 10, "Joint Mode");
  
  // Status anzeigen
  if (axisConfirmed) {
    displayPtr->drawStr(70, 10, "BEWEGUNG");
  } else {
    displayPtr->drawStr(70, 10, "AUSWAHL");
  }
  
  // Scrollbare Achsenanzeige
  // Wir zeigen maximal 3 Achsen gleichzeitig an
  const int visibleAxes = 3;
  const int axisHeight = 12;
  const int totalAxes = 7; // Jetzt 7 Elemente total
  
  // KORRIGIERT: Berechne den ersten anzuzeigenden Index
  // Stelle sicher, dass die ausgewählte Achse immer sichtbar ist
  int startIdx;
  
  if (selectedAxis < visibleAxes) {
    // Wenn wir bei den ersten Achsen sind, zeige von Anfang an
    startIdx = 0;
  } else if (selectedAxis >= totalAxes - 1) {
    // Wenn wir bei der letzten Achse sind, zeige die letzten 3 Achsen
    startIdx = totalAxes - visibleAxes;
  } else {
    // Normalerweise: Zeige die ausgewählte Achse in der Mitte
    startIdx = selectedAxis - 1;
  }
  
  // Sicherheitscheck für die Grenzen
  if (startIdx < 0) startIdx = 0;
  if (startIdx > totalAxes - visibleAxes) startIdx = totalAxes - visibleAxes;
  
  
  // Zeichne Aufwärtspfeil wenn es Achsen vor dem Anzeigebereich gibt
  if (startIdx > 0) {
    displayPtr->drawTriangle(64, 16, 58, 20, 70, 20);
  }
  
  // Achsenpositionen anzeigen
  char buffer[30];
  for (int i = 0; i < visibleAxes && (i + startIdx) < totalAxes; i++) {
    int axisIdx = i + startIdx;
    int yPos = 28 + (i * axisHeight);
    
    // Rahmen für die ausgewählte Achse
    if (axisIdx == selectedAxis) {
      displayPtr->drawRFrame(0, yPos - 9, 128, axisHeight, 2);
      if (axisConfirmed) {
        displayPtr->drawBox(0, yPos - 9, 128, axisHeight);
        displayPtr->setDrawColor(0); // Invertierter Text
      }
    }
    
    // Achsennummer, Name und Position anzeigen
    if (axisIdx == 6) { // Greifer (jetzt Index 6)
      sprintf(buffer, "%d: %s %d°", axisIdx + 1, axisNames[axisIdx], servoPosition);
    } else if (axisIdx < 6) { // Normale Stepper-Achsen
      float degree = stepsToDegree(axisIdx, motors[axisIdx].currentPosition());
      sprintf(buffer, "%d: %s %.1f°", axisIdx + 1, axisNames[axisIdx], degree);
    }
    
    displayPtr->drawStr(4, yPos, buffer);
    
    // Zurück zu normaler Textfarbe
    displayPtr->setDrawColor(1);
  }
  
  // Zeichne Abwärtspfeil wenn es Achsen nach dem Anzeigebereich gibt
  if (startIdx + visibleAxes < totalAxes) {
    displayPtr->drawTriangle(64, 50, 58, 46, 70, 46);
  }
  
  // Statuszeile mit Bedienungshinweisen
  displayPtr->drawHLine(0, 52, 128);
  if (axisConfirmed) {
    if (selectedAxis == 6) { // Greifer-Modus (jetzt Index 6)
      displayPtr->drawStr(2, 62, "L-Joy: Greifer | R-Btn: Zurück");
    } else {
      char speedInfo[30];
      sprintf(speedInfo, "Speed: %.0f", currentmotorspeeds[selectedAxis]);
      displayPtr->drawStr(2, 62, speedInfo);
    }
  } else {
    displayPtr->drawStr(2, 62, "R-Joy: Wählen | R-Btn: OK");
  }
  
  displayPtr->sendBuffer();
}
