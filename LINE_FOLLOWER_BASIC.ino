#include <TFT_eSPI.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

// ===== PIN DEFINITIONS =====
#define BTN_START 19
#define BTN_UP    21
#define BTN_DOWN  5

// IR Sensor Pins (Physical Layout: Left -> Right)
#define IR1 36 // L1 (Far Left)
#define IR2 39 // L2 (Inner Left)
#define IR3 34 // C1 (Center Front)
#define IR4 35 // C2 (Center Back)
#define IR5 32 // R2 (Inner Right)
#define IR6 33 // R1 (Far Right)

// Motor Driver Pins (TB6612FNG)
#define PWMA 14
#define AIN1 26
#define AIN2 25
#define STBY 27
#define PWMB 15
#define BIN1 13
#define BIN2 12

// Hardware Pins
#define PIN_BUZZER 4
#define PIN_HALL 16
#define PIN_RGB 0

// ===== WS2812 LED CONFIGURATION =====
#define NUM_LEDS 3
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN_RGB, NEO_GRB + NEO_KHZ800);

// ===== SYSTEM VARIABLES =====
int baseSpeed;
float Kp, Ki, Kd;

float lastError = 0;
float integral = 0;
int s1, s2, s3, s4, s5, s6;

// Sequence Variables
int startMagnetCount; 
int magnetCount = 0;
unsigned long lastHallDetectTime = 0;
const int HALL_COOLDOWN = 1500; 
int ROTATION_TIME;

// Manual Thresholds
int manualBlackThresh;
int manualWhiteThresh;

// Line Logic Tracking
bool whiteLineOnBlackBg = false; 

// Menu & State Machine
enum SystemState { STATE_MENU, STATE_RUN, STATE_IR_VIEW, STATE_LED_TEST, STATE_MAG_TEST };
SystemState currentState = STATE_MENU;

int menuIndex = 0;
const int MENU_ITEMS = 13; // Increased for Mag Test
bool isEditing = false;
bool redrawMenu = true;

unsigned long startButtonHeldTime = 0; 
int lastDisplayedMode = -1; // Track displayed logic (0 = Black, 1 = White)

// ===== ROBUST DEBOUNCE LOGIC =====
struct Button {
    uint8_t pin;
    bool lastReading;
    bool stableState;
    unsigned long lastDebounceTime;
};

Button btnStart = {BTN_START, HIGH, HIGH, 0};
Button btnUp    = {BTN_UP, HIGH, HIGH, 0};
Button btnDown  = {BTN_DOWN, HIGH, HIGH, 0};
const int debounceDelay = 150; 

bool checkPress(Button &b) {
    bool currentReading = digitalRead(b.pin);
    bool isPressed = false;
    
    if (currentReading != b.lastReading) {
        b.lastDebounceTime = millis();
    }
    
    if ((millis() - b.lastDebounceTime) > debounceDelay) {
        if (currentReading != b.stableState) {
            b.stableState = currentReading;
            if (b.stableState == LOW) {
                isPressed = true;
            }
        }
    }
    b.lastReading = currentReading;
    return isPressed;
}

// ===== HELPER FUNCTIONS =====

void setMotor(int in1, int in2, int pwmPin, int speed) {
    if (speed >= 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speed = -speed;
    }
    analogWrite(pwmPin, constrain(speed, 0, 220));
}

// ===== MENU LOGIC =====

void updateMenuDisplay() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("--- SYSTEM MENU ---");

    const char* items[] = {
        "Start Run Mode", 
        "Start Task",
        "IR View", 
        "Black Thresh", 
        "White Thresh",
        "Rot Time",
        "Test Spin",
        "LED Test",       
        "Test Magnet",    // New Option
        "Base Speed", 
        "Kp", 
        "Ki", 
        "Kd"
    };
    
    for (int i = 0; i < MENU_ITEMS; i++) {
        if (i == menuIndex) {
            tft.setTextColor(TFT_BLACK, TFT_YELLOW); 
            if (isEditing) tft.setTextColor(TFT_BLACK, TFT_RED); 
        } else {
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
        }
        
        tft.print(items[i]);
        
        if (i == 1 || (i >= 3 && i <= 5) || i >= 9) {
            tft.print(": ");
            switch(i) {
                case 1: tft.println(startMagnetCount); break;
                case 3: tft.println(manualBlackThresh); break;
                case 4: tft.println(manualWhiteThresh); break;
                case 5: tft.println(ROTATION_TIME); break;
                case 9: tft.println(baseSpeed); break;
                case 10: tft.println(Kp, 3); break;
                case 11: tft.println(Ki, 4); break;
                case 12: tft.println(Kd, 3); break;
            }
        } else {
            tft.println();
        }
        yield(); 
    }
    redrawMenu = false;
}

void processButtons() {
    bool pressedStart = checkPress(btnStart);
    bool pressedUp    = checkPress(btnUp);
    bool pressedDown  = checkPress(btnDown);

    if (currentState == STATE_MENU) {
        if (!isEditing) {
            if (pressedDown) { menuIndex = (menuIndex + 1) % MENU_ITEMS; redrawMenu = true; }
            if (pressedUp)   { menuIndex = (menuIndex - 1 + MENU_ITEMS) % MENU_ITEMS; redrawMenu = true; }
            
            if (pressedStart) { 
                if (menuIndex == 0) {
                    currentState = STATE_RUN;
                    tft.fillScreen(TFT_BLACK);
                    
                    integral = 0;
                    lastError = 0;
                    magnetCount = startMagnetCount; 
                    lastDisplayedMode = -1; // Reset screen state tracker
                    whiteLineOnBlackBg = false; // Default baseline on start
                    
                    strip.clear();
                    if (magnetCount == 2) strip.setPixelColor(2, strip.Color(255, 0, 0));
                    else if (magnetCount == 3) strip.setPixelColor(2, strip.Color(0, 255, 0));
                    else if (magnetCount >= 4) strip.setPixelColor(2, strip.Color(0, 0, 255));
                    strip.show();
                } 
                else if (menuIndex == 2) {
                    currentState = STATE_IR_VIEW;
                    tft.fillScreen(TFT_BLACK);
                }
                else if (menuIndex == 6) {
                    tft.fillScreen(TFT_BLACK);
                    tft.setTextSize(3);
                    tft.setTextColor(TFT_CYAN, TFT_BLACK);
                    tft.setCursor(10, 100);
                    tft.println("TEST SPIN...");
                    delay(2000);
                    setMotor(AIN1, AIN2, PWMA, 150); 
                    setMotor(BIN1, BIN2, PWMB, -150);
                    delay(ROTATION_TIME);
                    setMotor(AIN1, AIN2, PWMA, 0); 
                    setMotor(BIN1, BIN2, PWMB, 0);
                    redrawMenu = true;
                }
                else if (menuIndex == 7) { 
                    currentState = STATE_LED_TEST;
                    tft.fillScreen(TFT_BLACK);
                }
                else if (menuIndex == 8) { // New Magnet Test Trigger
                    currentState = STATE_MAG_TEST;
                    tft.fillScreen(TFT_BLACK);
                }
                else if (menuIndex == 1 || (menuIndex >= 3 && menuIndex <= 5) || menuIndex >= 9) {
                    isEditing = true; 
                    redrawMenu = true; 
                }
            }
        } else {
            if (pressedStart) { 
                isEditing = false; 
                switch(menuIndex) {
                    case 1: prefs.putInt("startMag", startMagnetCount); break;
                    case 3: prefs.putInt("blkThresh", manualBlackThresh); break;
                    case 4: prefs.putInt("whtThresh", manualWhiteThresh); break;
                    case 5: prefs.putInt("rotTime", ROTATION_TIME); break;
                    case 9: prefs.putInt("baseSpeed", baseSpeed); break;
                    case 10: prefs.putFloat("Kp", Kp); break;
                    case 11: prefs.putFloat("Ki", Ki); break;
                    case 12: prefs.putFloat("Kd", Kd); break;
                }
                redrawMenu = true; 
            }
            
            if (pressedUp) {
                switch(menuIndex) {
                    case 1: startMagnetCount++; if(startMagnetCount > 5) startMagnetCount = 5; break;
                    case 3: manualBlackThresh += 50; if(manualBlackThresh > 4095) manualBlackThresh = 4095; break;
                    case 4: manualWhiteThresh += 50; if(manualWhiteThresh > 4095) manualWhiteThresh = 4095; break;
                    case 5: ROTATION_TIME += 10; break;
                    case 9: baseSpeed += 10; break;
                    case 10: Kp += 0.01; break;
                    case 11: Ki += 0.001; break;
                    case 12: Kd += 0.01; break;
                }
                redrawMenu = true;
            }
            if (pressedDown) {
                switch(menuIndex) {
                    case 1: startMagnetCount--; if(startMagnetCount < 0) startMagnetCount = 0; break;
                    case 3: manualBlackThresh -= 50; if(manualBlackThresh < 0) manualBlackThresh = 0; break;
                    case 4: manualWhiteThresh -= 50; if(manualWhiteThresh < 0) manualWhiteThresh = 0; break;
                    case 5: ROTATION_TIME -= 10; if(ROTATION_TIME < 0) ROTATION_TIME = 0; break;
                    case 9: baseSpeed -= 10; break;
                    case 10: Kp -= 0.01; break;
                    case 11: Ki -= 0.001; break;
                    case 12: Kd -= 0.01; break;
                }
                redrawMenu = true;
            }
        }
    } 
    else {
        // Exit any sub-state
        if (pressedStart) {
            currentState = STATE_MENU;
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            digitalWrite(PIN_BUZZER, LOW); // Stop buzzer if it was running
            strip.clear(); 
            strip.show();
            redrawMenu = true;
        }
    }
    yield(); 
}

// ===== MAIN ROUTINES =====

void setup() {
    delay(2000); 
    
    prefs.begin("robot", false);
    startMagnetCount = prefs.getInt("startMag", 0);
    manualBlackThresh = prefs.getInt("blkThresh", 3000);
    manualWhiteThresh = prefs.getInt("whtThresh", 1000);
    ROTATION_TIME = prefs.getInt("rotTime", 600);
    
    baseSpeed = prefs.getInt("baseSpeed", 130);
    Kp = prefs.getFloat("Kp", 0.45);
    Ki = prefs.getFloat("Ki", 0.00);
    Kd = prefs.getFloat("Kd", 0.25);
    
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

    pinMode(IR1, INPUT); pinMode(IR2, INPUT); pinMode(IR3, INPUT);
    pinMode(IR4, INPUT); pinMode(IR5, INPUT); pinMode(IR6, INPUT);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); 
    pinMode(PIN_HALL, INPUT_PULLUP); 

    strip.begin();
    strip.show(); 
    strip.setBrightness(50); 

    tft.init();
    tft.setRotation(1);
    updateMenuDisplay();
}

void loop() {
    if (digitalRead(BTN_START) == LOW && (currentState == STATE_MENU)) {
        if (startButtonHeldTime == 0) {
            startButtonHeldTime = millis();
        } 
        else if (millis() - startButtonHeldTime > 5000) {
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            tft.fillScreen(TFT_RED);
            tft.setTextColor(TFT_WHITE);
            tft.setTextSize(3);
            tft.setCursor(20, 100);
            tft.println("SYSTEM RESET");
            delay(1000);
            ESP.restart(); 
        }
    } else {
        startButtonHeldTime = 0; 
    }

    processButtons();

    if (currentState == STATE_MENU) {
        if (redrawMenu) updateMenuDisplay();
    } 
    else if (currentState == STATE_IR_VIEW) {
        static unsigned long lastDebugRefresh = 0;
        if (millis() - lastDebugRefresh > 300) {
            s1 = analogRead(IR1); s2 = analogRead(IR2); s3 = analogRead(IR3);
            s4 = analogRead(IR4); s5 = analogRead(IR5); s6 = analogRead(IR6);

            tft.setTextSize(2);
            tft.setCursor(0, 0);
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.println("-- IR SENSOR VIEW --\n");
            
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.printf("L1 (IR1): %04d  L2 (IR2): %04d\n", s1, s2);
            tft.printf("C1 (IR3): %04d  C2 (IR4): %04d\n", s3, s4);
            tft.printf("R2 (IR5): %04d  R1 (IR6): %04d\n", s5, s6);
            
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.println("\n[START] to Exit     ");
            
            lastDebugRefresh = millis();
        }
        yield(); 
    }
    else if (currentState == STATE_MAG_TEST) { // NEW: MAGNET TEST
        static unsigned long lastMagRefresh = 0;
        if (millis() - lastMagRefresh > 150) {
            bool magDetected = (digitalRead(PIN_HALL) == LOW);
            
            tft.setTextSize(2);
            tft.setCursor(10, 40);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.println("-- HALL SENSOR TEST --\n");
            
            tft.setCursor(10, 80);
            tft.setTextSize(3);
            if (magDetected) {
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.println("DETECTED!   ");
                digitalWrite(PIN_BUZZER, HIGH); // Sound buzzer
                strip.setPixelColor(0, strip.Color(0, 255, 0)); 
                strip.setPixelColor(1, strip.Color(0, 255, 0));
                strip.setPixelColor(2, strip.Color(0, 255, 0));
            } else {
                tft.setTextColor(TFT_RED, TFT_BLACK);
                tft.println("NO MAGNET   ");
                digitalWrite(PIN_BUZZER, LOW);
                strip.clear();
            }
            strip.show();
            
            tft.setTextSize(2);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(10, 140);
            tft.println("\n[START] to Exit");
            
            lastMagRefresh = millis();
        }
        yield();
    }
    else if (currentState == STATE_LED_TEST) { 
        static unsigned long lastLedStep = 0;
        static int ledStep = 0;

        if (millis() - lastLedStep > 500) {
            tft.setTextSize(2);
            tft.setCursor(10, 40);
            tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
            tft.println("-- WS2812 LED TEST --\n");
            
            strip.clear();
            if (ledStep == 0) {
                tft.setTextColor(TFT_RED, TFT_BLACK);
                tft.println("COLOR: RED    ");
                for(int i=0; i<NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
            } else if (ledStep == 1) {
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.println("COLOR: GREEN  ");
                for(int i=0; i<NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
            } else if (ledStep == 2) {
                tft.setTextColor(TFT_BLUE, TFT_BLACK);
                tft.println("COLOR: BLUE   ");
                for(int i=0; i<NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(0, 0, 255));
            } else if (ledStep == 3) {
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.println("COLOR: WHITE  ");
                for(int i=0; i<NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(255, 255, 255));
            }
            strip.show();

            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.println("\n[START] to Exit");

            ledStep = (ledStep + 1) % 4;
            lastLedStep = millis();
        }
        yield();
    }
    else if (currentState == STATE_RUN) {
        
        // --- MAGNET SEQUENCE LOGIC ---
        if (digitalRead(PIN_HALL) == LOW && (millis() - lastHallDetectTime > HALL_COOLDOWN)) {
            lastHallDetectTime = millis();
            magnetCount++;

            if (magnetCount == 1) {
                setMotor(AIN1, AIN2, PWMA, 0);
                setMotor(BIN1, BIN2, PWMB, 0);
                
                for (int i = 0; i < 5; i++) {
                    digitalWrite(PIN_BUZZER, HIGH);
                    delay(100);
                    digitalWrite(PIN_BUZZER, LOW);
                    delay(900);
                    yield(); 
                }
            } 
            else if (magnetCount == 2) {
                strip.setPixelColor(2, strip.Color(255, 0, 0)); 
                strip.show();
            }
            else if (magnetCount == 3) {
                strip.setPixelColor(2, strip.Color(0, 255, 0)); 
                strip.show();
            }
            else if (magnetCount == 4) {
                strip.setPixelColor(2, strip.Color(0, 0, 255)); 
                strip.show();
            }
            else if (magnetCount == 5) {
                setMotor(AIN1, AIN2, PWMA, 150); 
                setMotor(BIN1, BIN2, PWMB, -150);
                delay(ROTATION_TIME); 
                setMotor(AIN1, AIN2, PWMA, 0); 
                setMotor(BIN1, BIN2, PWMB, 0);
            }
        }

        // ---- NEW DYNAMIC LINE DETECTION BLOCK ----
        int raw[6] = {analogRead(IR1), analogRead(IR2), analogRead(IR3), 
                      analogRead(IR4), analogRead(IR5), analogRead(IR6)};

        bool farLeftIsBlack  = (raw[0] > manualBlackThresh);
        bool farRightIsBlack = (raw[5] > manualBlackThresh);
        bool farLeftIsWhite  = (raw[0] < manualWhiteThresh);
        bool farRightIsWhite = (raw[5] < manualWhiteThresh);

        // State update based on outermost sensors
        if (farLeftIsBlack && farRightIsBlack) {
            whiteLineOnBlackBg = true;  // Track White line logic
        } 
        else if (farLeftIsWhite && farRightIsWhite) {
            whiteLineOnBlackBg = false; // Track Black line logic
        }
        // Else: If they are mixed (e.g. crossing a line or corner), keep the last known logic (do nothing).

        // --- TFT DISPLAY OF ACTIVE LINE LOGIC ---
        int currentMode = whiteLineOnBlackBg ? 1 : 0;
        if (currentMode != lastDisplayedMode) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(10, 80);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.println("RUNNING.......");
            
            tft.setTextSize(2);
            tft.setCursor(10, 130);
            if (whiteLineOnBlackBg) {
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.println("MODE: WHITE LOGIC");
            } else {
                tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                tft.println("MODE: BLACK LOGIC");
            }
            lastDisplayedMode = currentMode;
        }

        bool lineDetected[6];
        for(int i = 0; i < 6; i++) {
            if (whiteLineOnBlackBg) {
                lineDetected[i] = (raw[i] < manualWhiteThresh);
            } else {
                lineDetected[i] = (raw[i] > manualBlackThresh);
            }
        }

        // PHYSICAL MAPPING: Left-to-Right
        bool l1_farLeft    = lineDetected[0]; // IR1 (Far Left)
        bool l2_innerLeft  = lineDetected[1]; // IR2 (Inner Left)
        bool c1_centerFwd  = lineDetected[2]; // IR3 (Center Front)
        bool c2_centerBack = lineDetected[3]; // IR4 (Center Back)
        bool r2_innerRight = lineDetected[4]; // IR5 (Inner Right)
        bool r1_farRight   = lineDetected[5]; // IR6 (Far Right)

        // --- END ZONE LOGIC ---
        if (magnetCount >= 5 && !l1_farLeft && !l2_innerLeft && !c1_centerFwd && !c2_centerBack && !r2_innerRight && !r1_farRight) {
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            currentState = STATE_MENU;
            strip.clear();
            strip.show();
            redrawMenu = true;
            return; 
        }

        // ---- GEOMETRY-CALIBRATED ERROR LOGIC ----
        float error = 0.0;
        
        // 1. Dead Center Tracking
        if ((c1_centerFwd || c2_centerBack) && (!l1_farLeft && !r1_farRight)) {
            if (l2_innerLeft && !r2_innerRight)       error = -0.4; // Drifting Right, turn Left
            else if (!l2_innerLeft && r2_innerRight)  error = 0.4;  // Drifting Left, turn Right
            else                                      error = 0.0;  // Dead center
        }
        // 2. Mild Curves
        else if (l2_innerLeft && !r1_farRight) {
            error = -1.2; // Turn Left
        }
        else if (r2_innerRight && !l1_farLeft) {
            error = 1.2;  // Turn Right
        }
        // 3. Sharp 90-Degree Turns
        else if (l1_farLeft) {
            error = -2.8; // Hard Left
        }
        else if (r1_farRight) {
            error = 2.8;  // Hard Right
        }
        // 4. Line Lost Recovery
        else {
            if (lastError < -0.5)       error = -2.0; // Keep turning Left
            else if (lastError > 0.5)   error = 2.0;  // Keep turning Right
            else                        error = 0.0;  
        }

        // --- PID CALCULATION ---
        integral += error;
        integral = constrain(integral, -300, 300);
        float derivative = error - lastError;
        
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;

        // --- OPPOSITE DIRECTION MOTOR SPEED CALCULATION ---
        int leftSpeed  = baseSpeed - (output * 30);
        int rightSpeed = baseSpeed + (output * 30);

        // Constrained between -120 (active pivot reverse) and 220 max PWM
        leftSpeed  = constrain(leftSpeed, -120, 220);
        rightSpeed = constrain(rightSpeed, -120, 220);

        setMotor(AIN1, AIN2, PWMA, rightSpeed);
        setMotor(BIN1, BIN2, PWMB, leftSpeed);
    }
    delay(5);
    yield(); 
}