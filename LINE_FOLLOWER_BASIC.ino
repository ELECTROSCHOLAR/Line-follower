#include <TFT_eSPI.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

// ===== PIN DEFINITIONS =====
#define BTN_START 19
#define BTN_UP    21
#define BTN_DOWN  5

// IR Sensor Pins
#define IR1 36 // R1 (Far Left)
#define IR2 39 // R2 (Inner Left)
#define IR3 34 // C1 (Center Front)
#define IR4 35 // C2 (Center Back)
#define IR5 32 // L2 (Inner Right)
#define IR6 33 // L1 (Far Right)

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

// Menu & State Machine
enum SystemState { STATE_MENU, STATE_RUN, STATE_IR_VIEW, STATE_LED_TEST };
SystemState currentState = STATE_MENU;

int menuIndex = 0;
const int MENU_ITEMS = 11;
bool isEditing = false;
bool redrawMenu = true;

unsigned long startButtonHeldTime = 0; 

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
        
        if (i == 1 || (i >= 3 && i <= 5) || i >= 7) {
            tft.print(": ");
            switch(i) {
                case 1: tft.println(startMagnetCount); break;
                case 3: tft.println(manualBlackThresh); break;
                case 4: tft.println(manualWhiteThresh); break;
                case 5: tft.println(ROTATION_TIME); break;
                case 7: tft.println(baseSpeed); break;
                case 8: tft.println(Kp, 3); break;
                case 9: tft.println(Ki, 4); break;
                case 10: tft.println(Kd, 3); break;
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
                    tft.setTextSize(3);
                    tft.setTextColor(TFT_GREEN, TFT_BLACK);
                    tft.setCursor(40, 100);
                    tft.println("Running.......");
                    
                    integral = 0;
                    lastError = 0;
                    magnetCount = startMagnetCount; 
                    
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
                else if (menuIndex == 1 || (menuIndex >= 3 && menuIndex <= 5) || menuIndex >= 7) {
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
                    case 7: prefs.putInt("baseSpeed", baseSpeed); break;
                    case 8: prefs.putFloat("Kp", Kp); break;
                    case 9: prefs.putFloat("Ki", Ki); break;
                    case 10: prefs.putFloat("Kd", Kd); break;
                }
                redrawMenu = true; 
            }
            
            if (pressedUp) {
                switch(menuIndex) {
                    case 1: startMagnetCount++; if(startMagnetCount > 5) startMagnetCount = 5; break;
                    case 3: manualBlackThresh += 50; if(manualBlackThresh > 4095) manualBlackThresh = 4095; break;
                    case 4: manualWhiteThresh += 50; if(manualWhiteThresh > 4095) manualWhiteThresh = 4095; break;
                    case 5: ROTATION_TIME += 10; break;
                    case 7: baseSpeed += 10; break;
                    case 8: Kp += 0.01; break;
                    case 9: Ki += 0.001; break;
                    case 10: Kd += 0.01; break;
                }
                redrawMenu = true;
            }
            if (pressedDown) {
                switch(menuIndex) {
                    case 1: startMagnetCount--; if(startMagnetCount < 0) startMagnetCount = 0; break;
                    case 3: manualBlackThresh -= 50; if(manualBlackThresh < 0) manualBlackThresh = 0; break;
                    case 4: manualWhiteThresh -= 50; if(manualWhiteThresh < 0) manualWhiteThresh = 0; break;
                    case 5: ROTATION_TIME -= 10; if(ROTATION_TIME < 0) ROTATION_TIME = 0; break;
                    case 7: baseSpeed -= 10; break;
                    case 8: Kp -= 0.01; break;
                    case 9: Ki -= 0.001; break;
                    case 10: Kd -= 0.01; break;
                }
                redrawMenu = true;
            }
        }
    } 
    else {
        if (pressedStart) {
            currentState = STATE_MENU;
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
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
    
    // Calibrated defaults for 7.4V battery with N20 motors
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
            tft.printf("R1: %04d  R2: %04d\n", s1, s2);
            tft.printf("C1: %04d  C2: %04d\n", s3, s4);
            tft.printf("L2: %04d  L1: %04d\n", s5, s6);
            
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.println("\n[START] to Exit     ");
            
            lastDebugRefresh = millis();
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

        // ---- DYNAMIC LINE DETECTION BLOCK ----
        int raw[6] = {analogRead(IR1), analogRead(IR2), analogRead(IR3), 
                      analogRead(IR4), analogRead(IR5), analogRead(IR6)};

        int outerBlackCount = 0;
        if (raw[0] > manualBlackThresh) outerBlackCount++;
        if (raw[1] > manualBlackThresh) outerBlackCount++;
        if (raw[4] > manualBlackThresh) outerBlackCount++;
        if (raw[5] > manualBlackThresh) outerBlackCount++;

        bool whiteLineOnBlackBg = (outerBlackCount >= 3);

        bool lineDetected[6];
        for(int i = 0; i < 6; i++) {
            if (whiteLineOnBlackBg) {
                lineDetected[i] = (raw[i] < manualWhiteThresh);
            } else {
                lineDetected[i] = (raw[i] > manualBlackThresh);
            }
        }

        // PHYSICAL MAPPING: Left-to-Right
        bool r1_farLeft    = lineDetected[0]; // IR1 (Far Left, x = -22.5mm)
        bool r2_innerLeft  = lineDetected[1]; // IR2 (Inner Left, x = -12.5mm)
        bool c1_centerFwd  = lineDetected[2]; // IR3 (Center Front)
        bool c2_centerBack = lineDetected[3]; // IR4 (Center Back)
        bool l2_innerRight = lineDetected[4]; // IR5 (Inner Right, x = +12.5mm)
        bool l1_farRight   = lineDetected[5]; // IR6 (Far Right, x = +22.5mm)

        // --- END ZONE LOGIC ---
        if (magnetCount >= 5 && !r1_farLeft && !r2_innerLeft && !c1_centerFwd && !c2_centerBack && !l2_innerRight && !l1_farRight) {
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
        if ((c1_centerFwd || c2_centerBack) && (!r1_farLeft && !l1_farRight)) {
            if (r2_innerLeft && !l2_innerRight)       error = -0.4; 
            else if (!r2_innerLeft && l2_innerRight)  error = 0.4;  
            else                                      error = 0.0;  
        }
        // 2. Mild Curves
        else if (r2_innerLeft && !l1_farRight) {
            error = -1.2; 
        }
        else if (l2_innerRight && !r1_farLeft) {
            error = 1.2;  
        }
        // 3. Sharp 90-Degree Turns
        else if (r1_farLeft) {
            error = -2.8; 
        }
        else if (l1_farRight) {
            error = 2.8;  
        }
        // 4. Line Lost Recovery
        else {
            if (lastError > 0.5)        error = 2.0;  
            else if (lastError < -0.5)  error = -2.0; 
            else                        error = 0.0;  
        }

        // --- PID CALCULATION ---
        integral += error;
        integral = constrain(integral, -300, 300);
        float derivative = error - lastError;
        
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;

        // --- MOTOR SPEED CALCULATION (Scaled for 7.4V Battery) ---
        int leftSpeed  = baseSpeed + (output * 30);
        int rightSpeed = baseSpeed - (output * 30);

        // Constrained between -120 (active pivot reverse) and 220 max PWM limit
        leftSpeed  = constrain(leftSpeed, -120, 220);
        rightSpeed = constrain(rightSpeed, -120, 220);

        setMotor(AIN1, AIN2, PWMA, rightSpeed);
        setMotor(BIN1, BIN2, PWMB, leftSpeed);
    }
    delay(5);
    yield(); 
}