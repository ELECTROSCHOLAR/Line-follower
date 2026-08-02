#include <TFT_eSPI.h>
#include <Adafruit_NeoPixel.h>

TFT_eSPI tft = TFT_eSPI();

// ===== PIN DEFINITIONS =====
#define BTN_START 19
#define BTN_UP    21
#define BTN_DOWN  5

// IR Sensor Pins
#define IR1 36
#define IR2 39
#define IR3 34
#define IR4 35
#define IR5 32
#define IR6 33

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
int baseSpeed = 150;
float Kp = 0.5;
float Ki = 0.0;
float Kd = 1.0;

float lastError = 0;
float integral = 0;
int s1, s2, s3, s4, s5, s6;

// Sequence Variables
int startMagnetCount = 0; // Editable in menu to resume specific tasks
int magnetCount = 0;
unsigned long lastHallDetectTime = 0;
const int HALL_COOLDOWN = 1500; 
int ROTATION_TIME = 600;  // Editable in menu

// Dual Calibration Thresholds
int whiteLevel[6] = {4095, 4095, 4095, 4095, 4095, 4095};
int blackLevel[6] = {0, 0, 0, 0, 0, 0};
int sensorThreshold[6] = {2000, 2000, 2000, 2000, 2000, 2000};

// Menu & State Machine
enum SystemState { STATE_MENU, STATE_RUN, STATE_IR_VIEW, STATE_CALIB_WHITE, STATE_CALIB_BLACK, STATE_LED_TEST };
SystemState currentState = STATE_MENU;

int menuIndex = 0;
const int MENU_ITEMS = 12; 
bool isEditing = false;
bool redrawMenu = true;

// Calibration Sub-State Control
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
    analogWrite(pwmPin, constrain(speed, 0, 255));
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
        "Calibrate Dual", 
        "LED Test", 
        "Rot Time",
        "Test Spin",
        "Threshold", 
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
        
        // Only print values for items that have them
        if (i == 1 || i == 5 || i >= 7) {
            tft.print(": ");
            switch(i) {
                case 1: tft.println(startMagnetCount); break;
                case 5: tft.println(ROTATION_TIME); break;
                case 7: tft.println("DYNAMIC"); break;
                case 8: tft.println(baseSpeed); break;
                case 9: tft.println(Kp, 3); break;
                case 10: tft.println(Ki, 4); break;
                case 11: tft.println(Kd, 3); break;
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
                    
                    // Reset variables
                    integral = 0;
                    lastError = 0;
                    magnetCount = startMagnetCount; // Start at the user-defined task
                    
                    // Pre-load Info LED based on Starting Task
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
                else if (menuIndex == 3) {
                    currentState = STATE_CALIB_WHITE;
                    tft.fillScreen(TFT_BLACK);
                    tft.setCursor(0,0);
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);
                    tft.println("STEP 1: WHITE");
                    tft.println("Place on WHITE");
                    tft.println("Press START");
                }
                else if (menuIndex == 4) {
                    currentState = STATE_LED_TEST;
                    tft.fillScreen(TFT_BLACK);
                    tft.setTextSize(2);
                    tft.setCursor(0, 0);
                    tft.setTextColor(TFT_CYAN, TFT_BLACK);
                    tft.println("-- LED TEST MODE --");
                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.println("\n[START] to Exit");
                }
                else if (menuIndex == 6) {
                    // Test Spin Mode
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
                else if (menuIndex == 1 || menuIndex == 5 || menuIndex >= 8) {
                    // Only allow editing for variables that can actually be changed
                    isEditing = true; 
                    redrawMenu = true; 
                }
            }
        } else {
            if (pressedStart) { 
                isEditing = false; 
                redrawMenu = true; 
            }
            
            if (pressedUp) {
                switch(menuIndex) {
                    case 1: startMagnetCount++; if(startMagnetCount > 5) startMagnetCount = 5; break;
                    case 5: ROTATION_TIME += 10; break;
                    case 8: baseSpeed += 10; break;
                    case 9: Kp += 0.01; break;
                    case 10: Ki += 0.001; break;
                    case 11: Kd += 0.01; break;
                }
                redrawMenu = true;
            }
            if (pressedDown) {
                switch(menuIndex) {
                    case 1: startMagnetCount--; if(startMagnetCount < 0) startMagnetCount = 0; break;
                    case 5: ROTATION_TIME -= 10; if(ROTATION_TIME < 0) ROTATION_TIME = 0; break;
                    case 8: baseSpeed -= 10; break;
                    case 9: Kp -= 0.01; break;
                    case 10: Ki -= 0.001; break;
                    case 11: Kd -= 0.01; break;
                }
                redrawMenu = true;
            }
        }
    } 
    else if (currentState == STATE_CALIB_WHITE) {
        if (pressedStart) {
            whiteLevel[0] = analogRead(IR1); whiteLevel[1] = analogRead(IR2); whiteLevel[2] = analogRead(IR3);
            whiteLevel[3] = analogRead(IR4); whiteLevel[4] = analogRead(IR5); whiteLevel[5] = analogRead(IR6);

            currentState = STATE_CALIB_BLACK;
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(0,0);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.println("STEP 2: BLACK");
            tft.println("Place on BLACK");
            tft.println("Press START");
        }
    }
    else if (currentState == STATE_CALIB_BLACK) {
        if (pressedStart) {
            blackLevel[0] = analogRead(IR1); blackLevel[1] = analogRead(IR2); blackLevel[2] = analogRead(IR3);
            blackLevel[3] = analogRead(IR4); blackLevel[4] = analogRead(IR5); blackLevel[5] = analogRead(IR6);

            for(int i = 0; i < 6; i++) {
                sensorThreshold[i] = (whiteLevel[i] + blackLevel[i]) / 2;
            }

            tft.fillScreen(TFT_BLACK);
            tft.setCursor(0,0);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.println("CALIB SUCCESS!");
            delay(1500);

            currentState = STATE_MENU;
            redrawMenu = true;
        }
    }
    else {
        // Exit from RUN, IR_VIEW, or LED_TEST modes
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
    // --- 5-SECOND HARDWARE RESET CHECK ---
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
    else if (currentState == STATE_LED_TEST) {
        static unsigned long lastLedUpdate = 0;
        static long firstPixelHue = 0;
        
        if (millis() - lastLedUpdate > 10) { 
            strip.setPixelColor(0, strip.ColorHSV(firstPixelHue));
            strip.setPixelColor(1, strip.ColorHSV(firstPixelHue));
            strip.setPixelColor(2, strip.Color(255, 165, 0)); 
            strip.show();
            
            firstPixelHue += 256; 
            if (firstPixelHue >= 65536) firstPixelHue = 0; 
            
            lastLedUpdate = millis();
        }
        yield();
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

        // ---- PID Execution Block ----
        int raw[6] = {analogRead(IR1), analogRead(IR2), analogRead(IR3), 
                      analogRead(IR4), analogRead(IR5), analogRead(IR6)};

        bool lineDetected[6];
        for(int i = 0; i < 6; i++) {
            if (whiteLevel[i] > blackLevel[i]) {
                lineDetected[i] = (raw[i] < sensorThreshold[i]); 
            } else {
                lineDetected[i] = (raw[i] > sensorThreshold[i]); 
            }
        }

        bool r  = lineDetected[0]; 
        bool r2 = lineDetected[1];
        bool c1 = lineDetected[2]; 
        bool c2 = lineDetected[3];
        bool l2 = lineDetected[4]; 
        bool l  = lineDetected[5];

        // --- END ZONE LOGIC ---
        if (magnetCount >= 5 && !l && !l2 && !c1 && !c2 && !r2 && !r) {
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            currentState = STATE_MENU;
            strip.clear();
            strip.show();
            redrawMenu = true;
            return; 
        }

        float weightSum = 0; float activeSum = 0;

        if (l)  { weightSum += -3; activeSum += 1; }
        if (l2) { weightSum += -2; activeSum += 1; }
        if (c1) { weightSum += 0;  activeSum += 1; }
        if (c2) { weightSum += 0;  activeSum += 1; }
        if (r2) { weightSum += 2;  activeSum += 1; }
        if (r)  { weightSum += 3;  activeSum += 1; }

        float error;
        if (activeSum > 0) {
            error = weightSum / activeSum;
        } else {
            error = (lastError > 0) ? 3.0 : -3.0;
        }

        integral += error;
        integral = constrain(integral, -1000, 1000);
        float derivative = error - lastError;
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;

        int leftSpeed  = baseSpeed + output * 100;
        int rightSpeed = baseSpeed - output * 100;

        leftSpeed  = constrain(leftSpeed, 0, 205);
        rightSpeed = constrain(rightSpeed, 0, 205);

        setMotor(AIN1, AIN2, PWMA, rightSpeed);
        setMotor(BIN1, BIN2, PWMB, leftSpeed);
    }
    delay(5);
    yield(); 
}