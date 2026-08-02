#include <TFT_eSPI.h>

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

// New Hardware Pins
#define PIN_BUZZER 4
#define PIN_HALL 16

// ===== SYSTEM VARIABLES =====
int baseSpeed = 150;
float Kp = 0.5;
float Ki = 0.0;
float Kd = 1.0;

float lastError = 0;
float integral = 0;
int s1, s2, s3, s4, s5, s6;

// Dual Calibration Thresholds for each sensor (Index 0-5)
int whiteLevel[6] = {4095, 4095, 4095, 4095, 4095, 4095};
int blackLevel[6] = {0, 0, 0, 0, 0, 0};
int sensorThreshold[6] = {2000, 2000, 2000, 2000, 2000, 2000};

// Buzzer State Machine Variables
unsigned long buzzerTimer = 0;
unsigned long lastHallDetectTime = 0;
int beepState = 0; 
const int BEEP_DURATION = 100; // duration of a single beep in ms
const int BEEP_GAP = 100;      // gap between double beeps in ms
const int HALL_COOLDOWN = 1000;// cooldown to prevent rapid re-triggering

// Menu & State Machine
enum SystemState { STATE_MENU, STATE_RUN, STATE_IR_VIEW, STATE_CALIB_WHITE, STATE_CALIB_BLACK };
SystemState currentState = STATE_MENU;

int menuIndex = 0;
const int MENU_ITEMS = 8; 
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

// Non-blocking buzzer logic to prevent crashing or halting the PID loop
void handleBuzzer() {
    // Check if hall sensor goes low and we are not already beeping or in cooldown
    if (digitalRead(PIN_HALL) == LOW && beepState == 0 && (millis() - lastHallDetectTime > HALL_COOLDOWN)) {
        beepState = 1;
        buzzerTimer = millis();
        digitalWrite(PIN_BUZZER, HIGH); // Start first beep
        lastHallDetectTime = millis();
    }

    // State 1: First beep is active
    if (beepState == 1 && millis() - buzzerTimer > BEEP_DURATION) {
        digitalWrite(PIN_BUZZER, LOW); // Turn off
        beepState = 2;
        buzzerTimer = millis();
    }
    // State 2: Gap between beeps
    else if (beepState == 2 && millis() - buzzerTimer > BEEP_GAP) {
        digitalWrite(PIN_BUZZER, HIGH); // Start second beep
        beepState = 3;
        buzzerTimer = millis();
    }
    // State 3: Second beep is active
    else if (beepState == 3 && millis() - buzzerTimer > BEEP_DURATION) {
        digitalWrite(PIN_BUZZER, LOW); // Turn off
        beepState = 0; // Reset state machine
    }
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
        "IR View", 
        "Calibrate Dual", 
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
        
        if (i >= 3) {
            tft.print(": ");
            switch(i) {
                case 3: tft.println("DYNAMIC"); break;
                case 4: tft.println(baseSpeed); break;
                case 5: tft.println(Kp, 3); break;
                case 6: tft.println(Ki, 4); break;
                case 7: tft.println(Kd, 3); break;
            }
        } else {
            tft.println();
        }
        yield(); // Prevent watchdog triggers during heavy text rendering loops
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
                } 
                else if (menuIndex == 1) {
                    currentState = STATE_IR_VIEW;
                    tft.fillScreen(TFT_BLACK);
                }
                else if (menuIndex == 2) {
                    currentState = STATE_CALIB_WHITE;
                    tft.fillScreen(TFT_BLACK);
                    tft.setCursor(0,0);
                    tft.setTextColor(TFT_WHITE, TFT_BLACK);
                    tft.println("STEP 1: WHITE");
                    tft.println("Place on WHITE");
                    tft.println("Press START");
                }
                else {
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
                    case 4: baseSpeed += 10; break;
                    case 5: Kp += 0.01; break;
                    case 6: Ki += 0.001; break;
                    case 7: Kd += 0.01; break;
                }
                redrawMenu = true;
            }
            if (pressedDown) {
                switch(menuIndex) {
                    case 4: baseSpeed -= 10; break;
                    case 5: Kp -= 0.01; break;
                    case 6: Ki -= 0.001; break;
                    case 7: Kd -= 0.01; break;
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
        if (pressedStart) {
            currentState = STATE_MENU;
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            redrawMenu = true;
        }
    }
    yield(); // Ensure background processing between button cycles
}

// ===== MAIN ROUTINES =====

void setup() {
    delay(2000); // 2 seconds power stabilization delay
    
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

    pinMode(IR1, INPUT); pinMode(IR2, INPUT); pinMode(IR3, INPUT);
    pinMode(IR4, INPUT); pinMode(IR5, INPUT); pinMode(IR6, INPUT);

    // Initialize New Hardware Pins
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); // Ensure buzzer is off by default
    pinMode(PIN_HALL, INPUT_PULLUP); // Use pullup for hall sensor

    tft.init();
    tft.setRotation(1);
    updateMenuDisplay();
}

void loop() {
    // Handle the non-blocking buzzer state machine
    handleBuzzer();

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
        yield(); // Crucial yield point for high frequency display rendering states
    }
    else if (currentState == STATE_RUN) {
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

        float weightSum = 0; float activeSum = 0;

        if (l)  { weightSum += -2.5; activeSum += 1; }
        if (l2) { weightSum += -1.5; activeSum += 1; }
        if (c1) { weightSum += -0.5;  activeSum += 1; }
        if (c2) { weightSum += 0.5;  activeSum += 1; }
        if (r2) { weightSum += 1.5;  activeSum += 1; }
        if (r)  { weightSum += 2.5;  activeSum += 1; }

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
    yield(); // Global safety yield to feed the FreeRTOS task watchdog timer
}