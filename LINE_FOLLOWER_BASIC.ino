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

// ===== SYSTEM VARIABLES =====
int baseSpeed = 150;
int threshold = 2000;
float Kp = 0.05;
float Ki = 0.0;
float Kd = 0.02;

float lastError = 0;
float integral = 0;
int s1, s2, s3, s4, s5, s6;

// Menu & State Machine
enum SystemState { STATE_MENU, STATE_RUN, STATE_IR_VIEW, STATE_CALIBRATE };
SystemState currentState = STATE_MENU;

int menuIndex = 0;
const int MENU_ITEMS = 8; // Expanded menu
bool isEditing = false;
bool redrawMenu = true;

// Calibration Variables
unsigned long calibStartTime = 0;
int calibMin = 4095;
int calibMax = 0;

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
const int debounceDelay = 50; 

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
        "IR View", 
        "Calibrate IR", 
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
        
        // Print item name
        tft.print(items[i]);
        
        // Only print values for editable parameters (index 3 to 7)
        if (i >= 3) {
            tft.print(": ");
            switch(i) {
                case 3: tft.println(threshold); break;
                case 4: tft.println(baseSpeed); break;
                case 5: tft.println(Kp, 3); break;
                case 6: tft.println(Ki, 4); break;
                case 7: tft.println(Kd, 3); break;
            }
        } else {
            tft.println(); // Just a new line for commands
        }
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
                    tft.setCursor(0,0);
                    tft.setTextColor(TFT_GREEN, TFT_BLACK);
                    tft.println("RUNNING PID...");
                    integral = 0;
                    lastError = 0;
                } 
                else if (menuIndex == 1) {
                    currentState = STATE_IR_VIEW;
                    tft.fillScreen(TFT_BLACK);
                }
                else if (menuIndex == 2) {
                    currentState = STATE_CALIBRATE;
                    tft.fillScreen(TFT_BLACK);
                    tft.setCursor(0,0);
                    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                    tft.println("CALIBRATING...");
                    tft.println("Sweep sensors");
                    tft.println("across the line!");
                    calibMin = 4095;
                    calibMax = 0;
                    calibStartTime = millis();
                }
                else {
                    isEditing = true; 
                    redrawMenu = true; 
                }
            }
        } else {
            // Edit Mode (Only applies to indices 3 through 7)
            if (pressedStart) { 
                isEditing = false; 
                redrawMenu = true; 
            }
            
            if (pressedUp) {
                switch(menuIndex) {
                    case 3: threshold += 100; break;
                    case 4: baseSpeed += 10; break;
                    case 5: Kp += 0.01; break;
                    case 6: Ki += 0.001; break;
                    case 7: Kd += 0.01; break;
                }
                redrawMenu = true;
            }
            if (pressedDown) {
                switch(menuIndex) {
                    case 3: threshold -= 100; break;
                    case 4: baseSpeed -= 10; break;
                    case 5: Kp -= 0.01; break;
                    case 6: Ki -= 0.001; break;
                    case 7: Kd -= 0.01; break;
                }
                redrawMenu = true;
            }
        }
    } 
    else {
        // If in Run, IR View, or Calibrate mode, pressing START halts and returns to MENU
        if (pressedStart) {
            currentState = STATE_MENU;
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            redrawMenu = true;
        }
    }
}

// ===== MAIN ROUTINES =====

void setup() {
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

    pinMode(IR1, INPUT); pinMode(IR2, INPUT); pinMode(IR3, INPUT);
    pinMode(IR4, INPUT); pinMode(IR5, INPUT); pinMode(IR6, INPUT);

    tft.init();
    tft.setRotation(1);
    updateMenuDisplay();
}

void loop() {
    processButtons();

    if (currentState == STATE_MENU) {
        if (redrawMenu) updateMenuDisplay();
    } 
    else if (currentState == STATE_IR_VIEW) {
        static unsigned long lastDebugRefresh = 0;
        if (millis() - lastDebugRefresh > 200) {
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
    }
    else if (currentState == STATE_CALIBRATE) {
        // Read all sensors rapidly
        int readings[6] = {
            analogRead(IR1), analogRead(IR2), analogRead(IR3), 
            analogRead(IR4), analogRead(IR5), analogRead(IR6)
        };
        
        // Find min and max
        for(int i = 0; i < 6; i++) {
            if (readings[i] < calibMin) calibMin = readings[i];
            if (readings[i] > calibMax) calibMax = readings[i];
        }

        // Auto-exit calibration after 5 seconds
        if (millis() - calibStartTime > 5000) {
            // Calculate perfect midpoint threshold
            threshold = (calibMin + calibMax) / 2;
            
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(0,0);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.println("DONE!");
            tft.print("New Thresh: ");
            tft.println(threshold);
            delay(1500); // Show result briefly before returning to menu
            
            currentState = STATE_MENU;
            redrawMenu = true;
        }
    }
    else if (currentState == STATE_RUN) {
        // ---- PID Execution Block ----
        s1 = analogRead(IR1); s2 = analogRead(IR2); s3 = analogRead(IR3);
        s4 = analogRead(IR4); s5 = analogRead(IR5); s6 = analogRead(IR6);

        bool r = s1 > threshold; bool r2 = s2 > threshold;
        bool c1 = s3 > threshold; bool c2 = s4 > threshold;
        bool l2 = s5 > threshold; bool l = s6 > threshold;

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
        
        delay(10);
    }
}