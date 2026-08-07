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

// ===== CHASSIS GEOMETRY (mm) — used to tune line-loss recovery =====
const float SENSOR_ARRAY_WIDTH_MM = 60.0;   // width of the 6-IR sensor bar
const float CASTER_SPAN_MM        = 110.0;  // spacing between the two front caster balls
const float WHEEL_TRACK_MM        = 160.0;  // spacing between the rear N20 drive motors
const float SENSOR_TO_AXLE_MM     = 75.0;   // front sensor bar to rear drive axis (pivot) distance

// A shorter sensor-to-axle "lookahead" means a given lateral/weighted-average
// error corresponds to a tighter curve happening close to the robot's pivot,
// so recovery should be a bit punchier. Normalized against a 100mm baseline.
const float LOOKAHEAD_NORM_MM = 100.0;
const float LOOKAHEAD_GAIN = LOOKAHEAD_NORM_MM / SENSOR_TO_AXLE_MM;

// ===== ERROR HISTORY / TREND EXTRAPOLATION =====
// Keeps the last few VALID (line-seen) errors so that, at the instant the
// line is lost, we can tell "error was climbing steadily toward an edge"
// (sharp curve exiting the sensor array) apart from "error was flat/centered
// then suddenly vanished" (a real dropout / gap / junction).
#define ERR_HISTORY_SIZE 5
float errorHistory[ERR_HISTORY_SIZE] = {0, 0, 0, 0, 0};
int errHistIndex = 0;
int errHistCount = 0;

bool wasLineLost = false;
unsigned long lineLostSince = 0;

// Classification thresholds
const float TREND_CURVE_THRESHOLD = 0.35;  // min per-sample slope to call it a "curve exit"
const int   TREND_CONSISTENCY_MIN = 3;     // consecutive same-direction samples required
const unsigned long DROPOUT_GRACE_MS = 200; // grace window to ride out a small gap before escalating

// Sequence Variables
int startMagnetCount; 
int magnetCount = 0;
unsigned long lastHallDetectTime = 0;

// MODIFIED: 1-second (1000ms) cooldown delay between detecting consecutive magnets
const int HALL_COOLDOWN = 1000; 
int ROTATION_TIME;

// Manual Thresholds
int manualBlackThresh;
int manualWhiteThresh;

// Manual Line Logic Selection (0 = Black Line on White, 1 = White Line on Black)
int selectedLineLogic = 0; 

// Menu & State Machine
enum SystemState { STATE_MENU, STATE_RUN, STATE_IR_VIEW, STATE_LED_TEST, STATE_MAG_TEST };
SystemState currentState = STATE_MENU;

int menuIndex = 0;
const int MENU_ITEMS = 14; 
bool isEditing = false;
bool redrawMenu = true;

unsigned long startButtonHeldTime = 0; 
int lastDisplayedMode = -1; 

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

// ===== ERROR HISTORY / TREND HELPERS =====

void pushErrorHistory(float e) {
    errorHistory[errHistIndex] = e;
    errHistIndex = (errHistIndex + 1) % ERR_HISTORY_SIZE;
    if (errHistCount < ERR_HISTORY_SIZE) errHistCount++;
}

// Returns the stored errors in chronological order (oldest -> newest)
void getOrderedHistory(float *out) {
    int start = (errHistIndex - errHistCount + ERR_HISTORY_SIZE) % ERR_HISTORY_SIZE;
    for (int i = 0; i < errHistCount; i++) {
        out[i] = errorHistory[(start + i) % ERR_HISTORY_SIZE];
    }
}

void resetErrorHistory() {
    for (int i = 0; i < ERR_HISTORY_SIZE; i++) errorHistory[i] = 0;
    errHistIndex = 0;
    errHistCount = 0;
    wasLineLost = false;
    lineLostSince = 0;
}

// Least-squares slope of the recent error samples (per-sample rate of change),
// plus how many consecutive samples (walking backward from the newest) moved
// in the same direction. A steady, consistent slope right before the line
// disappeared is the signature of a curve sliding out of the sensor's view;
// a flat/inconsistent history right before disappearance is the signature of
// a genuine dropout (gap, junction, crossing) rather than a curve.
float computeErrorTrend(int &consistentCount) {
    float ordered[ERR_HISTORY_SIZE];
    getOrderedHistory(ordered);
    consistentCount = 0;

    if (errHistCount < 2) return 0.0;

    int sign = 0;
    for (int i = errHistCount - 1; i > 0; i--) {
        float diff = ordered[i] - ordered[i - 1];
        int s = (diff > 0.02) ? 1 : (diff < -0.02) ? -1 : 0;
        if (s == 0) break;
        if (sign == 0) sign = s;
        else if (s != sign) break;
        consistentCount++;
    }

    int n = errHistCount;
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    for (int i = 0; i < n; i++) {
        sumX += i;
        sumY += ordered[i];
        sumXY += (float)i * ordered[i];
        sumXX += (float)i * i;
    }
    float denom = (n * sumXX - sumX * sumX);
    if (fabs(denom) < 1e-6) return 0.0;
    return (n * sumXY - sumX * sumY) / denom;
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
        "Line Color",     
        "Rot Time",
        "Test Spin",
        "LED Test",       
        "Test Magnet",    
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
        
        if (i == 1 || (i >= 3 && i <= 6) || i >= 10) {
            tft.print(": ");
            switch(i) {
                case 1: tft.println(startMagnetCount); break;
                case 3: tft.println(manualBlackThresh); break;
                case 4: tft.println(manualWhiteThresh); break;
                case 5: tft.println(selectedLineLogic == 0 ? "Black" : "White"); break;
                case 6: tft.println(ROTATION_TIME); break;
                case 10: tft.println(baseSpeed); break;
                case 11: tft.println(Kp, 3); break;
                case 12: tft.println(Ki, 4); break;
                case 13: tft.println(Kd, 3); break;
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
                    resetErrorHistory();
                    magnetCount = startMagnetCount; 
                    lastDisplayedMode = -1; 
                    
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
                else if (menuIndex == 7) {
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
                else if (menuIndex == 8) { 
                    currentState = STATE_LED_TEST;
                    tft.fillScreen(TFT_BLACK);
                    
                    tft.setTextSize(2);
                    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
                    tft.setCursor(10, 40);
                    tft.println("-- LED TEST --\n");
                    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                    tft.println("[START] to Exit");
                }
                else if (menuIndex == 9) { 
                    currentState = STATE_MAG_TEST;
                    tft.fillScreen(TFT_BLACK);
                }
                else if (menuIndex == 1 || (menuIndex >= 3 && menuIndex <= 6) || menuIndex >= 10) {
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
                    case 5: prefs.putInt("lineLogic", selectedLineLogic); break;
                    case 6: prefs.putInt("rotTime", ROTATION_TIME); break;
                    case 10: prefs.putInt("baseSpeed", baseSpeed); break;
                    case 11: prefs.putFloat("Kp", Kp); break;
                    case 12: prefs.putFloat("Ki", Ki); break;
                    case 13: prefs.putFloat("Kd", Kd); break;
                }
                redrawMenu = true; 
            }
            
            if (pressedUp) {
                switch(menuIndex) {
                    case 1: startMagnetCount++; if(startMagnetCount > 5) startMagnetCount = 5; break;
                    case 3: manualBlackThresh += 50; if(manualBlackThresh > 4095) manualBlackThresh = 4095; break;
                    case 4: manualWhiteThresh += 50; if(manualWhiteThresh > 4095) manualWhiteThresh = 4095; break;
                    case 5: selectedLineLogic = (selectedLineLogic + 1) % 2; break;
                    case 6: ROTATION_TIME += 10; break;
                    case 10: baseSpeed += 10; break;
                    case 11: Kp += 0.01; break;
                    case 12: Ki += 0.001; break;
                    case 13: Kd += 0.01; break;
                }
                redrawMenu = true;
            }
            if (pressedDown) {
                switch(menuIndex) {
                    case 1: startMagnetCount--; if(startMagnetCount < 0) startMagnetCount = 0; break;
                    case 3: manualBlackThresh -= 50; if(manualBlackThresh < 0) manualBlackThresh = 0; break;
                    case 4: manualWhiteThresh -= 50; if(manualWhiteThresh < 0) manualWhiteThresh = 0; break;
                    case 5: selectedLineLogic = (selectedLineLogic + 1) % 2; break;
                    case 6: ROTATION_TIME -= 10; if(ROTATION_TIME < 0) ROTATION_TIME = 0; break;
                    case 10: baseSpeed -= 10; break;
                    case 11: Kp -= 0.01; break;
                    case 12: Ki -= 0.001; break;
                    case 13: Kd -= 0.01; break;
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
            digitalWrite(PIN_BUZZER, LOW); 
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
    manualBlackThresh = prefs.getInt("blkThresh", 300);
    manualWhiteThresh = prefs.getInt("whtThresh", 800);
    selectedLineLogic = prefs.getInt("lineLogic", 0);
    ROTATION_TIME = prefs.getInt("rotTime", 600);
    
    baseSpeed = prefs.getInt("baseSpeed", 80); 
    Kp = prefs.getFloat("Kp", 1.31);
    Ki = prefs.getFloat("Ki", 0.00);
    Kd = prefs.getFloat("Kd", 3.60); 
    
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
            tft.printf("L1 (IR1): %04d\n  L2 (IR2): %04d\n", s1, s2);
            tft.printf("C1 (IR3): %04d\n  C2 (IR4): %04d\n", s3, s4);
            tft.printf("R2 (IR5): %04d\n  R1 (IR6): %04d\n", s5, s6);
            
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.println("\n[START] to Exit     ");
            
            lastDebugRefresh = millis();
        }
        yield(); 
    }
    else if (currentState == STATE_MAG_TEST) { 
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
                digitalWrite(PIN_BUZZER, HIGH); 
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
        static uint16_t pixelHue = 0; 

        if (millis() - lastLedStep > 15) { 
            strip.clear();
            
            strip.setPixelColor(0, strip.gamma32(strip.ColorHSV(pixelHue)));
            strip.setPixelColor(1, strip.gamma32(strip.ColorHSV(pixelHue + 10000)));
            strip.setPixelColor(2, strip.Color(255, 165, 0));

            strip.show();
            pixelHue += 300; 
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
                // INVERSION OF CURRENT LOGIC AFTER 3RD TASK/MAGNET
                selectedLineLogic = (selectedLineLogic == 0) ? 1 : 0;
                
                digitalWrite(PIN_BUZZER, HIGH);
                delay(300);
                digitalWrite(PIN_BUZZER, LOW);

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
                resetErrorHistory();
                // Will immediately resume line tracking on next loop evaluation!
            }
        }

        int raw[6] = {analogRead(IR1), analogRead(IR2), analogRead(IR3), 
                      analogRead(IR4), analogRead(IR5), analogRead(IR6)};

        // Update display text dynamically if logic flips or starts
        if (selectedLineLogic != lastDisplayedMode) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(10, 80);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.println("RUNNING.....");
            
            tft.setTextSize(2);
            tft.setCursor(10, 130);
            if (selectedLineLogic == 1) {
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.println("MODE:WHITE ON BLACK");
            } else {
                tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                tft.println("MODE:BLACK ON WHITE");
            }
            lastDisplayedMode = selectedLineLogic;
        }

        // --- CONTINUOUS WEIGHTED AVERAGE POSITION CALCULATION ---
        long weightedSum = 0;
        long totalSum = 0;
        bool anyLineDetected = false;

        for(int i = 0; i < 6; i++) {
            int evaluatedVal = 0;
            if (selectedLineLogic == 1) {
                // White Line on Black Background logic: active when sensor reads low (white)
                if (raw[i] < manualWhiteThresh) evaluatedVal = 4095; 
            } else {
                // Black Line on White Background logic: active when sensor reads high (black)
                if (raw[i] > manualBlackThresh) evaluatedVal = 4095;
            }

            if (evaluatedVal > 0) {
                anyLineDetected = true;
            }

            int weight = (i - 2.5) * 1000; 
            
            weightedSum += (long)evaluatedVal * weight;
            totalSum += evaluatedVal;
        }

        // --- 90 DEGREE TURN HANDLING ---
        bool l1_farLeft    = (selectedLineLogic == 1 ? raw[0] < manualWhiteThresh : raw[0] > manualBlackThresh);
        bool r1_farRight   = (selectedLineLogic == 1 ? raw[5] < manualWhiteThresh : raw[5] > manualBlackThresh);

        // --- MODIFIED: END ZONE WHITE BOX CHECK ---
        // A white box means ALL sensors read a value < manualWhiteThresh
        bool allWhiteBox = (raw[0] < manualWhiteThresh && raw[1] < manualWhiteThresh && 
                            raw[2] < manualWhiteThresh && raw[3] < manualWhiteThresh && 
                            raw[4] < manualWhiteThresh && raw[5] < manualWhiteThresh);

        // Stop condition ONLY triggers if past the 5th magnet AND fully inside the white box
        if (magnetCount >= 5 && allWhiteBox) {
            setMotor(AIN1, AIN2, PWMA, 0);
            setMotor(BIN1, BIN2, PWMB, 0);
            currentState = STATE_MENU;
            strip.clear();
            strip.show();
            redrawMenu = true;
            return; 
        }

        float error = 0.0;

        // FIXED LOGIC: Prioritize sharp turn detection OVER standard line detection
        if (l1_farLeft && !r1_farRight) {
            error = -3.5; // Hard sharp left turn catch
            pushErrorHistory(error);
            wasLineLost = false;
        } 
        else if (r1_farRight && !l1_farLeft) {
            error = 3.5;  // Hard sharp right turn catch
            pushErrorHistory(error);
            wasLineLost = false;
        } 
        else if (anyLineDetected) {
            float position = (float)weightedSum / totalSum; 
            error = position / 1000.0; 
            pushErrorHistory(error);
            wasLineLost = false;
        } 
        else {
            // --- LINE LOST: discriminate "curve exiting sensor range" vs "real dropout" ---
            int consistentCount = 0;
            float trend = computeErrorTrend(consistentCount); // per-sample slope from recent valid readings

            if (!wasLineLost) {
                wasLineLost = true;
                lineLostSince = millis();
            }
            unsigned long lostDuration = millis() - lineLostSince;

            // Curve vs jump discrimination: a steady run of same-direction error
            // growth just before loss means the line was sliding out of sensor
            // range on a curve. No prior trend (error was flat/centered) means
            // this is more likely a genuine dropout/gap.
            bool isCurveExit = (fabs(trend) >= TREND_CURVE_THRESHOLD) &&
                                (consistentCount >= TREND_CONSISTENCY_MIN);

            if (isCurveExit) {
                // Error trend extrapolation: bias recovery hard in the direction
                // the error was already heading. Scale slightly with how steep
                // the trend was (sharper curve -> punchier correction), and with
                // the sensor's lookahead distance from the pivot axle.
                float dirSign = (trend > 0) ? 1.0 : -1.0;
                float magnitude = fabs(trend) * 4.0 * LOOKAHEAD_GAIN;
                magnitude = constrain(magnitude, 3.0, 3.5); // stay within normal hard-turn range
                error = dirSign * magnitude;
            }
            else if (lostDuration < DROPOUT_GRACE_MS) {
                // Sudden jump to zero detection with no prior trend: ride it out
                // briefly at reduced authority rather than committing to a hard
                // turn, in case it's just a small gap/junction under the array.
                error = lastError * 0.4;
            }
            else {
                // Dropout has persisted past the grace window without ever
                // showing a curve-like trend: fall back to edge-seeking, but
                // less aggressively than a confirmed curve exit.
                if (lastError < 0)      error = -2.5;
                else if (lastError > 0) error = 2.5;
                else                    error = 0.0;
            }
        }

        // --- PID CALCULATION ---
        integral += error;
        integral = constrain(integral, -300, 300);
        float derivative = error - lastError;
        
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;

        // --- MODIFIED: DYNAMIC MOTOR SPEED (RAMP BOOST) ---
        int currentBaseSpeed = baseSpeed;
        int maxLimit = 180; // Standard maximum constraint
        
        // If we are between magnet 3 and magnet 4 (the 25-degree ramp area)
        // push the robot to near-maximum speed.
        if (magnetCount == 3) {
            currentBaseSpeed = 200; // Drastically boosted base speed
            maxLimit = 220;         // Allow motors to use the full top-end PWM range
        }

        int leftSpeed  = currentBaseSpeed - (output * 35);
        int rightSpeed = currentBaseSpeed + (output * 35);

        leftSpeed  = constrain(leftSpeed, -120, maxLimit);
        rightSpeed = constrain(rightSpeed, -120, maxLimit);

        setMotor(AIN1, AIN2, PWMA, rightSpeed);
        setMotor(BIN1, BIN2, PWMB, leftSpeed);
    }
    delay(1);
    yield(); 
}