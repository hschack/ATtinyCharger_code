#include <Arduino.h>
#include <EEPROM.h>

// ------------------- Pin constants -------------------
const uint8_t carBatPin        = PIN_A1;
const uint8_t liefpoBatPin     = PIN_A2;
const uint8_t currentPin       = PIN_A3; // ACS758
const uint8_t pwmPin           = PIN_A5; // PA5 for PWM
const uint8_t ignPin           = PIN_A7;

// ------------------- ADC / sensor constants -------------------
constexpr float Vref           = 5.0;
constexpr float scaleDirect    = Vref / 4095.0;
constexpr float R1             = 8200.0;
constexpr float R2             = 2200.0;
constexpr float scaleDivider   = (Vref / 4095.0) * ((R1 + R2) / R2);
constexpr float acsOffset      = 2048.0; // 0A
constexpr float acsSens        = 0.030518; // 40 mV per 1A

// ------------------- Safety thresholds -----------------------
constexpr float BAT_DIFF_MAX   = 0.0;   
constexpr float LIFEPO_MAX     = 13.8;  
constexpr float LIFEPO_RECOVER = 13.4;  
constexpr float ACS_MIN        = -1.0;  

// ------------------- PWM control constants -------------------
constexpr float SETPOINT_A     = 7.0;  
#define PWM_STEP_FAST            10     
#define PWM_STEP_SLOW            2      
constexpr int   PWM_MAX        = 255;
constexpr int   PWM_MIN        = 0;

// ------------------- Timing -------------------
#define ADC_INTERVAL             25      
#define PWM_INTERVAL             1000   
#define PRINT_INTERVAL           1000    
#define FILTER_CONSTANT          0.3

// ------------------- Filtered ADC -------------------
static float filtCurrent       = 2048.0;
static float filtCarBat        = 0.0;
static float filtLiFePO4       = 0.0;

// EEPROM address to store flag
constexpr uint8_t EEPROM_ADDR_FLAG = 0;

// ------------------- Function prototypes -------------------
void setup();
void loop();
void sampleAdc();
float filteredUpdate(float oldVal, float newVal);
int controlPWM(float measuredAmp, bool doCharge);
void printStatus(float measuredAmp, float carVolt, float lifepoVolt, int pwmOut, bool doCharge);
bool batterySafetyCheck(float carVolt, float lifepoVolt, float measuredAmp);
bool readEepromFlag();
void writeEepromFlag(bool flag);
void feed_wdt();
void analogWritePA5(uint8_t value);

// ------------------- Setup ----------------------------
void setup() {
    // Route TCB0 output to PA5 (ALT0)
    PORTMUX.TCBROUTEA = 0x00; // ALT0 = PA5 output

    // Set PA5 as output
    PORTA.DIR |= (1 << 5);

    // Stop TCB0 while configuring
    TCB0.CTRLA = 0;

    // Set 8-bit PWM mode and enable output
    TCB0.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm;

    // Set TOP value for 8-bit PWM
    TCB0.CCMP = 255;

    // Start with 0% duty cycle
    TCB0.CCMPH = PWM_MIN;

    // Enable timer with prescaler DIV64 (~1.2 kHz @ 20 MHz)
    TCB0.CTRLA = TCB_ENABLE_bm | 5;

    Serial.begin(115200);
    analogReadResolution(12);
    Serial.println("Setup done, PWM on PA5!");
}

// ------------------- Main loop ------------------------
void loop() {
    static uint32_t lastAdcTime = 0;
    static uint32_t lastPwmTime = 0;
    static uint32_t lastPrintTime = 0;
    uint32_t now = millis();

    // ADC sampling
    if ((now - lastAdcTime) >= ADC_INTERVAL) {
        lastAdcTime = now;
        sampleAdc();
    }

    // Control PWM
    if ((now - lastPwmTime) >= PWM_INTERVAL) {
        lastPwmTime = now;

        float carVolt     = filtCarBat * scaleDivider;
        float lifepoVolt  = filtLiFePO4 * scaleDivider;
        float measuredAmp = (filtCurrent - acsOffset) * acsSens;

        bool doCharge = batterySafetyCheck(carVolt, lifepoVolt, measuredAmp);
        int pwmOut = controlPWM(measuredAmp, doCharge);

        // Print status
        lastPrintTime += PWM_INTERVAL;
        if (lastPrintTime >= PRINT_INTERVAL) {
            printStatus(measuredAmp, carVolt, lifepoVolt, pwmOut, doCharge);
            lastPrintTime = 0;
        }
    }

    feed_wdt(); 
}

// ------------------- Functions ------------------------
void sampleAdc() {
    filtCurrent   = filteredUpdate(filtCurrent,   analogRead(currentPin));
    filtCarBat    = filteredUpdate(filtCarBat,    analogRead(carBatPin));
    filtLiFePO4   = filteredUpdate(filtLiFePO4,   analogRead(liefpoBatPin));
}

float filteredUpdate(float oldVal, float newVal) {
    return (oldVal * (1.0-FILTER_CONSTANT)) + (newVal * FILTER_CONSTANT);
}

int controlPWM(float measuredAmp, bool doCharge) {
    static int16_t pwmOutUpdated = -1;
    int16_t pwmOut;
    int8_t step = 0;
   
    if(doCharge) {
        if (measuredAmp < (SETPOINT_A * 0.8)) step = PWM_STEP_FAST;
        else if (measuredAmp < SETPOINT_A) step = PWM_STEP_SLOW;
        else if(measuredAmp > (SETPOINT_A*1.2)) step = -PWM_STEP_SLOW;
        
        pwmOut = pwmOutUpdated + step;
        if (pwmOut > PWM_MAX) pwmOut = PWM_MAX;
        if (pwmOut < PWM_MIN) pwmOut = PWM_MIN;
    } else {
        pwmOut = 0;
    }

    if(pwmOutUpdated != pwmOut) {
        analogWritePA5(pwmOut);
        Serial.println("PWM set to " + String(pwmOut));
        pwmOutUpdated = pwmOut;
    }
    return pwmOut;
}

bool batterySafetyCheck(float carVolt, float lifepoVolt, float measuredAmp) {
    static bool doCharge = false;
    static bool ChargingPaused = false;
    static bool prevFlag = ChargingPaused;
    static bool FirstRun = true;

    if(FirstRun) {
        ChargingPaused = readEepromFlag();
        prevFlag = ChargingPaused;
        FirstRun = false;
    }

    if (ChargingPaused && (lifepoVolt > LIFEPO_RECOVER)) doCharge = false;
    else if ((carVolt - lifepoVolt) < BAT_DIFF_MAX) doCharge = false;
    else if (lifepoVolt > LIFEPO_MAX) { doCharge = false; ChargingPaused = true; }
    else if (measuredAmp < ACS_MIN) doCharge = false;
    else if (lifepoVolt < LIFEPO_MAX) { ChargingPaused = false; doCharge = true; }

    if (ChargingPaused != prevFlag) {
        writeEepromFlag(ChargingPaused);
        prevFlag = ChargingPaused;
    }

    return doCharge;
}

void printStatus(float measuredAmp, float carVolt, float lifepoVolt, int pwmOut, bool doCharge) {
    Serial.print("Current: "); Serial.print(measuredAmp, 1);
    Serial.print("A, C: "); Serial.print(carVolt, 1);
    Serial.print(", L: "); Serial.print(lifepoVolt, 2);
    Serial.print(", pwm: "); Serial.print(pwmOut);
    Serial.print(", Dif: "); Serial.print(carVolt - lifepoVolt ,2);
    Serial.print(", Charge: "); Serial.println(doCharge ? "YES" : "NO");
}

bool readEepromFlag() {
    byte val = EEPROM.read(EEPROM_ADDR_FLAG);
    return (val == 0xFF);
}

void writeEepromFlag(bool flag) {
    EEPROM.update(EEPROM_ADDR_FLAG, flag ? 0xFF : 0x00);
}

void feed_wdt() {
    __asm__ __volatile__("wdr");
}

// ------------------- PWM wrapper for PA5 -------------------
void analogWritePA5(uint8_t value) {
    TCB0.CCMPH = value;  // 0–255 duty cycle
}
