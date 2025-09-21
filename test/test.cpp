#include <Arduino.h>
#include <EEPROM.h>

// ------------------- Pin constants -------------------
//const uint8_t ledPin           = LED_BUILTIN;
const uint8_t carBatPin        = PIN_A1;
const uint8_t liefpoBatPin     = PIN_A2;
const uint8_t currentPin       = PIN_A3; // ACS758
const uint8_t pwmPin           = PIN_A6;
const uint8_t ignPin           = PIN_A7;

// ------------------- ADC / sensor constants -------------------
constexpr float Vref           = 5.0;
constexpr float scaleDirect    = Vref / 4095.0;

// Voltage dividers for batteries (carBat & LiFePO4)
constexpr float R1             = 8200.0;
constexpr float R2             = 2200.0;
constexpr float scaleDivider   = (Vref / 4095.0) * ((R1 + R2) / R2);

// ACS758 parameters
constexpr float acsOffset      = 2048.0; // 0A
constexpr float acsSens        = 0.12207; // 40 mV per 1A

// ------------------- Safety thresholds -----------------------
constexpr float BAT_DIFF_MAX   = 0.0;   // V, carBat - LiFePO4
constexpr float LIFEPO_MAX     = 13.8;  // V, High stop charge
constexpr float LIFEPO_RECOVER = 13.1;  // V, resume charging
constexpr float ACS_MIN        = -1.0;  // A, stop if current goes negative

// ------------------- PWM control constants -------------------
constexpr float SETPOINT_A     = 7.0;  // target current
#define PWM_STEP_FAST            20      // 5% PWM step
#define PWM_STEP_SLOW            2       // 2% PWM step
constexpr int   PWM_MAX        = 255;
constexpr int   PWM_MIN        = 0;

// ------------------- Timing -------------------
#define ADC_INTERVAL             25      // in 25ms => 40 Hz
#define PRINT_INTERVAL           5000    // in 1000ms => 2 Hz
#define FILTER_CONSTANT          0.3
// ------------------- Filtered ADC -------------------
static float filtCurrent       = 2048.0;
static float filtCarBat        = 0.0;
static float filtLiFePO4       = 0.0;

// EEPROM address to store flag
constexpr uint8_t EEPROM_ADDR_FLAG = 0;

// ------------------- PWM settings -------------------
#define PWM_TARGET_PIN   PIN_PA6       // A6
#define PWM_8BIT_TOP     255           // 8-bit resolution

// Vælg frekvens: 0 = ~1.22 kHz, 1 = ~4.88 kHz, 2 = ~19.5 kHz
#define PWM_FREQ_SETTING 1

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

// PWM functions
void pwm_init_A6(uint8_t duty);
void pwm_set_duty(uint8_t duty);

// ------------------- Main loop ------------------------
void loop() {
   static uint32_t lastAdcTime = 0;
   static uint32_t lastPrintTime = 0;
   uint32_t now = millis();

    // ADC sampling 25 Hz
    if ((now - lastAdcTime) >= ADC_INTERVAL) {
        lastAdcTime = now;
        sampleAdc();
    }

    // Control + safety check + print 1 Hz
    if ((now - lastPrintTime) >= PRINT_INTERVAL) {
        lastPrintTime = now;

        // Convert ADCs to voltage / current
        float carVolt     = filtCarBat * scaleDivider;
        float lifepoVolt  = filtLiFePO4 * scaleDivider;
        float measuredAmp = (filtCurrent - acsOffset) * acsSens;

        // Safety check and update doCharge
        bool doCharge = batterySafetyCheck(carVolt, lifepoVolt, measuredAmp);

        // Only control PWM if allowed
        int Pwm = controlPWM(measuredAmp, doCharge);

        // Print status
        printStatus(measuredAmp, carVolt, lifepoVolt, Pwm, doCharge);
        //digitalWrite(ledPin, !digitalRead(ledPin));
    }
}

// ------------------- Functions ------------------------

// Sample all ADC channels and apply simple filter
void sampleAdc() {
    filtCurrent   = filteredUpdate(filtCurrent,   analogRead(currentPin));
    filtCarBat    = filteredUpdate(filtCarBat,    analogRead(carBatPin));
    filtLiFePO4   = filteredUpdate(filtLiFePO4,   analogRead(liefpoBatPin));
}

float filteredUpdate(float oldVal, float newVal) {
    return (oldVal * (1.0-FILTER_CONSTANT)) + (newVal * FILTER_CONSTANT);
}

// Simple incremental PWM control
int controlPWM(float measuredAmp, bool doCharge) {
    static int16_t pwmOutUpdated = -1; // -1 => will guarantee to update pwm output at first run
    int16_t pwmOut;
    int8_t step = 0;
   
    if(doCharge) { // true
        if (measuredAmp < (SETPOINT_A * 0.8)) {
            step = PWM_STEP_FAST; // inc pwm by PWM_STEP_FAST
        } else if (measuredAmp < SETPOINT_A) {
            step = PWM_STEP_SLOW;   // inc pwm by PWM_STEP_SLOW
        } else if(measuredAmp > (SETPOINT_A*1.2)) {
            step = -PWM_STEP_SLOW;  // dec pwm by PWM_STEP_SLOW
        }
        
        pwmOut = pwmOutUpdated + step;
        if (pwmOut > PWM_MAX) pwmOut = PWM_MAX;
        if (pwmOut < PWM_MIN) pwmOut = PWM_MIN;
    }
    else {
        pwmOut = 0;
    }

    if(pwmOutUpdated != pwmOut) {
        pwm_set_duty(pwmOut);
        pwmOutUpdated = pwmOut;
    }
    return pwmOut;
}

// ------------------- Battery safety check -------------------
bool batterySafetyCheck(float carVolt, float lifepoVolt, float measuredAmp) {
    static bool doCharge = false;
    static bool ChargingPaused = false;
    static bool prevFlag = ChargingPaused;
    static bool FirstRun = true;

    if(FirstRun) {
        ChargingPaused = readEepromFlag();
        prevFlag = ChargingPaused;
        Serial.print("doCharge from eeprom = ");
        Serial.println(doCharge ? "YES" : "NO");
        FirstRun = false;
    }
    
    if (ChargingPaused && (lifepoVolt > LIFEPO_RECOVER)) {
        doCharge = false;
    }
    else if ((carVolt - lifepoVolt) < BAT_DIFF_MAX) {
        Serial.print("Dif Negativ: ");
        Serial.println((carVolt-lifepoVolt), 2);
        doCharge = false;  // stop charging
    } else if (lifepoVolt > LIFEPO_MAX) {
        Serial.print("Full Charge: ) ");
        Serial.println((lifepoVolt), 2);
        doCharge = false;  // stop charging
        ChargingPaused = true;
    } else if (measuredAmp < ACS_MIN) {
        Serial.print("ACS  lader fra lifePo til bil: ");
        Serial.println((measuredAmp), 1);
        doCharge = false;  // Stop if current goes negative beyond threshold
    } else if (lifepoVolt < LIFEPO_RECOVER) {
        ChargingPaused = false;
        doCharge = true;   // resume charging
    }

    // Write flag to EEPROM if changed
    if (ChargingPaused != prevFlag) {
        Serial.println("ChargingPaused written to eeprom");
        writeEepromFlag(ChargingPaused);
        prevFlag = ChargingPaused;
    }
    return doCharge;
}

// --------------- Print current & PWM ------------------
void printStatus(float measuredAmp, float carVolt, float lifepoVolt, int pwmOut, bool doCharge) {
    Serial.print("Current: ");
    Serial.print(measuredAmp, 1);
    Serial.print("A, C: ");
    Serial.print(carVolt, 1);
    Serial.print(", L: ");
    Serial.print(lifepoVolt, 2);
    Serial.print(", pwm: ");
    Serial.print(pwmOut);
    Serial.print(", ");
    Serial.print(", Dif: ");
    Serial.print(carVolt - lifepoVolt ,2);
    Serial.print(", Charge: ");
    Serial.println(doCharge ? "YES" : "NO");
}

// ------------------- Setup ----------------------------
void setup() {
    pinMode(pwmPin, OUTPUT);
    pwm_init_A6(PWM_MIN);   // hardware PWM på A6
    Serial.begin(115200);
    analogReadResolution(12);
}

// ------------------- EEPROM helpers -------------------
bool readEepromFlag() {
    byte val = EEPROM.read(EEPROM_ADDR_FLAG);
    return (val == 0xFF);
}

void writeEepromFlag(bool flag) {
    EEPROM.update(EEPROM_ADDR_FLAG, flag ? 0xFF : 0x00);
}

// ------------------- PWM functions -------------------
void pwm_init_A6(uint8_t duty) {
    pinMode(PWM_TARGET_PIN, OUTPUT);

    // Stop timer mens vi konfigurerer
    TCA0.SINGLE.CTRLA = 0;

    // Single slope PWM, enable WO2 (A6)
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc | TCA_SINGLE_CMP2EN_bm;

    // 8-bit resolution
    TCA0.SINGLE.PER = PWM_8BIT_TOP;

    // Duty
    TCA0.SINGLE.CMP2 = duty;

    // Vælg prescaler baseret på frekvensvalg
    uint8_t prescaler;
    switch(PWM_FREQ_SETTING) {
        case 0: prescaler = TCA_SINGLE_CLKSEL_DIV64_gc; break;   // ~1.22 kHz
        case 1: prescaler = TCA_SINGLE_CLKSEL_DIV16_gc; break;   // ~4.88 kHz
        case 2: prescaler = TCA_SINGLE_CLKSEL_DIV4_gc; break;    // ~19.5 kHz
        default: prescaler = TCA_SINGLE_CLKSEL_DIV16_gc; break;  // fallback
    }

    TCA0.SINGLE.CTRLA = prescaler | TCA_SINGLE_ENABLE_bm;
}

// ------------------------- PWM freq -----------------------------
void pwm_set_duty(uint8_t duty) {
    TCA0.SINGLE.CMP2 = duty;   // 0–255 duty
}
