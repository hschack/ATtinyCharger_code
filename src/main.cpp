#include <Arduino.h>
#include <EEPROM.h>
#include "avr/io.h"
#include "avr/wdt.h"

// FUSES = {
//   0x0A, // WDTCFG {PERIOD=4KCLK, WINDOW=OFF}
//   0x00, // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHZ, LVL=BODLEVEL0}
//   0x7E, // OSCCFG {FREQSEL=20MHZ, OSCLOCK=CLEAR}
//   {0, 0},
//   0xF6, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=UPDI, TOUTDIS=SET, CRCSRC=NOCRC}
//   0xFF, // SYSCFG1 {SUT=64MS}
//   0x00, // APPEND {APPEND=User range:  0x0 - 0xFF}
//   0x00, // BOOTEND {BOOTEND=User range:  0x0 - 0xFF}
// };

//  LOCKBITS = 0xC5; // {LB=NOLOCK}

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
constexpr float acsSens        = 0.030518; // 0.12207; // 40 mV per 1A

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
#define PRINT_INTERVAL           1000    // in 1000ms => 1 Hz
#define FILTER_CONSTANT          0.3
// ------------------- Filtered ADC -------------------
static float filtCurrent       = 2048.0;
static float filtCarBat        = 0.0;
static float filtLiFePO4       = 0.0;

// EEPROM address to store flag
constexpr uint8_t EEPROM_ADDR_FLAG = 0;

// ------------------- Function prototypes -------------------
void setup();
void sampleAdc();
float filteredUpdate(float oldVal, float newVal);
int controlPWM(float measuredAmp, bool doCharge);
void printStatus(float measuredAmp, float carVolt, float lifepoVolt, int pwmOut, bool doCharge);
bool batterySafetyCheck(float carVolt, float lifepoVolt, float measuredAmp);
bool readEepromFlag();
void writeEepromFlag(bool flag);
void feed_wdt();

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
        feed_wdt(); // feed watchdog
        // delay(3500); // test wdt time out 3500 ms ok med 0A ca 4 sek
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
        if (pwmOut > PWM_MAX) {
            pwmOut = PWM_MAX;
        }
        if (pwmOut < PWM_MIN) {
            pwmOut = PWM_MIN;
        }
    }
    else {
        pwmOut = 0;
    }

    if(pwmOutUpdated != pwmOut) {
        // Change pwm output, if pwmOut has changed
        analogWrite(pwmPin, pwmOut);
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
        Serial.print("Full Charge: > ");
        Serial.println((lifepoVolt), 2);
        doCharge = false;  // stop charging
        ChargingPaused = true;
    } else if (measuredAmp < ACS_MIN) {
        Serial.print("ACS  lader fra lifePo til bil: ");
        Serial.println((measuredAmp), 1);
        doCharge = false;  // Stop if current goes negative beyond threshold
    } else if (lifepoVolt < LIFEPO_MAX) {
        ChargingPaused = false;
        doCharge = true;   // resume charging
    }

    // Write flag to EEPROM if changed
    if (ChargingPaused != prevFlag) {
        Serial.println("ChargingPaused written to eeprom");
        // writeEepromFlag(ChargingPaused);
        prevFlag = ChargingPaused;
    }
    Serial.println(doCharge ? "doCharfe = YES" : "doCharfe = NO");
   
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
 //   pinMode(ledPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, PWM_MIN);
    Serial.begin(115200);
    analogReadResolution(12);
    // CPU_CCP = CCP_IOREG_gc; // allow changes
    // WDT.CTRLA = WDT_WINDOW_4KCLK_gc | WDT_PERIOD_4KCLK_gc; // 4s period and window
    // CPU_CCP = CCP_IOREG_gc;
    // WDT.STATUS = WDT_LOCK_bm;
    // while(WDT.STATUS & WDT_SYNCBUSY_bm){
    // } // wait until WDT is synchronized  
    Serial.println("Setup done !");
}

// ------------------- EEPROM helpers -------------------
bool readEepromFlag() {
    byte val = EEPROM.read(EEPROM_ADDR_FLAG);
    return (val == 0xFF);
}

void writeEepromFlag(bool flag) {
    EEPROM.update(EEPROM_ADDR_FLAG, flag ? 0xFF : 0x00);
}

// Feed watchdog
void feed_wdt(void) {
  __asm__ __volatile__("wdr");
}