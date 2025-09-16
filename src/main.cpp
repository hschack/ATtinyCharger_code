#include <Arduino.h>
#include <Wire.h>

#define PWM_PIN 6    // PA6 = TCB0
#define LED_PIN 5    // optional debug LED

void setup() {
    // Initialize UART
    Serial.begin(115200);

    // Initialize I2C on default pins PB0/PB1
    Wire.begin();

    // Setup PWM on PA6 (TCB0)
    pinMode(PWM_PIN, OUTPUT);

    analogReadResolution(12);  // 0–4095 range  

    TCB0.CTRLB = TCB_CNTMODE_PWM8_gc;       // 8-bit PWM
    TCB0.CCMP = (F_CPU / 2000) - 1;         // TOP value for ~2 kHz
    TCB0.CCMPH = 127;                        // 50% duty cycle
    TCB0.CTRLA = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;

    // Optional LED
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);

    Serial.println("ATtiny3224 test running...");

    for (uint8_t duty = 0; duty <= 255; duty++) {
        TCB0.CCMPH = duty;
        delay(5);
    }
}
