#include <Arduino.h>
#include "ServoControl.h"
#include <Adafruit_NeoPixel.h>
#include "I2C_LCD.h"
#include <Wire.h>
#include <Seeed_Arduino_SSCMA.h>

#define SDA_PIN 5
#define SCL_PIN 6
#define NUM_LEDS 4
#define DATA_PIN 38
#define BUZZER_PIN 8
#define SERVO_PIN 14

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);
ServoControl servoControl(SERVO_PIN);
I2C_LCD lcd(0x27);

SSCMA Infer;

void playTone(int pin, int frequency, int duration) {
    int period = 1000000L / frequency;
    int pulse = period / 2;
    for (long i = 0; i < duration * 1000L; i += period) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(pin, LOW);
        delayMicroseconds(pulse);
    }
}

void playAlertSound() {
    int frequencies[] = {370, 415, 330, 233, 311};
    int durations[] = {50, 50, 50, 50, 300};
    for (int i = 0; i < 5; i++) {
        playTone(BUZZER_PIN, frequencies[i], durations[i]);
    }
    digitalWrite(BUZZER_PIN, LOW);
}

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

void setup() {
    delay(5000);
    Serial.begin(115200);
    Infer.begin();
    
    pinMode(BUZZER_PIN, OUTPUT);
    strip.begin();
    lcd.setCursor(0, 0);
    lcd.print("Starting...");

    // Green during startup
    setLEDColor(0, 255, 0); 

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.begin(16, 2);
    lcd.clear();

    // Center out servo
    servoControl.setAngle(0);

    // Blue during scanning after startup
    setLEDColor(0, 0, 255); 
}

void loop() {
    static bool personDetected = false;
    static bool alertGiven = false;
    static unsigned long lastDetectionTime = 0;

    if (!Infer.invoke()) {
        if (Infer.boxes().size() > 0) {
            personDetected = true;
            lastDetectionTime = millis();

            if (!alertGiven) {
                // Red when person is detected
                setLEDColor(255, 0, 0); 
                playAlertSound();
                alertGiven = true;
            }

            int personX = Infer.boxes()[0].x;
            int personWidth = Infer.boxes()[0].w;

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Width: ");
            lcd.print(personWidth);
            lcd.setCursor(0, 1);
            lcd.print("X: ");
            lcd.print(personX);

        } else {
            if (personDetected && (millis() - lastDetectionTime > 10000)) {
                setLEDColor(0, 0, 255); 
                personDetected = false;
                alertGiven = false;
            }
        }
    }
}
