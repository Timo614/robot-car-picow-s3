#include <Arduino.h>
#include "MotorControl.h"
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

// Define trigger pin as 17 and echo pin as 47
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);
MotorControl motorControl;
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
    setLEDColor(0, 255, 0);

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.begin(16, 2);
    lcd.clear();

    // Reset servo to point forward
    servoControl.setAngle(0);

    // Blue indicating scanning mode
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
                setLEDColor(255, 0, 0); 
                playAlertSound();
                alertGiven = true;
            }

            int personX = Infer.boxes()[0].x;
            int personWidth = Infer.boxes()[0].w;

            bool moveLeft = personX < 80;
            bool moveRight = personX > 140;
            bool moveForward = personWidth < 45;
            bool moveBackward = personWidth > 50;
            bool turnLeft = personX < 40;
            bool turnRight = personX > 180;

            lcd.clear();
            lcd.setCursor(0, 0);

            if (turnLeft) {
                Serial.println("Turn Left");
                motorControl.moveCar(MOVE, TURN_LEFT, 50);
                delay(100);
                motorControl.stopAllMotors();
                lcd.print("Turn Left");            
            } else if (turnRight) {
                Serial.println("Turn right");
                motorControl.moveCar(MOVE, TURN_RIGHT, 50);
                delay(100);
                motorControl.stopAllMotors();
                lcd.print("Turn Right");            
            } else if (moveForward) {
                lcd.print("Action: Moving");
                lcd.setCursor(0, 1);
                if (moveLeft) {
                    Serial.println("Moving left forward");
                    motorControl.moveCar(MOVE, LEFT_FORWARD, 50);
                    lcd.print("Left Forward");
                } else if (moveRight) {
                    Serial.println("Moving right forward");
                    motorControl.moveCar(MOVE, RIGHT_FORWARD, 50);
                    lcd.print("Right Forward");
                } else {
                    Serial.println("Moving forward");
                    motorControl.moveCar(MOVE, FORWARD, 50);
                    lcd.print("Forward");
                }
            } else if (moveBackward) {
                lcd.print("Action: Moving");
                lcd.setCursor(0, 1);
                if (moveLeft) {
                    Serial.println("Moving left backward");
                    motorControl.moveCar(MOVE, LEFT_BACKWARD, 50);
                    lcd.print("Left Backward");
                } else if (moveRight) {
                    Serial.println("Moving right backward");
                    motorControl.moveCar(MOVE, RIGHT_BACKWARD, 50);
                    lcd.print("Right Backward");
                } else {
                    Serial.println("Moving backward");
                    motorControl.moveCar(MOVE, BACKWARD, 50);
                    lcd.print("Backward");
                }
            } else if (moveLeft) {
                lcd.print("Action: Moving");
                lcd.setCursor(0, 1);
                Serial.println("Moving left");
                motorControl.moveCar(MOVE, LEFT, 50);
                lcd.print("Left");
            } else if (moveRight) {
                lcd.print("Action: Moving");
                lcd.setCursor(0, 1);
                Serial.println("Moving right");
                motorControl.moveCar(MOVE, RIGHT, 50);
                lcd.print("Right");
            } else {
                Serial.println("Stopping all motors");
                motorControl.stopAllMotors();
                lcd.print("Action: Stopped");
                lcd.setCursor(0, 1);
            }
        } else {
            if (personDetected && (millis() - lastDetectionTime > 10000)) {
                setLEDColor(0, 0, 255);
                motorControl.stopAllMotors();
                personDetected = false;
                alertGiven = false;
            } else if (millis() - lastDetectionTime > 500) {
                motorControl.stopAllMotors();
            }
        }
    } else {
        motorControl.stopAllMotors();
    }

    if (!personDetected) {
        lcd.clear();
        lcd.setCursor(0, 0);
        Serial.println("Scanning for faces");
        setLEDColor(0, 0, 255);
        motorControl.moveCar(MOVE, TURN_RIGHT, 50);
        delay(100);
        motorControl.stopAllMotors();
    }
}
