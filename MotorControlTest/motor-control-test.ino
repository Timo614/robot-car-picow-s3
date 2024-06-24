#include <Arduino.h>
#include "Ultrasonic.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include <Adafruit_NeoPixel.h>
#include "I2C_LCD.h"
#include <Wire.h>

#define SDA_PIN 5
#define SCL_PIN 6
#define NUM_LEDS 4
#define DATA_PIN 38
#define BUZZER_PIN 8

// Define trigger pin as 17 and echo pin as 47
Ultrasonic ultrasonic(17, 47); 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

ServoControl servoControl(14); 
MotorControl motorControl;
I2C_LCD lcd(0x27);

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

void setup() {
    delay(5000);
    Serial.begin(9600);

    pinMode(BUZZER_PIN, OUTPUT);

    strip.begin();
    strip.show();
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
    }
    strip.show();

    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.begin(16, 2);

    lcd.setCursor(0, 0);
    lcd.print("Starting...");

    // Testing alert sound
    playAlertSound();


    delay(500);
    // Test servo: sweep from -90 to 90 degrees and back
    for (int angle = -90; angle <= 90; angle += 10) {
        Serial.print("Setting servo to ");
        Serial.print(angle);
        Serial.println(" degrees");
        lcd.setCursor(0, 1);
        lcd.print("Servo: ");
        lcd.print(angle);
        servoControl.setAngle(angle);
        delay(500);
        lcd.clear();
    }
    
    servoControl.setAngle(0);

    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 255, 0));
    }
    strip.show();

    // Test each motor in each direction for one second
    Serial.println("Testing forward");
    lcd.setCursor(0, 1);
    lcd.print("Forward ");
    motorControl.moveCar(MOVE, FORWARD, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing backward");
    lcd.setCursor(0, 1);
    lcd.print("Backward");
    motorControl.moveCar(MOVE, BACKWARD, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing left");
    lcd.setCursor(0, 1);
    lcd.print("Left    ");
    motorControl.moveCar(MOVE, LEFT, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing right");
    lcd.setCursor(0, 1);
    lcd.print("Right   ");
    motorControl.moveCar(MOVE, RIGHT, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing left forward");
    lcd.setCursor(0, 1);
    lcd.print("L Forward");
    motorControl.moveCar(MOVE, LEFT_FORWARD, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing right forward");
    lcd.setCursor(0, 1);
    lcd.print("R Forward");
    motorControl.moveCar(MOVE, RIGHT_FORWARD, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing left backward");
    lcd.setCursor(0, 1);
    lcd.print("L Backwrd");
    motorControl.moveCar(MOVE, LEFT_BACKWARD, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing right backward");
    lcd.setCursor(0, 1);
    lcd.print("R Backwrd");
    motorControl.moveCar(MOVE, RIGHT_BACKWARD, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing turn left");
    lcd.setCursor(0, 1);
    lcd.print("Turn Left");
    motorControl.moveCar(MOVE, TURN_LEFT, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();

    Serial.println("Testing turn right");
    lcd.setCursor(0, 1);
    lcd.print("Turn Right");
    motorControl.moveCar(MOVE, TURN_RIGHT, 50);
    delay(1000);
    motorControl.stopAllMotors();
    lcd.clear();
    
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 255));
    }
    strip.show();
}

void loop() {
    // Get distance from ultrasonic sensor
    float distance = ultrasonic.getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Display distance on LCD
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(distance);
    lcd.print(" cm");

    delay(1000);
}
