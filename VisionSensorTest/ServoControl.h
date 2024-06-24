#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoControl {
public:
    ServoControl(int pin);
    void setAngle(int angle);

private:
    Servo servo;
    int servoPin;
    const int pwmMax = 2500;
    const int pwmMin = 500;
    const int period = 65535;

    int mapAngleToDutyCycle(int angle);
};

#endif // SERVOCONTROL_H
