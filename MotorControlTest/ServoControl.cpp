#include "ServoControl.h"

ServoControl::ServoControl(int pin) : servoPin(pin) {
    servo.attach(servoPin);
}

void ServoControl::setAngle(int angle) {
    if (angle < -90) {
        angle = -90;
    }
    if (angle > 90) {
        angle = 90;
    }

    int dutyCycle = mapAngleToDutyCycle(angle);
    servo.writeMicroseconds(dutyCycle);
}

int ServoControl::mapAngleToDutyCycle(int angle) {
    float normalizedAngle = float(angle + 90) / 180.0; // Normalize angle to range 0-1
    int dutyCycle = pwmMin + (normalizedAngle * (pwmMax - pwmMin));
    return dutyCycle;
}
