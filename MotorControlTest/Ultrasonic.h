#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {
  private:
    uint8_t trigPin;
    uint8_t echoPin;

  public:
    Ultrasonic(uint8_t trig, uint8_t echo);
    float getDistance();
};

#endif
