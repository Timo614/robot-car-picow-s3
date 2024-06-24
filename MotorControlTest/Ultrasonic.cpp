#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig, uint8_t echo) {
  trigPin = trig;
  echoPin = echo;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float Ultrasonic::getDistance() {
  // Send a pulse to the trig pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (cm) based on the speed of sound (343 m/s)
  float distance = (duration * 0.0343) / 2;

  // Return the distance value
  return distance;
}
