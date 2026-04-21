#include "ultrasonic.h"
#include <Arduino.h>

void setupUltrasonic() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long timeout = micros() + 25000;
  while (digitalRead(ECHO_PIN) == LOW && micros() < timeout);
  
  unsigned long start = micros();
  while (digitalRead(ECHO_PIN) == HIGH && micros() < timeout);
  unsigned long duration = micros() - start;
  
  float distance = duration * 0.034 / 2.0;
  if (distance < 2.0 || distance > 400.0) return 0.0;
  return distance;
}