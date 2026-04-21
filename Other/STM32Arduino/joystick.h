#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include <stdint.h>

extern int joystickX;
extern int joystickY;
extern bool buttonA;
extern bool buttonB;
extern bool buttonX;
extern bool buttonY;
extern bool joystickConnected;

void setupJoystick();
bool checkJoystick();
void processJoystick();

#endif