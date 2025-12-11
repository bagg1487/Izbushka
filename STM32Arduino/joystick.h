#ifndef JOYSTICK_H
#define JOYSTICK_H

extern int joystickX;
extern int joystickY;
extern bool buttonA;
extern bool buttonB;
extern bool buttonX;
extern bool buttonY;

void setupJoystick();
void readJoystick();

#endif