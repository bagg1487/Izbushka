#include "joystick.h"
#include "config.h"
#include <Arduino.h>

int joystickX = 0;
int joystickY = 0;
bool buttonA = false;
bool buttonB = false;
bool buttonX = false;
bool buttonY = false;
bool joystickConnected = false;

#define JOYSTICK_X_PIN PA0
#define JOYSTICK_Y_PIN PA1
#define BUTTON_A_PIN PA2
#define BUTTON_B_PIN PA3
#define BUTTON_X_PIN PA4
#define BUTTON_Y_PIN PA5

void setupJoystick() {
  Serial.println("Инициализация эмуляции джойстика...");
  
  pinMode(JOYSTICK_X_PIN, INPUT_ANALOG);
  pinMode(JOYSTICK_Y_PIN, INPUT_ANALOG);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_X_PIN, INPUT_PULLUP);
  pinMode(BUTTON_Y_PIN, INPUT_PULLUP);
  joystickConnected = true;
  
  Serial.println("Эмуляция джойстика готова");
}

bool checkJoystick() {
  return joystickConnected;
}

void processJoystick() {
  int rawX = analogRead(JOYSTICK_X_PIN);
  int rawY = analogRead(JOYSTICK_Y_PIN);
  
  joystickX = map(rawX, 0, 4095, -32767, 32767);
  joystickY = map(rawY, 0, 4095, -32767, 32767);
  
  buttonA = !digitalRead(BUTTON_A_PIN);
  buttonB = !digitalRead(BUTTON_B_PIN);
  buttonX = !digitalRead(BUTTON_X_PIN);
  buttonY = !digitalRead(BUTTON_Y_PIN);
  
  if (abs(joystickX) < JOYSTICK_DEADZONE) joystickX = 0;
  if (abs(joystickY) < JOYSTICK_DEADZONE) joystickY = 0;
}