#include "joystick.h"

USB Usb;
USBHub Hub(&Usb);
XBOXRECV Xbox(&Usb);

int joystickX = 0;
int joystickY = 0;
bool joystickConnected = false;
bool buttonA = false;
bool buttonB = false;
bool buttonX = false;
bool buttonY = false;

void setupJoystick() {
  if (Usb.Init() == -1) {
    Serial.println("USB Host не инициализирован");
    return;
  }
  Serial.println("USB Host готов");
}

bool checkJoystick() {
  Usb.Task();
  joystickConnected = Xbox.XboxReceiverConnected;
  return joystickConnected;
}

void processJoystick() {
  if (!joystickConnected) return;
  
  if (Xbox.Xbox360Connected[0]) {
    joystickX = Xbox.getAnalogHat(LeftHatX, 0);
    joystickY = Xbox.getAnalogHat(LeftHatY, 0);
    
    buttonA = Xbox.getButtonPress(A, 0);
    buttonB = Xbox.getButtonPress(B, 0);
    buttonX = Xbox.getButtonPress(X, 0);
    buttonY = Xbox.getButtonPress(Y, 0);
    
    if (abs(joystickX) < JOYSTICK_DEADZONE) joystickX = 0;
    if (abs(joystickY) < JOYSTICK_DEADZONE) joystickY = 0;
  }
}