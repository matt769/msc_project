// NOTE - I DON'T THINK FLIPPING OF DIRECTION WORKS PROPERLY

#include "Motors.h"

// Should I handle pin modes here as well?

Motor::Motor(const uint8_t pinDir, const uint8_t pinPwm): pinDir(pinDir), pinPwm(pinPwm) {
  //state = MotorState::OFF;
  init();
};

Motor::Motor(const uint8_t pinDir, const uint8_t pinPwm, bool reverse): pinDir(pinDir), pinPwm(pinPwm) {
  //state = MotorState::OFF;
  init();
  flipDir = reverse;
};


void Motor::init() {
  pwmValue = 0;
  pwmValuePrev = 0;
  direction = MotorDirection::FORWARD;
  directionPrev = MotorDirection::BACKWARD; // just to force a write in update function()
  pinMode(pinDir, OUTPUT);
  pinMode(pinPwm, OUTPUT);
  update();
}

MotorState::Enum Motor::getState() {
  if (pwmValue != 0) {
    return MotorState::ON;
  }
  else {
    return MotorState::OFF;
  }
}

MotorDirection::Enum Motor::getDirection() {
  return direction;
}

uint8_t Motor::getPwmValue() {
  return pwmValue;
}

void Motor::setPwmValue(uint8_t newPwm) {
  pwmValue = newPwm;
  update();
}

void Motor::setDir(MotorDirection::Enum newDir){
  direction = newDir;
  update();
}

void Motor::setDirAndPwmValue(MotorDirection::Enum newDir, uint8_t newPwm) {
  // add control of forward/backward transition?
  setDir(newDir);
  setPwmValue(newPwm);
}

//void Motor::setState(MotorState::Enum newState) {
//  state = newState;
//}


void Motor::setDirAndPwmValue(int16_t signedPwm){
  MotorDirection::Enum newDir;
  if(signedPwm >= 0){
    newDir = MotorDirection::FORWARD;
  }
  else if (signedPwm < 0){
    newDir = MotorDirection::BACKWARD;
  }
  uint16_t newPwm = abs(signedPwm);
  newPwm = constrain(newPwm, 0, 255);
  
  setDirAndPwmValue(newDir, (uint8_t)newPwm);
}

//void Motor::adjustSpeed(int16_t signedPwmChange){
//  // TODO remove this - it doesn't really make sense since it can't handle direction correctly
//  // with the way the class operates at the moment
//}



void Motor::update() {
  if (pwmValue != pwmValuePrev) {
    analogWrite(pinPwm, pwmValue);
  }
  
  if (direction != directionPrev) {
    uint8_t directionMod;
    if (flipDir) {
      directionMod = 1 - (uint8_t)direction;
    }
    else {
      directionMod = (uint8_t)direction;
    }
    digitalWriteFast(pinDir, directionMod);
  }
  
  pwmValuePrev = pwmValue;
  directionPrev = direction;
  //  statePrev = state;
}
