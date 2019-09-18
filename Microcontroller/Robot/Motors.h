// NOTE - I DON'T THINK FLIPPING OF DIRECTION WORKS PROPERLY

#ifndef MOTORS_H
#define MOTORS_H

#include<Arduino.h>

namespace MotorState {
enum Enum {
  ON,
  OFF
};
}

namespace MotorDirection {
enum Enum {
  FORWARD = 1,
  BACKWARD = 0
};
}

class Motor {
  public:
    Motor(uint8_t pinDir, uint8_t pinPwm);
    Motor(uint8_t pinDir, uint8_t pinPwm, bool reverse);
    MotorState::Enum getState();
    MotorDirection::Enum getDirection();
    uint8_t getPwmValue();
    void setPwmValue(uint8_t newPwm);
    void setDir(MotorDirection::Enum newDir);
    void setDirAndPwmValue(MotorDirection::Enum newDir, uint8_t newPwm);
    void setDirAndPwmValue(int16_t signedPwm);
    void adjustSpeed(int16_t signedPwmChange);
//    void setState(MotorState::Enum newState);
    

  private:
    const uint8_t pinDir;
    const uint8_t pinPwm;
    MotorDirection::Enum direction;
    MotorDirection::Enum directionPrev;
    uint8_t pwmValue;
    uint8_t pwmValuePrev;
//    MotorState::Enum state;
//    MotorState::Enum statePrev;
    bool flipDir;
    void init();
    void update();
};


#endif
