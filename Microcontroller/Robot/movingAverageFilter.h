#ifndef MOVING_AVERAGE_FILTER_H
#define MOVING_AVERAGE_FILTER_H

#include <Arduino.h>

class movingAverageFilter {
  public:
    static const uint8_t size = 20;
    float buffer[size];
    uint8_t idx;
    float total;
    float output;
    float update(float newReading);
    float latest();
};


#endif
