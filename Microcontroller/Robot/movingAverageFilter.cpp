#include"movingAverageFilter.h"

float movingAverageFilter::update(float newReading) {
  idx++;
  if (idx >= size) {
    idx = 0;
  }
  total -= buffer[idx];
  buffer[idx] = newReading;
  total += buffer[idx];
  output = total / size;
  return output;
}

float movingAverageFilter::latest() {
  return buffer[idx];
}
