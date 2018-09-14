#include "Parameter.h"
#include <iostream>
#include <stdlib.h>

void Parameter::AddNoise(float rnd) {
  // rnd in [-1,1]

  float alpha = 0.6f;
  float variation = alpha * value + (1 - alpha) * (upperBound - lowerBound);
  float amplitude = 0.2f;
  float newValue = value + variation * amplitude * rnd;
  if (newValue > upperBound)
    newValue = upperBound;
  else if (newValue < lowerBound)
    newValue = lowerBound;
  value = newValue;
}
