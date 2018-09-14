#include "NNLogger.h"
#include <iostream>

NNLogger::NNLogger(const char *filename, float interval) {
  file.open(filename, std::ios::out);
  logInterval = interval;

  // Force log point at t=0
  elapsedSinceLastLog = logInterval;
}
void NNLogger::Close(void) { file.close(); }
void NNLogger::AddDataPoint(NeuralNetwork &nn, float deltaT, float totalTime) {
  elapsedSinceLastLog += deltaT;
  if (elapsedSinceLastLog >= logInterval) {
    elapsedSinceLastLog = 0;
    file << totalTime;

    for (int i = 0; i < nn.GetSize(); i++) {
      file << "\t";
      file << nn.GetPotentialOf(i);
    }
    file << std::endl;
  }
}
