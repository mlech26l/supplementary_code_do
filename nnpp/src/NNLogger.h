#ifndef NN_LOGGER_H_
#define NN_LOGGER_H_

#include "neural_network.h"
#include <fstream>
#include <string>

class NNLogger {
private:
  std::ofstream file;
  float logInterval;
  float elapsedSinceLastLog;

public:
  NNLogger(const char *filename, float interval);
  void Close(void);
  void AddDataPoint(NeuralNetwork &nn, float deltaT, float totalTime);
};
#endif
