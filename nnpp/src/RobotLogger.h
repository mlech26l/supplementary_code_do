#ifndef ROBOT_LOGGER_H_
#define ROBOT_LOGGER_H_

#include "RobotInterface.h"
#include <fstream>
#include <string>

class RobotLogger {
private:
  std::ofstream file;
  float logInterval;
  float elapsedSinceLastLog;

public:
  RobotLogger(const char *filename, float interval);
  void Close(void);
  void AddDataPoint(RobotInterface &robot, float deltaT, float totalTime);
};
#endif
