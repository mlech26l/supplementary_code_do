#include "RobotLogger.h"
#include <iostream>

RobotLogger::RobotLogger(const char *filename, float interval) {
  file.open(filename, std::ios::out);
  logInterval = interval;

  // Force log point at t=0
  elapsedSinceLastLog = logInterval;
}
void RobotLogger::Close(void) { file.close(); }
void RobotLogger::AddDataPoint(RobotInterface &robot, float deltaT,
                               float totalTime) {
  elapsedSinceLastLog += deltaT;
  if (elapsedSinceLastLog >= logInterval) {
    elapsedSinceLastLog = 0;
    file << totalTime;

    file << "\t" << robot.GetX();
    file << "\t" << robot.GetY();
    file << "\t" << robot.GetTheta();
    file << "\t" << robot.GetVelocity();
    file << "\t" << robot.GetRotationSpeed();

    file << std::endl;
    file << std::flush;
  }
}
