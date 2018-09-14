#include "RobotTrace.h"
#include "RobotInterface.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

RobotTrace::RobotTrace() { nextPoint = 0; }
void RobotTrace::Reset() { nextPoint = 0; }
void RobotTrace::AddPoint(float t, float x, float y, bool containsTheta,
                          float theta) {
  points.push_back(RobotState(t, x, y, containsTheta, theta));
}
bool RobotTrace::EndOfTraceReached() { return nextPoint >= (int)points.size(); }
RobotTrace *RobotTrace::Clone() {
  RobotTrace *clone = new RobotTrace();
  for (unsigned int i = 0; i < points.size(); i++) {
    clone->AddPoint(points[i].GetTimePoint(), points[i].GetX(),
                    points[i].GetY(), points[i].GetContainsTheta(),
                    points[i].GetTheta());
  }
  return clone;
}
float RobotTrace::GetDifferenceFromTrace(RobotInterface &robot,
                                         float totalTime) {
  if (EndOfTraceReached())
    return 0;

  float nextTime = points[nextPoint].GetTimePoint();
  if (totalTime >= nextTime) {
    float retval = points[nextPoint].Distance(robot);
    // std::cout << "Distance of robot: " << retval << " (" << robot.GetX() <<
    // ","
    //           << robot.GetY() << ") vs (" << points[nextPoint].GetX() << ","
    //           << points[nextPoint].GetY() << ")" << std::endl;
    nextPoint++;
    return retval;
  }
  return 0;
}
void RobotTrace::LoadFromFile(std::string filename) {
  std::ifstream file;
  file.open(filename, std::ios::in);
  while (!file.eof()) {
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};

    if (tokens.size() != 3 && tokens.size() != 4)
      continue;

    float t = std::stof(tokens[0]);
    float x = std::stof(tokens[1]);
    float y = std::stof(tokens[2]);

    float theta = 0;
    bool containsTheta = false;

    if (tokens.size() == 4) {
      containsTheta = true;
      theta = std::stof(tokens[3]);
    }
    AddPoint(t, x, y, containsTheta, theta);
  }
  file.close();
}
