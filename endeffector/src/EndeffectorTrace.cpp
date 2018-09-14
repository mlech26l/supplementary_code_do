#include "EndeffectorTrace.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
EndeffectorTrace::EndeffectorTrace(bool _verbose) {
  nextPoint = 0;
  verbose = _verbose;
  currentObjectRecgnizedValue = 0;
}
void EndeffectorTrace::Reset() {
  nextPoint = 0;
  currentObjectRecgnizedValue = 0;
}
bool EndeffectorTrace::EndOfTraceReached() {
  return nextPoint >= (int)points.size();
}
EndeffectorTrace *EndeffectorTrace::Clone() {
  // no dynamic members -> can use default copy constructor
  EndeffectorTrace *clone = new EndeffectorTrace(*this);
  return clone;
}
float EndeffectorTrace::GetDifferenceFromTrace(EndeffectorSystem &endeffector,
                                               GripperEffector &gripper,
                                               float totalTime) {
  if (EndOfTraceReached())
    return 0;

  float nextTime = points[nextPoint].GetTimePoint();
  if (totalTime >= nextTime) {
    currentObjectRecgnizedValue = points[nextPoint].GetObjectRecognizedValue();
    float retval = points[nextPoint].Distance(endeffector, gripper);
    if (verbose) {
      std::cout << "Cost function term at " << totalTime << ": " << retval
                << std::endl;
    }
    // std::cout << "Distance of robot: " << retval << " (" << robot.GetX() <<
    // ","
    //           << robot.GetY() << ") vs (" << points[nextPoint].GetX() << ","
    //           << points[nextPoint].GetY() << ")" << std::endl;
    nextPoint++;
    return retval;
  }
  return 0;
}
float EndeffectorTrace::GetObjectRecognizedValue() {
  return currentObjectRecgnizedValue;
}
void EndeffectorTrace::LoadFromFile(std::string filename) {
  std::ifstream file;
  file.open(filename, std::ios::in);
  while (!file.eof()) {
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    std::vector<std::string> tokens{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};

    if (tokens.size() != 9)
      continue;

    float t = std::stof(tokens[0]);
    float x = std::stof(tokens[1]);
    float y = std::stof(tokens[2]);
    float z = std::stof(tokens[3]);
    float roll = std::stof(tokens[4]);
    float pitch = std::stof(tokens[5]);
    float yaw = std::stof(tokens[6]);
    float gripper = std::stof(tokens[7]);
    float _or = std::stof(tokens[8]);

    points.push_back(
        EndeffectorState(t, x, y, z, roll, pitch, yaw, gripper, _or));
  }
  file.close();
}
