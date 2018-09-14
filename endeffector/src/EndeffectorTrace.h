#ifndef ENDEFFECTOR_TRACE_H
#define ENDEFFECTOR_TRACE_H

#include "EndeffectorState.h"

#include <string>
#include <vector>

class EndeffectorTrace {
private:
  std::vector<EndeffectorState> points;
  int nextPoint;
  float currentObjectRecgnizedValue;
  bool verbose;

public:
  EndeffectorTrace(bool _verbose = false);
  void Reset();
  void LoadFromFile(std::string filename);
  float GetObjectRecognizedValue();
  bool EndOfTraceReached();
  float GetDifferenceFromTrace(EndeffectorSystem &endeffector,
                               GripperEffector &gripper, float totalTime);
  EndeffectorTrace *Clone();
};

#endif /* end of include guard: ENDEFFECTOR_TRACE_H */
