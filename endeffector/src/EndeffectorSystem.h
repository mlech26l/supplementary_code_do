#ifndef ENDEFFECTOR_SYSTEM_H
#define ENDEFFECTOR_SYSTEM_H

#include "EffectorLink.h"
#include "GripperEffector.h"
#include <tuple>
#include <vector>

class EndeffectorSystem {
private:
  std::vector<EffectorLink *> links;

public:
  EndeffectorSystem();
  void AddLink(EffectorLink *link);
  std::tuple<float, float, float> GetGripperPosition();
  std::tuple<float, float, float> GetGripperDirection();
};

#endif /* end of include guard: ENDEFFECTOR_SYSTEM_H */
