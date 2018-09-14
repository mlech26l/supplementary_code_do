#include "GripperEffector.h"

void GripperEffector::SetDesiredGripperState(float state) {
  desiredGripperState = state;
  if (desiredGripperState > 1)
    desiredGripperState = 1;
  if (desiredGripperState < 0)
    desiredGripperState = 0;
}
void GripperEffector::Update(float deltaT) {
  if (desiredGripperState > gripperState) {
    // opening
    gripperState += deltaT * gripperSpeed;
    if (gripperState > desiredGripperState)
      gripperState = desiredGripperState;
  } else {
    gripperState -= deltaT * gripperSpeed;
    if (gripperState < desiredGripperState)
      gripperState = desiredGripperState;
  }
}
