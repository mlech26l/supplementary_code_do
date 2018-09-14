#ifndef GRIPPER_EFFECTOR_H
#define GRIPPER_EFFECTOR_H

class GripperEffector {
private:
  // between 0 and 1
  float gripperState;

  float desiredGripperState;
  // full open/close in 1 second(s)
  float gripperSpeed = 1.0f / 1.0f;

public:
  GripperEffector() {
    gripperState = 0;
    desiredGripperState = 0;
  }
  void SetDesiredGripperState(float state);
  float GetGripperState() { return gripperState; }
  void Update(float deltaT);
  void Reset() {
    gripperState = 0;
    desiredGripperState = 0;
  }
};
#endif /* end of include guard: GRIPPER_EFFECTOR)H */
