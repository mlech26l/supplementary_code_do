#ifndef JOINT_EFFECTOR_H
#define JOINT_EFFECTOR_H

#include "EffectorLink.h"
#include <cmath>
enum class JointType { Roll, Pitch, Yaw, NegativePitch };
class JointEffector {
private:
  EffectorLink *link;
  JointType jointType;

  float minimumAngle;
  float maximumAngle;

  float currentAngle;

  float desiredAngle;

  // 90 degree in 1 seoncds (actual arm is 180 degree per second)
  float angularVelocity = (M_PI / 2) / 1;
  void SetAngle(float angle); // TODO: Make private
public:
  JointEffector(EffectorLink *correspondingLink, JointType typ, float min,
                float max);
  void SetDesiredAngle(float angle);

  /// -1 = minimumAngle, 0=0, 1=maximumAngle
  void SetDesiredAngleNormalized(float val);

  float GetAngle() { return currentAngle; }
  float GetAngleNormalized();
  void Reset() {
    currentAngle = 0;
    desiredAngle = 0;
  }
  void Update(float deltaT);
};
#endif /* end of include guard: JOINT_EFFECTOR_H */
