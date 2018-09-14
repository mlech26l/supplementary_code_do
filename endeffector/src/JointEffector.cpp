#include "JointEffector.h"
#include <iostream>

JointEffector::JointEffector(EffectorLink *correspondingLink, JointType typ,
                             float min, float max) {
  link = correspondingLink;
  jointType = typ;
  minimumAngle = min;
  maximumAngle = max;
  currentAngle = 0;
  desiredAngle = 0;
}
void JointEffector::SetDesiredAngle(float angle) {
  if (angle > maximumAngle)
    desiredAngle = maximumAngle;
  else if (angle < minimumAngle)
    desiredAngle = minimumAngle;
  else
    desiredAngle = angle;
}
void JointEffector::SetDesiredAngleNormalized(float val) {
  if (val < 0)
    SetDesiredAngle(-val * minimumAngle);
  else
    SetDesiredAngle(val * maximumAngle);
}
void JointEffector::SetAngle(float angle) {
  currentAngle = angle;
  if (jointType == JointType::Roll) {
    link->SetRoll(currentAngle);
  } else if (jointType == JointType::Pitch) {
    link->SetPitch(currentAngle);
  } else if (jointType == JointType::NegativePitch) {
    link->SetPitch(-currentAngle);
  } else {
    link->SetYaw(currentAngle);
  }
}
void JointEffector::Update(float deltaT) {
  if (desiredAngle >= currentAngle) {
    currentAngle += deltaT * angularVelocity;
    if (currentAngle > desiredAngle)
      currentAngle = desiredAngle;
  } else {
    currentAngle -= deltaT * angularVelocity;
    if (currentAngle < desiredAngle)
      currentAngle = desiredAngle;
  }
  SetAngle(currentAngle);
}
float JointEffector::GetAngleNormalized() {
  if (currentAngle >= 0)
    return currentAngle / maximumAngle;
  else
    return -(currentAngle / minimumAngle);
}
