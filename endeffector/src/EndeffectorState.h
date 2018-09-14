#ifndef ENDEFFECTOR_STATE_H
#define ENDEFFECTOR_STATE_H

#include "EndeffectorSystem.h"
#include "GripperEffector.h"
class EndeffectorState {
private:
  float t;
  float x, y, z;
  float roll, pitch, yaw;
  float gripperState;
  float object_recognized;

public:
  EndeffectorState(float _t, float _x, float _y, float _z, float _roll,
                   float _pitch, float _yaw, float _gripper, float _or) {
    t = _t;
    x = _x;
    y = _y;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    gripperState = _gripper;
    object_recognized = _or;
  }
  float GetTimePoint() { return t; }
  float GetObjectRecognizedValue() { return object_recognized; }
  float Distance(EndeffectorSystem &endeffector, GripperEffector &gripper);
};
#endif /* end of include guard: ENDEFFECTOR_STATE_H */
