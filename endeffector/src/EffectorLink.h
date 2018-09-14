#ifndef EFFECTOR_LINK_H
#define EFFECTOR_LINK_H

#include <cmath>
#include <iostream>
class EffectorLink {
private:
  float roll, pitch, yaw;

  /// Pre compute sin and cos of angle for speedup: Not needed right now
  // float s_roll, c_roll;
  // float s_pitch, c_pitch;
  // float s_yaw, c_yaw;

  float offset_x, offset_y, offset_z;

public:
  EffectorLink(float ox, float oy, float oz);
  void ApplyEffect(float &x, float &y, float &z);
  void ApplyRotation(float &x, float &y, float &z);
  void SetRoll(float r) {
    roll = r;
    // s_roll = std::sin(roll);
    // c_roll = std::cos(roll);
  }
  void SetPitch(float p) {
    pitch = p;
    // s_pitch = std::sin(pitch);
    // c_pitch = std::cos(pitch);
  }
  void SetYaw(float y) {
    yaw = y;
    // s_yaw = std::sin(yaw);
    // c_yaw = std::cos(yaw);
  }
  void Print() { std::cout << roll << " " << pitch << " " << yaw << std::endl; }
};
#endif /* end of include guard:  */
