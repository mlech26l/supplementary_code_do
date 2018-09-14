#include "EffectorLink.h"

EffectorLink::EffectorLink(float ox, float oy, float oz) {
  offset_x = ox;
  offset_y = oy;
  offset_z = oz;

  SetPitch(0);
  SetRoll(0);
  SetYaw(0);
}
void EffectorLink::ApplyEffect(float &x, float &y, float &z) {
  float new_x = cos(yaw) * cos(pitch) * x +
                cos(yaw) * (sin(pitch) * sin(roll) - sin(yaw) * cos(roll)) * y +
                (cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)) * z;

  float new_y = sin(yaw) * cos(pitch) * x +
                (sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll)) * y +
                (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)) * z;

  float new_z =
      -sin(pitch) * x + cos(pitch) * sin(roll) * y + cos(pitch) * cos(roll) * z;

  new_x += offset_x;
  new_y += offset_y;
  new_z += offset_z;

  x = new_x;
  y = new_y;
  z = new_z;
}
void EffectorLink::ApplyRotation(float &x, float &y, float &z) {
  float new_x = cos(yaw) * cos(pitch) * x +
                cos(yaw) * (sin(pitch) * sin(roll) - sin(yaw) * cos(roll)) * y +
                (cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)) * z;

  float new_y = sin(yaw) * cos(pitch) * x +
                (sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll)) * y +
                (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)) * z;

  float new_z =
      -sin(pitch) * x + cos(pitch) * sin(roll) * y + cos(pitch) * cos(roll) * z;

  x = new_x;
  y = new_y;
  z = new_z;
}
