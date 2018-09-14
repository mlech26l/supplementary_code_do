#include <fstream>
#include <iostream>

#include "EffectorLink.h"
#include "EndeffectorSystem.h"
#include "JointEffector.h"

int main(int argc, char const *argv[]) {

  EffectorLink *shoulder_roll = new EffectorLink(0, 0, 54);
  EffectorLink *shoulder_pitch = new EffectorLink(0, 0, 66);
  EffectorLink *elbow_roll = new EffectorLink(0, 0, 70.4);
  EffectorLink *elbow_pitch = new EffectorLink(0, 0, 70.4);
  EffectorLink *elbow_yaw = new EffectorLink(0, 0, 71.8);
  EffectorLink *wrist_pitch = new EffectorLink(0, 0, 71.8);
  EffectorLink *wrist_roll = new EffectorLink(0, 0, 124.6);

  EndeffectorSystem endeffector;
  endeffector.AddLink(shoulder_roll);
  endeffector.AddLink(shoulder_pitch);
  endeffector.AddLink(elbow_roll);
  endeffector.AddLink(elbow_pitch);
  endeffector.AddLink(elbow_yaw);
  endeffector.AddLink(wrist_pitch);
  endeffector.AddLink(wrist_roll);

  JointEffector *shoulder_roll_effector =
      new JointEffector(shoulder_roll, JointType::Yaw, -2.618, 2.618);
  JointEffector *shoulder_pitch_effector =
      new JointEffector(shoulder_pitch, JointType::Roll, -1.8326, 1.8326);
  JointEffector *elbow_roll_effector =
      new JointEffector(elbow_roll, JointType::Yaw, -1.8326, 1.8326);
  JointEffector *elbow_pitch_effector =
      new JointEffector(elbow_pitch, JointType::Roll, -1.8326, 1.8326);
  JointEffector *elbow_yaw_effector =
      new JointEffector(elbow_yaw, JointType::NegativePitch, -1.8326, 1.8326);
  JointEffector *wrist_pitch_effector =
      new JointEffector(wrist_pitch, JointType::Roll, -1.8326, 1.8326);
  JointEffector *wrist_roll_effector =
      new JointEffector(wrist_roll, JointType::Yaw, -2.61799, 2.61799);

  float x;
  float y;
  float z;

  std::tie(x, y, z) = endeffector.GetGripperPosition();

  std::cout << "Endeffector Position: " << x << ", " << y << ", " << z
            << std::endl;

  std::tie(x, y, z) = endeffector.GetGripperDirection();
  std::cout << "Endeffector Direction: " << x << ", " << y << ", " << z
            << std::endl;

  // elbow_pitch->SetRoll(M_PI / 2);
  elbow_pitch_effector->SetDesiredAngle(M_PI / 2);
  elbow_pitch_effector->Update(1.0f);
  // elbow_pitch->Print();

  std::tie(x, y, z) = endeffector.GetGripperPosition();

  std::cout << "Endeffector Position: " << x << ", " << y << ", " << z
            << std::endl;

  std::tie(x, y, z) = endeffector.GetGripperDirection();
  std::cout << "Endeffector Direction: " << x << ", " << y << ", " << z
            << std::endl;
  return 0;
}
