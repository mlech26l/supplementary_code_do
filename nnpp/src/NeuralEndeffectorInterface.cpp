#include "NeuralEndeffectorInterface.h"
#include "EffectorLink.h"
#include <iostream>
#include <tuple>
NeuralEndeffectorInterface::NeuralEndeffectorInterface()
    : shoulder_roll_input_interface(1, 2), shoulder_pitch_input_interface(3, 4),
      elbow_roll_input_interface(5, 6), elbow_pitch_input_interface(7, 8),
      elbow_yaw_input_interface(9, 10), wrist_pitch_input_interface(11, 12),
      wrist_roll_input_interface(13, 14),
      shoulder_roll_output_interface(15, 16),
      shoulder_pitch_output_interface(17, 18),
      elbow_roll_output_interface(19, 20), elbow_pitch_output_interface(21, 22),
      elbow_yaw_output_interface(23, 24), wrist_pitch_output_interface(25, 26),
      wrist_roll_output_interface(27, 28), gripper_input_interface(29),
      gripper_output_interface(30), object_recognized_interface(31) {
  EffectorLink *shoulder_roll = new EffectorLink(0, 0, 54);
  EffectorLink *shoulder_pitch = new EffectorLink(0, 0, 66);
  EffectorLink *elbow_roll = new EffectorLink(0, 0, 70.4);
  EffectorLink *elbow_pitch = new EffectorLink(0, 0, 70.4);
  EffectorLink *elbow_yaw = new EffectorLink(0, 0, 71.8);
  EffectorLink *wrist_pitch = new EffectorLink(0, 0, 71.8);
  EffectorLink *wrist_roll = new EffectorLink(0, 0, 124.6);

  endeffector.AddLink(shoulder_roll);
  endeffector.AddLink(shoulder_pitch);
  endeffector.AddLink(elbow_roll);
  endeffector.AddLink(elbow_pitch);
  endeffector.AddLink(elbow_yaw);
  endeffector.AddLink(wrist_pitch);
  endeffector.AddLink(wrist_roll);

  shoulder_roll_effector =
      new JointEffector(shoulder_roll, JointType::Yaw, -2.618, 2.618);
  shoulder_pitch_effector =
      new JointEffector(shoulder_pitch, JointType::Roll, -1.8326, 1.8326);
  elbow_roll_effector =
      new JointEffector(elbow_roll, JointType::Yaw, -1.8326, 1.8326);
  elbow_pitch_effector =
      new JointEffector(elbow_pitch, JointType::Roll, -1.8326, 1.8326);
  elbow_yaw_effector =
      new JointEffector(elbow_yaw, JointType::NegativePitch, -1.8326, 1.8326);
  wrist_pitch_effector =
      new JointEffector(wrist_pitch, JointType::Roll, -1.8326, 1.8326);
  wrist_roll_effector =
      new JointEffector(wrist_roll, JointType::Yaw, -2.61799, 2.61799);

  // shoulder_roll_input_interface(1, 2);
  // shoulder_pitch_input_interface(3, 4);
  // elbow_roll_input_interface(5, 6);
  // elbow_pitch_input_interface(7, 8);
  // elbow_yaw_input_interface(9, 10);
  // wrist_pitch_input_interface(11, 12);
  // wrist_roll_input_interface(13, 14);
  //
  // shoulder_roll_output_interface(15, 16);
  // shoulder_pitch_output_interface(17, 18);
  // elbow_roll_output_interface(19, 20);
  // elbow_pitch_output_interface(21, 22);
  // elbow_yaw_output_interface(23, 24);
  // wrist_pitch_output_interface(25, 26);
  // wrist_roll_output_interface(27, 28);
  //
  // gripper_input_interface(29);
  // gripper_output_interface(30);
}
void NeuralEndeffectorInterface::SetObjectRecognizedValue(float value) {
  object_recognized_interface.SetValue(value);
}
void NeuralEndeffectorInterface::Reset() {
  shoulder_roll_input_interface.Reset();
  shoulder_pitch_input_interface.Reset();
  elbow_roll_input_interface.Reset();
  elbow_pitch_input_interface.Reset();
  elbow_yaw_input_interface.Reset();
  wrist_pitch_input_interface.Reset();
  wrist_roll_input_interface.Reset();
  gripper_input_interface.Reset();

  gripper_output_interface.Reset();
  shoulder_roll_output_interface.Reset();
  shoulder_pitch_output_interface.Reset();
  elbow_roll_output_interface.Reset();
  elbow_pitch_output_interface.Reset();
  elbow_yaw_output_interface.Reset();
  wrist_pitch_output_interface.Reset();
  wrist_roll_output_interface.Reset();

  shoulder_roll_effector->Reset();
  shoulder_pitch_effector->Reset();
  elbow_roll_effector->Reset();
  elbow_pitch_effector->Reset();
  elbow_yaw_effector->Reset();
  wrist_pitch_effector->Reset();
  wrist_roll_effector->Reset();

  gripper.Reset();
}
void NeuralEndeffectorInterface::SyncWithNetwork(NeuralNetwork &nn) {
  shoulder_roll_input_interface.Sync(nn);
  shoulder_pitch_input_interface.Sync(nn);
  elbow_roll_input_interface.Sync(nn);
  elbow_pitch_input_interface.Sync(nn);
  elbow_yaw_input_interface.Sync(nn);
  wrist_pitch_input_interface.Sync(nn);
  wrist_roll_input_interface.Sync(nn);
  gripper_input_interface.Sync(nn);

  object_recognized_interface.Sync(nn);

  gripper_output_interface.Sync(nn);
  shoulder_roll_output_interface.Sync(nn);
  shoulder_pitch_output_interface.Sync(nn);
  elbow_roll_output_interface.Sync(nn);
  elbow_pitch_output_interface.Sync(nn);
  elbow_yaw_output_interface.Sync(nn);
  wrist_pitch_output_interface.Sync(nn);
  wrist_roll_output_interface.Sync(nn);
}
void NeuralEndeffectorInterface::Update(float deltaT) {
  shoulder_roll_effector->Update(deltaT);
  shoulder_pitch_effector->Update(deltaT);
  elbow_roll_effector->Update(deltaT);
  elbow_pitch_effector->Update(deltaT);
  elbow_yaw_effector->Update(deltaT);
  wrist_pitch_effector->Update(deltaT);
  wrist_roll_effector->Update(deltaT);

  gripper.Update(deltaT);
}
void NeuralEndeffectorInterface::SyncWithEndEffectorSystem() {

  // std::cout << "Desired elbow roll " <<
  // elbow_roll_output_interface.GetValue()
  //           << std::endl;
  // std::cout << "Desired gripper state " <<
  // gripper_output_interface.GetValue()
  //           << std::endl;
  // std::cout << "Current gripper state " << gripper.GetGripperState()
  //           << std::endl;
  shoulder_roll_input_interface.SetValue(
      shoulder_roll_effector->GetAngleNormalized());

  shoulder_pitch_input_interface.SetValue(
      shoulder_pitch_effector->GetAngleNormalized());

  elbow_roll_input_interface.SetValue(
      elbow_roll_effector->GetAngleNormalized());

  elbow_pitch_input_interface.SetValue(
      elbow_pitch_effector->GetAngleNormalized());

  elbow_yaw_input_interface.SetValue(elbow_yaw_effector->GetAngleNormalized());

  wrist_pitch_input_interface.SetValue(
      wrist_pitch_effector->GetAngleNormalized());

  wrist_roll_input_interface.SetValue(
      wrist_roll_effector->GetAngleNormalized());

  gripper_input_interface.SetValue(gripper.GetGripperState());

  gripper.SetDesiredGripperState(gripper_output_interface.GetValue());

  shoulder_roll_effector->SetDesiredAngleNormalized(
      shoulder_roll_output_interface.GetValue());

  shoulder_pitch_effector->SetDesiredAngleNormalized(
      shoulder_pitch_output_interface.GetValue());

  elbow_roll_effector->SetDesiredAngleNormalized(
      elbow_roll_output_interface.GetValue());

  elbow_pitch_effector->SetDesiredAngleNormalized(
      elbow_pitch_output_interface.GetValue());

  elbow_yaw_effector->SetDesiredAngleNormalized(
      elbow_yaw_output_interface.GetValue());

  wrist_pitch_effector->SetDesiredAngleNormalized(
      wrist_pitch_output_interface.GetValue());

  wrist_roll_effector->SetDesiredAngleNormalized(
      wrist_roll_output_interface.GetValue());
}
void NeuralEndeffectorInterface::Log(float totalTime, std::ofstream &file) {
  file << totalTime << " ";

  float x, y, z;
  std::tie(x, y, z) = endeffector.GetGripperPosition();
  file << x << " ";
  file << y << " ";
  file << z << " ";
  std::tie(x, y, z) = endeffector.GetGripperDirection();
  file << x << " ";
  file << y << " ";
  file << z << " ";

  file << shoulder_roll_effector->GetAngle() << " ";
  file << shoulder_pitch_effector->GetAngle() << " ";
  file << elbow_roll_effector->GetAngle() << " ";
  file << elbow_pitch_effector->GetAngle() << " ";
  file << elbow_yaw_effector->GetAngle() << " ";
  file << wrist_pitch_effector->GetAngle() << " ";
  file << wrist_roll_effector->GetAngle() << " ";
  file << gripper.GetGripperState() << " ";
  file << std::endl;
}
