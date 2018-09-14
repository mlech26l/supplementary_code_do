#ifndef NEURAL_ENDEFFECTOR_INTERFACE
#define NEURAL_ENDEFFECTOR_INTERFACE

#include "EndeffectorSystem.h"
#include "GripperEffector.h"
#include <fstream>

#include "JointEffector.h"
#include "NeuralInterface.h"
class NeuralEndeffectorInterface {
private:
  NeuralBiInterfaceInput shoulder_roll_input_interface;
  NeuralBiInterfaceInput shoulder_pitch_input_interface;
  NeuralBiInterfaceInput elbow_roll_input_interface;
  NeuralBiInterfaceInput elbow_pitch_input_interface;
  NeuralBiInterfaceInput elbow_yaw_input_interface;
  NeuralBiInterfaceInput wrist_pitch_input_interface;
  NeuralBiInterfaceInput wrist_roll_input_interface;

  NeuralBiInterfaceOutput shoulder_roll_output_interface;
  NeuralBiInterfaceOutput shoulder_pitch_output_interface;
  NeuralBiInterfaceOutput elbow_roll_output_interface;
  NeuralBiInterfaceOutput elbow_pitch_output_interface;
  NeuralBiInterfaceOutput elbow_yaw_output_interface;
  NeuralBiInterfaceOutput wrist_pitch_output_interface;
  NeuralBiInterfaceOutput wrist_roll_output_interface;

  EndeffectorSystem endeffector;
  JointEffector *shoulder_roll_effector;
  JointEffector *shoulder_pitch_effector;
  JointEffector *elbow_roll_effector;
  JointEffector *elbow_pitch_effector;
  JointEffector *elbow_yaw_effector;
  JointEffector *wrist_pitch_effector;
  JointEffector *wrist_roll_effector;

  GripperEffector gripper;
  NeuralInterfaceInput gripper_input_interface;
  NeuralInterfaceOutput gripper_output_interface;

  NeuralInterfaceInput object_recognized_interface;

public:
  NeuralEndeffectorInterface();
  void SyncWithNetwork(NeuralNetwork &nn);
  void SyncWithEndEffectorSystem();
  void Update(float deltaT);
  void Log(float totalTime, std::ofstream &file);
  void Reset();

  void SetObjectRecognizedValue(float value);
  GripperEffector *GetGripperObject() { return &gripper; }
  EndeffectorSystem *GetEndeffectorSystemObject() { return &endeffector; }
};
#endif /* end of include guard: NEURAL_ENDEFFECTOR_INTERFACE */
