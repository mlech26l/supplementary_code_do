#include "RobotInterface.h"
#include "neuron_util.h"
#include <iostream>
#include <math.h>

RobotInterface::RobotInterface() { Reset(); }
void RobotInterface::Reset() {
  x = 0;
  y = 0;
  v = 0;
  theta = 0;
  w = 0;
}
RobotInterface *RobotInterface::Clone() {
  RobotInterface *clone = new RobotInterface();
  clone->BindToNeuralNetwork(sensoryNeuronX, sensoryNeuronY, sensoryNeuronTheta,
                             motoryNeuronMotor, motoryNeuronMotorBackwards,
                             motoryNeuronRotatePositiv,
                             motoryNeuronRotateNegativ);
  return clone;
}
void RobotInterface::BindToNeuralNetwork(int s_x, int s_y, int s_t, int m_fwd,
                                         int m_bwd, int m_rpos, int m_rneg) {
  sensoryNeuronX = s_x;
  sensoryNeuronY = s_y;
  sensoryNeuronTheta = s_t;
  motoryNeuronMotor = m_fwd;
  motoryNeuronMotorBackwards = m_bwd;
  motoryNeuronRotatePositiv = m_rpos;
  motoryNeuronRotateNegativ = m_rneg;
}
void RobotInterface::Actuate(NeuralNetwork &nn) {

  // Rotation
  // default value = min
  float neuronPotentialRotatePositiv = MinmumActuatePotentialSteering;
  if (motoryNeuronRotatePositiv >= 0)
    neuronPotentialRotatePositiv = nn.GetPotentialOf(motoryNeuronRotatePositiv);

  // default value = min
  float neuronPotentialRotateNegativ = MinmumActuatePotentialSteering;
  if (motoryNeuronRotateNegativ >= 0)
    neuronPotentialRotateNegativ = nn.GetPotentialOf(motoryNeuronRotateNegativ);

  float w_pos = bounded_affine(MinmumActuatePotentialSteering, 0,
                               MaximumActuatePotentialSteering, SteeringSpeed,
                               neuronPotentialRotatePositiv);
  float w_neg = bounded_affine(MinmumActuatePotentialSteering, 0,
                               MaximumActuatePotentialSteering, SteeringSpeed,
                               neuronPotentialRotateNegativ);
  w = w_pos - w_neg;

  // Linear Movement
  // default value = min
  float neuronPotentialMotor = MinmumActuatePotential;
  if (motoryNeuronMotor >= 0)
    neuronPotentialMotor = nn.GetPotentialOf(motoryNeuronMotor);

  // default value = min
  float neuronPotentialMotorBackwards = MinmumActuatePotential;
  if (motoryNeuronMotorBackwards >= 0)
    neuronPotentialMotorBackwards =
        nn.GetPotentialOf(motoryNeuronMotorBackwards);

  float v_pos =
      bounded_affine(MinmumActuatePotential, 0, MaximumActuatePotential,
                     MotorSpeedForward, neuronPotentialMotor);
  float v_neg =
      bounded_affine(MinmumActuatePotential, 0, MaximumActuatePotential,
                     MotorSpeedBackward, neuronPotentialMotorBackwards);

  v = v_pos - v_neg;
  // std::cout << "w(" << w_pos << "/" << w_neg << "/" << w << "), v(" << v_pos
  // << "/" << v_neg << "/" << v << ")" << std::endl;
}
void RobotInterface::Sense(NeuralNetwork &nn) {
  float Vx = (MaximumSensePotential - MimiumSensePotential) / AreaSize * x +
             MimiumSensePotential;

  float Vy = (MaximumSensePotential - MimiumSensePotential) / AreaSize * y +
             MimiumSensePotential;

  if (sensoryNeuronX >= 0)
    nn.ForcePotentialOf(sensoryNeuronX, Vx);
  if (sensoryNeuronY >= 0)
    nn.ForcePotentialOf(sensoryNeuronY, Vy);

  float Vtheta =
      (MaximumSensePotential - MimiumSensePotential) / ThetaMax * theta +
      MimiumSensePotential;
  if (sensoryNeuronTheta >= 0)
    nn.ForcePotentialOf(sensoryNeuronTheta, Vtheta);
}
void RobotInterface::AddSensoryNoise(NeuralNetwork &nn, float n_x, float n_y,
                                     float n_t) {
  if (sensoryNeuronX >= 0) {
    float v = nn.GetPotentialOf(sensoryNeuronX);
    nn.ForcePotentialOf(sensoryNeuronX, v + n_x);
  }
  if (sensoryNeuronY >= 0) {
    float v = nn.GetPotentialOf(sensoryNeuronY);
    nn.ForcePotentialOf(sensoryNeuronY, v + n_y);
  }

  if (sensoryNeuronTheta >= 0) {
    float v = nn.GetPotentialOf(sensoryNeuronTheta);
    nn.ForcePotentialOf(sensoryNeuronTheta, v + n_t);
  }
}
