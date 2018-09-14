#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_
#include "neural_network.h"
#include <math.h>

class RobotInterface {
private:
  float MaximumActuatePotential = -20;
  float MinmumActuatePotential = -60;

  float MaximumActuatePotentialSteering = -30;
  float MinmumActuatePotentialSteering = -60;

  // 360 degree in 3 seconds
  float SteeringSpeed = 0.18;

  float AreaSize = 2;
  // whole area in 3 seconds
  // float MotorSpeedForward = 0.30;
  float MotorSpeedForward = 0.16;
  float MotorSpeedBackward = 0.16;

  float ThetaMax = M_PI / 2;
  float MaximumSensePotential = 0;
  float MimiumSensePotential = -70;

  float x;
  float y;
  float theta;
  float v;
  float w;

  int sensoryNeuronX;
  int sensoryNeuronY;
  int sensoryNeuronTheta;
  int motoryNeuronMotor;
  int motoryNeuronMotorBackwards;
  int motoryNeuronRotatePositiv;
  int motoryNeuronRotateNegativ;

public:
  RobotInterface();
  void Reset();
  void BindToNeuralNetwork(int s_x, int s_y, int s_t, int m_fwd, int m_bwd,
                           int m_rpos, int m_rneg);
  void Actuate(NeuralNetwork &nn);
  void Sense(NeuralNetwork &nn);
  void AddSensoryNoise(NeuralNetwork &nn, float n_x, float n_y, float n_t);

  void SetX(float nx) { x = nx; }
  void SetY(float ny) { y = ny; }
  void SetTheta(float nt) { theta = nt; }
  float GetX() { return x; }
  float GetY() { return y; }
  float GetTheta() { return theta; }
  float GetVelocity() { return v; }
  float GetRotationSpeed() { return w; }
  void SetActuators(float new_v, float new_w) {
    v = new_v;
    w = new_w;
  }
  RobotInterface *Clone();
  // void SetMaximumMotorSpeed(float maxMotorSpeed) { MotorSpeed =
  // maxMotorSpeed; }

public:
};
#endif
