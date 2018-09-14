#include "RobotDynamics.h"
#include <iostream>

RobotDynamics::RobotDynamics() { Reset(); }
void RobotDynamics::Reset() {
  totalTime = 0;
  x = 0;
  y = 0;
  v_is = 0;
  v_want = 0;
  theta = 0;
  w_is = 0;
  w_want = 0;

  while (!linearActuatorCommandQueue.empty()) {
    linearActuatorCommandQueue.pop();
  }
  while (!angularActuatorCommandQueue.empty()) {
    angularActuatorCommandQueue.pop();
  }
}

void RobotDynamics::CheckActuatorCommandQueue() {
  if (linearActuatorCommandQueue.size() > 0) {
    std::pair<float, float> firstItemLinearActuator =
        linearActuatorCommandQueue.front();
    if (firstItemLinearActuator.first <= totalTime) {
      v_want = firstItemLinearActuator.second;
      linearActuatorCommandQueue.pop();
    }
  }

  if (angularActuatorCommandQueue.size() > 0) {
    std::pair<float, float> firstItemAngularActuator =
        angularActuatorCommandQueue.front();
    if (firstItemAngularActuator.first <= totalTime) {
      w_want = firstItemAngularActuator.second;
      angularActuatorCommandQueue.pop();
    }
  }
}
void RobotDynamics::DoSimulationStep(float deltaT) {
  totalTime += deltaT;

  CheckActuatorCommandQueue();

  bool accelerate = v_want >= v_is;

  // Acclerate to v_want
  if (accelerate) {
    v_is += Acceleration * deltaT;
    if (v_is > v_want)
      v_is = v_want;
  } else {
    v_is -= Acceleration * deltaT;
    // Decclerate to v_want
    if (v_is < v_want)
      v_is = v_want;
  }

  bool angularAccelerate = w_want >= w_is;

  // Acclerate to w_want
  if (angularAccelerate) {
    w_is += AngularAcceleration * deltaT;
    if (w_is > w_want)
      w_is = w_want;
  } else {
    w_is -= AngularAcceleration * deltaT;
    // Decclerate to w_want
    if (w_is < w_want)
      w_is = w_want;
  }

  x += deltaT * cos(theta) * v_is;
  y += deltaT * sin(theta) * v_is;

  theta += deltaT * w_is;
  if (theta > M_PI / 2)
    theta -= M_PI;
  if (theta < -M_PI / 2)
    theta += M_PI;
}
void RobotDynamics::GetCommandsFromInterface(RobotInterface &robotInterface) {

  std::pair<float, float> enqueItemLinearActuator = std::make_pair(
      totalTime + linearActuatorDelay, robotInterface.GetVelocity());
  linearActuatorCommandQueue.push(enqueItemLinearActuator);

  std::pair<float, float> enqueItemAngularActuator = std::make_pair(
      totalTime + angularActuatorDelay, robotInterface.GetRotationSpeed());
  angularActuatorCommandQueue.push(enqueItemAngularActuator);
}
void RobotDynamics::SendInputsToInterface(RobotInterface &robotInterface) {
  robotInterface.SetTheta(theta);
  robotInterface.SetX(x);
  robotInterface.SetY(y);
}
