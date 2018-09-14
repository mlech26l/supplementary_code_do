#include "RobotTrajectoryCostModel.h"

#include "pulse_neuron.h"
#include <iostream>

float RobotTrajectoryCostModel::Evaluate(NeuralNetwork &nn) {
  RobotDynamics robotDynamics;
  robot->Reset();
  nn.Reset();
  robotTrace->Reset();
  float totalTime = 0;

  float cost = 0;
  float deltaT = 0.01f;

  float sync_counter = 0;
  float sync_value = 0.1f;
  while (!robotTrace->EndOfTraceReached()) {
    if (verbose && totalTime < 0.1f) {
      std::cout << "T: " << totalTime << ": " << robot->GetX() << ", "
                << robot->GetY() << ", " << robot->GetTheta()
                << robot->GetVelocity() << ", " << robot->GetRotationSpeed()
                << std::endl;
    }
    pulse.Update(nn, totalTime);

    robot->Sense(nn);
    nn.DoSimulationStep(deltaT);
    robot->Actuate(nn);
    robotDynamics.DoSimulationStep(deltaT);

    if (sync_counter >= sync_value) {
      sync_counter = 0;
      robotDynamics.GetCommandsFromInterface(*robot);
      robotDynamics.SendInputsToInterface(*robot);
    }
    sync_counter += deltaT;
    totalTime += deltaT;
    float add = robotTrace->GetDifferenceFromTrace(*robot, totalTime);
    if (verbose && add != 0) {
      std::cout << "Adding cost " << add << " at " << totalTime << ": "
                << robot->GetX() << ", " << robot->GetY() << ", "
                << robot->GetTheta() << std::endl;
    }
    cost += add;
  }
  return cost;
}
OptimizerCostModel *RobotTrajectoryCostModel::Clone() {
  RobotTrace *rTrace = robotTrace->Clone();
  RobotInterface *rInterface = robot->Clone();
  OptimizerCostModel *clone =
      new RobotTrajectoryCostModel(rInterface, rTrace, pulse);
  return clone;
}
RobotTrajectoryCostModel::RobotTrajectoryCostModel(
    RobotInterface *robotInterface, RobotTrace *learnData,
    PulseNeuron &startPulse)
    : pulse(startPulse) {
  robot = robotInterface;
  robotTrace = learnData;
}
