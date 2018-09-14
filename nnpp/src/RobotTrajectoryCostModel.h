#ifndef ROBOT_TRAJECTORY_COST_MODEL_H_
#define ROBOT_TRAJECTORY_COST_MODEL_H_

#include "OptimizerCostModel.h"
#include "RobotDynamics.h"
#include "RobotInterface.h"
#include "RobotTrace.h"
#include "pulse_neuron.h"

class RobotTrajectoryCostModel : public OptimizerCostModel {
private:
  RobotTrace *robotTrace;
  RobotInterface *robot;
  PulseNeuron pulse;

public:
  RobotTrajectoryCostModel(RobotInterface *robotInterface,
                           RobotTrace *learnData, PulseNeuron &startPulse);
  float Evaluate(NeuralNetwork &nn);
  OptimizerCostModel *Clone();
};

#endif /* end of include guard: ROBOT_TRAJECTORY_COST_MODEL_H_ */
