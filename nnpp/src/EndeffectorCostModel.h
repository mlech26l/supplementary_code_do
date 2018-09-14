#ifndef ENDEFFECTOR_COST_MODEL_H_
#define ENDEFFECTOR_COST_MODEL_H_

#include "EndeffectorTrace.h"
#include "NeuralEndeffectorInterface.h"
#include "NeuronPotentialCostPenalty.h"
#include "OptimizerCostModel.h"
#include <vector>
class EndeffectorCostModel : public OptimizerCostModel {
private:
  EndeffectorTrace *trace;
  NeuralEndeffectorInterface *interface;
  std::vector<NeuronPotentialCostPenalty> penalties;

  float penaltyCheckInterval = 0.05f;

public:
  EndeffectorCostModel(EndeffectorTrace *endeffectorTrace);
  float Evaluate(NeuralNetwork &nn);
  void AddPenalty(NeuronPotentialCostPenalty &penalty);
  OptimizerCostModel *Clone();
};

#endif /* end of include guard: ROBOT_TRAJECTORY_COST_MODEL_H_ */
