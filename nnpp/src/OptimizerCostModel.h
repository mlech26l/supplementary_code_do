#ifndef OPTIMIZER_COST_MODEL_H_
#define OPTIMIZER_COST_MODEL_H_

#include "neural_network.h"

class OptimizerCostModel {
protected:
  bool verbose = false;

public:
  virtual float Evaluate(NeuralNetwork &nn) {}
  virtual OptimizerCostModel *Clone() {}
  void SetVerbose(bool v) { verbose = v; }
};
#endif /* end of include guard: OPTIMIZER_COST_MODEL_H_ */
