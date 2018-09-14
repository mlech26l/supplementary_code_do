#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "OptimizeWorker.h"
#include "OptimizerCostModel.h"
#include "Parameter.h"
#include "PopulationPool.h"
#include "neural_network.h"
#include <cstddef>
#include <vector>

enum class EffortLevel { Fast, Balanced, High, Max };
class Optimizer {
private:
  int numOfThreads;
  std::vector<OptimizeWorker> workers;
  PopulationPool populationPool;
  NeuralNetwork *nn;

  float initialCost;

public:
  static int SeedLinear;
  static int SeedQuadratic;
  static int NumberOfThread;
  Optimizer(NeuralNetwork *network, OptimizerCostModel *costModel,
            std::vector<Parameter> &params);

  void Run(EffortLevel level = EffortLevel::Balanced);
  void InjectParameters(std::vector<Parameter> &params);
};
#endif
