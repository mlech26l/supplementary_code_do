#ifndef OPTIMIZE_WORKER_H_
#define OPTIMIZE_WORKER_H_

#include "OptimizerCostModel.h"
#include "StochasticParameter.h"
#include "neural_network.h"
#include <cstddef>
#include <vector>

class PopulationPool;

class OptimizeWorker {
private:
  std::vector<Parameter> *originParameters;
  std::vector<StochasticParameter> parameters;
  OptimizerCostModel *costModel;
  NeuralNetwork *nn;
  PopulationPool *populationPool;

  float costValue;
  void DoSingleStep();

  int lookahead;
  int MaxLookahead = 0;
  int stepsSinceLastImprovement = 0;
  unsigned int randomState;
  void CreateStochasticParameters();
  void WriteBackStochasticParameters();
  /// old stuff
  void Reheat();

public:
  OptimizeWorker(NeuralNetwork *network, OptimizerCostModel *optimizerCostModel,
                 PopulationPool *pool);

  OptimizeWorker *Clone();
  float EvaluateCost();
  void PerformWorkload();
  void Finalize(std::vector<Parameter> *params);
  void SetRandomState(unsigned int ran) { randomState = ran; }

  OptimizerCostModel *GetCostModel() { return costModel; }
  /// Old methods
  // void RandomizeAllParameters();
  // void SetAllParametersToDefaultValue();
  // void AddSynapseWeightParameter(int src, int dest, float lb, float ub);
  // void AddConstNeuronPotentialParamter(int neuron, float lb, float ub);
  // void AddNeuronCapacityParameter(int neuron, float lb, float ub);
  // void RunSteps(int n, int reheatInterval);
  //
  // void AddAllSynapseWeightsAsParameters(float lb, float ub);
  // void AddAllConstNeuronPotentialsAsParameters(float lb, float ub);
};
#endif /* end of include guard: OPTIMIZE_WORKER_H_ */
