#ifndef NEURON_POTENTIAL_COST_PENALTY_H_
#define NEURON_POTENTIAL_COST_PENALTY_H_

#include "neural_network.h"
enum class PenaltyThresholdType { Above, Below };
enum class PenaltyTermporalType { After, Before, Between };

class NeuronPotentialCostPenalty {
private:
  int neuronId;
  float threshold;
  PenaltyThresholdType thresholdType;

  float time;
  float endTime;
  PenaltyTermporalType temporalType;

  float penalty;

public:
  NeuronPotentialCostPenalty(int neuron, float threshld,
                             PenaltyThresholdType thresType, float t,
                             PenaltyTermporalType tempType,
                             float penaltyAmount = 1.0f, float endT = 1.0f) {
    neuronId = neuron;
    threshold = threshld;
    thresholdType = thresType;
    time = t;
    temporalType = tempType;
    penalty = penaltyAmount;
    endTime = endT;
  }
  float AddPenalty(NeuralNetwork &nn, float totalTime);
};

#endif /* end of include guard: NEURON_POTENTIAL_COST_PENALTY_H_ */
