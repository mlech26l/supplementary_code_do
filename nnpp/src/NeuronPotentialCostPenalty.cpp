#include "NeuronPotentialCostPenalty.h"

float NeuronPotentialCostPenalty::AddPenalty(NeuralNetwork &nn,
                                             float totalTime) {
  if ((temporalType == PenaltyTermporalType::After && totalTime > time) ||
      (temporalType == PenaltyTermporalType::Before && totalTime < time) ||
      (temporalType == PenaltyTermporalType::Between && totalTime > time &&
       totalTime < endTime)) {
    float potential = nn.GetPotentialOf(neuronId);
    if ((thresholdType == PenaltyThresholdType::Above &&
         potential > threshold) ||
        (thresholdType == PenaltyThresholdType::Below &&
         potential < threshold)) {
      return penalty;
    }
  }
  return 0;
}
