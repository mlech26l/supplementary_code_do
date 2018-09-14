#ifndef STOCHASTIC_PARAMETER_H_
#define STOCHASTIC_PARAMETER_H_

#include "Parameter.h"
#include "neural_network.h"
#include <vector>
class StochasticParameter {
private:
  Parameter *parameter;
  float lowerBound;
  float upperBound;

  std::vector<float *> valuePtrs;

  float variation;
  float undoValue;
  float initialValue;
  float initialVariation;

  void AddSynapseWeightParameter(NeuralNetwork &nn, int src, int dest,
                                 int type);
  void AddConstNeuronPotentialParamter(NeuralNetwork &nn, int neuron);
  void AddNeuronCapacityParameter(NeuralNetwork &nn, int neuron);

public:
  StochasticParameter(Parameter &param, NeuralNetwork &nn);

  void WriteBack();
  void AddNoise(float rnd);
  // void Penalize(float alpha);
  void Undo();
  void CommitChange();
  // void SetToDefaultValue();

  // void SetInitialValue(float value);
  // float GetRatioDifferenceToIntitialValue();
  float GetValue();
  // float GetVariation() { return variation; }
  // float GetInitialVariation() { return initialVariation; }
  // float GetInitialValue() { return initialValue; }
  // void SetVariation(float newVariation) { variation = newVariation; }
};

#endif /* end of include guard: STOCHASTIC_PARAMETER_H_ */
