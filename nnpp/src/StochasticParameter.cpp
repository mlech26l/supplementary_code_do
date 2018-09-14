#include "StochasticParameter.h"
#include <iostream>

StochasticParameter::StochasticParameter(Parameter &param, NeuralNetwork &nn) {
  if (param.GetType() == ParameterType::Synapse) {
    for (int i = 0; i < param.LinkedParametersCount(); i++) {
      AddSynapseWeightParameter(nn, param.GetIndexA(i), param.GetIndexB(i),
                                param.GetIndexC(i));
    }

  } else {
    std::cerr << "Unspecified Parameter type!" << std::endl;
  }

  parameter = &param;
  for (size_t i = 0; i < valuePtrs.size(); i++) {
    *valuePtrs[i] = param.GetValue();
  }
  undoValue = param.GetValue();
  lowerBound = param.GetLowerBound();
  upperBound = param.GetUpperBound();
  float alpha = 0.05;
  variation = (upperBound - lowerBound) * alpha;
  variation = 0.2f;
}

void StochasticParameter::AddNeuronCapacityParameter(NeuralNetwork &nn,
                                                     int neuron) {
  float *ptr = nn.GetValuePointerOfCapacity(neuron);
  if (ptr == NULL) {
    std::cerr << "Could not find neuron  " << neuron << std::endl;
    return;
  }
  valuePtrs.push_back(ptr);
}
void StochasticParameter::Undo() {
  for (size_t i = 0; i < valuePtrs.size(); i++) {
    *valuePtrs[i] = undoValue;
  }
}
void StochasticParameter::CommitChange() { undoValue = *valuePtrs[0]; }

// void SetToDefaultValue();

// void SetInitialValue(float value);
// float GetRatioDifferenceToIntitialValue();
float StochasticParameter::GetValue() { return *valuePtrs[0]; }
void StochasticParameter::AddSynapseWeightParameter(NeuralNetwork &nn, int src,
                                                    int dest, int type) {
  SynapseType typ = Excitatory;
  if (type == 1)
    typ = Inhibitory;
  else if (type == 2)
    typ = GapJunction;
  float *ptr = nn.GetValuePointerOfSynapse(src, dest, typ);
  if (ptr == NULL) {
    std::cerr << "Could not find synapse form " << src << " to " << dest
              << " of type " << typ << std::endl;
    return;
  }
  valuePtrs.push_back(ptr);
  if (type == 2) {
    float *ptr2 = nn.GetValuePointerOfSynapse(dest, src, typ);
    if (ptr2 != NULL)
      valuePtrs.push_back(ptr2);
  }
}
void StochasticParameter::AddConstNeuronPotentialParamter(NeuralNetwork &nn,
                                                          int neuron) {
  float *ptr = nn.GetValuePointerOfConstNeuron(neuron);
  if (ptr == NULL) {
    std::cerr << "Could not find const neuron " << neuron << std::endl;
    return;
  }
  valuePtrs.push_back(ptr);
}
void StochasticParameter::WriteBack() { parameter->SetValue(*valuePtrs[0]); }
void StochasticParameter::AddNoise(float rnd) {
  float newValue = *valuePtrs[0] + variation * rnd;
  if (newValue > upperBound)
    newValue = upperBound;
  else if (newValue < lowerBound)
    newValue = lowerBound;

  // std::cout << "Modify param from " << *valuePtr << " to " << newValue
  // << " using var " << variation << " with rand " << rnd << std::endl;
  for (size_t i = 0; i < valuePtrs.size(); i++) {
    *valuePtrs[i] = newValue;
  }
}
// void StochasticParameter::Penalize(float alpha) {
//   if (*valuePtr != undoValue) {
//     variation *= alpha;
//     // std::cout << "Penalize: new variation: " << variation << std::endl;
//   }
// }
