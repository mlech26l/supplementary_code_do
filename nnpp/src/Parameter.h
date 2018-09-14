#ifndef PARAMETER_H_
#define PARAMETER_H_

#include "neural_network.h"
#include <vector>
enum class ParameterType { Synapse, ConstantNeuron };
class Parameter {
private:
  float lowerBound;
  float upperBound;
  float value;

  ParameterType type;
  std::vector<int> indexA;
  std::vector<int> indexB;
  std::vector<int> indexC;

  Parameter *link;

public:
  Parameter(ParameterType param_type, int index1, int index2, int index3) {
    value = 0.6f;
    type = param_type;
    indexA.push_back(index1);
    indexB.push_back(index2);
    indexC.push_back(index3);
    if (type == ParameterType::Synapse) {
      lowerBound = 0;
      upperBound = 2;
    }
  }
  ParameterType GetType() { return type; }
  int GetIndexA(int indx = 0) { return indexA[indx]; }
  int GetIndexB(int indx = 0) { return indexB[indx]; }
  int GetIndexC(int indx = 0) { return indexC[indx]; }
  int LinkedParametersCount() { return (int)indexA.size(); }
  void LinkParameter(int index1, int index2, int index3) {
    indexA.push_back(index1);
    indexB.push_back(index2);
    indexC.push_back(index3);
  }

  float GetValue() { return value; }
  void SetValue(float val) { value = val; }
  float GetLowerBound() { return lowerBound; }
  float GetUpperBound() { return upperBound; }
  void AddNoise(float rnd);
};
#endif
