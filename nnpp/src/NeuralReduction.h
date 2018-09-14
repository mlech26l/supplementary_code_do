#ifndef NEURAL_REDUCATION_H_
#define NEURAL_REDUCATION_H_
#include "neural_network.h"
#include <vector>
class NeuralReduction {
private:
  std::vector<int> motorNeurons;

  void ReachabilityOfMotorNeuronCut(NeuralNetwork &nn);

  void DFSreach(NeuralNetwork &nn, int currentNeuron, bool *checkerbord);

  void CutMultiSynapses(NeuralNetwork &nn);

  void CutBySum(NeuralNetwork &nn);

  float SumUpSynapseWeights(NeuralNetwork &nn, int neuron, SynapseType typ);

  void CutByThreshold(NeuralNetwork &nn);
  void ZeroInputCut(NeuralNetwork &nn);

public:
  void AddMotorNeuron(std::vector<int> neurons);
  void AddMotorNeuron(int neuron);
  void Reduce(NeuralNetwork &nn);
};
#endif /* end of include guard: NEURAL_REDUCATION_H_ */
