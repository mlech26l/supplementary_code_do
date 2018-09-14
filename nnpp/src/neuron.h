#ifndef NEURON_H_
#define NEURON_H_

#include "synapse.h"
#include <vector>

class NeuralNetwork;
class Neuron {
private:
  float v;
  float v_next;
  float Cm;
  float Gleak;
  std::vector<Synapse> synapses;

public:
  void Initialize(float v_eq, float cm);
  float GetCm() { return Cm; }
  void SetGleak(float value) { Gleak = value; }
  void AddSynapse(float w, SynapseType type, int src);
  std::vector<Synapse> &GetSynapses() { return synapses; }
  // float GetInflowCurrent(NeuralNetwork &nn);
  void ComputeV_next(float deltaT, NeuralNetwork &nn);

  void UseVNext(void);
  void ForcePotential(float value);
  float GetPotential(void);
  void PrintDebug(NeuralNetwork &nn);

  void RemoveSynapse(int source, SynapseType synType);
  float *GetValuePointerOfSynapse(int source, SynapseType synType);
  float *GetValuePointerOfCapacity() { return &Cm; }
};

#endif
