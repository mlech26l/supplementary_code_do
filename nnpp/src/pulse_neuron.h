#ifndef PULSE_NEURON_H_
#define PULSE_NEURON_H_

#include "neural_network.h"

class PulseNeuron {
private:
  int neuron;
  float startTime;
  float duration;
  float activationPotential;

public:
  PulseNeuron(int n, float start, float dur, float activationPot);
  void Update(NeuralNetwork &nn, float totalTime);
};
#endif
