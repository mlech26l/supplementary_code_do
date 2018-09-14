#include "pulse_neuron.h"
#include "neuron.h"
#include <math.h>

PulseNeuron::PulseNeuron(int n, float start, float dur, float activationPot) {
  neuron = n;
  startTime = start;
  duration = dur;
  activationPotential = activationPot;
}
void PulseNeuron::Update(NeuralNetwork &nn, float totalTime) {
  if (totalTime >= startTime && totalTime <= startTime + duration) {
    nn.ForcePotentialOf(neuron, activationPotential);
  } else {
    nn.ForcePotentialOf(neuron, nn.V_Leak);
  }
}
