#include "EndeffectorCostModel.h"

#include "pulse_neuron.h"

EndeffectorCostModel::EndeffectorCostModel(EndeffectorTrace *endeffectorTrace) {
  trace = endeffectorTrace;
  interface = new NeuralEndeffectorInterface();
}
float EndeffectorCostModel::Evaluate(NeuralNetwork &nn) {
  nn.Reset();
  trace->Reset();
  interface->Reset();
  float totalTime = 0;

  PulseNeuron pulse(0, 0.0f, 0.3f, -20);
  PulseNeuron pulse2(48, 7.0f, 0.3f, -20);
  PulseNeuron pulse3(49, 10.0f, 0.3f, -20);
  float cost = 0;
  float deltaT = 0.01f;

  float sync_counter = 0;
  float sync_value = 0.1f;
  float penaltyCounter = 0;
  while (!trace->EndOfTraceReached()) {
    pulse.Update(nn, totalTime);
    pulse2.Update(nn, totalTime);
    pulse3.Update(nn, totalTime);
    nn.DoSimulationStep(deltaT);
    interface->SyncWithNetwork(nn);
    interface->SetObjectRecognizedValue(trace->GetObjectRecognizedValue());
    interface->Update(deltaT);
    if (sync_counter >= sync_value) {
      sync_counter = 0;
      interface->SyncWithEndEffectorSystem();
    }
    if (penaltyCounter >= penaltyCheckInterval) {
      penaltyCounter = 0;
      for (unsigned int i = 0; i < penalties.size(); i++) {
        cost += penalties[i].AddPenalty(nn, totalTime);
      }
    }
    penaltyCounter += deltaT;
    sync_counter += deltaT;
    totalTime += deltaT;
    cost += trace->GetDifferenceFromTrace(
        *interface->GetEndeffectorSystemObject(),
        *interface->GetGripperObject(), totalTime);
  }
  return cost;
}
void EndeffectorCostModel::AddPenalty(NeuronPotentialCostPenalty &penalty) {
  penalties.push_back(penalty);
}
OptimizerCostModel *EndeffectorCostModel::Clone() {
  EndeffectorCostModel *clone = new EndeffectorCostModel(trace->Clone());

  // assignment of std::vector is element wise copy
  clone->penalties = penalties;
  return clone;
}
