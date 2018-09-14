#include "OptimizeWorker.h"
#include "PopulationPool.h"
#include "neural_net_serializer.h"
#include <iostream>

OptimizeWorker::OptimizeWorker(NeuralNetwork *network,
                               OptimizerCostModel *optimizerCostModel,
                               PopulationPool *pool) {
  nn = network;
  costModel = optimizerCostModel;
  populationPool = pool;
}

OptimizeWorker *OptimizeWorker::Clone() {
  NeuralNetwork *network = nn->Clone();
  OptimizerCostModel *optimizerCostModel = costModel->Clone();
  return new OptimizeWorker(network, optimizerCostModel, populationPool);
}
void OptimizeWorker::PerformWorkload() {
  originParameters = populationPool->GetOpen();
  while (originParameters != NULL) {
    CreateStochasticParameters();

    costValue = EvaluateCost();
    lookahead = 0;
    // std::cout << "Starting workload with " << costValue << std::endl;
    for (int i = 0; i < 500; i++) {
      DoSingleStep();
    }
    // Undo changes if lookahead > 0
    for (size_t i = 0; i < parameters.size(); i++) {
      parameters[i].Undo();
    }
    // std::cout << "Done workload with " << costValue << std::endl;
    WriteBackStochasticParameters();

    // replace later with a more efficient implementation
    parameters.clear();

    populationPool->SetProcessed(originParameters, costValue);
    originParameters = populationPool->GetOpen();
  }
}
void OptimizeWorker::Finalize(std::vector<Parameter> *params) {
  originParameters = params;
  CreateStochasticParameters();

  costValue = EvaluateCost();
  // std::cout << "Cost before finalize " << costValue << std::endl;
  lookahead = 0;
  // std::cout << "Starting workload with " << costValue << std::endl;
  for (int i = 0; i < 2000; i++) {
    DoSingleStep();
  }
  // Undo changes if lookahead > 0
  for (size_t i = 0; i < parameters.size(); i++) {
    parameters[i].Undo();
  }
  // std::cout << "Done workload with " << costValue << std::endl;
  WriteBackStochasticParameters();

  // NeuralNetSerializer serializer("finalize.bnn");
  // serializer.Serialize(nn);

  // replace later with a more efficient implementation
  parameters.clear();
}
void OptimizeWorker::CreateStochasticParameters() {
  if (parameters.size() == 0) {
    for (unsigned int i = 0; i < originParameters->size(); i++) {
      StochasticParameter stochasticParam((*originParameters)[i], *nn);
      parameters.push_back(stochasticParam);
    }
  } else {
    std::cout << "Assertion" << std::endl;
  }
}
void OptimizeWorker::WriteBackStochasticParameters() {
  for (unsigned int i = 0; i < parameters.size(); i++) {
    parameters[i].WriteBack();
    // (*originParameters)[i].SetValue(parameters[i].GetValue());
  }
}
void OptimizeWorker::DoSingleStep() {
  stepsSinceLastImprovement++;
  // int numModifiedParams = (rand_r(&randomState) % (parameters.size() - 1)) +
  // Modify params between 1 and 8
  int numModifiedParams = 1 + (rand_r(&randomState) % 5);
  // numModifiedParams = 2;
  for (int i = 0; i < numModifiedParams; i++) {
    float rnd = ((float)rand_r(&randomState) / (float)(RAND_MAX / 2)) - 1.0f;
    int which = (rand_r(&randomState)) % parameters.size();
    parameters[which].AddNoise(rnd);
  }
  float newCost = EvaluateCost();
  // std::cout << "Evaluated cost: " << newCost;
  if (newCost < costValue) {
    // std::cout << " [Accept]" << std::endl;
    costValue = newCost;
    stepsSinceLastImprovement = 0;
    MaxLookahead--;
    if (MaxLookahead < 0)
      MaxLookahead = 0;
    lookahead = 0;
    for (size_t i = 0; i < parameters.size(); i++) {
      // parameters[i].Penalize(0.99f);
      parameters[i].CommitChange();
    }
  } else {
    if (stepsSinceLastImprovement > 40) {
      stepsSinceLastImprovement = 0;
      MaxLookahead++;
    }
    lookahead++;
    if (lookahead > MaxLookahead) {
      lookahead = 0;
      // std::cout << " [Discard]" << std::endl;
      // not better -> undo last step
      for (size_t i = 0; i < parameters.size(); i++) {
        // parameters[i].Penalize(0.99f);
        parameters[i].Undo();
      }
    }
  }
  // std::cin.get();
}
float OptimizeWorker::EvaluateCost() { return costModel->Evaluate(*nn); }
/////////////////// OLD STUFFF
// OptimizeWorker::Optimizer(NeuralNetwork *network, RobotInterface *rbot,
//                           int startButtonNeuron, float startPulseTime) {
//   nn = network;
//   robot = rbot;
//   startNeuron = startButtonNeuron;
//   startTime = startPulseTime;
//   currentBestCost = 1e10;
// }
// void OptimizeWorker::SetTrace(RobotTrace *rt) { robotTrace = rt; }
//
// void OptimizeWorker::AddNeuronCapacityParameter(int neuron, float lb,
//                                                 float ub) {
//   float *valuePtr = nn->GetValuePointerOfCapacity(neuron);
//   if (valuePtr == NULL) {
//     std::cerr << "Could not find neuron  " << neuron << std::endl;
//     return;
//   }
//   Parameter param(valuePtr, lb, ub);
//   parameters.push_back(param);
// }
// void OptimizeWorker::AddSynapseWeightParameter(int src, int dest, float lb,
//                                                float ub) {
//   float *valuePtr = nn->GetValuePointerOfSynapse(src, dest);
//   if (valuePtr == NULL) {
//     std::cerr << "Could not find synapse form " << src << " to " << dest
//               << std::endl;
//     return;
//   }
//   Parameter param(valuePtr, lb, ub);
//   parameters.push_back(param);
// }
// void OptimizeWorker::AddConstNeuronPotentialParamter(int neuron, float lb,
//                                                      float ub) {
//   float *valuePtr = nn->GetValuePointerOfConstNeuron(neuron);
//   if (valuePtr == NULL) {
//     std::cerr << "Could not find const neuron " << neuron << std::endl;
//     return;
//   }
//   // default value for const neurons
//   // *valuePtr = -30;
//   Parameter param(valuePtr, lb, ub);
//   parameters.push_back(param);
// }
//
//
// void OptimizeWorker::AddAllSynapseWeightsAsParameters(float lb, float ub) {
//   for (int i = 0; i < nn->GetSize(); i++) {
//     std::vector<Synapse> &synapses = nn->GetSynapsesOf(i);
//     for (size_t j = 0; j < synapses.size(); j++) {
//       AddSynapseWeightParameter(synapses[j].source, i, lb, ub);
//     }
//   }
// }
// void OptimizeWorker::AddAllConstNeuronPotentialsAsParameters(float lb,
//                                                              float ub) {
//   std::vector<std::pair<int, float>> &constNeurons = nn->GetConstNeurons();
//   for (size_t i = 0; i < constNeurons.size(); i++) {
//     AddConstNeuronPotentialParamter(constNeurons[i].first, lb, ub);
//   }
// }
// void OptimizeWorker::Reheat() {
//   for (size_t i = 0; i < parameters.size(); i++) {
//     float x = parameters[i].GetRatioDifferenceToIntitialValue();
//     float minDiff = 0;
//     float maxDiff = 0.4f;
//     if (x < minDiff)
//       x = 0;
//     else if (x > maxDiff)
//       x = maxDiff;
//     x /= maxDiff;
//
//     float reheatValue = x * parameters[i].GetVariation() +
//                         (1 - x) * parameters[i].GetInitialVariation();
//     // std::cout << "Reheat param " << i << " from "
//     // << parameters[i].GetVariation() << " to " << reheatValue
//     // << std::endl;
//     parameters[i].SetVariation(reheatValue);
//   }
//   // std::cout << "-----" << std::endl;
// }
// void OptimizeWorker::DoSingleStep() {
//   int numModifiedParams = (rand() % (parameters.size() - 1)) + 1;
//   bool permutation[parameters.size()];
//   for (int i = 0; i < (int)parameters.size(); i++) {
//     permutation[i] = i < numModifiedParams;
//   }
//   FisherYatesPermutate(permutation, parameters.size());
//   for (size_t i = 0; i < parameters.size(); i++) {
//     parameters[i].AddNoise(!permutation[i]);
//   }
//   float newCost = EvaluateCost();
//   // std::cout << "Evaluated cost: " << newCost;
//   if (newCost < currentBestCost) {
//     // std::cout << " [Accept]" << std::endl;
//     currentBestCost = newCost;
//   } else {
//     // std::cout << " [Discard]" << std::endl;
//     // not better -> undo last step
//     for (size_t i = 0; i < parameters.size(); i++) {
//       parameters[i].Penalize(0.99f);
//       parameters[i].Undo();
//     }
//   }
// }
// void OptimizeWorker::RunSteps(int n, int reheatInterval) {
//   std::chrono::time_point<std::chrono::system_clock> start =
//       std::chrono::system_clock::now();
//
//   currentBestCost = EvaluateCost();
//   float startcost = currentBestCost;
//   std::cout << "Start cost: " << currentBestCost << std::endl;
//   for (int i = 0; i < n; i++) {
//     if ((i + 1) % reheatInterval == 0) {
//       Reheat();
//     }
//     DoSingleStep();
//   }
//   std::chrono::time_point<std::chrono::system_clock> end =
//       std::chrono::system_clock::now();
//   std::chrono::duration<double> elapsed_seconds = end - start;
//
//   // std::cout << "Total change of parameters: " << std::endl;
//   // for (size_t i = 0; i < parameters.size(); i++) {
//   //   std::cout << "Param " << i << " : " <<
//   parameters[i].GetInitialValue()
//   //             << " -> " << parameters[i].GetValue() << std::endl;
//   // }
//   std::cout << "-------" << std::endl;
//
//   std::cout << "Cost after " << n << " steps: " << currentBestCost << " ("
//             << startcost / currentBestCost << " x decrease)" << std::endl;
//   std::cout << "Took " << elapsed_seconds.count() << "s with "
//             << (n / elapsed_seconds.count()) << " simulations/s" <<
//             std::endl;
// }
// float OptimizeWorker::EvaluateCost() {
//   PulseNeuron pulse(startNeuron, startTime, 0.2f, 0.1f, -20);
//
//   RobotDynamics robotDynamics;
//   robot->Reset();
//   nn->Reset();
//   robotTrace->Reset();
//   float totalTime = 0;
//
//   float cost = 0;
//   float deltaT = 0.01f;
//
//   float sync_counter = 0;
//   float sync_value = 0.1f;
//   while (!robotTrace->EndOfTraceReached()) {
//     pulse.Update(*nn, totalTime);
//
//     robot->Sense(*nn);
//     nn->DoSimulationStep(deltaT);
//     robot->Actuate(*nn);
//     robotDynamics.DoSimulationStep(deltaT);
//
//     if (sync_counter >= sync_value) {
//       sync_counter = 0;
//       robotDynamics.GetCommandsFromInterface(*robot);
//       robotDynamics.SendInputsToInterface(*robot);
//     }
//     sync_counter += deltaT;
//     totalTime += deltaT;
//     cost += robotTrace->GetDifferenceFromTrace(*robot, totalTime);
//   }
//   return cost;
// }
// void OptimizeWorker::SetAllParametersToDefaultValue() {
//   for (size_t i = 0; i < parameters.size(); i++) {
//     parameters[i].SetToDefaultValue();
//   }
// }
// void OptimizeWorker::RandomizeAllParameters() {
//   for (size_t i = 0; i < parameters.size(); i++) {
//     parameters[i].SetToDefaultValue();
//     parameters[i].AddNoise(false);
//   }
// }
