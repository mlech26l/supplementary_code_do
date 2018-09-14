#include "Optimizer.h"
#include "RobotDynamics.h"
#include "pulse_neuron.h"
#include <chrono>
#include <cstddef>
#include <iostream>
#include <thread>
int Optimizer::SeedLinear = 43543;
int Optimizer::SeedQuadratic = 7632;
int Optimizer::NumberOfThread = 0;

Optimizer::Optimizer(NeuralNetwork *network, OptimizerCostModel *costModel,
                     std::vector<Parameter> &params)
    : populationPool(params) {
  nn = network;
  numOfThreads = NumberOfThread;
  if (numOfThreads <= 0 || numOfThreads > 20)
    numOfThreads = std::thread::hardware_concurrency();

  std::cout << "Using up to " << numOfThreads << " threds" << std::endl;
  OptimizeWorker *worker =
      new OptimizeWorker(network, costModel->Clone(), &populationPool);

  initialCost = worker->EvaluateCost();

  populationPool.SetRandomState(3 * Optimizer::SeedLinear +
                                Optimizer::SeedQuadratic * 76);
  worker->SetRandomState(numOfThreads * SeedLinear +
                         numOfThreads * numOfThreads * SeedQuadratic);
  workers.push_back(*worker);
  for (int i = 1; i < numOfThreads; i++) {
    worker = worker->Clone();
    worker->SetRandomState(i * SeedLinear + i * i * SeedQuadratic);
    workers.push_back(*worker);
  }
}
void Optimizer::Run(EffortLevel level) {
  std::cout << "Starting optimization with intial cost: " << initialCost
            << std::endl;
  std::cout << "Effort Level: ";
  if (level == EffortLevel::Fast) {
    std::cout << "[Fast]";
    populationPool.SetTerminationThreshold(5);
  } else if (level == EffortLevel::Balanced) {
    std::cout << "[Balanced]";
    populationPool.SetTerminationThreshold(12);
  } else if (level == EffortLevel::High) {
    std::cout << "[High]";
    populationPool.SetTerminationThreshold(20);
  } else {
    std::cout << "[Max]";
    populationPool.SetTerminationThreshold(30);
    // max
  }
  std::cout << std::endl;

  do {
    std::vector<std::thread> threads;
    // spawn threads
    for (int i = 1; i < workers.size(); i++) {
      // std::cout << "Spawning Thread " << i << std::endl;
      std::thread t(&OptimizeWorker::PerformWorkload, &workers[i]);
      threads.push_back(move(t));
    }
    // let main thread handle workload 0
    workers[0].PerformWorkload();

    for (int i = 0; i < threads.size(); i++) {
      threads[i].join();
    }
  } while (populationPool.NextEpoch());

  float factor = initialCost / populationPool.GetBestCost();

  std::vector<Parameter> bestParams = populationPool.GetBestParameter();
  workers[0].Finalize(&bestParams);
  InjectParameters(bestParams);

  std::cout << "Optimization done! Global best: "
            << populationPool.GetBestCost() << ", (x" << factor << " decrease)"
            << std::endl;
}
void Optimizer::InjectParameters(std::vector<Parameter> &params) {
  for (unsigned int i = 0; i < params.size(); i++) {
    StochasticParameter write(params[i], *nn);
  }
}
