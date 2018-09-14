#ifndef POPULATION_POOL_H_
#define POPULATION_POOL_H_

#include "OptimizeWorker.h"
#include "Parameter.h"
#include <mutex>
#include <thread>
#include <vector>

class PopulationPool {
private:
  std::vector<Parameter> bestParameters;
  float currentBestCost;

  std::vector<std::vector<Parameter> *> openPool;
  std::vector<std::pair<std::vector<Parameter> *, float>> processedPool;

  std::mutex pullMutex;
  std::mutex pushMutex;

  int populationSize;

  int improvementAgo = 0;
  int terminationThreshold = 3;
  int epochsPassed;
  const int MaxEpochs = 3;
  unsigned int randomState = 887481;
  void RandomizeVector(std::vector<Parameter> *param);

public:
  PopulationPool(std::vector<Parameter> &params);
  std::vector<Parameter> *GetOpen();
  void SetProcessed(std::vector<Parameter> *param, float costValue);
  bool NextEpoch();
  void SetRandomState(unsigned int rands) { randomState = rands; }
  static bool CompareBPair(std::pair<std::vector<Parameter> *, float> a,
                           std::pair<std::vector<Parameter> *, float> b) {
    return a.second < b.second;
  }
  float GetBestCost() { return currentBestCost; }
  std::vector<Parameter> GetBestParameter() { return bestParameters; }

  void SetTerminationThreshold(int r) { terminationThreshold = r; }
};

#endif /* end of include guard: POPULATION_POOL_H_ */
