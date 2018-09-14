#include "PopulationPool.h"
#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <fstream>
#include <iostream>
std::vector<Parameter> *PopulationPool::GetOpen() {
  std::vector<Parameter> *ret = NULL;
  pullMutex.lock();
  if (openPool.size() > 0) {
    ret = openPool[openPool.size() - 1];
    openPool.pop_back();
  }
  // std::cout << "Removing parameter from open! Remaining: " << openPool.size()
  // << std::endl;
  pullMutex.unlock();
  return ret;
}
void PopulationPool::SetProcessed(std::vector<Parameter> *param,
                                  float costValue) {
  pushMutex.lock();
  processedPool.push_back(std::make_pair(param, costValue));
  pushMutex.unlock();
}
PopulationPool::PopulationPool(std::vector<Parameter> &params) {
  populationSize = std::thread::hardware_concurrency() * 8;
  bestParameters = params;
  epochsPassed = 0;

  currentBestCost = FLT_MAX;

  for (size_t i = 0; i < populationSize; i++) {
    std::vector<Parameter> *paramSet =
        new std::vector<Parameter>(bestParameters);
    RandomizeVector(paramSet);
    openPool.push_back(paramSet);
  }
}
void PopulationPool::RandomizeVector(std::vector<Parameter> *param) {
  for (unsigned int i = 0; i < param->size(); i++) {
    float rnd = ((float)rand_r(&randomState) / (float)(RAND_MAX / 2)) - 1.0f;
    // float val = ((*param)[i].GetUpperBound() - (*param)[i].GetLowerBound()) /
    // 2;
    // float val = 1.7f;
    // set to center and give 30% randomess
    float val = (*param)[i].GetValue();
    val += rnd * val * 0.1f;
    if (val < (*param)[i].GetLowerBound())
      val = (*param)[i].GetLowerBound();
    else if (val > (*param)[i].GetUpperBound())
      val = (*param)[i].GetUpperBound();
    (*param)[i].SetValue(val);
  }
}
//#define USE_IMPORTANCE_SAMPLING
#ifdef USE_IMPORTANCE_SAMPLING
bool PopulationPool::NextEpoch() {
  epochsPassed++;
  // std::sort(processedPool.begin(), processedPool.end(),
  // PopulationPool::CompareBPair);

  float sum = 1.0f / currentBestCost;
  float epochBest = processedPool[0].second;
  std::vector<Parameter> *bestParam = processedPool[0].first;
  for (unsigned int i = 0; i < processedPool.size(); i++) {
    if (processedPool[i].second < epochBest) {
      epochBest = processedPool[i].second;
      bestParam = processedPool[i].first;
    }
    sum += 1.0f / processedPool[i].second;

    // std::cout << "Number " << i << " has prob "
    // << 1.0f / processedPool[i].second << " and cost "
    // << processedPool[i].second << std::endl;
  }
  improvementAgo++;

  if (epochBest < currentBestCost) {
    currentBestCost = epochBest;
    bestParameters = *processedPool[0].first;
    improvementAgo = 0;
  }
  // if (epochsPassed >= MaxEpochs)
  //   return false;
  if (improvementAgo > terminationThreshold)
    return false;

  // std::cout << "End of epoch best x: " << epochBest << std::endl;
  // terminate if 3 epochs without improvement

  // Importance sampling
  for (unsigned int i = 0; i < populationSize; i++) {
    float importance = ((float)rand_r(&randomState) / (float)(RAND_MAX)) * sum;
    // std::cout << "Sum is " << sum << " rand is " << importance << std::endl;
    std::vector<Parameter> *parent = &bestParameters;

    float partial_sum = 0;
    for (unsigned int x = 0; x < processedPool.size(); x++) {
      partial_sum += 1.0f / processedPool[x].second;
      if (partial_sum > importance) {
        // std::cout << "Choose number " << x << " that has prob "
        // << 1.0f / processedPool[x].second << std::endl;
        parent = processedPool[x].first;
        break;
      }
    }
    // if (partial_sum <= importance)
    // std::cout << "Use global best with prob " << 1.0f / currentBestCost
    // << std::endl;

    std::vector<Parameter> *newParam = new std::vector<Parameter>(*parent);
    for (unsigned int i = 0; i < newParam->size(); i++) {
      float rnd = ((float)rand_r(&randomState) / (float)(RAND_MAX / 2)) - 1.0f;
      (*newParam)[i].AddNoise(rnd);
    }
    openPool.push_back(newParam);
  }
  for (unsigned int x = 0; x < processedPool.size(); x++) {
    delete processedPool[x].first;
  }
  processedPool.clear();
  return true;
}
#else
bool PopulationPool::NextEpoch() {
  epochsPassed++;
  std::sort(processedPool.begin(), processedPool.end(),
            PopulationPool::CompareBPair);
  float epochBest = processedPool[0].second;
  // std::cout << "End of epoch, best: " << epochBest << std::endl;
  // for (unsigned int i = 0; i < processedPool.size(); i++) {
  //   std::cout << "Solution no " << i << ": " << processedPool[i].second
  //             << std::endl;
  // }
  improvementAgo++;

  if (epochBest < currentBestCost) {
    currentBestCost = epochBest;
    bestParameters = *processedPool[0].first;
    improvementAgo = 0;
  }

  // std::cout << "End of epoch best y: " << epochBest << std::endl;
  // terminate if 3 epochs without improvement
  // if (epochsPassed >= MaxEpochs)
  //   return false;

  if (improvementAgo > terminationThreshold)
    return false;

  float quantile = 0.2f;
  unsigned int quantileCount = (int)(quantile * populationSize);
  if (quantileCount <= 0)
    quantileCount = 1;

  int bestCounter = -1;

  // replace bad solutions with variations of the good solutions
  while (processedPool.size() > quantileCount) {
    std::vector<Parameter> *newParam =
        processedPool[processedPool.size() - 1].first;
    processedPool.pop_back();

    // select global best also for further optimization
    std::vector<Parameter> *parent;
    if (bestCounter < 0) {
      parent = &bestParameters;
    } else {
      parent = processedPool[bestCounter].first;
    }
    for (unsigned int i = 0; i < newParam->size(); i++) {
      (*newParam)[i].SetValue((*parent)[i].GetValue());
      float rnd = ((float)rand_r(&randomState) / (float)(RAND_MAX / 2)) - 1.0f;
      (*newParam)[i].AddNoise(rnd);
    }
    openPool.push_back(newParam);
    bestCounter++;
    if (bestCounter >= quantile)
      bestCounter = -1;
  }
  // Add noise to the good solutions too
  while (processedPool.size() > 0) {
    std::vector<Parameter> *newParam =
        processedPool[processedPool.size() - 1].first;
    processedPool.pop_back();
    for (unsigned int i = 0; i < newParam->size(); i++) {
      float rnd = ((float)rand_r(&randomState) / (float)(RAND_MAX / 2)) - 1.0f;
      (*newParam)[i].AddNoise(rnd);
    }
    openPool.push_back(newParam);
  }
  return true;
}
#endif
