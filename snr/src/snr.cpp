#include "NNLogger.h"
#include "NeuralInterface.h"
#include "Optimizer.h"
#include "RobotDynamics.h"
#include "RobotLogger.h"
#include "RobotTrajectoryCostModel.h"
#include "neural_net_serializer.h"
#include "neural_network.h"
#include "pulse_neuron.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

std::random_device rd;
std::mt19937 gen(rd());

float Replay(NeuralNetwork *nn, float variance);
float Replay(std::string filename, float variance) {
  NeuralNetSerializer serializer(filename);
  NeuralNetwork *nn = serializer.Deserialize();
  float cost = Replay(nn, variance);
  delete nn;
  return cost;
}
int main(int argc, char const *argv[]) {
  // Replay("opt_1.bnn", 20);
  // return 0;
  int N = 100;
  std::cout << "Cost with increased variance" << std::endl;
  for (float variance = 0; variance <= 50; variance += 1.0f) {
    double cost = 0;
    for (int i = 0; i < N; i++) {
      cost += Replay("modelred2.bnn", variance);
    }
    std::cout << variance << " " << (cost / N) << std::endl;
  }
  return 1;
}
double sq(double v) { return v * v; }
float Replay(NeuralNetwork *nn, float variance) {

  std::normal_distribution<float> dx(0, variance);
  std::normal_distribution<float> dy(0, variance);
  std::normal_distribution<float> dt(0, variance);

  // std::cout << "Run replay!" << std::endl;
  float logInterval = 0.1;
  // NNLogger logger("log.dat", logInterval);
  // RobotLogger rlog("robot.dat", logInterval);
  RobotDynamics *robot = new RobotDynamics();
  RobotInterface *robotInterface = new RobotInterface();
  // robot->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);
  robotInterface->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);
  PulseNeuron pulse(7, 0.1f, 0.2f, -20);

  RobotTrace *rt = new RobotTrace();
  rt->LoadFromFile("robot_trace.dat");
  // RobotTrajectoryCostModel *costModel =
  // new RobotTrajectoryCostModel(robotInterface, rt, pulse);

  // robotInterface->SetMaximumMotorSpeed(0.3f);

  robotInterface->Reset();
  nn->Reset();
  rt->Reset();

  // nn->PrintStats();

  float cost = 0;
  float SimulationTime = 30.0f;
  float deltaT = 0.01f;
  int N = SimulationTime / deltaT;

  float totalTime = 0;

  float sync_counter = 0;
  float sync_value = 0.1f;

  cost = 0;
  double pNoiseX = 0;
  double pNoiseY = 0;
  double pNoiseT = 0;
  double pSigX = 0;
  double pSigY = 0;
  double pSigT = 0;
  int sensoryNeuronX = 1;
  int sensoryNeuronY = 9;
  int sensoryNeuronTheta = 10;
  for (int i = 0; i < N; i++) {
    pulse.Update(*nn, totalTime);

    robotInterface->Sense(*nn);
    float noiseX = dx(gen);
    float vx = nn->GetPotentialOf(sensoryNeuronX);
    pSigX += sq(vx + 70);
    pNoiseX += sq(noiseX);
    nn->ForcePotentialOf(sensoryNeuronX, vx + noiseX);
    // nn->ForcePotentialOf(sensoryNeuronX, -70);

    float noiseY = dy(gen);
    float vy = nn->GetPotentialOf(sensoryNeuronY);
    pSigY += sq(vy + 70);
    pNoiseY += sq(noiseY);
    nn->ForcePotentialOf(sensoryNeuronY, vy + noiseY);
    // nn->ForcePotentialOf(sensoryNeuronY, -70);

    float noiseT = dt(gen);
    float vt = nn->GetPotentialOf(sensoryNeuronTheta);
    pSigT += sq(vt + 70);
    pNoiseT += sq(noiseT);
    nn->ForcePotentialOf(sensoryNeuronTheta, vt + noiseT);
    // nn->ForcePotentialOf(sensoryNeuronTheta, -70);

    nn->DoSimulationStep(deltaT);
    robotInterface->Actuate(*nn);
    robot->DoSimulationStep(deltaT);

    // logger.AddDataPoint(*nn, deltaT, totalTime);
    // rlog.AddDataPoint(*robotInterface, deltaT, totalTime);

    if (sync_counter >= sync_value) {
      sync_counter = 0;
      robot->GetCommandsFromInterface(*robotInterface);
      robot->SendInputsToInterface(*robotInterface);
    }
    sync_counter += deltaT;
    totalTime += deltaT;

    float add = rt->GetDifferenceFromTrace(*robotInterface, totalTime);
    cost += add;
  }

  // logger.Close();
  // rlog.Close();
  double snrX = 10 * std::log10(pSigX / pNoiseX);
  double snrY = 10 * std::log10(pSigY / pNoiseY);
  double snrT = 10 * std::log10(pSigT / pNoiseT);
  // std::cout << "SNR X: " << snrX << std::endl;
  // std::cout << "SNR Y: " << snrY << std::endl;
  // std::cout << "SNR T: " << snrT << std::endl;
  // std::cout << "Replay cost: " << cost << std::endl;
  return cost;
}
