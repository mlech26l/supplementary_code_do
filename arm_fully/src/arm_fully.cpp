#include "EndeffectorCostModel.h"
#include "EndeffectorTrace.h"
#include "NNLogger.h"
#include "NeuralEndeffectorInterface.h"
#include "NeuralReduction.h"
#include "Optimizer.h"
#include "RobotDynamics.h"
#include "RobotLogger.h"
#include "RobotTrace.h"
#include "RobotTrajectoryCostModel.h"
#include "neural_net_serializer.h"
#include "neural_network.h"
#include "pulse_neuron.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

std::vector<Parameter> GetAllSynapses(NeuralNetwork &nn) {
  std::vector<Parameter> params;
  for (size_t i = 0; i < nn.GetSize(); i++) {
    std::vector<Synapse> &syns = nn.GetSynapsesOf(i);
    for (size_t j = 0; j < syns.size(); j++) {
      int typ = 0;
      if (syns[j].type == Inhibitory)
        typ = 1;
      else if (syns[j].type == GapJunction)
        typ = 2;
      Parameter p(ParameterType::Synapse, syns[j].source, i, typ);
      p.SetValue(syns[j].w);
      params.push_back(p);
    }
  }
  return params;
}
void PrintToLatex(NeuralNetwork *nn, std::string outfile) {

  std::ofstream file;
  file << std::fixed;
  file << std::setprecision(2);
  file.open(outfile, std::ios::out);
  for (size_t dest = 0; dest < nn->GetSize(); dest++) {
    std::vector<Synapse> &syns = nn->GetSynapsesOf(dest);
    for (size_t j = 0; j < syns.size(); j++) {
      file << "\\draw[";
      if (syns[j].type == Excitatory)
        file << "ex";
      else if (syns[j].type == Inhibitory)
        file << "inh";
      else if (syns[j].type == GapJunction)
        file << "gj";
      file << ",line width=" << syns[j].w;

      file << "] (N" << syns[j].source << ") to[";
      if (syns[j].source == dest) {
        file << "out=295,in=315,looseness=10";
      }
      file << "] (N" << dest << ");" << std::endl;
    }
  }
  file.close();
}
void Cut(NeuralNetwork *nn) {
  NeuralReduction reduction;
  for (int i = 15; i <= 28; i++) {
    reduction.AddMotorNeuron(i);
  }
  reduction.AddMotorNeuron(30);
  reduction.Reduce(*nn);
}
NeuralNetwork *CreateFullyConnectedNetwork();
void Replay(std::string bnnFile);
void Reduce() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetwork *nn = CreateFullyConnectedNetwork();
  std::ofstream logFile;
  // logFile.open("cost_over_iterations.log", std::ios::out);
  logFile.open("cost_over_iterations.log", std::ios::app);

  int round_count = 0;
  int roundsWithoutReduction = 0;
  int lastSynapseCount = nn->CountSynapses();
  while (true) {
    std::ostringstream graph_raw_file;
    graph_raw_file << "latex/round" << round_count << ".tex";
    PrintToLatex(nn, graph_raw_file.str());

    std::cout << "Starting optimizer round " << round_count << " with "
              << nn->CountSynapses() << " synapses" << std::endl;

    std::vector<Parameter> allSynapses = GetAllSynapses(*nn);
    Optimizer optimizer(nn, costModel, allSynapses);
    optimizer.Run(EffortLevel::High);

    std::ostringstream graph_opt_file;
    graph_opt_file << "latex/opt" << round_count << ".tex";
    PrintToLatex(nn, graph_opt_file.str());

    std::ostringstream opt_bnn_file;
    opt_bnn_file << "bnn/opt_round_" << round_count << ".bnn";
    NeuralNetSerializer serializer2(opt_bnn_file.str());
    serializer2.Serialize(nn);

    float costAfterOpt = costModel->Evaluate(*nn);
    std::cout << "Cost after round " << round_count << ": " << costAfterOpt
              << std::endl;

    std::cout << "Cut!" << std::endl;
    Cut(nn);
    logFile << round_count << " " << nn->CountSynapses() << " " << costAfterOpt
            << std::endl
            << std::flush;

    int synapseCount = nn->CountSynapses();
    if (synapseCount == lastSynapseCount) {
      roundsWithoutReduction++;
      if (roundsWithoutReduction >= 3) {
        std::cout << "Terminate!" << std::endl;
        logFile.close();
        return;
      }
    } else {
      lastSynapseCount = synapseCount;
      roundsWithoutReduction = 0;
    }
    // Replay(opt_bnn_file.str());
    round_count++;
  }

  delete nn;
  delete trace;
}
int main(int argc, char const *argv[]) {
   if (argc >= 2) {
    Optimizer::SeedLinear = std::stoi(argv[1]);
    std::cout << "Using linar seed " << Optimizer::SeedLinear << std::endl;
  }
  if (argc >= 3) {
    Optimizer::SeedQuadratic = std::stoi(argv[2]);
    std::cout << "Using quadratic seed " << Optimizer::SeedQuadratic
              << std::endl;
  }
   OptimizeForAllPoints();
  // Replay("narm_reduced_opt.bnn");
  Reduce()
  return 0;
}

void Replay(std::string bnnFile) {
  NeuralNetSerializer serializer(bnnFile);
  NeuralNetwork *nn = serializer.Deserialize();
  NNLogger logger("arm-log.dat", 0.1f);
  EndeffectorTrace *trace = new EndeffectorTrace(true);
  trace->LoadFromFile("arm_trace_final.dat");

  std::ofstream arm_log;
  arm_log.open("arm-effector.log", std::ios::out);
  NeuralEndeffectorInterface neuralArm;
  nn->PrintStats();

  PulseNeuron pulse(0, 0.0f, 0.3f, -20);
  PulseNeuron pulse2(48, 7.0f, 0.3f, -20);
  PulseNeuron pulse3(49, 10.0f, 0.3f, -20);
  float SimulationTime = 13.0f;
  float deltaT = 0.01f;
  int N = SimulationTime / deltaT;

  float totalTime = 0;
  float cost = 0;
  float sync_counter = 0;
  float sync_value = 0.1f;
  for (int i = 0; i < N; i++) {
    pulse.Update(*nn, totalTime);
    pulse2.Update(*nn, totalTime);
    pulse3.Update(*nn, totalTime);
    nn->DoSimulationStep(deltaT);
    neuralArm.SyncWithNetwork(*nn);
    neuralArm.SetObjectRecognizedValue(trace->GetObjectRecognizedValue());
    neuralArm.Update(deltaT);
    logger.AddDataPoint(*nn, deltaT, totalTime);
    sync_counter += deltaT;
    if (sync_counter >= sync_value) {
      sync_counter = 0;
      neuralArm.SyncWithEndEffectorSystem();
      neuralArm.Log(totalTime, arm_log);
    }
    cost +=
        trace->GetDifferenceFromTrace(*neuralArm.GetEndeffectorSystemObject(),
                                      *neuralArm.GetGripperObject(), totalTime);

    totalTime += deltaT;
  }
  std::cout << "Total Optimized Cost: " << cost << std::endl;
  arm_log.close();
  logger.Close();
}
float GetRandomWeight() {
  float rnd = ((float)std::rand() / (float)(RAND_MAX));
  return 0.6f + rnd * 0.8f;
}
void AddAllSynapseTypes(NeuralNetwork *nn, int from, int to) {
  nn->AddExcitatorySynapse(from, to, GetRandomWeight());
  nn->AddInhibitorySynapse(from, to, GetRandomWeight());
  nn->AddGapJunction(from, to, 0.4f);
}
void AddEISynapseTypes(NeuralNetwork *nn, int from, int to) {
  nn->AddExcitatorySynapse(from, to, GetRandomWeight());
  nn->AddInhibitorySynapse(from, to, GetRandomWeight());
}
void ConnectToInter(NeuralNetwork *nn, int from) {
  AddAllSynapseTypes(nn, from, 39);
  AddAllSynapseTypes(nn, from, 40);
  AddAllSynapseTypes(nn, from, 41);
  AddAllSynapseTypes(nn, from, 42);
  AddAllSynapseTypes(nn, from, 44);
  AddAllSynapseTypes(nn, from, 45);
  AddAllSynapseTypes(nn, from, 46);
}
void ConnectToEncode(NeuralNetwork *nn, int from) {
  AddEISynapseTypes(nn, from, 37);
  AddEISynapseTypes(nn, from, 38);
  AddEISynapseTypes(nn, from, 51);
  AddEISynapseTypes(nn, from, 53);
  AddEISynapseTypes(nn, from, 54);
}
void ConnectToCommand(NeuralNetwork *nn, int from) {
  AddEISynapseTypes(nn, from, 33);
  AddEISynapseTypes(nn, from, 34);
  AddEISynapseTypes(nn, from, 35);
  AddEISynapseTypes(nn, from, 50);
  AddEISynapseTypes(nn, from, 52);
}
void InterconnectCommand(NeuralNetwork *nn) {
  AddEISynapseTypes(nn, 33, 33);
  AddEISynapseTypes(nn, 33, 34);
  AddEISynapseTypes(nn, 33, 35);
  AddEISynapseTypes(nn, 33, 50);
  AddEISynapseTypes(nn, 33, 52);

  AddEISynapseTypes(nn, 34, 33);
  AddEISynapseTypes(nn, 34, 34);
  AddEISynapseTypes(nn, 34, 35);
  AddEISynapseTypes(nn, 34, 50);
  AddEISynapseTypes(nn, 34, 52);

  AddEISynapseTypes(nn, 35, 33);
  AddEISynapseTypes(nn, 35, 34);
  AddEISynapseTypes(nn, 35, 35);
  AddEISynapseTypes(nn, 35, 50);
  AddEISynapseTypes(nn, 35, 52);

  AddEISynapseTypes(nn, 50, 33);
  AddEISynapseTypes(nn, 50, 34);
  AddEISynapseTypes(nn, 50, 35);
  AddEISynapseTypes(nn, 50, 50);
  AddEISynapseTypes(nn, 50, 52);

  AddEISynapseTypes(nn, 52, 33);
  AddEISynapseTypes(nn, 52, 34);
  AddEISynapseTypes(nn, 52, 35);
  AddEISynapseTypes(nn, 52, 50);
  AddEISynapseTypes(nn, 52, 52);
}
void ConnectToMotor(NeuralNetwork *nn, int from) {
  for (int i = 15; i <= 28; i++) {
    AddEISynapseTypes(nn, from, i);
  }
  AddAllSynapseTypes(nn, from, 30);
}
NeuralNetwork *CreateFullyConnectedNetwork() {
  NeuralNetwork *nn = new NeuralNetwork(55);
  nn->AddConstNeuron(43, -20);

  for (int i = 1; i <= 14; i++) {
    ConnectToInter(nn, i);
  }
  ConnectToInter(nn, 43);
  ConnectToEncode(nn, 29);
  ConnectToEncode(nn, 31);

  ConnectToEncode(nn, 39);
  ConnectToEncode(nn, 40);
  ConnectToEncode(nn, 41);
  ConnectToEncode(nn, 42);
  ConnectToEncode(nn, 44);
  ConnectToEncode(nn, 45);
  ConnectToEncode(nn, 46);

  ConnectToCommand(nn, 0);
  ConnectToCommand(nn, 37);
  ConnectToCommand(nn, 38);
  ConnectToCommand(nn, 51);
  ConnectToCommand(nn, 53);
  ConnectToCommand(nn, 54);

  InterconnectCommand(nn);

  ConnectToMotor(nn, 33);
  ConnectToMotor(nn, 34);
  ConnectToMotor(nn, 35);
  ConnectToMotor(nn, 50);
  ConnectToMotor(nn, 52);

  return nn;
}
