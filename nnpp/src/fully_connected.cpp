#include "EndeffectorCostModel.h"
#include "EndeffectorTrace.h"
#include "NNLogger.h"
#include "NeuralEndeffectorInterface.h"
#include "Optimizer.h"
#include "RobotDynamics.h"
#include "RobotLogger.h"
#include "RobotTrace.h"
#include "RobotTrajectoryCostModel.h"
#include "neural_net_serializer.h"
#include "neural_network.h"
#include "pulse_neuron.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
void DoCut(NeuralNetwork *nn) {
  for (int source = 0; source < nn->GetSize(); source++) {
    for (int dest = 0; dest < nn->GetSize(); dest++) {
      Synapse *ex = nn->GetSynapse(source, dest, Excitatory);
      Synapse *inh = nn->GetSynapse(source, dest, Inhibitory);
      Synapse *gj = nn->GetSynapse(source, dest, GapJunction);

      float REMOVE_SYNAPSE_W = 0.3f;
      float REMOVE_GAP_JUNCTION_W = 0.2f;
      if (gj != NULL && gj->w < REMOVE_GAP_JUNCTION_W) {
        // std::cout << "Gap junction below threshold (" << source << ", " <<
        // dest
        //           << ")" << std::endl;
        nn->RemoveSynapse(source, dest, GapJunction);
        gj = NULL;
      }
      if (ex != NULL && ex->w < REMOVE_SYNAPSE_W) {
        // std::cout << "Excitatory syn. below threshold (" << source << ", "
        //           << dest << ")" << std::endl;
        nn->RemoveSynapse(source, dest, Excitatory);
        ex = NULL;
      }
      if (inh != NULL && inh->w < REMOVE_SYNAPSE_W) {
        // std::cout << "Inhibitory syn. below threshold (" << source << ", "
        //           << dest << ")" << std::endl;
        nn->RemoveSynapse(source, dest, Inhibitory);
        inh = NULL;
      }
      if (ex != NULL && inh != NULL) {
        float wex = ex->w;
        float wix = inh->w;
        float DIFF_CUT_THRESHOLD = 0.7f;

        if (wex + DIFF_CUT_THRESHOLD < wix) {
          // std::cout << "Inhibition stronger than excitation (" << source <<
          // ", "
          //           << dest << ")" << std::endl;
          nn->RemoveSynapse(source, dest, Excitatory);
          ex = NULL;
          inh->w -= 0.5f;
          if (inh->w < 0.3f)
            inh->w = 0.3f;
        } else if (wix + DIFF_CUT_THRESHOLD < wex) {
          // std::cout << "Excitation stronger than inhibition (" << source <<
          // ", "
          // << dest << ")" << std::endl;
          nn->RemoveSynapse(source, dest, Inhibitory);
          inh = NULL;
          ex->w -= 0.5f;
          if (ex->w < 0.3f)
            ex->w = 0.3f;
        }
        float EQUALITY_THRESHOLD = 0.2f;
        if (std::fabs(wex - wix) < EQUALITY_THRESHOLD) {
          ex->w = 0.42f;
          inh->w = 0.42f;
        }
      }
      if (ex != NULL && ex->w > 2.2) {
        ex->w -= 0.8f;
      } else if (ex != NULL && ex->w > 1.8) {
        ex->w -= 0.6f;
      }
      if (inh != NULL && inh->w > 2.2) {
        inh->w -= 0.8f;
      } else if (inh != NULL && inh->w > 1.8) {
        inh->w -= 0.6f;
      }
    }
  }
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
void PrintToLatexFromFile(std::string bnnFile, std::string outfile) {
  NeuralNetSerializer serializer(bnnFile);
  NeuralNetwork *nn = serializer.Deserialize();
  PrintToLatex(nn, outfile);
  delete nn;
}
std::vector<Parameter> GetAllSynapsesCommingFrom(NeuralNetwork &nn, int from) {
  std::vector<Parameter> params;
  for (size_t i = 0; i < nn.GetSize(); i++) {
    std::vector<Synapse> &syns = nn.GetSynapsesOf(i);
    for (size_t j = 0; j < syns.size(); j++) {
      if (syns[j].source == from) {
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
  }
  return params;
}
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
void Replay(std::string filename);
NeuralNetwork *CreateFullyConnectedNetwork();
// 264 synapses
// 173
// 134
// 115
// 100
// 91
// 83
// 71
// 61
// 58
// 50
// 48
// 46
// 38
//
int round_count = 0;
void DoRound() {
  RobotInterface *robot = new RobotInterface();
  robot->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);

  PulseNeuron pulse(7, 4.0f, 0.4f, -20);

  RobotTrace *rt = new RobotTrace();
  rt->LoadFromFile("robot_trace_full.dat");
  RobotTrajectoryCostModel *costModel =
      new RobotTrajectoryCostModel(robot, rt, pulse);

  std::ofstream logFile;
  // logFile.open("cost_over_iterations.log", std::ios::out);
  logFile.open("cost_over_iterations.log", std::ios::app);
  // NeuralNetSerializer serializer("opt_round_21.bnn");
  // NeuralNetwork *nn = serializer.Deserialize();
  // DoCut(nn);
  // round_count = 22;

  NeuralNetwork *nn = CreateFullyConnectedNetwork();
  // std::cout << "Round " << round_count << std::endl;

  int roundsWithoutReduction = 0;
  int lastSynapseCount = nn->CountSynapses();
  while (true) {

    // NeuralNetSerializer serializer("opt_in_again13.bnn");
    // NeuralNetwork *nn = serializer.Deserialize();
    if (round_count > 0) {
      std::ostringstream graph_opt_file;
      graph_opt_file << "latex/opt" << round_count << ".tex";
      PrintToLatex(nn, graph_opt_file.str());
      std::cout << "Before cut: ";
      nn->PrintStats();
      DoCut(nn);
      std::cout << "After cut: ";
      std::ostringstream graph_cut_file;
      graph_cut_file << "latex/cut" << round_count << ".tex";
      PrintToLatex(nn, graph_cut_file.str());
    }
    // NeuralNetSerializer serializer3("cut.bnn");
    // serializer3.Serialize(nn);
    nn->PrintStats();
    //
    // return 0;
    // NeuralNetwork *nn = new NeuralNetwork(29);
    // AddSynapsesForParking(*nn);

    // nn->PrintStats();
    // NeuralNetSerializer serializer1("noopt.bnn");
    // serializer1.Serialize(nn);

    std::cout << "Adding all syapses as parameter...";
    std::vector<Parameter> allSynapses = GetAllSynapses(*nn);
    std::cout << " [Done]" << std::endl;

    Optimizer optimizer(nn, costModel, allSynapses);
    std::cout << "Starting optimizer..." << std::endl;
    optimizer.Run();
    std::cout << "Optimizer Done!" << std::endl;

    std::ostringstream opt_bnn_file;
    opt_bnn_file << "opt_round_" << round_count << ".bnn";
    NeuralNetSerializer serializer2(opt_bnn_file.str());
    serializer2.Serialize(nn);

    float costAfterOpt = costModel->Evaluate(*nn);
    std::cout << "Cost after round " << round_count << ": " << costAfterOpt
              << std::endl;
    logFile << round_count << " " << nn->CountSynapses() << " " << costAfterOpt
            << std::endl
            << std::flush;

    int synapseCount = nn->CountSynapses();
    if (synapseCount == lastSynapseCount) {
      roundsWithoutReduction++;
      if (roundsWithoutReduction >= 5) {
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
}
void PrintFiles() {
  std::cout << "int arr[] = {";
  for (int round_count = 0; round_count < 48; round_count++) {
    std::ostringstream bnnFile;
    bnnFile << "exp1/opt_round_" << round_count << ".bnn";
    NeuralNetSerializer serializer(bnnFile.str());
    NeuralNetwork *nn = serializer.Deserialize();
    // NeuralNetSerializer serializer("opt_in_again13.bnn");
    // NeuralNetwork *nn = serializer.Deserialize();
    std::ostringstream graph_opt_file;
    graph_opt_file << "latex/opt" << round_count << ".tex";
    PrintToLatex(nn, graph_opt_file.str());
    std::cout << nn->CountSynapses() << ", ";
    DoCut(nn);
    std::ostringstream graph_cut_file;
    graph_cut_file << "latex/cut" << round_count << ".tex";
    PrintToLatex(nn, graph_cut_file.str());
    // Neur
    delete nn;
  }
  std::cout << "0};" << std::endl;
}
int main(int argc, char const *argv[]) {
  // PrintFiles();
  // return 0;
  if (argc >= 2) {
    Optimizer::SeedLinear = std::stoi(argv[1]);
    std::cout << "Using linar seed " << Optimizer::SeedLinear << std::endl;
  }
  if (argc >= 3) {
    Optimizer::SeedQuadratic = std::stoi(argv[2]);
    std::cout << "Using quadratic seed " << Optimizer::SeedQuadratic
              << std::endl;
  }
  // NeuralNetwork *nn2 = CreateFullyConnectedNetwork();
  // PrintToLatex(nn2, "latex/full.tex");
  // PrintToLatexFromFile("opt_in_again.bnn", "latex/round1.tex");
  // PrintToLatexFromFile("opt_in_again2.bnn", "latex/round2.tex");
  // PrintToLatexFromFile("opt_in_again3.bnn", "latex/round3.tex");
  // PrintToLatexFromFile("opt_in_again4.bnn", "latex/round4.tex");
  // PrintToLatexFromFile("opt_in_again5.bnn", "latex/round5.tex");
  // PrintToLatexFromFile("opt_in_again6.bnn", "latex/round6.tex");
  // PrintToLatexFromFile("opt_in_again7.bnn", "latex/round7.tex");
  // PrintToLatexFromFile("opt_in_again8.bnn", "latex/round8.tex");
  // PrintToLatexFromFile("opt_in_again9.bnn", "latex/round9.tex");
  // return 0;
  DoRound();
  return 0;
}

void Replay(std::string filename) {
  std::cout << "Run replay!" << std::endl;
  float logInterval = 0.1;
  NNLogger logger("log.dat", logInterval);
  RobotLogger rlog("robot.dat", logInterval);

  RobotInterface *robotInterface = new RobotInterface();
  RobotDynamics *robot = new RobotDynamics();
  robotInterface->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);

  // robotInterface->SetMaximumMotorSpeed(0.3f);

  NeuralNetSerializer serializer(filename);
  NeuralNetwork *nn = serializer.Deserialize();

  PulseNeuron pulse(7, 4.0f, 0.4f, -20);
  RobotTrace *rt = new RobotTrace();
  rt->LoadFromFile("robot_trace_full.dat");
  RobotTrajectoryCostModel *costModel =
      new RobotTrajectoryCostModel(robotInterface, rt, pulse);

  float cost = costModel->Evaluate(*nn);
  nn->Reset();
  robotInterface->Reset();
  std::cout << "Cost model evaluation: " << cost << std::endl;

  nn->PrintStats();

  float SimulationTime = 30.0f;
  float deltaT = 0.01f;
  int N = SimulationTime / deltaT;

  float totalTime = 0;

  float sync_counter = 0;
  float sync_value = 0.1f;
  for (int i = 0; i < N; i++) {
    pulse.Update(*nn, totalTime);

    robotInterface->Sense(*nn);
    nn->DoSimulationStep(deltaT);
    robotInterface->Actuate(*nn);
    robot->DoSimulationStep(deltaT);

    logger.AddDataPoint(*nn, deltaT, totalTime);
    rlog.AddDataPoint(*robotInterface, deltaT, totalTime);
    sync_counter += deltaT;
    if (sync_counter >= sync_value) {
      sync_counter = 0;
      robot->GetCommandsFromInterface(*robotInterface);
      robot->SendInputsToInterface(*robotInterface);
    }

    totalTime += deltaT;
  }

  logger.Close();
  rlog.Close();
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
  AddAllSynapseTypes(nn, from, 3);
  AddAllSynapseTypes(nn, from, 14);
  AddAllSynapseTypes(nn, from, 18);
  AddAllSynapseTypes(nn, from, 21);
  AddAllSynapseTypes(nn, from, 27);
  AddAllSynapseTypes(nn, from, 25);
}
void ConnectToCommand(NeuralNetwork *nn, int from) {
  AddEISynapseTypes(nn, from, 6);
  AddEISynapseTypes(nn, from, 4);
  AddEISynapseTypes(nn, from, 15);
  AddEISynapseTypes(nn, from, 16);
  AddEISynapseTypes(nn, from, 22);
  AddEISynapseTypes(nn, from, 23);
}
void InterconnectCommand(NeuralNetwork *nn) {
  AddEISynapseTypes(nn, 6, 4);
  AddEISynapseTypes(nn, 6, 15);
  AddEISynapseTypes(nn, 6, 16);
  AddEISynapseTypes(nn, 6, 22);
  AddEISynapseTypes(nn, 6, 23);

  AddEISynapseTypes(nn, 4, 6);
  AddEISynapseTypes(nn, 4, 15);
  AddEISynapseTypes(nn, 4, 16);
  AddEISynapseTypes(nn, 4, 22);
  AddEISynapseTypes(nn, 4, 23);

  AddEISynapseTypes(nn, 15, 4);
  AddEISynapseTypes(nn, 15, 6);
  AddEISynapseTypes(nn, 15, 16);
  AddEISynapseTypes(nn, 15, 22);
  AddEISynapseTypes(nn, 15, 23);

  AddEISynapseTypes(nn, 16, 4);
  AddEISynapseTypes(nn, 16, 6);
  AddEISynapseTypes(nn, 16, 15);
  AddEISynapseTypes(nn, 16, 22);
  AddEISynapseTypes(nn, 16, 23);

  AddEISynapseTypes(nn, 22, 4);
  AddEISynapseTypes(nn, 22, 6);
  AddEISynapseTypes(nn, 22, 15);
  AddEISynapseTypes(nn, 22, 16);
  AddEISynapseTypes(nn, 22, 23);

  AddEISynapseTypes(nn, 23, 4);
  AddEISynapseTypes(nn, 23, 6);
  AddEISynapseTypes(nn, 23, 15);
  AddEISynapseTypes(nn, 23, 16);
  AddEISynapseTypes(nn, 23, 22);

  AddEISynapseTypes(nn, 4, 4);
  AddEISynapseTypes(nn, 6, 6);
  AddEISynapseTypes(nn, 15, 15);
  AddEISynapseTypes(nn, 16, 16);
  AddEISynapseTypes(nn, 22, 22);
  AddEISynapseTypes(nn, 23, 23);
}
void ConnectToMotor(NeuralNetwork *nn, int from) {
  AddEISynapseTypes(nn, from, 8);
  AddEISynapseTypes(nn, from, 11);
  AddEISynapseTypes(nn, from, 12);
}
NeuralNetwork *CreateFullyConnectedNetwork() {
  NeuralNetwork *nn = new NeuralNetwork(28);
  nn->AddConstNeuron(2, -20);
  ConnectToInter(nn, 1);
  ConnectToInter(nn, 2);
  ConnectToInter(nn, 9);
  ConnectToInter(nn, 10);

  ConnectToCommand(nn, 3);
  ConnectToCommand(nn, 14);
  ConnectToCommand(nn, 18);
  ConnectToCommand(nn, 21);
  ConnectToCommand(nn, 27);
  ConnectToCommand(nn, 25);

  InterconnectCommand(nn);

  ConnectToMotor(nn, 4);
  ConnectToMotor(nn, 6);
  ConnectToMotor(nn, 15);
  ConnectToMotor(nn, 16);
  ConnectToMotor(nn, 22);
  ConnectToMotor(nn, 23);

  ConnectToCommand(nn, 7);
  return nn;
}
