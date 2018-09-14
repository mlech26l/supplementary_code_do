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
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

std::vector<Parameter> GetAllSynapsesCommingFromExcept(NeuralNetwork &nn,
                                                       int from, int except) {
  std::vector<Parameter> params;
  for (size_t i = 0; i < nn.GetSize(); i++) {
    if (i == except)
      continue;
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
void AddSynapse(std::vector<Parameter> &params, NeuralNetwork &nn, int source,
                int dest) {
  std::vector<Synapse> &syns = nn.GetSynapsesOf(dest);
  for (size_t j = 0; j < syns.size(); j++) {
    if (syns[j].source == source) {
      int typ = 0;
      if (syns[j].type == Inhibitory)
        typ = 1;
      else if (syns[j].type == GapJunction)
        typ = 2;
      Parameter p(ParameterType::Synapse, syns[j].source, dest, typ);
      p.SetValue(syns[j].w);
      params.push_back(p);
    }
  }
}
void GetAllSynapsesGoingTo(std::vector<Parameter> &params, NeuralNetwork &nn,
                           int dest) {
  std::vector<Synapse> &syns = nn.GetSynapsesOf(dest);
  for (size_t j = 0; j < syns.size(); j++) {
    int typ = 0;
    if (syns[j].type == Inhibitory)
      typ = 1;
    else if (syns[j].type == GapJunction)
      typ = 2;
    Parameter p(ParameterType::Synapse, syns[j].source, dest, typ);
    p.SetValue(syns[j].w);
    params.push_back(p);
  }
}
void GetAllSynapsesCommingFromAndTo(std::vector<Parameter> &params,
                                    NeuralNetwork &nn, int id) {
  std::vector<Parameter> subParams =
      GetAllSynapsesCommingFromExcept(nn, id, id);
  for (int i = 0; i < subParams.size(); i++) {
    params.push_back(subParams[i]);
  }
  std::vector<Synapse> &syns = nn.GetSynapsesOf(id);
  for (size_t j = 0; j < syns.size(); j++) {
    int typ = 0;
    if (syns[j].type == Inhibitory)
      typ = 1;
    else if (syns[j].type == GapJunction)
      typ = 2;
    Parameter p(ParameterType::Synapse, syns[j].source, id, typ);
    p.SetValue(syns[j].w);
    params.push_back(p);
  }
}
void RemoveDuplicates(std::vector<Parameter> &params) {
  bool dupFound = false;
  do {
    dupFound = false;
    for (unsigned int i = 0; i < params.size(); i++) {
      for (unsigned int j = 0; j < params.size(); j++) {
        if (i == j)
          continue;
        if (params[i].GetIndexA() == params[j].GetIndexA() &&
            params[i].GetIndexB() == params[j].GetIndexB() &&
            params[i].GetIndexC() == params[j].GetIndexC()) {
          params.erase(params.begin() + j);
          dupFound = true;
          // terminate loop
          i = params.size();
          j = params.size();
        }
      }
    }

  } while (dupFound);
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
void DoRobotEffectorCut(NeuralNetwork *nn, int source) {
  for (int i = 15; i < 29; i += 2) {
    int second = i + 1;
    Synapse *ex1 = nn->GetSynapse(source, i, Excitatory);
    Synapse *ex2 = nn->GetSynapse(source, second, Excitatory);
    if (ex1->w >= ex2->w) {
      nn->RemoveSynapse(source, second, Excitatory);
    } else {
      nn->RemoveSynapse(source, i, Excitatory);
    }
  }
}

void Replay(std::string bnnFile);
void OptimizeForPoint() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_p2.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("narm_raw_for_out.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);

  nn->PrintStats();
  std::cout << "Adding all syapses as parameter...";
  std::vector<Parameter> allSynapses =
      GetAllSynapsesCommingFromExcept(*nn, 34, 34);
  std::cout << " [Done]" << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run();
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  std::cout << "Cutting weak synapses ..." << std::endl;
  DoRobotEffectorCut(nn, 34);
  std::cout << "Starting optimizer again..." << std::endl;
  std::vector<Parameter> allSynapsesAgain =
      GetAllSynapsesCommingFromExcept(*nn, 34, 34);
  for (int i = 0; i < allSynapsesAgain.size(); i++) {
    allSynapsesAgain[i].SetValue(0.5f);
    allSynapsesAgain[i].AddNoise(1);
  }
  Optimizer optimizerAgain(nn, costModel, allSynapsesAgain);
  optimizerAgain.Run();
  std::cout << "Optimizer Done second time!" << std::endl;
  NeuralNetSerializer serializer("narm_opt_for_out.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void OptimizeFull() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_full.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  EndeffectorTrace *traceV = new EndeffectorTrace(true);
  traceV->LoadFromFile("arm_trace_full.dat");
  EndeffectorCostModel *costModelV = new EndeffectorCostModel(traceV);

  NeuralNetSerializer serializer_load("narm_full_3.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);

  std::cout << "Before optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;

  nn->PrintStats();
  std::vector<Parameter> allSynapses;
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 39);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 40);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 41);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 42);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 44);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 45);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 46);
  AddSynapse(allSynapses, *nn, 37, 35);
  AddSynapse(allSynapses, *nn, 38, 34);

  NeuronPotentialCostPenalty penalty(33, -40, PenaltyThresholdType::Above, 7,
                                     PenaltyTermporalType::After, 0.1f);
  costModel->AddPenalty(penalty);

  std::cout << "Adding " << allSynapses.size() << " synapses as parameter"
            << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run(EffortLevel::Max);
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer("narm_full_3_opt.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void OptimizeFinal() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("narm_final_3.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);
  EndeffectorTrace *traceV = new EndeffectorTrace(true);
  traceV->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModelV = new EndeffectorCostModel(traceV);

  std::cout << "Before optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;

  nn->PrintStats();
  std::vector<Parameter> allSynapses;
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 51);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 53);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 54);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 50);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 52);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 34);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 33);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 35);
  RemoveDuplicates(allSynapses);

  NeuronPotentialCostPenalty penalty(33, -40, PenaltyThresholdType::Above, 9,
                                     PenaltyTermporalType::After, 0.1f);
  costModel->AddPenalty(penalty);

  std::cout << "Adding " << allSynapses.size() << " synapses as parameter"
            << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run(EffortLevel::Max);
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer("narm_final_3_opt.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  std::cout << "After optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void OptimizeSequence() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_full.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("narm_reduced.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);

  nn->PrintStats();
  std::vector<Parameter> allSynapses;
  AddSynapse(allSynapses, *nn, 37, 35);
  AddSynapse(allSynapses, *nn, 38, 34);
  AddSynapse(allSynapses, *nn, 31, 33);

  AddSynapse(allSynapses, *nn, 33, 35);
  AddSynapse(allSynapses, *nn, 35, 34);
  AddSynapse(allSynapses, *nn, 34, 33);
  AddSynapse(allSynapses, *nn, 33, 34);
  AddSynapse(allSynapses, *nn, 34, 35);
  AddSynapse(allSynapses, *nn, 35, 33);
  std::cout << "Adding " << allSynapses.size() << "synapses as parameter"
            << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run();
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer("narm_reduced_opt.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void OptimizeForAllPoints() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_full.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("narm_reduced.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);

  nn->PrintStats();
  std::vector<Parameter> allSynapses;
  GetAllSynapsesGoingTo(allSynapses, *nn, 15);
  GetAllSynapsesGoingTo(allSynapses, *nn, 16);
  GetAllSynapsesGoingTo(allSynapses, *nn, 17);
  GetAllSynapsesGoingTo(allSynapses, *nn, 18);
  GetAllSynapsesGoingTo(allSynapses, *nn, 19);
  GetAllSynapsesGoingTo(allSynapses, *nn, 20);
  GetAllSynapsesGoingTo(allSynapses, *nn, 21);
  GetAllSynapsesGoingTo(allSynapses, *nn, 22);
  GetAllSynapsesGoingTo(allSynapses, *nn, 23);
  GetAllSynapsesGoingTo(allSynapses, *nn, 24);
  GetAllSynapsesGoingTo(allSynapses, *nn, 25);
  GetAllSynapsesGoingTo(allSynapses, *nn, 26);
  GetAllSynapsesGoingTo(allSynapses, *nn, 27);
  GetAllSynapsesGoingTo(allSynapses, *nn, 28);
  std::cout << "Adding " << allSynapses.size() << "synapses as parameter"
            << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run();
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer("narm_reduced_opt.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void OptimizeForRelease() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("narm_release_4.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);
  EndeffectorTrace *traceV = new EndeffectorTrace(true);
  traceV->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModelV = new EndeffectorCostModel(traceV);
  std::cout << "Before optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;

  nn->PrintStats();
  // std::vector<Parameter> allSynapses  = GetAllSynapses(*nn);
  std::vector<Parameter> allSynapses;

  RemoveDuplicates(allSynapses);
  std::cout << "Adding " << allSynapses.size() << "synapses as parameter"
            << std::endl;

  NeuronPotentialCostPenalty penalty(33, -40, PenaltyThresholdType::Above, 9,
                                     PenaltyTermporalType::After, 0.1f);
  costModel->AddPenalty(penalty);

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run(EffortLevel::Max);
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer("narm_release_5.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;

  std::cout << "After optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void TakeGreater(NeuralNetwork &nn, int from1, int to1, SynapseType typ1,
                 int from2, int to2, SynapseType typ2) {
  Synapse *syn1 = nn.GetSynapse(from1, to1, typ1);
  Synapse *syn2 = nn.GetSynapse(from2, to2, typ2);
  if (syn1 == NULL) {
    std::cout << "ERROR: syn1 " << from1 << " to " << to1 << " not found"
              << std::endl;
  }
  if (syn2 == NULL) {
    std::cout << "ERROR: syn2 " << from2 << " to " << to2 << " not found"
              << std::endl;
  }
  if (syn1->w >= syn2->w) {
    nn.RemoveSynapse(from2, to2, typ2);
  } else {
    nn.RemoveSynapse(from1, to1, typ1);
  }
}
void OptimizeForReleaseIn() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("narm_release_in.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);
  EndeffectorTrace *traceV = new EndeffectorTrace(true);
  traceV->LoadFromFile("arm_trace_final.dat");
  EndeffectorCostModel *costModelV = new EndeffectorCostModel(traceV);
  std::cout << "Before optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;

  nn->PrintStats();
  std::vector<Parameter> allSynapses;
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 39);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 40);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 41);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 42);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 44);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 45);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 46);

  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 37);
  GetAllSynapsesCommingFromAndTo(allSynapses, *nn, 38);

  RemoveDuplicates(allSynapses);
  std::cout << "Adding " << allSynapses.size() << "synapses as parameter"
            << std::endl;

  NeuronPotentialCostPenalty penalty(33, -40, PenaltyThresholdType::Above, 9,
                                     PenaltyTermporalType::After, 0.1f);
  costModel->AddPenalty(penalty);

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run(EffortLevel::High);
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer("narm_release_between.bnn");
  serializer.Serialize(nn);
  std::cout << "After optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;

  std::cout << "Perform cut" << std::endl;
  TakeGreater(*nn, 1, 39, SynapseType::GapJunction, 2, 39,
              SynapseType::GapJunction);
  TakeGreater(*nn, 3, 40, SynapseType::GapJunction, 4, 40,
              SynapseType::GapJunction);
  TakeGreater(*nn, 5, 41, SynapseType::GapJunction, 6, 41,
              SynapseType::GapJunction);
  TakeGreater(*nn, 7, 42, SynapseType::GapJunction, 8, 42,
              SynapseType::GapJunction);
  TakeGreater(*nn, 9, 44, SynapseType::GapJunction, 10, 44,
              SynapseType::GapJunction);
  TakeGreater(*nn, 11, 45, SynapseType::GapJunction, 12, 45,
              SynapseType::GapJunction);
  TakeGreater(*nn, 13, 46, SynapseType::GapJunction, 14, 46,
              SynapseType::GapJunction);

  TakeGreater(*nn, 39, 38, SynapseType::Excitatory, 39, 38,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 40, 38, SynapseType::Excitatory, 40, 38,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 41, 38, SynapseType::Excitatory, 41, 38,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 42, 38, SynapseType::Excitatory, 42, 38,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 44, 38, SynapseType::Excitatory, 44, 38,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 45, 38, SynapseType::Excitatory, 45, 38,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 46, 38, SynapseType::Excitatory, 46, 38,
              SynapseType::Inhibitory);

  TakeGreater(*nn, 39, 37, SynapseType::Excitatory, 39, 37,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 40, 37, SynapseType::Excitatory, 40, 37,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 41, 37, SynapseType::Excitatory, 41, 37,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 42, 37, SynapseType::Excitatory, 42, 37,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 44, 37, SynapseType::Excitatory, 44, 37,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 45, 37, SynapseType::Excitatory, 45, 37,
              SynapseType::Inhibitory);
  TakeGreater(*nn, 46, 37, SynapseType::Excitatory, 46, 37,
              SynapseType::Inhibitory);

  std::vector<Parameter> allSynapses2;
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 39);
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 40);
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 41);
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 42);
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 44);
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 45);
  GetAllSynapsesCommingFromAndTo(allSynapses2, *nn, 46);
  RemoveDuplicates(allSynapses2);
  std::cout << "Adding " << allSynapses2.size() << "synapses as parameter"
            << std::endl;

  Optimizer optimizer2(nn, costModel, allSynapses2);
  std::cout << "Starting optimizer again..." << std::endl;
  optimizer2.Run(EffortLevel::High);
  std::cout << "Optimizer Done!" << std::endl;
  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  NeuralNetSerializer serializer4("narm_release_4.bnn");
  serializer4.Serialize(nn);
  std::cout << "After optimization cost V: " << costModelV->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void CutGj() {
  NeuralNetSerializer serializer_load("narm_full_opt.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();

  NeuralReduction reduction;
  for (int i = 1; i <= 31; i++) {
    reduction.AddMotorNeuron(i);
  }
  reduction.Reduce(*nn);

  NeuralNetSerializer serializer("narm_full_2.bnn");
  serializer.Serialize(nn);
}
int main(int argc, char const *argv[]) {
  // OptimizeForAllPoints();
  // Replay("narm_reduced_opt.bnn");
  if (argc >= 2) {
    Optimizer::SeedLinear = std::stoi(argv[1]);
    std::cout << "Using linar seed " << Optimizer::SeedLinear << std::endl;
  }
  if (argc >= 3) {
    Optimizer::SeedQuadratic = std::stoi(argv[2]);
    std::cout << "Using quadratic seed " << Optimizer::SeedQuadratic
              << std::endl;
  }
  // OptimizeSequence();
  // Replay("narm_reduced.bnn");
  // Replay("narm_reduced_opt.bnn");
  // OptimizeForPoint();
  // Replay("narm_opt_for_out.bnn");
  // return 0;
  // return 0;
  // OptimizeFull();
  // OptimizeFinal();
  // Replay("narm_release_6.bnn");
  // OptimizeForRelease();
  // OptimizeFull();
  OptimizeForReleaseIn();
  // Replay("narm_release_3.bnn");
  // Replay("narm_full_3_opt.bnn");
  // Replay("narm_final_3_opt.bnn");
  // return 0;
  return 0;
}
// Red raw
// Cost function term at 0: 0
// Cost function term at 4.01: 0
// Cost function term at 6.50005: 5.97235e-05
// Cost function term at 9.50012: 17.8656
// Cost function term at 11.0002: 0.885017
// Total Optimized Cost: 18.7506

// Red opt
// Cost function term at 0: 0
// Cost function term at 4.01: 0
// Cost function term at 6.50005: 0.000372367
// Cost function term at 9.50012: 8.66615
// Cost function term at 11.0002: 0.672435
// Total Optimized Cost: 9.33896

void Replay(std::string bnnFile) {
  NeuralNetSerializer serializer(bnnFile);
  NeuralNetwork *nn = serializer.Deserialize();
  NNLogger logger("arm-log.dat", 0.1f);
  EndeffectorTrace *trace = new EndeffectorTrace(true);
  trace->LoadFromFile("arm_trace_final2.dat");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

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
