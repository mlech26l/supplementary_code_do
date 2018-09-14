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
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

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
void PerformArmTest();
void OptimizeArm();
void Replay(std::string filename, std::string logFile);
void AddSynapsesForParking(NeuralNetwork &nn);
void RunRobotSimulationWithoutFeedback();
void ReplayTW(std::string filename);
void OptimizeNPark();
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
  if (argc >= 4) {
    Optimizer::NumberOfThread = std::stoi(argv[3]);
    std::cout << "Using number of threads " << Optimizer::NumberOfThread
              << std::endl;
  }
  Replay("modelred1.bnn", "mr1.dat");
  Replay("modelred2.bnn", "mr2.dat");
  Replay("modelred3.bnn", "mr3.dat");
  Replay("modelred4.bnn", "mr4.dat");
  // ReplayFaulty("ftol_1.bnn");
  // OptimizeNPark();
  // Benchmark();
  return 0;
  // OptimizeArm();
  // return 0;
  // PerformArmTest();
  // return 0;
  // ReplayTW("opt.bnn");
  // ReplayTW("tw1.bnn");
  // ReplayTW("opt.bnn");
  // return 0;
  // RunRobotSimulationWithoutFeedback();
  // return 0;
  // Replay("opt_1.bnn");
  // return 0;

  RobotInterface *robot = new RobotInterface();
  // robot->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);
  int n_x = 0;
  int n_start = 1;
  int n_y = 6;
  int n_t = 7;
  // y=7, t=1, x=0 best: 1.80817, (x6.99721 decrease)
  // y=1,t=7, x=0  Global best: 1.80569, (x7.00699 decrease)

  robot->BindToNeuralNetwork(n_x, n_y, n_t, 9, -1, 11, 10);
  PulseNeuron pulse(n_start, 3.0f, 30.0f, -20);

  RobotTrace *rt = new RobotTrace();
  rt->LoadFromFile("learn_tw.dat");
  RobotTrajectoryCostModel *costModel =
      new RobotTrajectoryCostModel(robot, rt, pulse);

  NeuralNetSerializer serializer_load("tap_withdrawl.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);

  nn->PrintStats();
  // NeuralNetSerializer serializer1("noopt.bnn");
  // serializer1.Serialize(nn);

  std::cout << "Adding all syapses as parameter...";
  std::vector<Parameter> allSynapses = GetAllSynapses(*nn);
  std::cout << " [Done]" << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run();
  std::cout << "Optimizer Done!" << std::endl;
  NeuralNetSerializer serializer("opt.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  ReplayTW("opt.bnn");
  delete nn;
  delete robot;
  delete rt;
  return 0;
}
void Replay(std::string filename, std::string logFile) {
  std::cout << "Run replay!" << std::endl;
  float logInterval = 0.1;
  NNLogger logger(logFile.c_str(), logInterval);
  RobotLogger rlog("robot.dat", logInterval);

  RobotInterface *robotInterface = new RobotInterface();
  RobotDynamics *robot = new RobotDynamics();
  robotInterface->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);

  // robotInterface->SetMaximumMotorSpeed(0.3f);

  NeuralNetSerializer serializer(filename);
  NeuralNetwork *nn = serializer.Deserialize();

  PulseNeuron pulse(7, 0.1f, 0.2f, -20);
  RobotTrace *rt = new RobotTrace();
  rt->LoadFromFile("robot_trace.dat");
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
void OptimizeNPark() {
  RobotInterface *robot = new RobotInterface();
  // robot->BindToNeuralNetwork(1, 9, 10, 8, -1, 11, 12);
  int n_x = 0;
  int n_start = 1;
  int n_y = 6;
  int n_t = 7;
  // y=7, t=1, x=0 best: 1.80817, (x6.99721 decrease)
  // y=1,t=7, x=0  Global best: 1.80569, (x7.00699 decrease)

  robot->BindToNeuralNetwork(n_x, n_y, n_t, 9, -1, 11, 10);
  PulseNeuron pulse(n_start, 0.1f, 0.2f, -20);

  RobotTrace *rt = new RobotTrace();
  rt->LoadFromFile("robot_trace.dat");
  RobotTrajectoryCostModel *costModel =
      new RobotTrajectoryCostModel(robot, rt, pulse);

  NeuralNetwork *nn = new NeuralNetwork(29);
  AddSynapsesForParking(*nn);

  // nn->PrintStats();
  // NeuralNetSerializer serializer1("noopt.bnn");
  // serializer1.Serialize(nn);

  // std::cout << "Adding all syapses as parameter...";
  std::vector<Parameter> allSynapses = GetAllSynapses(*nn);
  // std::cout << " [Done]" << std::endl;

  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

  Optimizer optimizer(nn, costModel, allSynapses);

  std::cout << "Starting optimizer..." << std::endl;
  start = std::chrono::high_resolution_clock::now();
  optimizer.Run();
  end = std::chrono::high_resolution_clock::now();

  int elapsed_seconds =
      std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
  std::cout << "Execution took " << elapsed_seconds << "s with "
            << Optimizer::NumberOfThread << " threads!" << std::endl;
  std::cout << "Optimizer Done!" << std::endl;
  NeuralNetSerializer serializer("npark_opt.bnn");
  serializer.Serialize(nn);

  // std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
  // << std::endl;
  Replay("opt.bnn", "log.dat");
  // std::ofstream file;
  // file.open("twbench_res_mlx.log", std::ios::app);
  // file << costModel->Evaluate(*nn) << std::endl;
  // file.close();
  delete nn;
  delete robot;
  delete rt;
  return;
}
void AddSynapsesForParking(NeuralNetwork &nn) {
  // Synapses for the goto X
  float w_trigger = 0.4;
  float gap_weight = 1.8;
  // float gap_weight_turn = 2;
  float gap_weight_turn = 1.8;
  float gap_weight_y = 1.8;
  nn.AddGapJunction(1, 3, gap_weight);
  nn.AddGapJunction(2, 3, gap_weight);
  nn.AddExcitatorySynapse(3, 4, w_trigger);

  // self ext., inh. pre
  nn.AddExcitatorySynapse(4, 4, 2);
  nn.AddInhibitorySynapse(4, 6, 2);

  nn.AddExcitatorySynapse(6, 4, w_trigger);

  nn.AddExcitatorySynapse(6, 6, 2);
  nn.AddExcitatorySynapse(7, 6, 2);
  nn.AddExcitatorySynapse(6, 8, 2);
  nn.AddExcitatorySynapse(4, 11, 1);

  // Synapses for goto park
  nn.AddExcitatorySynapse(4, 15, w_trigger);
  nn.AddExcitatorySynapse(15, 15, 2);
  nn.AddInhibitorySynapse(15, 4, 2);
  nn.AddInhibitorySynapse(15, 6, 2);

  nn.AddGapJunction(10, 14, gap_weight_turn);
  nn.AddGapJunction(13, 14, gap_weight_turn);

  nn.AddExcitatorySynapse(14, 15, w_trigger);
  nn.AddExcitatorySynapse(15, 8, 2);

  // Synapses for turning back
  nn.AddExcitatorySynapse(15, 16, w_trigger);

  nn.AddGapJunction(9, 18, gap_weight_y);
  nn.AddGapJunction(17, 18, gap_weight_y);
  nn.AddExcitatorySynapse(18, 16, w_trigger);
  nn.AddExcitatorySynapse(16, 16, 2);

  nn.AddExcitatorySynapse(16, 12, 2);
  nn.AddInhibitorySynapse(16, 15, 2);
  nn.AddInhibitorySynapse(16, 4, 2);

  // Synapses for driving straight again
  // w_stop_turning = 1;
  nn.AddGapJunction(10, 21, gap_weight);
  nn.AddGapJunction(20, 21, gap_weight);

  nn.AddInhibitorySynapse(21, 27, 2);

  nn.AddExcitatorySynapse(26, 27, 1);

  // Activation
  nn.AddExcitatorySynapse(27, 22, w_trigger);
  nn.AddExcitatorySynapse(16, 22, w_trigger);
  // self
  nn.AddExcitatorySynapse(22, 22, 2);

  nn.AddInhibitorySynapse(22, 16, 2);
  nn.AddInhibitorySynapse(22, 15, 2);
  nn.AddInhibitorySynapse(22, 4, 2);

  nn.AddExcitatorySynapse(22, 8, 2);

  // Synapses for stopping

  nn.AddGapJunction(24, 25, gap_weight);
  nn.AddGapJunction(1, 25, gap_weight);

  nn.AddExcitatorySynapse(25, 23, w_trigger);
  nn.AddExcitatorySynapse(22, 23, w_trigger);
  nn.AddExcitatorySynapse(23, 23, 2);

  nn.AddInhibitorySynapse(23, 22, 2);
  nn.AddInhibitorySynapse(23, 16, 2);
  nn.AddInhibitorySynapse(23, 15, 2);
  nn.AddInhibitorySynapse(23, 4, 2);

  // Strong inhibitory synapses between turning left and turning right
  nn.AddInhibitorySynapse(11, 12, 5);
  nn.AddInhibitorySynapse(12, 11, 5);

  /// Set cm
  float Cfast = 50e-3;
  // float Cnormal = 100e-3;
  // float Cslow = 300e-3;
  float Cslow = 50e-3;
  nn.SetCmOf(6, Cslow);
  nn.SetCmOf(4, Cslow);
  nn.SetCmOf(15, Cslow);
  nn.SetCmOf(16, Cslow);
  nn.SetCmOf(22, Cslow);
  nn.SetCmOf(23, Cslow);

  nn.SetCmOf(8, Cfast);
  nn.SetCmOf(11, Cfast);
  nn.SetCmOf(12, Cfast);

  // Set const neurons
  nn.AddConstNeuron(2, -20);
  // nn.AddConstNeuron(13, -30);
  nn.AddConstNeuron(13, -20);
  // nn.AddConstNeuron(17, -10);
  nn.AddConstNeuron(17, -20);
  nn.AddConstNeuron(20, -20);
  nn.AddConstNeuron(26, -20);
  nn.AddConstNeuron(24, -20);

  // for rounded tunring
  nn.AddExcitatorySynapse(4, 8, 0.8f);
  nn.AddExcitatorySynapse(16, 8, 0.8f);
}
void RunRobotSimulationWithoutFeedback() {
  RobotLogger rlog("tw_trace.dat", 1);

  std::unique_ptr<RobotInterface> robotInterface(new RobotInterface);
  std::unique_ptr<RobotDynamics> robot(new RobotDynamics);

  float SimulationTime = 20.0f;
  float deltaT = 0.01f;
  int N = SimulationTime / deltaT;

  float totalTime = 0;

  float v_want = 0.08f;
  float w_want = 0.18f;

  float phaseStart = 3.0f;
  float phaseTurn = 6.0f;

  float sync_counter = 0;
  float sync_value = 0.1f;

  for (int i = 0; i < N; i++) {
    if (totalTime <= phaseStart) {
      robotInterface->SetActuators(0.0f, 0.0f);
    } else if (totalTime <= phaseStart + phaseTurn) {
      robotInterface->SetActuators(v_want, w_want);
    } else if (totalTime <= phaseStart + 2 * phaseTurn) {
      robotInterface->SetActuators(v_want, -w_want);
    } else
      robotInterface->SetActuators(0.0f, 0.0f);
    if (sync_counter >= sync_value) {
      sync_counter = 0;
      robot->GetCommandsFromInterface(*robotInterface);
      robot->SendInputsToInterface(*robotInterface);
    }
    sync_counter += deltaT;
    robot->DoSimulationStep(deltaT);

    rlog.AddDataPoint(*robotInterface, deltaT, totalTime);
    totalTime += deltaT;
  }

  rlog.Close();
}
void OptimizeArm() {

  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  NeuralNetSerializer serializer_load("bnn/narm.bnn");
  NeuralNetwork *nn = serializer_load.Deserialize();
  // NeuralNetwork *nn = new NeuralNetwork(29);
  // AddSynapsesForParking(*nn);

  nn->PrintStats();
  std::cout << "Adding all syapses as parameter...";
  std::vector<Parameter> allSynapses = GetAllSynapsesCommingFrom(*nn, 32);
  std::cout << " [Done]" << std::endl;

  Optimizer optimizer(nn, costModel, allSynapses);
  std::cout << "Starting optimizer..." << std::endl;
  optimizer.Run();
  std::cout << "Optimizer Done!" << std::endl;
  NeuralNetSerializer serializer("bnn/narm_opt.bnn");
  serializer.Serialize(nn);

  std::cout << "After optimization cost: " << costModel->Evaluate(*nn)
            << std::endl;
  delete nn;
  delete trace;
}
void PerformArmTest() {
  NeuralNetSerializer serializer("bnn/narm.bnn");
  NeuralNetwork *nn = serializer.Deserialize();
  NNLogger logger("arm-log.dat", 0.1f);
  EndeffectorTrace *trace = new EndeffectorTrace();
  trace->LoadFromFile("arm_trace");
  EndeffectorCostModel *costModel = new EndeffectorCostModel(trace);

  std::ofstream arm_log;
  arm_log.open("arm-effector.log", std::ios::out);
  NeuralEndeffectorInterface neuralArm;
  nn->PrintStats();

  float SimulationTime = 10.0f;
  float deltaT = 0.01f;
  int N = SimulationTime / deltaT;

  float totalTime = 0;
  float cost = 0;
  float sync_counter = 0;
  float sync_value = 0.1f;
  for (int i = 0; i < N; i++) {
    nn->DoSimulationStep(deltaT);
    neuralArm.SyncWithNetwork(*nn);
    neuralArm.Update(deltaT);
    neuralArm.SetObjectRecognizedValue(trace->GetObjectRecognizedValue());
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
  std::cout << "Total Cost: " << cost << std::endl;
  arm_log.close();
  logger.Close();
}
