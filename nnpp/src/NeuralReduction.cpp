#include "NeuralReduction.h"
#include <cmath>
void NeuralReduction::AddMotorNeuron(std::vector<int> neurons) {
  for (int i = 0; i < neurons.size(); i++) {
    motorNeurons.push_back(neurons[i]);
  }
}
void NeuralReduction::AddMotorNeuron(int neuron) {
  motorNeurons.push_back(neuron);
}
void NeuralReduction::ReachabilityOfMotorNeuronCut(NeuralNetwork &nn) {
  bool *checkerbord = new bool[nn.GetSize()];
  for (int i = 0; i < nn.GetSize(); i++) {
    checkerbord[i] = false;
  }
  for (int i = 0; i < motorNeurons.size(); i++) {
    DFSreach(nn, motorNeurons[i], checkerbord);
  }

  for (int neuron = 0; neuron < nn.GetSize(); neuron++) {
    if (checkerbord[neuron] == false) {
      // std::cout << "Neuron without effect found: " << neuron << std::endl;
      std::vector<Synapse> &syns = nn.GetSynapsesOf(neuron);
      for (size_t j = 0; j < syns.size(); j++) {
        nn.RemoveSynapse(syns[j].source, neuron, SynapseType::Excitatory);
        nn.RemoveSynapse(syns[j].source, neuron, SynapseType::Inhibitory);
        nn.RemoveSynapse(syns[j].source, neuron, SynapseType::GapJunction);
        // std::cout << "DFS Remoe  " << syns[j].source << " -> " << neuron
        // << std::endl;
      }
    }
  }
  delete[] checkerbord;
}

void NeuralReduction::DFSreach(NeuralNetwork &nn, int neuron,
                               bool *checkerbord) {
  if (checkerbord[neuron] == false) {
    checkerbord[neuron] = true;
    std::vector<Synapse> &syns = nn.GetSynapsesOf(neuron);
    for (size_t j = 0; j < syns.size(); j++) {
      DFSreach(nn, syns[j].source, checkerbord);
    }
  }
}
float NeuralReduction::SumUpSynapseWeights(NeuralNetwork &nn, int dest,
                                           SynapseType typ) {
  std::vector<Synapse> &syns = nn.GetSynapsesOf(dest);
  float sum = 0;
  for (unsigned int i = 0; i < syns.size(); i++) {
    if (syns[i].type == SynapseType::Excitatory)
      sum += syns[i].w;
    else if (syns[i].type == SynapseType::Inhibitory)
      sum += syns[i].w;
    else if (syns[i].type == SynapseType::GapJunction)
      sum += syns[i].w;
  }
  return sum;
}
void NeuralReduction::CutBySum(NeuralNetwork &nn) {
  for (int neuron = 0; neuron < nn.GetSize(); neuron++) {
    std::vector<Synapse> &syns = nn.GetSynapsesOf(neuron);
    float sum_ex = SumUpSynapseWeights(nn, neuron, SynapseType::Excitatory);
    float sum_inh = SumUpSynapseWeights(nn, neuron, SynapseType::Inhibitory);
    float sum_gj = SumUpSynapseWeights(nn, neuron, SynapseType::GapJunction);

    float percentil = 0.05f;
    float percentil_gj = 0.1f;

    for (unsigned int i = 0; i < syns.size(); i++) {
      if (syns[i].type == SynapseType::Excitatory) {
        if (sum_ex * percentil >= syns[i].w) {
          // std::cout << "Ex Below 10% of sum " << syns[i].source << " -> "
          // << neuron << std::endl;
          nn.RemoveSynapse(syns[i].source, neuron, SynapseType::Excitatory);
          i--;
          sum_ex = SumUpSynapseWeights(nn, neuron, SynapseType::Excitatory);
        }
      } else if (syns[i].type == SynapseType::Inhibitory) {
        if (sum_inh * percentil >= syns[i].w) {
          // std::cout << "Inh Below 10% of sum " << syns[i].source << " -> "
          // << neuron << std::endl;
          nn.RemoveSynapse(syns[i].source, neuron, SynapseType::Inhibitory);
          i--;
          sum_inh = SumUpSynapseWeights(nn, neuron, SynapseType::Inhibitory);
        }
      } else if (syns[i].type == SynapseType::GapJunction) {
        if (sum_gj * percentil_gj >= syns[i].w) {
          // std::cout << "Gj Below 10% of sum " << syns[i].source << " -> "
          // << neuron << std::endl;
          nn.RemoveSynapse(syns[i].source, neuron, SynapseType::GapJunction);
          i--;
          sum_gj = SumUpSynapseWeights(nn, neuron, SynapseType::GapJunction);
        }
      }
    }
  }
}
void NeuralReduction::CutByThreshold(NeuralNetwork &nn) {
  for (int dest = 0; dest < nn.GetSize(); dest++) {
    std::vector<Synapse> &syns = nn.GetSynapsesOf(dest);
    for (unsigned int i = 0; i < syns.size(); i++) {
      if (syns[i].type == SynapseType::GapJunction) {
        if (syns[i].w < 0.25f) {
          nn.RemoveSynapse(syns[i].source, dest, SynapseType::GapJunction);
          // std::cout << "Cut threshold " << syns[i].source << " -> " << dest
          // << std::endl;
        }
      } else {
        if (syns[i].w < 0.25f) {
          nn.RemoveSynapse(syns[i].source, dest, syns[i].type);
          // std::cout << "Cut threshold " << syns[i].source << " -> " << dest
          // << std::endl;
        }
      }
    }
  }
}
void NeuralReduction::CutMultiSynapses(NeuralNetwork &nn) {
  for (int source = 0; source < nn.GetSize(); source++) {
    for (int dest = 0; dest < nn.GetSize(); dest++) {
      Synapse *ex = nn.GetSynapse(source, dest, Excitatory);
      Synapse *inh = nn.GetSynapse(source, dest, Inhibitory);
      Synapse *gj = nn.GetSynapse(source, dest, GapJunction);

      if (ex != NULL && inh != NULL) {
        float wex = ex->w;
        float wix = inh->w;
        float DIFF_CUT_THRESHOLD = 0.5f;

        if (wex + DIFF_CUT_THRESHOLD < wix) {
          // std::cout << "Inhibition stronger than excitation (" <<
          // source <<
          // ", "
          //           << dest << ")" << std::endl;
          nn.RemoveSynapse(source, dest, Excitatory);
          // std::cout << "Inh wins " << source << " -> " << dest << std::endl;
          ex = NULL;
          // inh->w -= 0.5f;
          // if (inh->w < 0.3f)
          //   inh->w = 0.3f;
        } else if (wix + DIFF_CUT_THRESHOLD < wex) {
          // std::cout << "Excitation stronger than inhibition (" <<
          // source <<
          // ", "
          // << dest << ")" << std::endl;
          nn.RemoveSynapse(source, dest, Inhibitory);
          // std::cout << "Ex wins " << source << " -> " << dest << std::endl;
          inh = NULL;
          // ex->w -= 0.5f;
          // if (ex->w < 0.3f)
          //   ex->w = 0.3f;
        }
        float EQUALITY_THRESHOLD = 0.2f;
        if (std::fabs(wex - wix) < EQUALITY_THRESHOLD) {
          ex->w = 0.42f;
          inh->w = 0.42f;
        }
      }
      // if (ex != NULL && ex->w > 2.2) {
      //   ex->w -= 0.8f;
      // } else if (ex != NULL && ex->w > 1.8) {
      //   ex->w -= 0.6f;
      // }
      // if (inh != NULL && inh->w > 2.2) {
      //   inh->w -= 0.8f;
      // } else if (inh != NULL && inh->w > 1.8) {
      //   inh->w -= 0.6f;
      // }
    }
  }
}
void NeuralReduction::ZeroInputCut(NeuralNetwork &nn) {
  for (int neuron = 0; neuron < nn.GetSize(); neuron++) {
    if (nn.GetSynapsesOf(neuron).empty()) {
      for (int dest = 0; dest < nn.GetSize(); dest++) {
        nn.RemoveSynapse(neuron, dest, Excitatory);
        nn.RemoveSynapse(neuron, dest, Inhibitory);
        nn.RemoveSynapse(neuron, dest, GapJunction);
      }
    }
  }
}
void NeuralReduction::Reduce(NeuralNetwork &nn) {
  CutByThreshold(nn);
  CutBySum(nn);
  CutMultiSynapses(nn);
  // Except sensory neurons from getting removed by this method
  // ZeroInputCut(nn);
  ReachabilityOfMotorNeuronCut(nn);
  CutByThreshold(nn);
  CutBySum(nn);
}
