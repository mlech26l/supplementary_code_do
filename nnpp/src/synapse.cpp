#include "synapse.h"

Synapse::Synapse(float weight, SynapseType typ, int src) {
  w = weight;
  type = typ;
  source = src;
}
