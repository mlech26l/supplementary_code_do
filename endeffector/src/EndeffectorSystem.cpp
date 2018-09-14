#include "EndeffectorSystem.h"
#include <iostream>
EndeffectorSystem::EndeffectorSystem() {}

std::tuple<float, float, float> EndeffectorSystem::GetGripperPosition() {
  float x = 0;
  float y = 0;
  float z = 0;
  for (int i = links.size() - 1; i >= 0; i--) {
    links[i]->ApplyEffect(x, y, z);
  }
  return std::make_tuple(x, y, z);
}
std::tuple<float, float, float> EndeffectorSystem::GetGripperDirection() {
  float x = 0;
  float y = 0;
  float z = 1;
  for (int i = links.size() - 1; i >= 0; i--) {
    links[i]->ApplyRotation(x, y, z);
  }
  return std::make_tuple(x, y, z);
}
void EndeffectorSystem::AddLink(EffectorLink *link) { links.push_back(link); }
