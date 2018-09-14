#include "EndeffectorState.h"
#include <tuple>

static float square(float x) { return x * x; }
float EndeffectorState::Distance(EndeffectorSystem &endeffector,
                                 GripperEffector &gripper) {
  float x_is, y_is, z_is;
  std::tie(x_is, y_is, z_is) = endeffector.GetGripperPosition();

  float roll_is, pitch_is, yaw_is;
  std::tie(roll_is, pitch_is, yaw_is) = endeffector.GetGripperDirection();

  float gripper_is = gripper.GetGripperState();

  float distance = 0;
  distance += square((x_is - x) / 100.0f);
  distance += square((y_is - y) / 100.0f);
  distance += square((z_is - z) / 100.0f);
  distance += square(roll_is - roll);
  distance += square(pitch_is - pitch);
  distance += square(yaw_is - yaw);
  distance += square(gripper_is - gripperState);
  // std::cout << "####### COST #####" << std::endl;
  // std::cout << "cost x    : " << square((x_is - x) / 100.0f) << ", " << x
  //           << " vs " << x_is << std::endl;
  // std::cout << "cost y    : " << square((y_is - y) / 100.0f) << ", " << y
  //           << " vs " << y_is << std::endl;
  // std::cout << "cost z    : " << square((z_is - z) / 100.0f) << ", " << z
  //           << " vs " << z_is << std::endl;
  // std::cout << "cost roll : " << square(roll_is - roll) << std::endl;
  // std::cout << "cost pitch: " << square(pitch_is - pitch) << std::endl;
  // std::cout << "cost yaw: " << square(yaw_is - yaw) << std::endl;
  // std::cout << "cost grip : " << square(gripper_is - gripperState) <<
  // std::endl;

  return distance;
}
