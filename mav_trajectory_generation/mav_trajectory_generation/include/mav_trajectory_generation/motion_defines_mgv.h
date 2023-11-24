#ifndef MGV_TRAJECTORY_MOTION_DEFINES_H_
#define MGV_TRAJECTORY_MOTION_DEFINES_H_

#include <string>

namespace mgv_trajectory_generation
{

  namespace derivative_order
  {
    static constexpr int POSITION = 0;
    static constexpr int VELOCITY = 1;
    static constexpr int ACCELERATION = 2;
    static constexpr int JERK = 3; // TBD
    static constexpr int SNAP = 4; // TBD

    static constexpr int ORIENTATION = 0;
    static constexpr int ANGULAR_VELOCITY = 1;
    static constexpr int ANGULAR_ACCELERATION = 2;

    static constexpr int INVALID = -1;
  }

  std::string positionDerivativeToString(int derivative);
  int positionDerivativeToInt(const std::string &string);

  std::string orintationDerivativeToString(int derivative);
  int orientationDerivativeToInt(const std::string &string);

} // namespace mav_trajectory_generation

#endif
