#ifndef mav_PATH_SMOOTHING_VELOCITY_RAMP_SMOOTHER_H_
#define mav_PATH_SMOOTHING_VELOCITY_RAMP_SMOOTHER_H_

#include "mav_path_smoothing/path_smoother_base_mgv.h"

namespace mgv_planning {

class VelocityRampSmoother : public PathSmootherBase {
 public:
  VelocityRampSmoother() : PathSmootherBase() {}
  virtual ~VelocityRampSmoother() {}

  virtual void setParametersFromRos(const ros::NodeHandle& nh);

  virtual bool getPathBetweenTwoPoints(
      const mgv_msgs::EigenTrajectoryPoint& start,
      const mgv_msgs::EigenTrajectoryPoint& goal,
      mgv_msgs::EigenTrajectoryPoint::Vector* path) const;
};

}  // namespace mgv_planning

#endif  // mav_PATH_SMOOTHING_VELOCITY_RAMP_SMOOTHER_H_
