#ifndef MAV_PATH_SMOOTHING_PATH_SMOOTHER_BASE_H_
#define MAV_PATH_SMOOTHING_PATH_SMOOTHER_BASE_H_

#include "/home/patton/voxblox_ws/src/mav_voxblox_planning/mgv_comm/mgv_msgs/include/mgv_msgs/eigen_mgv_msgs.h"
#include <ros/ros.h>

#include <mav_planning_common/physical_constraints_mgv.h>

namespace mgv_planning {

class PathSmootherBase {
 public:
  PathSmootherBase() : verbose_(true) {}
  virtual ~PathSmootherBase() {}

  virtual void setParametersFromRos(const ros::NodeHandle& nh);
  void setPhysicalConstraints(const PhysicalConstraints& constraints);
  const PhysicalConstraints& getPhysicalConstraints() const;

  void setVerbose(bool verbose) { verbose_ = verbose; }
  bool getVerbose() const { return verbose_; }

  // By default, getPathBetweenWaypoints just calls getPathBetweenTwoPoints
  // on consecutive waypoints.
  virtual bool getPathBetweenWaypoints(
      const mgv_msgs::EigenTrajectoryPoint::Vector& waypoints,
      mgv_msgs::EigenTrajectoryPoint::Vector* path) const;

  virtual bool getPathBetweenTwoPoints(
      const mgv_msgs::EigenTrajectoryPoint& start,
      const mgv_msgs::EigenTrajectoryPoint& goal,
      mgv_msgs::EigenTrajectoryPoint::Vector* path) const {
    return false;
  }

 protected:
  PhysicalConstraints constraints_;

  bool verbose_;
};

}  // namespace mgv_planning

#endif  // mgv_PATH_SMOOTHING_PATH_SMOOTHER_BASE_H_
