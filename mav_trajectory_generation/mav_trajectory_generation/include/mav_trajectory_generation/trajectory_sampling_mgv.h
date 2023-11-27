#ifndef mgv_TRAJECTORY_GENERATION_TRAJECTORY_SAMPLING_H_
#define mgv_TRAJECTORY_GENERATION_TRAJECTORY_SAMPLING_H_

#include <mgv_msgs/eigen_mgv_msgs.h>
#include "mav_trajectory_generation/trajectory_mgv.h"

namespace mgv_trajectory_generation
{

  // All of these functions sample a trajectory at a time, or a range of times,
  // into an EigenTrajectoryPoint (or vector). These support 3D or 4D
  // trajectories. If the trajectories are 4D, the 4th dimension is assumed to
  // be yaw.
  // If no yaw is set, then it is simply left at its current value (0 by default).

  bool sampleTrajectoryAtTime(const Trajectory &trajectory, double sample_time,
                              mgv_msgs::EigenTrajectoryPoint *state);

  bool sampleTrajectoryInRange(const Trajectory &trajectory, double min_time,
                               double max_time, double sampling_interval,
                               mgv_msgs::EigenTrajectoryPointVector *states);

  bool sampleTrajectoryStartDuration(
      const Trajectory &trajectory, double start_time, double duration,
      double sampling_interval, mgv_msgs::EigenTrajectoryPointVector *states);

  bool sampleWholeTrajectory(const Trajectory &trajectory,
                             double sampling_interval,
                             mgv_msgs::EigenTrajectoryPoint::Vector *states);

  bool sampleSegmentAtTime(const Segment &segment, double sample_time,
                           mgv_msgs::EigenTrajectoryPoint *state);

  template <class T>
  bool sampleFlatStateAtTime(const T &type, double sample_time,
                             mgv_msgs::EigenTrajectoryPoint *state);

} // namespace mgv_trajectory_generation

#endif // mgv_TRAJECTORY_GENERATION_TRAJECTORY_SAMPLING_H_
