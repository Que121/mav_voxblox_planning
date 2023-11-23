#ifndef MAV_PLANNING_COMMON_PATH_UTILS_H_
#define MAV_PLANNING_COMMON_PATH_UTILS_H_

#include "/home/patton/voxblox_ws/src/mav_voxblox_planning/mgv_comm/mgv_msgs/include/mgv_msgs/eigen_mgv_msgs.h"

namespace mgv_planning {

// Go through the entire trajectory and make the timings consistent (i.e.,
// monotonically increasing, always dt apart).
// Assumptions: consistent dt throughout the trajectory.
inline void retimeTrajectoryMonotonicallyIncreasing(
    mgv_msgs::EigenTrajectoryPointVector* trajectory) {
  if (trajectory->empty()) {
    return;
  }

  int64_t current_time_ns = (*trajectory)[0].time_from_start_ns;
  int64_t dt_ns = 0;
  for (size_t i = 1; i < trajectory->size(); ++i) {
    if (dt_ns <= 0) {
      dt_ns = (*trajectory)[i].time_from_start_ns - current_time_ns;
    }
    current_time_ns += dt_ns;
    if ((*trajectory)[i].time_from_start_ns != current_time_ns) {
      (*trajectory)[i].time_from_start_ns = current_time_ns;
    }
  }
}

inline void retimeTrajectoryWithStartTimeAndDt(
    int64_t start_time_ns, int64_t dt_ns,
    mgv_msgs::EigenTrajectoryPointVector* trajectory) {
  int64_t current_time_ns = start_time_ns;

  for (size_t i = 0; i < trajectory->size(); ++i) {
    (*trajectory)[i].time_from_start_ns = current_time_ns;
    current_time_ns += dt_ns;
  }
}

}  // namespace mgv_planning

#endif  // mgv_PLANNING_COMMON_PATH_UTILS_H_
