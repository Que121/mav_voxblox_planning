#ifndef mgv_PLANNING_COMMON_VISIBILITY_RESAMPLING_H_
#define mgv_PLANNING_COMMON_VISIBILITY_RESAMPLING_H_

#include <mgv_msgs/eigen_mgv_msgs.h>

#include "mav_planning_common/physical_constraints_mgv.h"


namespace mgv_planning {

void resampleWaypointsFromVisibilityGraph(
    int num_segments, const PhysicalConstraints& constraints,
    const mgv_msgs::EigenTrajectoryPoint::Vector& waypoints,
    mgv_msgs::EigenTrajectoryPoint::Vector* waypoints_out);

}  // namespace mgv_planning

#endif  // mgv_PLANNING_COMMON_VISIBILITY_RESAMPLING_H_
