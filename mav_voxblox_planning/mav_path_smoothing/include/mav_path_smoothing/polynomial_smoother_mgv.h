#ifndef MAV_PATH_SMOOTHING_POLYNOMIAL_SMOOTHER_H_
#define MAV_PATH_SMOOTHING_POLYNOMIAL_SMOOTHER_H_

#include <mav_trajectory_generation/trajectory_mgv.h>

#include <mav_path_smoothing/path_smoother_base_mgv.h>

namespace mgv_planning
{

  class PolynomialSmoother : public PathSmootherBase
  {
  public:
    PolynomialSmoother();
    virtual ~PolynomialSmoother() {}

    virtual void setParametersFromRos(const ros::NodeHandle &nh);

    virtual bool getTrajectoryBetweenWaypoints(
        const mgv_msgs::EigenTrajectoryPointMgv::Vector &waypoints,
        mgv_trajectory_generation::Trajectory *trajectory) const;

    virtual bool getPathBetweenWaypoints(
        const mgv_msgs::EigenTrajectoryPointMgv::Vector &waypoints,
        mgv_msgs::EigenTrajectoryPointMgv::Vector *path) const;

    virtual bool getPathBetweenTwoPoints(
        const mgv_msgs::EigenTrajectoryPointMgv &start,
        const mgv_msgs::EigenTrajectoryPointMgv &goal,
        mgv_msgs::EigenTrajectoryPointMgv::Vector *path) const;

    // Below are methods that are only needed if doing collision checking.
    typedef std::function<double(const Eigen::Vector3d &position)>
        MapDistanceFunctionType;
    typedef std::function<bool(const Eigen::Vector3d &position)>
        InCollisionFunctionType;

    // If using splitting at collisions, one of these needs to be set.
    // Map distance is compared against the radius in the physical constraints.
    void setMapDistanceCallback(const MapDistanceFunctionType &function)
    {
      map_distance_func_ = function;
    }
    // Function should return true if the position is in collision.
    void setInCollisionCallback(const InCollisionFunctionType &function)
    {
      in_collision_func_ = function;
    }

    // Uses whichever collision checking method is set to check for collisions.
    virtual bool isPositionInCollision(const Eigen::Vector3d &pos) const;
    // Try to do the minimum number of lookups in the map based on
    // min_col_check_resolution. Returns time of collision in t, if not null.
    virtual bool isPathInCollision(
        const mgv_msgs::EigenTrajectoryPointMgv::Vector &path, double *t) const;

    // Parameters.
    bool getOptimizeTime() const { return optimize_time_; }
    void setOptimizeTime(bool optimize_time) { optimize_time_ = optimize_time; }
    bool getSplitAtCollisions() const { return split_at_collisions_; }
    void setSplitAtCollisions(bool split_at_collisions)
    {
      split_at_collisions_ = split_at_collisions;
    }
    double getMinCollisionCheckResolution() const
    {
      return min_col_check_resolution_;
    }
    void setMinCollisionCheckResolution(double min_col_check_resolution)
    {
      min_col_check_resolution_ = min_col_check_resolution;
    }

  protected:
    // Add intermediate vertex for splitting.
    bool addVertex(double t,
                   const mgv_trajectory_generation::Trajectory &trajectory,
                   mgv_trajectory_generation::Vertex::Vector *vertices,
                   std::vector<double> *segment_times) const;

    // Figure out what kind of polynomial smoothing to do...

    // Wether to optimize the segment times to better meet the dynamic
    // constraints.
    bool optimize_time_;

    // Whether to add new vertices on the straight-line path if collisions with
    // the map are found.
    bool split_at_collisions_;

    // Minimum distance between collision checks.
    double min_col_check_resolution_;

    // Functions for collision checking.
    MapDistanceFunctionType map_distance_func_;
    InCollisionFunctionType in_collision_func_;
  };

} // namespace mgv_planning

#endif // mgv_PATH_SMOOTHING_POLYNOMIAL_SMOOTHER_H_
