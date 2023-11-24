#ifndef MGV_PATH_SMOOTHING_LOCO_SMOOTHER_H_
#define MGV_PATH_SMOOTHING_LOCO_SMOOTHER_H_

#include <loco_planner/loco.h>

#include "mav_path_smoothing/polynomial_smoother_mgv.h"

namespace mgv_planning
{

  class LocoSmoother : public PolynomialSmoother
  {
  public:
    typedef std::function<double(const Eigen::Vector3d &position,
                                 Eigen::Vector3d *gradient)>
        DistanceAndGradientFunctionType;

    LocoSmoother();
    virtual ~LocoSmoother() {}

    virtual void setParametersFromRos(const ros::NodeHandle &nh);

    virtual bool getTrajectoryBetweenWaypoints(
        const mgv_msgs::EigenTrajectoryPoint::Vector &waypoints,
        mav_trajectory_generation::Trajectory *trajectory) const;

    // Special case for num_waypoints = 2 (splits trajectory into num_segments).
    // Done
    virtual bool getTrajectoryBetweenTwoPoints(
        const mgv_msgs::EigenTrajectoryPointMgv &start,
        const mgv_msgs::EigenTrajectoryPointMgv &goal,
        mgv_trajectory_generation::Trajectory *trajectory) const;

    // DONE
    virtual bool getPathBetweenTwoPoints(
        const mgv_msgs::EigenTrajectoryPointMgv &start,
        const mgv_msgs::EigenTrajectoryPointMgv &goal,
        mgv_msgs::EigenTrajectoryPointMgvVector *path) const;

    // Parameters...
    bool getResampleTrajectory() const { return resample_trajectory_; }
    void setResampleTrajectory(bool resample_trajectory)
    {
      resample_trajectory_ = resample_trajectory;
    }
    bool getResampleVisibility() const { return resample_visibility_; }
    void setResampleVisibility(bool resample_visibility)
    {
      resample_visibility_ = resample_visibility;
    }
    int getNumSegments() const { return num_segments_; }
    void setNumSegments(int num_segments) { num_segments_ = num_segments; }
    // Controls whether waypoints are added as soft costs in the LOCO problem.
    bool getAddWaypoints() const { return add_waypoints_; }
    void setAddWaypoints(bool add_waypoints) { add_waypoints_ = add_waypoints; }

    // Use a function to get gradient.
    void setDistanceAndGradientFunction(
        const DistanceAndGradientFunctionType &function)
    {
      distance_and_gradient_function_ = function;
    }

  protected:
    double getMapDistanceAndGradient(const Eigen::VectorXd &position,
                                     Eigen::VectorXd *gradient) const;

    bool resample_trajectory_;
    bool resample_visibility_;
    int num_segments_;
    bool add_waypoints_;
    bool scale_time_;

    DistanceAndGradientFunctionType distance_and_gradient_function_;
  };

} // namespace mgv_planning

#endif // mgv_PATH_SMOOTHING_LOCO_SMOOTHER_H_
