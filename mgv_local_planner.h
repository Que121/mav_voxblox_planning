#ifndef MGV_LOCAL_PLANNER_MGV_LOCAL_PLANNER_H_
#define MGV_LOCAL_PLANNER_MGV_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_path_smoothing/polynomial_smoother.h>
#include <mav_path_smoothing/velocity_ramp_smoother.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_common/semaphore_mgv.h>
#include <mav_planning_common/yaw_policy.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>

namespace mgv_planning
{
  class MgvLocalPlanner
  {
  private:
    void startPublishingCommands_mgv();
    //void commandPublishTimerCallback_mgv();

    void planningTimerCallback_mgv(const ros::TimerEvent &event);
    void planningStep_mgv();

    bool planPathThroughWaypoints_mgv(
        const mav_msgs::EigenTrajectoryPointVector &waypointsOFmgv,
        mav_msgs::EigenTrajectoryPointVector *pathOFmgv);

    void replacePath_mgv(const mav_msgs::EigenTrajectoryPointVector &pathOFmgv);

    void commandPublishTimerCallback_mgv(const ros::TimerEvent &event);

    bool plan_to_startOFmgv_;
    bool avoid_collisions_;
    std::string smoother_name_;

    ros::ServiceClient position_hold_client_;

    int mpc_prediction_horizonOFmgv_;

    YawPolicy yaw_policy_;
    RosSemaphore should_replan_;

    RosSemaphore should_replan_;
    std::recursive_mutex path_mutexOFmgv_;
    std::recursive_mutex map_mutexOFmgv_;

    int mpc_prediction_horizonOFmgv_

  public:
  
    MgvLocalPlanner(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private);

    // input
    void odometryCallback_mgv(const nav_msgs::Odometry &msg);         // 机器人位置信息
    void waypointCallback_mgv(const geometry_msgs::PoseStamped &msg); // 单个路径点信息

    bool nextWaypoint();
    void finishWaypointsOFmgv();

    void clearTrajectory_mgv();

    void abort();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber waypoint_sub_; // sub路径点信息
    ros::Subscriber odometry_sub_; // sub机器人位置信息

    ros::Publisher command_pub_;

    ros::CallbackQueue command_publishing_queue_;
    ros::AsyncSpinner command_publishing_spinner_;
    ros::CallbackQueue planning_queue_;
    ros::AsyncSpinner planning_spinner_;

    ros::Timer command_publishing_timer_;
    ros::Timer planning_timer_;

    bool verbose_;
    double command_publishing_dt_;
    double replan_dt_;
    double replan_lookahead_sec_;

    std::string global_frame_id_;
    std::string local_frame_id_;

    RosSemaphore should_replan_;

    mav_msgs::EigenTrajectoryPointVector waypointsOFmgv_;
    int64_t current_waypointOFmgv_;

    mav_msgs::EigenTrajectoryPointVector path_queueOFmgv_;
    size_t path_indexOFmgv_;

    mav_msgs::EigenOdometry odometryOFmgv_;
  };
}

#endif // MGV_LOCAL_PLANNER_MGV_LOCAL_PLANNER_H_
