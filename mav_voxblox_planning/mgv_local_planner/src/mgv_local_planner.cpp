#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mgv_local_planner/mgv_local_planner.h"

namespace mgv_planning
{
  MgvLocalPlanner::MgvLocalPlanner(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        verbose_(false),
        global_frame_id_("map"),
        local_frame_id_("odom"),
        replan_dt_(1.0)
  {
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &MgvLocalPlanner::odometryCallback_mgv, this);

    waypoint_sub_ =
        nh_.subscribe("waypoint", 1, &MgvLocalPlanner::waypointCallback_mgv, this);

    ros::TimerOptions timer_options(
        ros::Duration(replan_dt_),
        boost::bind(&MgvLocalPlanner::planningTimerCallback_mgv, this, _1),
        &planning_queue_);

    planning_timer_ = nh_.createTimer(timer_options);

    // 启动异步线程
    command_publishing_spinner_.start();
    planning_spinner_.start();
  }

  void MgvLocalPlanner::odometryCallback_mgv(const nav_msgs::Odometry &msg)
  {
    // 消息类型需要更改
    mav_msgs::eigenOdometryFromMsg(msg, &odometryOFmgv_); // 将ros消息转换为eigen类型的变量
  }

  void MgvLocalPlanner::waypointCallback_mgv(const geometry_msgs::PoseStamped &msg)
  {
    clearTrajectoryOFmgv();

    // 消息类型需要更改
    mav_msgs::EigenTrajectoryPoint waypointsOFmgv;
    eigenTrajectoryPointFromPoseMsg(msg, &waypointsOFmgv);

    waypointsOFmgv_.clear();
    waypointsOFmgv_.push_back(waypointsOFmgv);

    current_waypointOFmgv_ = 0;

    planningStep_mgv();
    startPublishingCommands_mgv();
  }

  void MgvLocalPlanner::planningTimerCallback_mgv(const ros::TimerEvent &event)
  {
    if (should_replan_.wait_for(replan_dt_))
    {
      if (verbose_)
      {
        ROS_INFO("[Mgv Local Plnner][Plan Step] ")
      }
      planningStep_mgv();
    }
  }

  void MgvLocalPlanner::planningStep_mgv()
  {
  }

}
