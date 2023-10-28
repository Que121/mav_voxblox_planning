#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_local_planner/mav_local_planner.h"

namespace mav_planning
{

  MavLocalPlanner::MavLocalPlanner(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        command_publishing_spinner_(1, &command_publishing_queue_),
        planning_spinner_(1, &planning_queue_),
        verbose_(false),
        global_frame_id_("map"),
        local_frame_id_("odom"),
        mpc_prediction_horizon_(300),
        command_publishing_dt_(1.0),
        replan_dt_(1.0),
        replan_lookahead_sec_(0.1),
        avoid_collisions_(true), // 是否避障
        autostart_(true),
        plan_to_start_(true),
        smoother_name_("loco"), // 路径平滑器的算法
        current_waypoint_(-1),
        path_index_(0),
        max_failures_(5),
        num_failures_(0),
        esdf_server_(nh_, nh_private_),
        loco_planner_(nh_, nh_private_)
  {
    // Set up some settings.
    constraints_.setParametersFromRos(nh_private_);
    esdf_server_.setTraversabilityRadius(constraints_.robot_radius);
    loco_planner_.setEsdfMap(esdf_server_.getEsdfMapPtr());
    goal_selector_.setParametersFromRos(nh_private_);
    goal_selector_.setTsdfMap(esdf_server_.getTsdfMapPtr());

    nh_private_.param("verbose", verbose_, verbose_);
    nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
    nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
    nh_private_.param("mpc_prediction_horizon", mpc_prediction_horizon_,
                      mpc_prediction_horizon_);
    nh_private_.param("replan_dt", replan_dt_, replan_dt_);
    nh_private_.param("replan_lookahead_sec", replan_lookahead_sec_,
                      replan_lookahead_sec_);
    nh_private_.param("command_publishing_dt", command_publishing_dt_,
                      command_publishing_dt_);
    nh_private_.param("avoid_collisions", avoid_collisions_, avoid_collisions_);
    nh_private_.param("autostart", autostart_, autostart_);
    nh_private_.param("plan_to_start", plan_to_start_, plan_to_start_);
    nh_private_.param("smoother_name", smoother_name_, smoother_name_);

    // Publishers and subscribers.
    // 机器人位置信息
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &MavLocalPlanner::odometryCallback, this);
    // 单个路径点信息
    waypoint_sub_ =
        nh_.subscribe("waypoint", 1, &MavLocalPlanner::waypointCallback, this);

    // 路径表信息
    waypoint_list_sub_ = nh_.subscribe(
        "waypoint_list", 1, &MavLocalPlanner::waypointListCallback, this);

    // 发布运动轨迹信息
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

    // 发布生成路径（可视化）
    path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
        "local_path", 1, true);

    // 完整的路径轨迹信息
    full_trajectory_pub_ =
        nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            "full_trajectory", 1, true);

    // Services. 用于接收外部命令、以启动、暂停或停止路径规划器
    start_srv_ = nh_private_.advertiseService(
        "start", &MavLocalPlanner::startCallback, this);

    pause_srv_ = nh_private_.advertiseService(
        "pause", &MavLocalPlanner::pauseCallback, this);

    stop_srv_ = nh_private_.advertiseService(
        "stop", &MavLocalPlanner::stopCallback, this);

    position_hold_client_ =
        nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

    // Start the planning timer. Will no-op most cycles.
    // 创造定时器，定时执行plannningTimerCallback函数
    ros::TimerOptions timer_options(
        ros::Duration(replan_dt_),
        boost::bind(&MavLocalPlanner::planningTimerCallback, this, _1),
        &planning_queue_);

    // 初始化空服务用户端
    planning_timer_ = nh_.createTimer(timer_options);

    // Start the command publishing spinner.
    //  启动两个异步消息处理线程
    command_publishing_spinner_.start(); // 发送控制
    planning_spinner_.start();           // 处理路径规划和规划器相关信息

    // Set up yaw policy.
    // 设置偏航策略
    yaw_policy_.setPhysicalConstraints(constraints_);                 // 设置物理约束策略
    yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kVelocityVector); // 设置偏航策略

    // Set up smoothers.
    const double voxel_size = esdf_server_.getEsdfMapPtr()->voxel_size();
    // 获取体素的大小，用于设置smoother的分辨率

    // Straight-line smoother.
    ramp_smoother_.setParametersFromRos(nh_private_); // 从ros参数服务器中获取参数用于ramp

    // Poly smoother.
    poly_smoother_.setParametersFromRos(nh_private_);
    poly_smoother_.setMinCollisionCheckResolution(voxel_size); // 设置最小碰撞检查分辨率
    poly_smoother_.setMapDistanceCallback(
        std::bind(&MavLocalPlanner::getMapDistance, this, std::placeholders::_1)); // 获取指定点到最近障碍物的距离
    poly_smoother_.setOptimizeTime(true);                                          // 启用poly时间优化
    poly_smoother_.setSplitAtCollisions(avoid_collisions_);                        // 遇到碰撞是否分割轨迹以避开障碍物

    // Loco smoother!
    loco_smoother_.setParametersFromRos(nh_private_);
    loco_smoother_.setMinCollisionCheckResolution(voxel_size); // 设置最小碰撞检查分辨率
    loco_smoother_.setDistanceAndGradientFunction(
        std::bind(&MavLocalPlanner::getMapDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2));
    // 地图距离和梯度回调函数，该函数用于获取指定点到最近障碍物的距离和梯度。
    loco_smoother_.setOptimizeTime(true);       // 时间优化
    loco_smoother_.setResampleTrajectory(true); // 轨迹重采样
    loco_smoother_.setResampleVisibility(true); // 可见性采样
    loco_smoother_.setNumSegments(5);           // 轨迹分段处理的段数，影响光滑度和曲率
  }

  void MavLocalPlanner::odometryCallback(const nav_msgs::Odometry &msg)
  {
    mav_msgs::eigenOdometryFromMsg(msg, &odometry_); // 将ros消息转换为eigen类型的变量
  }

  // sub接受到一个新的waypont时调用
  void MavLocalPlanner::waypointCallback(const geometry_msgs::PoseStamped &msg)
  {
    // Plan a path from the current position to the target pose stamped.
    ROS_INFO("[Mav Local Planner] Got a waypoint!");
    // Cancel any previous trajectory on getting a new one.
    // 取消之前的轨迹规划，准备接收新的路径点
    clearTrajectory();

    // 获取新的位置点信息
    mav_msgs::EigenTrajectoryPoint waypoint;         // receive target pose information
    eigenTrajectoryPointFromPoseMsg(msg, &waypoint); // tranform pose information to Eigen

    // 清空之前的路径点，将新的目标点加入路径列表
    waypoints_.clear();
    waypoints_.push_back(waypoint); // push the new waypoint

    // 重置当前路径点索引
    current_waypoint_ = 0; // refresh index

    // Execute one planning step on main thread.
    // 执行一次路径规划
    planningStep();

    // 发布飞行指令
    startPublishingCommands();
  }

  // sub接受到一个新的waypointlist时调用
  void MavLocalPlanner::waypointListCallback(
      const geometry_msgs::PoseArray &msg)
  {
    // Plan a path from the current position to the target pose stamped.
    ROS_INFO("[Mav Local Planner] Got a list of waypoints, %zu long!",
             msg.poses.size());
    // Cancel any previous trajectory on getting a new one.
    // 取消之前的路径规划，准备接受新的路径点
    clearTrajectory();

    // 清空之前的路径点列表
    waypoints_.clear();

    // 遍历路径点消息，转换类型后加入路径点列表
    for (const geometry_msgs::Pose &pose : msg.poses)
    {
      mav_msgs::EigenTrajectoryPoint waypoint;
      eigenTrajectoryPointFromPoseMsg(pose, &waypoint);
      waypoints_.push_back(waypoint);
    }

    // 重置路径索引
    current_waypoint_ = 0;

    // Execute one planning step on main thread.
    // 执行一次路径规划
    planningStep();

    // 发布指令
    startPublishingCommands();
  }

  // 由ROS定时器触发的回调函数， 用于执行飞行规划
  void MavLocalPlanner::planningTimerCallback(const ros::TimerEvent &event)
  {
    // Wait on the condition variable from the publishing...
    // 等待发布器条件变量
    if (should_replan_.wait_for(replan_dt_))
    {
      // 是否详细输出，输出实际和预期时间，以及差异
      if (verbose_)
      {
        ROS_WARN(
            "[Mav Planning Timer] Difference between real and expected: %f Real: "
            "%f Expected: %f Now: %f",
            (event.current_real - event.current_expected).toSec(),
            event.current_real.toSec(), event.current_expected.toSec(), // real time and expected time
            ros::Time::now().toSec());
      }
      // 执行一次飞行路径规划
      planningStep();
    }
  }

  // 根据当前的路径队列和机器人状态，决定是直接规划到路径终点，还是进行避碰规划
  void MavLocalPlanner::planningStep()
  {
    // 输出当前处理的路径点索引和总路径点数。如果已经到达或超出路径的终点，且路径队列为空，函数将直接返回。
    // 这种情况通常出现在已规划至路径终点并且没有新的路径点需要添加时。
    ROS_INFO(
        "[Mav Local Planner][Plan Step] Waypoint index: %zd Total waypoints: %zu",
        current_waypoint_, waypoints_.size());

    // 路径点队列为空或index超出范围
    if (current_waypoint_ < 0 ||
        static_cast<int>(waypoints_.size()) <= current_waypoint_)
    {
      // This means that we probably planned to the end of the waypoints!

      // If we're done with sending waypoints, alllll good. Just quit.
      // 已经规划至路径终点并且没有新的路径点需要添加，直接返回
      if (path_index_ >= path_queue_.size() || path_queue_.empty())
      {
        return;
      }

      // If we're not though, we should probably double check the trajectory!
    }

    // 计时器记录规划的时间
    mav_trajectory_generation::timing::MiniTimer timer;

    // 设定一个距离阈值
    constexpr double kCloseToOdometry = 0.1;

    // First, easiest case: if we're not avoiding collisions, just use the
    // favorite path smoother. We only do this on the first planning call then
    // ignore all the rest.
    // 不需要规避障碍
    if (!avoid_collisions_)
    {
      // 如果计划从起点开始
      // std::vector<mav_msgs::EigenTrajectoryPoint> waypoints;
      mav_msgs::EigenTrajectoryPointVector waypoints;
      mav_msgs::EigenTrajectoryPoint current_point;
      current_point.position_W = odometry_.position_W;
      current_point.orientation_W_B = odometry_.orientation_W_B;

      if (plan_to_start_)
      {
        waypoints.push_back(current_point);
      }
      waypoints.insert(waypoints.end(), waypoints_.begin(), waypoints_.end());

      mav_msgs::EigenTrajectoryPointVector path;

      // 直接规划当前位置到路径终点的平滑路径
      if (planPathThroughWaypoints(waypoints, &path))
      {
        replacePath(path);
        current_waypoint_ = waypoints_.size();
      }
      else
      {
        ROS_ERROR("[Mav Local Planner] Waypoint planning failed!");
      }
    }

    // 如果路径队列为空
    else if (path_queue_.empty())
    {
      // First check how many waypoints we haven't covered yet are in free space.
      // 首先检查还有多少路径点没有被覆盖在自由空间中
      mav_msgs::EigenTrajectoryPointVector free_waypoints;
      // Do we need the odometry in here? Let's see.
      mav_msgs::EigenTrajectoryPoint current_point;
      current_point.position_W = odometry_.position_W;
      current_point.orientation_W_B = odometry_.orientation_W_B;

      // If the path doesn't ALREADY start near the odometry, the first waypoint
      // should be the current pose.
      // 如果路径点距离并不非常接近当前位置，第一个路径点应该是当前位置
      int waypoints_added = 0;
      if (plan_to_start_ &&
          (current_point.position_W - waypoints_.front().position_W).norm() >
              kCloseToOdometry)
      {
        free_waypoints.push_back(current_point);
        waypoints_added = 1;
      }

      // 将自由空间中的路径点加入free_waypoints中
      for (const mav_msgs::EigenTrajectoryPoint &waypoint : waypoints_)
      {
        if (getMapDistance(waypoint.position_W) < constraints_.robot_radius)
        {
          break;
        }
        free_waypoints.push_back(waypoint);
      }

      ROS_INFO("[Mav Local Planner] Of %zu waypoints, %zu are free.",
               waypoints_.size(), free_waypoints.size());
      bool success = false;

      // 如果free_waypoints只包含一个或两个路径点
      if (free_waypoints.size() <= static_cast<size_t>(waypoints_added) ||
          free_waypoints.size() == 2)
      {
        // Okay whatever just search for the first waypoint.
        // 只搜索第一个
        success = false;
      }
      else
      {
        // There is some hope! Maybe we can do path smoothing on these guys.
        // 尝试在这些自由空间的路径点上进行路径平滑
        mav_msgs::EigenTrajectoryPointVector path;
        // 路径规划function
        success = planPathThroughWaypoints(free_waypoints, &path);
        if (success)
        {
          // 成功规划出路径
          ROS_INFO(
              "[Mav Local Planner]  Successfully planned path through %zu free "
              "waypoints.",
              free_waypoints.size());

          // 检查规划出的路径是否与障碍物相交
          success = isPathCollisionFree(path);
          if (success)
          {
            // 如果规划出的path无碰撞，替换为当前path
            replacePath(path);

            // 更新当前路径点索引，确保飞行器继续朝前飞
            current_waypoint_ = std::min(free_waypoints.size() - waypoints_added,
                                         waypoints_.size() - 1);
            ROS_INFO(
                "[Mav Local Planner] Used smoothing through %zu waypoints! Total "
                "waypoint size: %zu, current point: %zd, added? %d",
                free_waypoints.size(), waypoints_.size(), current_waypoint_,
                waypoints_added);
          }
          else
          {
            // 如果规划出的路径与障碍物相交，输出警告
            ROS_WARN("[Mav Local Planner] But path was not collision free. :(");
          }
        }
      }
      // Give up!
      // 如果路径规划不成功，或者无法
      if (!success)
      {
        // 执行避障操作，尝试规划一条避开障碍物的新路径
        avoidCollisionsTowardWaypoint();
      }
    }

    // 否则继续探索
    else
    {
      // Otherwise let's just keep exploring.
      // 否则继续探索
      // 避免路径规划陷入死锁，继续尝试规划新路径
      avoidCollisionsTowardWaypoint();
    }

    // 输出规划完成，包括所花费的时间
    ROS_INFO("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f",
             timer.stop());

    // 可视化路径
    visualizePath();
  }

  // 尝试规划一条避开障碍物的到达目标点的path
  void MavLocalPlanner::avoidCollisionsTowardWaypoint()
  {
    // 检查当前路径点是否超过路径点总数
    if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()))
    {
      return;
    }

    // 从路径表中获取当前目标点的信息
    mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];

    // 定义一个距离阈值，表示无人机在多远就认为达到目标点
    const double kCloseEnough = 0.05;

    // 转换规划的时间间隔
    const int64_t kDtNs =
        mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

    // 打印当前的位置、目标点位置、路径点index
    ROS_INFO_STREAM("[Mav Local Planner][Plan Step] Current odometry: "
                    << odometry_.position_W.transpose() << " Tracking waypoint ["
                    << current_waypoint_
                    << "]: " << waypoint.position_W.transpose());

    // Save success and Trajectory.
    mav_trajectory_generation::Trajectory trajectory;
    bool success = false;

    // 路径队列不为空 the path queue is not empty
    if (!path_queue_.empty())
    {
      std::lock_guard<std::recursive_mutex> guard(path_mutex_); // lock path

      ROS_INFO(
          "[Mav Local Planner][Plan Step] Trying to replan on existing path.");
      mav_msgs::EigenTrajectoryPointVector path_chunk;
      // 选择一个起始点，以便在这个点之后的轨迹可以被重新规划，
      // 而不必重新规划整个路径。这种方法可以提高规划效率，因为无需从头开始规划整个路径，
      // 而只需重新规划一部分路径。
      size_t replan_start_index;
      {
        // 确定从路径队列的哪个位置开始重新规划路径，当前的路径队列的索引加上一个对应时间
        replan_start_index =
            std::min(path_index_ + static_cast<size_t>((replan_lookahead_sec_) /
                                                       constraints_.sampling_dt),
                     path_queue_.size());

        ROS_INFO(
            "[Mav Local Planner][Plan Step] Current path index: %zu Replan start "
            "index: %zu",
            path_index_, replan_start_index);
        // Cut out the remaining snippet of the trajectory so we can do
        // something with it.

        // 将剩余路径copy至path_chunk
        std::copy(path_queue_.begin() + replan_start_index, path_queue_.end(),
                  std::back_inserter(path_chunk));

        if (path_chunk.size() == 0) // 如果没有copy任何轨迹点
        {
          path_chunk.push_back(path_queue_.back()); // 包含最后一个点
          if (!nextWaypoint())                      // 获取下一个目标点
          {
            finishWaypoints(); // 无目标点完成规划
          }
        }
      }

      // 检查path_chunk是否是一个避免碰撞的path
      bool path_chunk_collision_free = isPathCollisionFree(path_chunk);
      ROS_INFO(
          "[Mav Local Planner][Plan Step] Existing chunk is collision free? %d",
          path_chunk_collision_free);

      // Check if the current path queue goes to the goal and is collision free.
      // 检查是否接近目标点 check it is whether close with waypoint
      if ((path_chunk.back().position_W - waypoint.position_W).norm() <
          kCloseEnough)
      {
        // Collision check the remaining chunk of the trajectory.、
        // 检查是否碰撞自由 check the freedom of collision
        if (path_chunk_collision_free)
        {
          ROS_INFO(
              "[Mav Local Planner][Plan Step] Current plan is valid, just "
              "rollin' with it.");

          // 路径有效 寻找下一个目标点 the path is effective and find the next one waypoint
          nextWaypoint();
          return;
        }
      }
      // Otherwise we gotta replan this thing anyway.
      // 重新规划路径 replan this path bacause the path_chunk is unsatisfy on close_waypoint or collision_free
      // 生成从path_chunk起始点到waypoint的新路径 creat the new path from the begin of 'path_chunk' to 'waypoint', and save it in 'trajectory'
      success = loco_planner_.getTrajectoryTowardGoal(path_chunk.front(),
                                                      waypoint, &trajectory);

      // 新路径规划失败  being fail to replan new path
      if (!success)
      {
        if (path_chunk_collision_free) // 再次检查是否碰撞自由 check the freedom of collision again
        {
          ROS_INFO(
              "[Mav Local Planner][Plan Step] Couldn't find a solution :( "
              "Continuing existing solution."); // ???
        }
        else
        {
          ROS_INFO(
              "[Mav Local Planner][Plan Step] ABORTING! No local solution "
              "found.");
          // TODO(helenol): which order to abort in?
          abort();
          dealWithFailure(); // 处理路径规划失败的情况  handle the path planning failure
        }
        return;
      }

      // 新路径规划成功 being sucessful to replan new path
      else
      {
        ROS_INFO("[Mav Local Planner][Plan Step] Appending new path chunk.");

        // 检查生成路径的最大时间是否非常小，说明路径短
        // check whether the maximum time for generating a path is very small, indicating that the path is sooooo short
        if (trajectory.getMaxTime() <= 1e-6)
        {
          nextWaypoint(); // 直接获取下一个目标点  thus get the next waypoint directly
        }

        // Otherwise 否则
        else
        {
          num_failures_ = 0;
          mav_msgs::EigenTrajectoryPointVector new_path_chunk;
          mav_trajectory_generation::sampleWholeTrajectory(
              trajectory, constraints_.sampling_dt, &new_path_chunk);
          //  函数将 trajectory 采样成一系列点

          // 进行时间重映射（retiming）和姿态调整
          retimeTrajectoryWithStartTimeAndDt(
              path_chunk.front().time_from_start_ns, kDtNs, &new_path_chunk);
          // 最终得到 new_path_chunk
          new_path_chunk.front().orientation_W_B =
              path_chunk.front().orientation_W_B;
          yaw_policy_.applyPolicyInPlace(&new_path_chunk); // ?

          // Remove what was in the trajectory before. 原有路径队列中从 replan_start_index 之后的轨迹删除
          if (replan_start_index < path_queue_.size())
          {
            path_queue_.erase(path_queue_.begin() + replan_start_index,
                              path_queue_.end());
          }
          // Stick the new one in. 将 new_path_chunk 加入路径队列
          path_queue_.insert(path_queue_.end(), new_path_chunk.begin(),
                             new_path_chunk.end());
        }
      }
    }

    // 路径队列为空 the path queue is empty
    // 初始规划或规划过程出现严重错误 a serious error in the initial planning or planning process
    else
    {
      ROS_INFO("[Mav Local Planner][Plan Step] Trying to plan from scratch.");

      // There's nothing planned so far! So we plan from the current odometry.
      mav_msgs::EigenTrajectoryPoint current_point;
      current_point.position_W = odometry_.position_W;
      current_point.orientation_W_B = odometry_.orientation_W_B;

      // Check if the current waypoint is basically the odometry.
      if ((current_point.position_W - waypoint.position_W).norm() <
          kCloseEnough) // 检查是否接近目标点 check it is whether close with waypoint
      {
        if (nextWaypoint())
        {
          waypoint = waypoints_[current_waypoint_]; // 更新目标点  refresh waypoint
        }
        else
        {
          return;
        }
      }

      success = loco_planner_.getTrajectoryTowardGoal(current_point, waypoint,
                                                      &trajectory);
      // 尝试从'current_point'规划到达目标点'waypoint'的路径
      ROS_INFO("[Mav Local Planner][Plan Step] Planning success? %d", success);

      if (success) // 规划成功 it is successful to plan
      {
        if (trajectory.getMaxTime() <= 0.1)
        {
          nextWaypoint();
        }
        else
        {
          //  采样成一系列点，然后调用 replacePath() 函数，将这些采样点替换掉路径队列中的内容。这样，无人机就有了一个从当前位置到达目标点的规划路径。
          // Sample a series of points and then call the replacePath() function to replace the contents of the path queue with those points
          // In this way, the drone has a planned path from its current location to the target point
          // Copy this straight into the queue.
          num_failures_ = 0;
          mav_msgs::EigenTrajectoryPointVector path;
          mav_trajectory_generation::sampleWholeTrajectory(
              trajectory, constraints_.sampling_dt, &path);
          replacePath(path);
        }
      }
      else // 规划失败 it is aborted to plan
      {
        dealWithFailure();
      }
    }
  }

  // 根据给定的路径点waypoints使用不同的路径平滑算法规划一条平滑的路径
  bool MavLocalPlanner::planPathThroughWaypoints(
      const mav_msgs::EigenTrajectoryPointVector &waypoints,
      mav_msgs::EigenTrajectoryPointVector *path)
  {
    CHECK_NOTNULL(path);          // 确保传入的 'path' 不为空
    bool success = false;         // 初始化success变量
    if (smoother_name_ == "loco") // 选择的算法为loco
    {
      if (waypoints.size() == 2) // 如果路径点数量为2
      {
        success = loco_smoother_.getPathBetweenTwoPoints(waypoints[0],
                                                         waypoints[1], path); // 规划两点之间的路径
      }
      else // 路径点数量大于2
      { 
        success = loco_smoother_.getPathBetweenWaypoints(waypoints, path); // 规划多点之间的路径
      }
    }
    else if (smoother_name_ == "polynomial") // 选择polynomial算法
    {
      success = poly_smoother_.getPathBetweenWaypoints(waypoints, path); // 多项式插值法规划路径
    }
    else if (smoother_name_ == "ramp") // 选择ramp算法
    {
      success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path); // 均匀速度规划路径
    }
    else
    {
      // Default case is ramp!
      ROS_ERROR(
          "[Mav Local Planner] Unknown smoother type %s, using ramp instead.",
          smoother_name_.c_str());
      success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
    return success;
  }

  // 在规划路径上移动到下一个路径点
  bool MavLocalPlanner::nextWaypoint()
  {
    if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()) - 1) // 到达最后一个点
    {
      current_waypoint_ = waypoints_.size() - 1;
      return false;
    }
    else
    {
      current_waypoint_++; // index增加1， 移动到下一个路径点
      return true;
    }
  }

  // 设置到达路径的终点
  void MavLocalPlanner::finishWaypoints()

  {
    current_waypoint_ = waypoints_.size();
  }

  // 用于替换路径队列中的内容为传入的新路径 path
  void MavLocalPlanner::replacePath(
      const mav_msgs::EigenTrajectoryPointVector &path)
  {
    // 锁定队列线程
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    // 清空路径队列
    path_queue_.clear();

    // 将新队列赋值给队列路径
    path_queue_ = path;

    // 调整新路径的起始姿态
    path_queue_.front().orientation_W_B = odometry_.orientation_W_B;

    // 应用姿态策略
    yaw_policy_.applyPolicyInPlace(&path_queue_);

    // 重置路径index
    path_index_ = 0;
  }

  // 用于启动无人机的命令发布功能，并设置了一个定时器，在定时器回调函数中发布无人机的控制命令
  void MavLocalPlanner::startPublishingCommands()
  {
    // Call the service call to takeover publishing commands.
    // 接管命令发布服务
    if (position_hold_client_.exists())
    {
      std_srvs::Empty empty_call;
      position_hold_client_.call(empty_call); // 向无人机发送一个空的服务请求，用于接管命令的发布
    }

    // Publish the first set immediately, on this thread.
    // 立刻发布第一组命令
    commandPublishTimerCallback(ros::TimerEvent());

    // Need advanced timer options to assign callback queue to this timer.
    // 设定定时器并关联回调函数
    ros::TimerOptions timer_options(
        ros::Duration(command_publishing_dt_),
        boost::bind(&MavLocalPlanner::commandPublishTimerCallback, this, _1),
        &command_publishing_queue_);

    // 创建定时器
    command_publishing_timer_ = nh_.createTimer(timer_options);
  }

  // 定时器回调函数，用于在规划的路径上按照一定频率发布控制命令
  void MavLocalPlanner::commandPublishTimerCallback(
      const ros::TimerEvent &event)
  {
    constexpr size_t kQueueBuffer = 0; // 表示路径队列的缓冲区大小

    // 如果路径索引小于路径队列的大小，即还有未到达的路径点
    if (path_index_ < path_queue_.size())
    {
      // 锁定线程
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);

      // 计算发布的路径点数量
      // 取决于发布的时间间隔和剩余未到达的路径点数量
      size_t number_to_publish = std::min<size_t>(
          std::floor(command_publishing_dt_ / constraints_.sampling_dt),
          path_queue_.size() - path_index_);

      // 设置路径队列的索引顺序
      size_t starting_index = 0;
      if (path_index_ != 0)
      {
        starting_index = path_index_ + kQueueBuffer;
        // 如果path_index_不为0，加上KQueueBuffer防止在边界上选择路径点出现问题
        if (starting_index >= path_queue_.size())
        {
          starting_index = path_index_;
        }
      }

      size_t number_to_publish_with_buffer = std::min<size_t>(
          number_to_publish + mpc_prediction_horizon_ - kQueueBuffer,
          path_queue_.size() - starting_index);

      // TODO(helenol): do this without copy! Use iterators properly!
      // 提取要发布的路径点
      mav_msgs::EigenTrajectoryPointVector::const_iterator first_sample =
          path_queue_.begin() + starting_index;
      mav_msgs::EigenTrajectoryPointVector::const_iterator last_sample =
          first_sample + number_to_publish_with_buffer;
      mav_msgs::EigenTrajectoryPointVector trajectory_to_publish(first_sample,
                                                                 last_sample);

      // 创建和发布控制指令信息
      trajectory_msgs::MultiDOFJointTrajectory msg; // 创建一个多自由度关节轨迹消息，用于存储控制指令
      msg.header.frame_id = local_frame_id_;
      msg.header.stamp = ros::Time::now();

      // 打印发布的控制信息
      ROS_INFO(
          "[Mav Local Planner][Command Publish] Publishing %zu samples of %zu. "
          "Start index: %zu Time: %f Start position: %f Start velocity: %f End "
          "time: %f End position: %f",
          trajectory_to_publish.size(), path_queue_.size(), starting_index,
          trajectory_to_publish.front().time_from_start_ns * 1.0e-9,
          trajectory_to_publish.front().position_W.x(),
          trajectory_to_publish.front().velocity_W.x(),
          trajectory_to_publish.back().time_from_start_ns * 1.0e-9,
          trajectory_to_publish.back().position_W.x());

      // 将 trajectory_to_publish 中的路径点转换为 ROS 的控制指令消息格式。
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_to_publish, &msg);

      command_pub_.publish(msg);
      // 更新路径索引并通知重新规划
      path_index_ += number_to_publish;
      should_replan_.notify();
    }
    // Does there need to be an else????
  }

  void MavLocalPlanner::abort()
  {
    // No need to check anything on stop, just clear all the paths.
    clearTrajectory();
    // Make sure to clear the queue in the controller as well (we send about a
    // second of trajectories ahead).
    sendCurrentPose();
  }

  void MavLocalPlanner::clearTrajectory()
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    command_publishing_timer_.stop();
    path_queue_.clear();
    path_index_ = 0;
  }

  void MavLocalPlanner::sendCurrentPose()
  {
    // Sends the current pose with velocity 0 to the controller to clear the
    // controller's trajectory queue.
    // More or less an abort operation.
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    trajectory_msgs::MultiDOFJointTrajectory msg;
    msgMultiDofJointTrajectoryFromEigen(current_point, &msg);
    command_pub_.publish(msg);
  }

  bool MavLocalPlanner::startCallback(std_srvs::Empty::Request &request,
                                      std_srvs::Empty::Response &response)
  {
    if (path_queue_.size() <= path_index_)
    {
      ROS_WARN("Trying to start an empty or finished trajectory queue!");
      return false;
    }
    startPublishingCommands();
    return true;
  }

  bool MavLocalPlanner::pauseCallback(std_srvs::Empty::Request &request,
                                      std_srvs::Empty::Response &response)
  {
    if (path_queue_.size() <= path_index_)
    {
      ROS_WARN("Trying to pause an empty or finished trajectory queue!");
      return false;
    }
    command_publishing_timer_.stop();
    return true;
  }

  bool MavLocalPlanner::stopCallback(std_srvs::Empty::Request &request,
                                     std_srvs::Empty::Response &response)
  {
    abort();
    return true;
  }

  // 函数将路径数据可视化为一个带有黑色线条的标记，并将其发布到 ROS 话题
  void MavLocalPlanner::visualizePath()
  {
    // TODO: Split trajectory into two chunks: before and after.
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker path_marker;
    {
      // 对路径数据加锁
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);

      // 创建路径标记 （路径数据、局部坐标系标识、路径标记的颜色、路径标记的名称、路径标记宽度）
      path_marker = createMarkerForPath(path_queue_, local_frame_id_,
                                        mav_visualization::Color::Black(),
                                        "local_path", 0.05);
    }

    // 将创建的路径标记 'path_marker' 添加到 'marker_array'
    marker_array.markers.push_back(path_marker);

    // 使用 'path_marker_pub_' 发布 'marker_array'
    path_marker_pub_.publish(marker_array);
  }

  // 通过查询地图，获取给定位置处的地图距离
  double MavLocalPlanner::getMapDistance(const Eigen::Vector3d &position) const
  {

    // 表示欲查询地图距离的位置
    double distance = 0.0;

    // 表示是否在两个地图格子之间插值
    const bool kInterpolate = false;

    // 获取给定位置的地图距离 （欲查询的位置、是否插值、存储地图距离的变量）
    if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
            position, kInterpolate, &distance))
    {
      // 获取失败、返回 '0.0'
      return 0.0;
    }

    // 返回得到的距离值
    return distance;
  }

  // 获取给定位置处的梯度和地图距离
  double MavLocalPlanner::getMapDistanceAndGradient(
      const Eigen::Vector3d &position, Eigen::Vector3d *gradient) const
  {
    // 存储地图距离的outcome
    double distance = 0.0;

    // 是否插值
    const bool kInterpolate = false;

    // 获取地图距离和梯度
    if (!esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
            position, kInterpolate, &distance, gradient))

    // 获取失败返回0.0
    {
      return 0.0;
    }
    // 成功获取，返回distance and gradient
    return distance;
  }

  // 检查路径上的点有没有碰撞
  // ensure that all points along the path are located within collision-safe areas in the map
  bool MavLocalPlanner::isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector &path) const
  {
    // 遍历待检查的路径，其中包含一系列的点
    for (const mav_msgs::EigenTrajectoryPoint &point : path)
    {

      // use getMapDistance to obtain the distance of the point in the map
      // it checks if this distance is less than radius - 0.1
      // why '-0.1' ??????
      if (getMapDistance(point.position_W) < constraints_.robot_radius - 0.1)
      {
        return false;
      }
    }

    // collision-free
    return true;
  }

  // 确保轨迹符合加速度和速度的约束
  // It ensures that the trajectory complies with acceleration and velocity constraints
  bool MavLocalPlanner::isPathFeasible(
      const mav_msgs::EigenTrajectoryPointVector &path) const
  {
    // This is easier to check in the trajectory but then we are limited in how
    // we do the smoothing.
    // It uses a 'for' loop to iterate through each waypoingt in the input trajectory
    for (const mav_msgs::EigenTrajectoryPoint &point : path)
    {
      // checks if the norm of acceleration is greater than 'constraints_.a_max + 1e-2'
      if (point.acceleration_W.norm() > constraints_.a_max + 1e-2)
      {
        return false;
      }

      // checks if the norm of velocity is greater than 'constraints_.a_max + 1e-2'
      if (point.velocity_W.norm() > constraints_.v_max + 1e-2)
      {
        return false;
      }
    }
    return true;
  }

  // 处理偏离计划轨迹的情况
  // handle deviations from the planned trajectory
  bool MavLocalPlanner::dealWithFailure()
  {

    // checks if 'current_waypoint_' is a vaild index
    if (current_waypoint_ < 0)
    {
      return false;
    }

    constexpr double kCloseEnough = 0.05;

    // 使用 'current_waypoint_'索引获取当前的 'waypoint'
    mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];

    // 初始化goal变量
    mav_msgs::EigenTrajectoryPoint goal = waypoint;

    // 选择新目标点
    // 如果 'temporary_goal_' 设置为true, 且存在更多的目标点
    if (temporary_goal_ &&
        static_cast<int64_t>(waypoints_.size()) > current_waypoint_ + 1)
    {
      goal = waypoints_[current_waypoint_ + 1];
    }

    //
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    mav_msgs::EigenTrajectoryPoint current_goal;
    if (!goal_selector_.selectNextGoal(goal, waypoint, current_point,
                                       &current_goal))
    {
      num_failures_++;
      if (num_failures_ > max_failures_)
      {
        current_waypoint_ = -1;
      }
      return false;
    }
    else
    {
      if ((current_goal.position_W - waypoint.position_W).norm() < kCloseEnough)
      {
        // Goal is unchanged. :(
        temporary_goal_ = false;
        return false;
      }
      else if ((current_goal.position_W - goal.position_W).norm() <
               kCloseEnough)
      {
        // This is just the next waypoint that we're trying to go to.
        current_waypoint_++;
        temporary_goal_ = false;
        return true;
      }
      else
      {
        // Then this is something different!
        temporary_goal_ = true;
        waypoints_.insert(waypoints_.begin() + current_waypoint_, current_goal);
        return true;
      }
    }
  }

} // namespace mav_planning
