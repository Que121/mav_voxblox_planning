#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mgv_local_planner/mgv_local_planner.h"

namespace mgv_planning
{
  // 构造函数
  MgvLocalPlanner::MgvLocalPlanner(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        command_publishing_spinner_(1, &command_publishing_queue_),
        planning_spinner_(1, &planning_queue_),
        verbose_(false), // 打印信息
        global_frame_id_("map"),
        local_frame_id_("odom"),
        replan_dt_(1.0),
        smoother_name_("loco"),   // 路径平滑器的算法
        avoid_collisions_mgv_(0), // 0:不需要避障
        plan_to_startOFmgv_(1),
        path_indexOFmgv_(0)
  {
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                  &MgvLocalPlanner::odometryCallback_mgv, this);

    // 订阅单个航点信息
    waypoint_sub_ =
        nh_.subscribe("waypoint", 1, &MgvLocalPlanner::waypointCallback_mgv, this);

    // 发布完整轨迹信息
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        mgv_msgs::default_topics::COMMAND_TRAJECTORY, 1);

    // 创建了一个ROS服务客户端
    position_hold_client_ =
        nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

    // 创建一个定时器，周期性触发函数planningTimerCallback_mgv
    ros::TimerOptions timer_options(
        ros::Duration(replan_dt_),
        boost::bind(&MgvLocalPlanner::planningTimerCallback_mgv, this, _1),
        &planning_queue_);

    // 初始化空服务用户端
    planning_timer_mgv_ = nh_.createTimer(timer_options);

    // 启动异步线程
    command_publishing_spinner_.start(); // 发送控制
    planning_spinner_.start();           // 处理路径规划和规划器相关信息

    // 需要？
    // 飞行设置
    yaw_policy_.setPhysicalConstraints(constraints_);                 // 设置物理约束策略
    yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kVelocityVector); // 设置偏航策略
  }

  void MgvLocalPlanner::odometryCallback_mgv(const nav_msgs::Odometry &msg)
  {
    // 消息类型需要更改
    mgv_msgs::eigenOdometryFromMsg(msg, &odometryOFmgv_); // 将ros消息转换为eigen类型的变量
  }

  // 回调   接收新的的航点
  void MgvLocalPlanner::waypointCallback_mgv(const geometry_msgs::PoseStamped &msg)
  {
    // 1.清除原来的轨迹信息
    clearTrajectory_mgv();

    // 消息类型需要更改
    mav_msgs::EigenTrajectoryPoint waypointsOFmgv;
    eigenTrajectoryPointFromPoseMsg(msg, &waypointsOFmgv);

    // 3.清除原来航点信息，将新的航点加入
    waypointsOFmgv_.clear();
    waypointsOFmgv_.push_back(waypointsOFmgv);

    // 4.设置航点索引
    current_waypointOFmgv_ = 0;

    // 5.进行路径规划
    planningStep_mgv();

    // 6.发布飞行信息
    startPublishingCommands_mgv();
  }

  // 回调  定时器执行飞行规划
  void MgvLocalPlanner::planningTimerCallback_mgv(const ros::TimerEvent &event)
  {
    // 等待发布器条件变量
    if (should_replan_.wait_for(replan_dt_))
    {
      // 设置--是否打印plan开始的信息
      if (verbose_)
        ROS_INFO("[Mgv Local Plnner][Plan Step] ");
    }
    // 执行一次飞行规划
    planningStep_mgv();
  }

  // 从目前位置到航点进行一次路径规划
  void MgvLocalPlanner::planningStep_mgv()
  {
    // 无人机类型需要修改的哦
    // 1.判断航点列表是否为空并且索引是否超出范围
    if (current_waypointOFmgv_ < 0 ||
        static_cast<int>(waypointsOFmgv_.size()) <= current_waypointOFmgv_)
    {
      // 已经规划至路径终点并且没有新的路径点需要添加，直接返回
      if (path_indexOFmgv_ >= path_queueOFmgv_.size() || path_queueOFmgv_.empty())
      {
        return;
      }
    }
    mav_trajectory_generation::timing::MiniTimer timer; //

    constexpr double kCloseToOdometry = 0.1; // 需要具体修改

    // 不需要避障
    // avoid_collisions_ = 0;
    if (!avoid_collisions_mgv_)
    {
      //
      mgv_msgs::EigenTrajectoryPointVector waypointsOFmgv;
      mgv_msgs::EigenTrajectoryPoint current_waypointOFmgv;

      current_waypointOFmgv.position_W = odometryOFmgv_.position_W; // odometry_ 目前无人机信息
      current_waypointOFmgv.orientation_W_B = odometryOFmgv_.orientation_W_B;

      if (plan_to_startOFmgv_)
      {
        waypointsOFmgv.push_back(current_waypointOFmgv);
      }
      waypointsOFmgv.insert(waypointsOFmgv.end(), waypointsOFmgv_.begin(), waypointsOFmgv_.end());

      mgv_msgs::EigenTrajectoryPointVector pathOFmgv;
      if (planPathThroughWaypoints_mgv(waypointsOFmgv, &pathOFmgv))
      {
        replacePath_mgv(pathOFmgv);
        current_waypointOFmgv_ = waypointsOFmgv_.size();
      }
      else
      {
        ROS_ERROR("[Mgv Local Planner] Waypoint planning failed!");
      }

      // 输出规划完成，包括所花费的时间
      ROS_INFO("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f",
               timer.stop());

      // 可视化路径
      visualizePath();
    }
  }

  // 根据给定的路径点waypoints使用不同的路平滑算法规划一条平滑的路径
  bool MgvLocalPlanner::planPathThroughWaypoints_mgv(
      const mgv_msgs::EigenTrajectoryPointVector &waypointsOFmgv,
      mgv_msgs::EigenTrajectoryPointVector *pathOFmgv)
  {
    // 1. 确保传入的 'path' 不为空
    CHECK_NOTNULL(pathOFmgv);

    // 2. 初始化success变量
    bool success = false;

    // 3.选择的算法为loco
    if (smoother_name_ == "loco")
    {
      // 4.判断航点数量
      if (waypointsOFmgv.size() == 2)
      {
        // 5.规划两点之间的路径
        success = loco_smoother_.getPathBetweenTwoPoints(waypointsOFmgv[0],
                                                         waypointsOFmgv[1], pathOFmgv);
      }
    }
    return success;
  }

  // 取代原来路径点的信息，传入新的路径点信息
  void MgvLocalPlanner::replacePath_mgv(
      const mgv_msgs::EigenTrajectoryPointVector &pathOFmgv)
  {
    // 1.上锁  // nm谁大多
    std::lock_guard<std::recursive_mutex> guard(path_mutexOFmgv_);

    // 2.清除原来路径点
    path_queueOFmgv_.clear();

    // 3.获取新的路径点
    path_queueOFmgv_ = pathOFmgv;

    // 4.设置路径点起始姿态
    path_queueOFmgv_.front().orientation_W_B = odometryOFmgv_.orientation_W_B;

    // 需要修改
    // 5.应用姿态策略
    yaw_policy_.applyPolicyInPlace(&path_queueOFmgv_);

    // 6.重置索引
    path_indexOFmgv_ = 0;
  }

  // 清除原来轨迹信息
  void MgvLocalPlanner::clearTrajectory_mgv()
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutexOFmgv_);
    command_publishing_timer_.stop();
    path_queueOFmgv_.clear();
    path_indexOFmgv_ = 0;
  }

  // 发布飞行指令
  void MgvLocalPlanner::startPublishingCommands_mgv()
  {
    // Call the service call to takeover publishing commands.
    // 1.接管命令发布服务
    if (position_hold_client_.exists())
    {
      std_srvs::Empty empty_call;
      // 向无人机发送一个空的服务请求，用于接管命令的发布
      position_hold_client_.call(empty_call);
    }

    // Publish the first set immediately, on this thread.
    // 2.立刻发布第一组命令
    commandPublishTimerCallback_mgv(ros::TimerEvent());

    // Need advanced timer options to assign callback queue to this timer.
    // 3.设定定时器并关联回调函数
    ros::TimerOptions timer_options(
        ros::Duration(command_publishing_dt_),
        boost::bind(&MgvLocalPlanner::commandPublishTimerCallback_mgv, this, _1),
        &command_publishing_queue_);

    // 4.创建定时器
    command_publishing_timer_ = nh_.createTimer(timer_options);
  }

  // 定时器回调函数，用于在规划的路径上按照一定频率发布控制命令
  void MgvLocalPlanner::commandPublishTimerCallback_mgv(
      const ros::TimerEvent &event)
  {
    // 1.路径队列的缓冲区大小
    constexpr size_t kQueueBuffer = 0;

    // 2.如果路径索引小于路径队列的大小，即还有未到达的路径点
    if (path_indexOFmgv_ < path_queueOFmgv_.size())
    {
      // 锁定线程
      std::lock_guard<std::recursive_mutex> guard(path_mutex_);

      // 3.计算发布的路径点数量
      //  取决于发布的时间间隔和剩余未到达的路径点数量
      size_t number_to_publish = std::min<size_t>(
          std::floor(command_publishing_dt_ / constraints_.sampling_dt),
          path_queueOFmgv_.size() - path_indexOFmgv_);

      // 4.设置路径队列的索引顺序
      size_t starting_index = 0;
      if (path_indexOFmgv_ != 0)
      {
        starting_index = path_indexOFmgv_ + kQueueBuffer;
        // 如果path_index_不为0，加上KQueueBuffer防止在边界上选择路径点出现问题
        if (starting_index >= path_queueOFmgv_.size())
        {
          starting_index = path_indexOFmgv_;
        }
      }

      size_t number_to_publish_with_buffer = std::min<size_t>(
          number_to_publish + mpc_prediction_horizonOFmgv_ - kQueueBuffer,
          path_queueOFmgv_.size() - starting_index);

      // 提取要发布的路径点
      mav_msgs::EigenTrajectoryPointVector::const_iterator first_sample =
          path_queueOFmgv_.begin() + starting_index;
      mgv_msgs::EigenTrajectoryPointVector::const_iterator last_sample =
          first_sample + number_to_publish_with_buffer;
      mgv_msgs::EigenTrajectoryPointVector trajectory_to_publish(first_sample,
                                                                 last_sample);

      // 6.创建和发布控制指令信息
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

      // 8.发布信息
      command_pub_.publish(msg);

      // 9.更新路径索引并通知重新规划
      path_indexOFmgv_ += number_to_publish;
      should_replan_.notify();
    }
    // Does there need to be an else????
  }
}