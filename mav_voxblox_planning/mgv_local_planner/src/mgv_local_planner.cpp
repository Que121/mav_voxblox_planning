// #include <mgv_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mgv_local_planner/mgv_local_planner.h"
#include "/home/patton/voxblox_ws/src/mav_voxblox_planning/mgv_comm/mgv_msgs/include/mgv_msgs/conversions.h" // 消息类型需要更改

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
        esdf_server_(nh_, nh_private_),
        loco_planner_(nh_, nh_private_),
        path_indexOFmgv_(0)
  {
    odometry_sub_ = nh_.subscribe("odometry", 1,
                                  &MgvLocalPlanner::odometryCallback_mgv, this);

    // 订阅单个航点信息
    waypoint_sub_ =
        nh_.subscribe("waypoint", 1, &MgvLocalPlanner::waypointCallback_mgv, this);

    // 发布完整轨迹信息
    command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "command/trajectory", 1);

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

  // 注意odometry为nav，此为自动生成的消息类型文件
  void MgvLocalPlanner::odometryCallback_mgv(const nav_msgs::Odometry &msg)
  {
    // 消息类型需要更改 // 不改了 感觉gv可以用
    mgv_msgs::eigenOdometryFromMsg(msg, &odometryOFmgv_); // 将ros消息转换为eigen类型的变量
  }

  // 接收新的的航点 done
  void MgvLocalPlanner::waypointCallback_mgv(const geometry_msgs::PoseStamped &msg)
  {
    clearTrajectory_mgv();
    mgv_msgs::EigenTrajectoryPointMgv waypointsOFmgv;
    mgv_msgs::eigenTrajectoryPointFromPoseMsgMgv(msg, &waypointsOFmgv);
    waypointsOFmgv_.clear();
    waypointsOFmgv_.push_back(waypointsOFmgv);
    current_waypointOFmgv_ = 0;

    planningStep_mgv();
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

  // 从目前位置到航点进行一次规划
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
    constexpr double kCloseToOdometry = 0.1;            // 需要具体修改
    if (!avoid_collisions_mgv_)                         // avoid_collisions_ = 0; // 不需要避障
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
      ROS_INFO("[Mgv Local Planner][Plan Step] Planning finished. Time taken: %f",
               timer.stop());

      // 可视化路径
      // visualizePath();
    }
  }

  // 根据给定的路径点waypoints使用不同的路平滑算法规划一条平滑的路径
  bool MgvLocalPlanner::planPathThroughWaypoints_mgv(
      const mgv_msgs::EigenTrajectoryPointVector &waypointsOFmgv,
      mgv_msgs::EigenTrajectoryPointVector *pathOFmgv)
  {
    CHECK_NOTNULL(pathOFmgv);     // 确保path不为空
    bool success = false;         // 初始化success变量 为规划是否成功的flag
    if (smoother_name_ == "loco") // 目前只使用loco算法
    {
      if (waypointsOFmgv.size() == 2) // 判断是否有两点用于规划
      {
        // 规划两点之间的路径
        success = loco_smoother_.getPathBetweenTwoPoints(waypointsOFmgv[0],
                                                         waypointsOFmgv[1], pathOFmgv);
      }
    }
    return success;
  }

  // 获取新的path
  void MgvLocalPlanner::replacePath_mgv(
      const mgv_msgs::EigenTrajectoryPointVector &pathOFmgv)
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutexOFmgv_);             // 上锁
    path_queueOFmgv_.clear();                                                  // 清除原来路径点
    path_queueOFmgv_ = pathOFmgv;                                              // 获取新的path
    path_queueOFmgv_.front().orientation_W_B = odometryOFmgv_.orientation_W_B; // 设置起始的姿态
    // ***可能需要更改***
    yaw_policy_.applyPolicyInPlace(&path_queueOFmgv_); // 应用姿态策略 人话：设置起始的yaw轴
    path_indexOFmgv_ = 0;                              // 重置索引
  }

  // 清除原来轨迹信息
  void MgvLocalPlanner::clearTrajectory_mgv()
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutexOFmgv_);
    command_publishing_timer_mgv_.stop(); // 停止发布命令定时器
    path_queueOFmgv_.clear();
    path_indexOFmgv_ = 0;
  }

  // 发布飞行指令
  void MgvLocalPlanner::startPublishingCommands_mgv()
  {
    if (position_hold_client_.exists()) // 接管命令发布服务
    {
      std_srvs::Empty empty_call;
      position_hold_client_.call(empty_call); // 向机器人发送一个空的服务请求，用于接管命令的发布
    }
    commandPublishTimerCallback_mgv(ros::TimerEvent()); // 立刻发布第一组命令
    ros::TimerOptions timer_options(
        ros::Duration(command_publishing_dt_),
        boost::bind(&MgvLocalPlanner::commandPublishTimerCallback_mgv, this, _1),
        &command_publishing_queue_);                                // 设定定时器并关联回调函数
    command_publishing_timer_mgv_ = nh_.createTimer(timer_options); // 创建定时器
  }

  // 定时器回调函数，用于在规划的路径上按照一定频率发布控制命令
  void MgvLocalPlanner::commandPublishTimerCallback_mgv(
      const ros::TimerEvent &event)
  {
    constexpr size_t kQueueBuffer = 0; // 设置path的缓冲区大小
    if (path_indexOFmgv_ < path_queueOFmgv_.size())
    {
      std::lock_guard<std::recursive_mutex> guard(path_mutexOFmgv_); // 锁定线程
      // 计算发布的path数量
      // 取决于发布的时间间隔和剩余未到达的路径点数量
      size_t number_to_publish = std::min<size_t>(
          std::floor(command_publishing_dt_ / constraints_.sampling_dt),
          path_queueOFmgv_.size() - path_indexOFmgv_); // std::floor 向下取整
      size_t starting_index = 0;                       // 设置path队列的索引顺序
      if (path_indexOFmgv_ != 0)                       // path的index不为零
      {
        starting_index = path_indexOFmgv_ + kQueueBuffer; // 如果path_index_不为0，加上KQueueBuffer防止在边界上选择点出现问题
        if (starting_index >= path_queueOFmgv_.size())
        {
          starting_index = path_indexOFmgv_; // reset开始的index为当前的index of path
        }
      }
      // caonima直接发不行阿？？？
      //=================================================================
      // size_t number_to_publish_with_buffer = std::min<size_t>(
      //     number_to_publish + mpc_prediction_horizonOFmgv_ - kQueueBuffer,
      //     path_queueOFmgv_.size() - starting_index);
      // mgv_msgs::EigenTrajectoryPointVector::const_iterator first_sample =
      //     path_queueOFmgv_.begin() + starting_index; // 提取发送path的头
      // mgv_msgs::EigenTrajectoryPointVector::const_iterator last_sample =
      //     first_sample + number_to_publish_with_buffer; // 提取发送path的尾
      // mgv_msgs::EigenTrajectoryPointVector trajectory_to_publish(first_sample,
      //                                                            last_sample);
      //=================================================================

      // 直接从头到尾
      mgv_msgs::EigenTrajectoryPointVector trajectory_to_publish(path_queueOFmgv_.begin() + starting_index,
                                                                 path_queueOFmgv_.end());
      trajectory_msgs::MultiDOFJointTrajectory msg; // 创建一个多自由度关节轨迹消息，用于存储控制指令

      msg.header.frame_id = local_frame_id_; // 将ROS消息（msg）中header的frame_id字段设置为存储在变量local_frame_id_中的值
      msg.header.stamp = ros::Time::now();   // 将ROS消息（msg）中header的stamp字段设置为当前ROS时间

      // 暂时不用打印
      //=================================================================
      // ROS_INFO(
      //     "[Mgv Local Planner][Command Publish] Publishing %zu samples of %zu. "
      //     "Start index: %zu Time: %f Start position: %f Start velocity: %f End "
      //     "time: %f End position: %f",
      //     trajectory_to_publish.size(), path_queueOFmgv_.size(), starting_index,
      //     trajectory_to_publish.front().time_from_start_ns * 1.0e-9,
      //     trajectory_to_publish.front().position_W.x(),
      //     trajectory_to_publish.front().velocity_W.x(),
      //     trajectory_to_publish.back().time_from_start_ns * 1.0e-9,
      //     trajectory_to_publish.back().position_W.x());
      //=================================================================
      mgv_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_to_publish, &msg); // 转换为ros消息发出

      command_pub_.publish(msg);             // 发布控制消息
      path_indexOFmgv_ += number_to_publish; // Update the path index and notify for replanning
      should_replan_.notify();
    }
  }

  void MgvLocalPlanner::ppUART_mgv(trajectory_msgs::MultiDOFJointTrajectory *msg)
  {
    std::string ppUART_mgv;
  }

  void MgvLocalPlanner::visualizePath() {} // TBD
}