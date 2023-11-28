#ifndef MGV_LOCAL_PLANNER_MGV_LOCAL_PLANNER_H_
#define MGV_LOCAL_PLANNER_MGV_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// 无人机类型需要修改
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <mgv_msgs/conversions.h>
#include <mgv_msgs/eigen_mgv_msgs.h>

#include <mav_path_smoothing/loco_smoother_mgv.h>
#include <mav_path_smoothing/polynomial_smoother_mgv.h>
#include <mav_path_smoothing/velocity_ramp_smoother_mgv.h>

#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_utils_mgv.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints_mgv.h>
#include <mav_planning_common/semaphore_mgv.h>
#include <mav_planning_common/yaw_policy_mgv.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints_mgv.h>
#include <mav_planning_common/semaphore_mgv.h>
#include <mav_planning_common/yaw_policy_mgv.h>

#include <mav_planning_msgs/PolynomialTrajectory4D.h>

#include <mav_visualization/helpers.h>
#include <voxblox_loco_planner/goal_point_selector.h>
#include <voxblox_loco_planner/voxblox_loco_planner_mgv.h>
#include <voxblox_ros/esdf_server.h>
#include <minkindr_conversions/kindr_msg.h>

namespace mgv_planning

{
  class MgvLocalPlanner
  {
  public:
    // 构造函数
    MgvLocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    // 数据输入
    void odometryCallback_mgv(const nav_msgs::Odometry &msg);         // 机器人位置信息
    void waypointCallback_mgv(const geometry_msgs::PoseStamped &msg); // 单个路径点信息

    // 停止path的pub, 并且清空现在所有的轨迹
    void clearTrajectvisualizePathory_mgv();

    void clearTrajectory_mgv();

    // 和上面功能相同，但是still发回现在的机器人位置 TBD
    void abort();

    // 将path展示在rviz TBD
    void visualizePath();

  private:
    // 控制指令发布
    void startPublishingCommands_mgv();
    void commandPublishTimerCallback_mgv(const ros::TimerEvent &event);

    // 路线规划控制器
    void planningTimerCallback_mgv(const ros::TimerEvent &event);
    void planningStep_mgv();

    // 如果存在下一个目标点，可以执行
    bool nextWaypoint();         // TBD
    void finishWaypointsOFmgv(); // TBD

    // 貌似是用在避障重规划路线里面的？
    void replacePath_mgv(const mgv_msgs::EigenTrajectoryPointMgvVector &pathOFmgv);

    // 得到两点或多点的path 目前只用到两点
    bool planPathThroughWaypoints_mgv(
        const mgv_msgs::EigenTrajectoryPointMgvVector &waypointsOFmgv,
        mgv_msgs::EigenTrajectoryPointMgvVector *pathOFmgv);

    // pp协议编码
    void ppUART_mgv(trajectory_msgs::MultiDOFJointTrajectory *msg);

    // 获取地图息 TBD
    double getMapDistance(const Eigen::Vector3d &position) const {}
    double getMapDistanceAndGradient(const Eigen::Vector3d &position,
                                     Eigen::Vector3d *gradient) const {}

    // 检测碰撞 TBD
    bool isPathCollisionFree(
        const mgv_msgs::EigenTrajectoryPointVector &path) const {}

    // 其他的内部成员？？？ TBD
    void sendCurrentPose() {}

    // 创建一个与ROS主系统通信的NodeHandle对象
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ROS的输入和输出
    ros::Subscriber waypoint_sub_;       // sub目标点
    ros::Subscriber odometry_sub_;       // sub机器人位置信息
    ros::Publisher command_pub_;         // pub控制命令
    ros::Publisher path_marker_pub_;     // pub轨迹点
    ros::Publisher full_trajectory_pub_; // pub完整轨迹到rive

    // 服务客户端获取MGV接口来监听发送的命令
    ros::ServiceClient position_hold_client_;

    // ROS的异步处理：用于命令发布的回调队列和微调器
    ros::CallbackQueue command_publishing_queue_;
    ros::AsyncSpinner command_publishing_spinner_;
    ros::CallbackQueue planning_queue_;
    ros::AsyncSpinner planning_spinner_;

    // 路线规划控制器定时器
    ros::Timer command_publishing_timer_mgv_;
    ros::Timer planning_timer_mgv_;

    // 设置--是否打印plan开始的信息
    bool verbose_;

    // 设置--frames
    std::string global_frame_id_;
    std::string local_frame_id_;

    // 设置--
    bool plan_to_startOFmgv_;
    bool avoid_collisions_mgv_;
    std::string smoother_name_;

    // 定义--机器人位置  // 无人机类型需要修改
    mgv_msgs::EigenOdometry odometryOFmgv_;

    // 定义--目标点  // 无人机类型需要修改
    int64_t current_waypointOFmgv_;
    mgv_msgs::EigenTrajectoryPointMgvVector waypointsOFmgv_;

    // 定义--轨迹path  // 无人机类型需要修改
    mgv_msgs::EigenTrajectoryPointMgvVector path_queueOFmgv_;
    size_t path_indexOFmgv_;

    // 线程锁
    std::recursive_mutex path_mutexOFmgv_;
    std::recursive_mutex map_mutexOFmgv_;

    // Settings -- constraints.
    PhysicalConstraints constraints_;

    // Settings -- controller interface. 不需要
    int mpc_prediction_horizonOFmgv_;

    // Map!  // 无Voxblox的包！！！
    voxblox::EsdfServer esdf_server_;

    double command_publishing_dt_;
    double replan_dt_;
    double replan_lookahead_sec_;

    // 规划器--yaw轴策略
    YawPolicy yaw_policy_; // ？？
    RosSemaphore should_replan_;

    // 规划器--本地loco规划
    VoxbloxLocoPlanner loco_planner_;

    // 规划器--path平滑.三种方式
    // VelocityRampSmoother ramp_smoother_;
    // PolynomialSmoother poly_smoother_;
    LocoSmoother loco_smoother_;
  };
}

#endif // MGV_LOCAL_PLANNER_MGV_LOCAL_PLANNER_H_
