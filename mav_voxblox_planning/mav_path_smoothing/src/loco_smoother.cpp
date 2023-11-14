#include <loco_planner/loco.h>
#include <mav_planning_common/visibility_resampling.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_path_smoothing/loco_smoother.h"

namespace mav_planning
{

  LocoSmoother::LocoSmoother()
      : PolynomialSmoother(),        // 调用了基类 ‘PolynomialSmoother’ 的构造函数
        resample_trajectory_(false), // 标识是否重新采样轨迹
        resample_visibility_(false), // 标识是否在可视化图上重新采样
        num_segments_(3),            // 表示轨迹被分成的段数
        add_waypoints_(false),       // 用于标识是否添加路径点
        scale_time_(true)            // 标识
  {
    split_at_collisions_ = false; // 发生碰撞是否增加顶点
  }

  // 获取参数值并将器赋给类的成员变量
  void LocoSmoother::setParametersFromRos(const ros::NodeHandle &nh)
  {
    PolynomialSmoother::setParametersFromRos(nh);
    nh.param("resample_trajectory", resample_trajectory_, resample_trajectory_);
    nh.param("resample_visbility", resample_visibility_, resample_visibility_);
    nh.param("add_waypoints", add_waypoints_, add_waypoints_);
    nh.param("num_segments", num_segments_, num_segments_);

    // Force some settings.
    split_at_collisions_ = false;
  }

  // 更根据给定的点生成平滑的轨迹，可选择是否在可视化图上进行重新采样，以及是否将路径点添加到规划里
  bool LocoSmoother::getTrajectoryBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint::Vector &waypoints,
      mav_trajectory_generation::Trajectory *trajectory) const
  {
    // If there's less than 3 waypoints, there are no free variables for loco.
    // 无足够的点生成平滑轨迹
    if (waypoints.size() < 2)
    {
      return false;
    }

    // 如果路径点为2，直接调用基类函数生成轨迹
    if (waypoints.size() == 2)
    {
      return PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints,
                                                               trajectory);
    }

    // 创建计时器，record the time of operation in LOCO
    mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

    // Create a loco object! So Loco!
    // 创建用于存储初始轨迹的变量
    mav_trajectory_generation::Trajectory traj_initial;

    // 如果需要可视化图上重新采样路径点，将给定的路径点均匀地划分到图上
    // ？？？？
    if (resample_visibility_)
    {
      // If resampling the visibility graph, then basically divide the whole thing
      // into evenly spaced waypoints on the graph.
      mav_msgs::EigenTrajectoryPoint::Vector resampled_waypoints;

      //
      resampleWaypointsFromVisibilityGraph(num_segments_, constraints_, waypoints,
                                           &resampled_waypoints);
      // 生成初始轨迹
      PolynomialSmoother::getTrajectoryBetweenWaypoints(resampled_waypoints,
                                                        &traj_initial);
    }

    // 不需要重新采样路径点
    else
    {
      // 生成初始轨迹
      PolynomialSmoother::getTrajectoryBetweenWaypoints(waypoints, &traj_initial);
    }

    constexpr int N = 10;
    constexpr int D = 3;
    // 创建LOCO规划器实例 N多项式阶数 D维度
    loco_planner::Loco<N> loco(D);

    // This is because our initial solution is nearly collision-free.
    // 设置权重，用于控制平滑度
    loco.setWd(0.1);

    // 设置机器人半径
    loco.setRobotRadius(constraints_.robot_radius);

    // 设置地图分辨率，用于碰撞检测
    loco.setMapResolution(min_col_check_resolution_);

    if (distance_and_gradient_function_)
    {
      loco.setDistanceAndGradientFunction(
          std::bind(&LocoSmoother::getMapDistanceAndGradient, this,
                    std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      loco.setDistanceFunction(map_distance_func_);
    }

    if (resample_trajectory_ && !resample_visibility_)
    {
      loco.setupFromTrajectoryAndResample(traj_initial, num_segments_);
    }
    else
    {
      loco.setupFromTrajectory(traj_initial);
    }
    if (add_waypoints_)
    {
      loco.setWaypointsFromTrajectory(traj_initial);
    }

    loco.solveProblem();
    loco.getTrajectory(trajectory);

    if (scale_time_)
    {
      trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                     constraints_.a_max);
    }

    return true;
  }

  bool LocoSmoother::getTrajectoryBetweenTwoPoints(
      const mav_msgs::EigenTrajectoryPoint &start,
      const mav_msgs::EigenTrajectoryPoint &goal,
      mav_trajectory_generation::Trajectory *trajectory) const // 生成两个点之间的轨迹
  {

    // 创建计时器记录执行时间
    mav_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

    // 确保“trajectory”不为空
    CHECK_NOTNULL(trajectory);

    // 定义多项式阶数和维度
    constexpr int N = 10;
    constexpr int D = 3;

    // 创建LOCO规划器实例
    loco_planner::Loco<N> loco(D);

    // 设置优化问题的权重，这里设置为0.1
    loco.setWd(0.1);

    // 设置机器人半径
    loco.setRobotRadius(constraints_.robot_radius);

    // 设置地图分辨率 VoxBlox.size()
    loco.setMapResolution(min_col_check_resolution_);

    //
    if (distance_and_gradient_function_)
    {
      loco.setDistanceAndGradientFunction(
          std::bind(&LocoSmoother::getMapDistanceAndGradient, this,
                    std::placeholders::_1, std::placeholders::_2));
    }

    else
    {
      loco.setDistanceFunction(map_distance_func_);
    }

    // 计算两个点之间的轨迹所需总时间，基于速度梯形规划算法
    double total_time = mav_trajectory_generation::computeTimeVelocityRamp(
        start.position_W, goal.position_W, constraints_.v_max,
        constraints_.a_max);

    // 设置优化问题，指定起始点、目标点、轨迹段数和总时间
    loco.setupFromTrajectoryPoints(start, goal, num_segments_, total_time);

    // 求解多项式优化问题，生成平滑轨迹
    loco.solveProblem();

    // 获取生成的轨迹
    loco.getTrajectory(trajectory);

    // 是否优化时间
    if (optimize_time_)
    {
      //
      trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                     constraints_.a_max);
    }
    return true;
  }

  bool LocoSmoother::getPathBetweenTwoPoints(
      const mav_msgs::EigenTrajectoryPoint &start,
      const mav_msgs::EigenTrajectoryPoint &goal,
      mav_msgs::EigenTrajectoryPoint::Vector *path) const
  {
    // 确保传入指针不为空
    CHECK_NOTNULL(path);

    // 存储两点之间的轨迹
    mav_trajectory_generation::Trajectory trajectory;
    bool success = getTrajectoryBetweenTwoPoints(start, goal, &trajectory); // 生成两个点之间的轨迹， 并存储在trajectory
    if (success)
    {
      // 将'trajectory'采样为一系列的路径点，保存在path里
      mav_trajectory_generation::sampleWholeTrajectory(
          trajectory, constraints_.sampling_dt, path);
      return true;
    }
    return false;
  }

  double LocoSmoother::getMapDistanceAndGradient(
      const Eigen::VectorXd &position, Eigen::VectorXd *gradient) const
  {
    CHECK(distance_and_gradient_function_);
    CHECK_EQ(position.size(), 3);
    if (gradient == nullptr)
    {
      return distance_and_gradient_function_(position, nullptr);
    }
    Eigen::Vector3d gradient_3d;
    double distance = distance_and_gradient_function_(position, &gradient_3d);
    *gradient = gradient_3d;
    return distance;
  }

} // namespace mav_planning
