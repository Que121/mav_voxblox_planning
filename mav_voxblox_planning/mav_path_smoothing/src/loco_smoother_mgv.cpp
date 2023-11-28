#include "loco_planner/impl/loco_impl_mgv.h"
#include <mav_planning_common/visibility_resampling_mgv.h>

#include "mav_path_smoothing/loco_smoother_mgv.h"
#include "mav_trajectory_generation/trajectory_sampling_mgv.h"

namespace mgv_planning
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
  // ============================================================================

  // 1111111111111111111111111111
  bool LocoSmoother::getPathBetweenTwoPoints(
      const mgv_msgs::EigenTrajectoryPointMgv &start,
      const mgv_msgs::EigenTrajectoryPointMgv &goal,
      mgv_msgs::EigenTrajectoryPointMgvVector *path) const
  {
    CHECK_NOTNULL(path); // 确保传入指针不为空
    mgv_trajectory_generation::Trajectory trajectory;
    // ***********************
    bool success = getTrajectoryBetweenTwoPoints(start, goal, &trajectory); // TBD // 生成两个点之间的轨迹， 并存储在trajectory
    if (success)
    {
      // 将'trajectory'采样为一系列的路径点，保存在path里 // 注意！！ path为指针传递 无需回传参数
      mgv_trajectory_generation::sampleWholeTrajectory(
          trajectory, constraints_.sampling_dt, path); // TBD
      return true;
    }
    return false;
  }

  // 222222222222222222222222222
  bool LocoSmoother::getTrajectoryBetweenTwoPoints(
      const mgv_msgs::EigenTrajectoryPointMgv &start,
      const mgv_msgs::EigenTrajectoryPointMgv &goal,
      mgv_trajectory_generation::Trajectory *trajectory) const // 生成两个点之间的轨迹 // TBD
  {

    // 创建计时器记录执行时间 DONE
    mgv_trajectory_generation::timing::Timer loco_timer("smoothing/poly_loco");

    // 确保“trajectory”不为空 DONE
    CHECK_NOTNULL(trajectory);

    // D 为维度 N 为阶数
    constexpr int N = 10;
    constexpr int D = 3; // TBD 改为 2

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
    double total_time = mgv_trajectory_generation::computeTimeVelocityRamp(
        start.position_W, goal.position_W, constraints_.v_max,
        constraints_.a_max);

    // 设置优化问题*********************
    loco.setupFromTrajectoryPoints(start, goal, num_segments_, total_time);
    // 设置优化问题*********************

    // 求解多项式优化问题，生成平滑轨迹
    loco.solveProblem();

    // 获取生成的轨迹**************
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

  // 333333333333333333333333333
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

} // namespace mgv_planning
