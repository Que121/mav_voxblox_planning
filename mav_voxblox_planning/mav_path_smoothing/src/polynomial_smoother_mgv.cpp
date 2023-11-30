#include <mav_trajectory_generation/polynomial_optimization_linear_mgv.h>
#include <mav_trajectory_generation/timing_mgv.h>
#include <mav_trajectory_generation/trajectory_sampling_mgv.h>
// #include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include "mav_path_smoothing/polynomial_smoother_mgv.h"

namespace mgv_planning
{

  PolynomialSmoother::PolynomialSmoother()
      : PathSmootherBase(),
        optimize_time_(true),
        split_at_collisions_(false),
        min_col_check_resolution_(0.1) {}

  void PolynomialSmoother::setParametersFromRos(const ros::NodeHandle &nh)
  {
    PathSmootherBase::setParametersFromRos(nh);
    nh.param("optimize_time", optimize_time_, optimize_time_);
    nh.param("split_at_collisions", split_at_collisions_, split_at_collisions_);
    nh.param("min_col_check_resolution", min_col_check_resolution_,
             min_col_check_resolution_);
  }


  bool PolynomialSmoother::getTrajectoryBetweenWaypoints(
      const mgv_msgs::EigenTrajectoryPointMgv::Vector &waypoints,
      mgv_trajectory_generation::Trajectory *trajectory) const
  {
    // 路径点小于2，没有足够的路径点用于平滑轨迹
    if (waypoints.size() < 2)
    {
      return false;
    }

    // I guess this is only if method is linear.
    // 创建计时器，用于记录执行时间
    mgv_trajectory_generation::timing::Timer linear_timer(
        "smoothing/poly_linear");

    constexpr int N = 10; // 设置多项式阶数为10
    constexpr int D = 3;  // 设置维度，这里为3， 即为三维空间

    // 创建多项式优化器的实例 N为阶数 D为维度
    mgv_trajectory_generation::PolynomialOptimization<N> poly_opt(D);

    // 获取路径点的数量
    int num_vertices = waypoints.size();

    // 选择优化的导数阶数，此处为JERK（三次导，加加速度）
    int derivative_to_optimize =
        mgv_trajectory_generation::derivative_order::JERK;

    // 创建顶点向量，存储路径点
    mgv_trajectory_generation::Vertex::Vector vertices(
        num_vertices, mgv_trajectory_generation::Vertex(D));

    // Add the first and last.
    // 起点和终点的位置约束被添加到了多项式优化问题中，
    // 确保了生成的轨迹从给定的起点出发，并以给定的终点结束。
    // 接下来的步骤将会进一步优化多项式系数，以生成平滑的轨迹。
    vertices.front().makeStartOrEnd(0, derivative_to_optimize);
    vertices.front().addConstraint(
        mgv_trajectory_generation::derivative_order::POSITION,
        waypoints.front().position_W);
    vertices.back().makeStartOrEnd(0, derivative_to_optimize);
    vertices.back().addConstraint(
        mgv_trajectory_generation::derivative_order::POSITION,
        waypoints.back().position_W);

    // Now do the middle bits.
    
    // j 用于
    size_t j = 1;

    for (size_t i = 1; i < waypoints.size() - 1; i += 1)
    {
      vertices[j].addConstraint(
          mgv_trajectory_generation::derivative_order::POSITION,
          waypoints[i].position_W);
      j++;
    }

    std::vector<double> segment_times =
        mgv_trajectory_generation::estimateSegmentTimesVelocityRamp(
            vertices, constraints_.v_max, constraints_.a_max, 1.2);

    poly_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    if (poly_opt.solveLinear())
    {
      poly_opt.getTrajectory(trajectory);
    }
    else
    {
      return false;
    }
    linear_timer.Stop();
    // Ok now do more stuff, if the method requires.
    if (split_at_collisions_)
    {
      mgv_trajectory_generation::timing::Timer split_timer(
          "smoothing/poly_split");

      mgv_msgs::EigenTrajectoryPointMgv::Vector path;

      // Sample it!
      double dt = constraints_.sampling_dt;
      mgv_trajectory_generation::sampleWholeTrajectory(*trajectory, dt, &path);// 改

      // Check if it's in collision.
      double t = 0.0;
      bool path_in_collision = isPathInCollision(path, &t);

      const int kMaxNumberOfAdditionalVertices = 10;
      int num_added = 0;
      while (path_in_collision)
      {
        if (!addVertex(t, *trajectory, &vertices, &segment_times))
        {
          // Well this isn't going anywhere.
          return false;
        }
        poly_opt.setupFromVertices(vertices, segment_times,
                                   derivative_to_optimize);
        poly_opt.solveLinear();
        poly_opt.getTrajectory(trajectory);
        mgv_trajectory_generation::sampleWholeTrajectory(*trajectory, dt, &path);// 改
        path_in_collision = isPathInCollision(path, &t);
        num_added++;
        if (num_added > kMaxNumberOfAdditionalVertices)
        {
          break;
        }
      }
      ROS_INFO("[SPLIT SMOOTHING] Added %d additional vertices.", num_added);

      // If we had to do this, let's scale the times back to make sure we're
      // still within constraints.
      if (optimize_time_)
      {
        trajectory->scaleSegmentTimesToMeetConstraints(constraints_.v_max,
                                                       constraints_.a_max);
      }
    }

    double v_max, a_max;
    trajectory->computeMaxVelocityAndAcceleration(&v_max, &a_max);

    ROS_INFO("[SMOOTHING] V max/limit: %f/%f, A max/limit: %f/%f", v_max,
             constraints_.v_max, a_max, constraints_.a_max);

    return true;
  }

  bool PolynomialSmoother::getPathBetweenWaypoints(
      const mgv_msgs::EigenTrajectoryPointMgvVector &waypoints,
      mgv_msgs::EigenTrajectoryPointMgv::Vector *path) const
  {
    mgv_trajectory_generation::Trajectory trajectory;
    bool success = getTrajectoryBetweenWaypoints(waypoints, &trajectory);
    if (success)
    {
      mgv_trajectory_generation::sampleWholeTrajectory(
          trajectory, constraints_.sampling_dt, path);//改
      return true;
    }
    return false;
  }

  bool PolynomialSmoother::getPathBetweenTwoPoints(
      const mgv_msgs::EigenTrajectoryPointMgv &start,
      const mgv_msgs::EigenTrajectoryPointMgv &goal,
      mgv_msgs::EigenTrajectoryPointMgv::Vector *path) const
  {
    mgv_msgs::EigenTrajectoryPointMgv::Vector waypoints;
    waypoints.push_back(start);
    waypoints.push_back(goal);

    return getPathBetweenWaypoints(waypoints, path);
  }

  bool PolynomialSmoother::addVertex(
      double t, const mgv_trajectory_generation::Trajectory &trajectory,
      mgv_trajectory_generation::Vertex::Vector *vertices,
      std::vector<double> *segment_times) const
  {
    // First, go through the trajectory segments and figure out between which two
    // segments the new vertex will lie.
    const mgv_trajectory_generation::Segment::Vector &segments =
        trajectory.segments();

    double time_so_far = 0.0;
    size_t seg_ind = 0;
    for (seg_ind = 0; seg_ind < segments.size(); ++seg_ind)
    {
      time_so_far += segments[seg_ind].getTime();
      if (time_so_far > t)
      {
        break;
      }
    }

    // Relative time within the segment.
    double seg_max_time = segments[seg_ind].getTime();
    double rel_time_sec = t - time_so_far + seg_max_time;
    // Get the start and goal positions of those segments.
    Eigen::VectorXd start_pos = segments[seg_ind].evaluate(0.0);
    Eigen::VectorXd end_pos = segments[seg_ind].evaluate(seg_max_time);

    // Get the relative time of the new vertex (relative to the start vertex),
    // and make sure it's not too close to the start or end to avoid numerical
    // issues.
    double rel_time_scaled = (rel_time_sec / seg_max_time);
    constexpr double kRelativeTimeMargin = 0.1;
    constexpr double kMinTimeSec = 0.1;
    rel_time_scaled =
        std::max(std::min(rel_time_scaled, 1.0 - kRelativeTimeMargin),
                 kRelativeTimeMargin);
    Eigen::VectorXd new_pos =
        (-start_pos + end_pos) * rel_time_scaled + start_pos;

    if (isPositionInCollision(new_pos.head<3>()))
    {
      ROS_ERROR(
          "Point along the straight line is occupied. Polynomial won't be "
          "collision-free.");
      return false;
    }

    rel_time_sec = std::max(rel_time_scaled * seg_max_time, kMinTimeSec);

    // Add the vertex with the correct constraints.
    mgv_trajectory_generation::Vertex new_vertex = (*vertices)[seg_ind];
    new_vertex.addConstraint(
        mgv_trajectory_generation::derivative_order::POSITION, new_pos);
    vertices->insert(vertices->begin() + seg_ind + 1, new_vertex);

    // Add the segment time in.
    segment_times->insert(segment_times->begin() + seg_ind, rel_time_sec);
    (*segment_times)[seg_ind + 1] =
        std::max(seg_max_time - rel_time_sec, kMinTimeSec);

    return true;
  }

  // Uses whichever collision checking method is set to check for collisions.
  bool PolynomialSmoother::isPositionInCollision(
      const Eigen::Vector3d &pos) const
  {
    if (map_distance_func_)
    {
      return map_distance_func_(pos) < constraints_.robot_radius;
    }
    if (in_collision_func_)
    {
      return in_collision_func_(pos);
    }
    return false;
  }

  bool PolynomialSmoother::isPathInCollision(
      const mgv_msgs::EigenTrajectoryPointMgv::Vector &path, double *t) const
  {
    if (path.size() < 1)
    {
      return true;
    }
    double distance_since_last_check = 0.0;
    Eigen::Vector3d last_pos = path[0].position_W;

    for (const mgv_msgs::EigenTrajectoryPointMgv &point : path)
    {
      distance_since_last_check += (point.position_W - last_pos).norm();
      if (distance_since_last_check > min_col_check_resolution_)
      {
        if (isPositionInCollision(point.position_W))
        {
          if (t != NULL)
          {
            *t = mgv_msgs::nanosecondsToSeconds(point.time_from_start_ns);
          }
          return true;
        }
        last_pos = point.position_W;
        distance_since_last_check = 0.0;
      }
    }
    return false;
  }

} // namespace mav_planning
