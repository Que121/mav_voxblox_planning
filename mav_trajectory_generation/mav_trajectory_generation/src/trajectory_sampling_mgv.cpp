#include "mav_trajectory_generation/trajectory_sampling_mgv.h"

namespace mgv_trajectory_generation
{

  const double kNumNanosecondsPerSecond = 1.e9;

  // 2222222222222222222222
  bool sampleTrajectoryInRange(const Trajectory &trajectory, double min_time,
                               double max_time, double sampling_interval,
                               mgv_msgs::EigenTrajectoryPointMgvVector *states)
  {
    CHECK_NOTNULL(states);
    // 判断是否在范围内
    if (min_time < trajectory.getMinTime() ||
        max_time > trajectory.getMaxTime())
    {
      LOG(ERROR) << "Sample time should be within [" << trajectory.getMinTime()
                 << " " << trajectory.getMaxTime() << "] but is [" << min_time
                 << " " << max_time << "]";
      return false;
    }

    // 自由度小于3，返回
    // 三维规划？？？
    if (trajectory.D() < 3)
    {
      LOG(ERROR) << "Dimension has to be at least 3, but is " << trajectory.D();
      return false;
    }

    std::vector<Eigen::VectorXd> position, velocity, acceleration, jerk, snap,
        yaw, yaw_rate;

    // constraints_.sampling_dt ----> sampling_interval ----> dt

    trajectory.evaluateRange(min_time, max_time, sampling_interval,
                             derivative_order::POSITION, &position);
    trajectory.evaluateRange(min_time, max_time, sampling_interval,
                             derivative_order::VELOCITY, &velocity);
    trajectory.evaluateRange(min_time, max_time, sampling_interval,
                             derivative_order::ACCELERATION, &acceleration);
    trajectory.evaluateRange(min_time, max_time, sampling_interval,
                             derivative_order::JERK, &jerk);
    trajectory.evaluateRange(min_time, max_time, sampling_interval,
                             derivative_order::SNAP, &snap);

    size_t n_samples = position.size();

    states->resize(n_samples);
    for (size_t i = 0; i < n_samples; ++i)
    {
      mgv_msgs::EigenTrajectoryPointMgv &state = (*states)[i];

      // EigenTrajectoryPointMgvVector trajectory ----> *path ---> *state
      // state.degrees_of_freedom = mgv_msgs::MavActuation::DOF4;
      state.position_W = position[i].head<3>();
      state.velocity_W = velocity[i].head<3>();
      state.acceleration_W = acceleration[i].head<3>();
      state.jerk_W = jerk[i].head<3>();
      state.snap_W = snap[i].head<3>();
      state.time_from_start_ns = static_cast<int64_t>((min_time + sampling_interval * i) * kNumNanosecondsPerSecond);

      // if (trajectory.D() == 4)
      // {
      //   state.setFromYaw(position[i](3));
      //   state.setFromYawRate(velocity[i](3));
      //   state.setFromYawAcc(acceleration[i](3));
      // }
    }
    return true;
  }

  // 对轨迹进行采样   1111111111111111111
  bool sampleWholeTrajectory(const Trajectory &trajectory,
                             double sampling_interval,
                             mgv_msgs::EigenTrajectoryPointMgvVector *states)
  {
    const double min_time = trajectory.getMinTime();
    const double max_time = trajectory.getMaxTime();

    // 在范围内采样
    return sampleTrajectoryInRange(trajectory, min_time, max_time,
                                   sampling_interval, states);
  }

  //==============================================================================================

  bool sampleTrajectoryAtTime(const Trajectory &trajectory, double sample_time,
                              mgv_msgs::EigenTrajectoryPoint *state)
  {
    CHECK_NOTNULL(state);
    if (sample_time < trajectory.getMinTime() ||
        sample_time > trajectory.getMaxTime())
    {
      LOG(ERROR) << "Sample time should be within [" << trajectory.getMinTime()
                 << " " << trajectory.getMaxTime() << "] but is " << sample_time;
      return false;
    }

    if (trajectory.D() < 3)
    {
      LOG(ERROR) << "Dimension has to be at least 3, but is " << trajectory.D();
      return false;
    }

    return sampleFlatStateAtTime<Trajectory>(trajectory, sample_time, state);
  }

  bool sampleTrajectoryStartDuration(
      const Trajectory &trajectory, double start_time, double duration,
      double sampling_interval, mgv_msgs::EigenTrajectoryPointMgvVector *states)
  {
    return sampleTrajectoryInRange(trajectory, start_time, start_time + duration,
                                   sampling_interval, states);
  }

  bool sampleSegmentAtTime(const Segment &segment, double sample_time,
                           mgv_msgs::EigenTrajectoryPoint *state)
  {
    CHECK_NOTNULL(state);
    if (sample_time < 0.0 || sample_time > segment.getTime())
    {
      LOG(ERROR) << "Sample time should be within [" << 0.0 << " "
                 << segment.getTime() << "] but is " << sample_time;
      return false;
    }

    return sampleFlatStateAtTime<Segment>(segment, sample_time, state);
  }

  template <class T>
  bool sampleFlatStateAtTime(const T &type, double sample_time,
                             mgv_msgs::EigenTrajectoryPoint *state)
  {
    if (type.D() < 3)
    {
      LOG(ERROR) << "Dimension has to be 3, 4, or 6 but is " << type.D();
      return false;
    }

    Eigen::VectorXd position = type.evaluate(sample_time, derivative_order::POSITION);
    Eigen::VectorXd velocity = type.evaluate(sample_time, derivative_order::VELOCITY);
    Eigen::VectorXd acceleration = type.evaluate(sample_time, derivative_order::ACCELERATION);

    state->degrees_of_freedom = mgv_msgs::mgvActuation::DOF4;
    state->position_W = position.head(3);
    state->velocity_W = velocity.head(3);
    state->acceleration_W = acceleration.head(3);
    state->jerk_W = type.evaluate(sample_time, derivative_order::JERK).head(3);
    state->snap_W = type.evaluate(sample_time, derivative_order::SNAP).head(3);

    if (type.D() == 4)
    {
      state->setFromYaw(position(3));
      state->setFromYawRate(velocity(3));
      state->setFromYawAcc(acceleration(3));
    }
    else if (type.D() == 6)
    {
      // overactuated, write quaternion from interpolated rotation vector
      Eigen::Vector3d rot_vec, rot_vec_vel, rot_vec_acc;
      rot_vec = position.tail(3);
      rot_vec_vel = velocity.tail(3);
      rot_vec_acc = acceleration.tail(3);
      Eigen::Matrix3d rot_matrix;
      mgv_msgs::matrixFromRotationVector(rot_vec, &rot_matrix);
      state->orientation_W_B = Eigen::Quaterniond(rot_matrix);
      state->angular_velocity_W = mgv_msgs::omegaFromRotationVector(rot_vec, rot_vec_vel);
      state->angular_acceleration_W = mgv_msgs::omegaDotFromRotationVector(rot_vec, rot_vec_vel, rot_vec_acc);
      state->degrees_of_freedom = mgv_msgs::mgvActuation::DOF6;
    }

    state->time_from_start_ns =
        static_cast<int64_t>(sample_time * kNumNanosecondsPerSecond);
    return true;
  }

} // namespace mgv_trajectory_generation
