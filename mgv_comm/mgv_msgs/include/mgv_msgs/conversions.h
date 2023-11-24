#ifndef MGV_MSGS_CONVERSIONS_H
#define MGV_MSGS_CONVERSIONS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "mgv_msgs/Actuators.h"
#include "mgv_msgs/AttitudeThrust.h"
#include "mgv_msgs/RateThrust.h"
#include "mgv_msgs/RollPitchYawrateThrust.h"
#include "mgv_msgs/TorqueThrust.h"
#include "mgv_msgs/common.h"
#include "mgv_msgs/default_values.h"
#include "mgv_msgs/eigen_mgv_msgs.h"

namespace mgv_msgs {

// 从msg获取eigen类型的变量 位姿、推力
inline void eigenAttitudeThrustFromMsg(const AttitudeThrust& msg,
                                       EigenAttitudeThrust* attitude_thrust) {
  assert(attitude_thrust != NULL);

  attitude_thrust->attitude = quaternionFromMsg(msg.attitude);
  attitude_thrust->thrust = vector3FromMsg(msg.thrust);
}

// 从msg获取eigen类型的变量 转角、旋转速度
inline void eigenActuatorsFromMsg(const Actuators& msg,
                                  EigenActuators* actuators) {
  assert(actuators != NULL);

  // Angle of the actuator in [rad].
  for (unsigned int i = 0; i < msg.angles.size(); ++i) {
    actuators->angles.resize(msg.angles.size());
    actuators->angles[i] = msg.angles[i];
  }

  // Angular velocities of the actuator in [rad/s].
  actuators->angular_velocities.resize(msg.angular_velocities.size());
  for (unsigned int i = 0; i < msg.angular_velocities.size(); ++i) {
    actuators->angular_velocities[i] = msg.angular_velocities[i];
  }

  // Normalized: Everything that does not fit the above, normalized
  // between [-1 ... 1].
  actuators->normalized.resize(msg.normalized.size());
  for (unsigned int i = 0; i < msg.normalized.size(); ++i) {
    actuators->normalized[i] = msg.normalized[i];
  }
}

// 从msg获取eigen类型的变量 角速度、推力
inline void eigenRateThrustFromMsg(const RateThrust& msg,
                                   EigenRateThrust* rate_thrust) {
  assert(rate_thrust != NULL);

  rate_thrust->angular_rates = vector3FromMsg(msg.angular_rates);
  rate_thrust->thrust = vector3FromMsg(msg.thrust);
}

// 从msg获取eigen类型的变量 扭矩、推力
inline void eigenTorqueThrustFromMsg(const TorqueThrust& msg,
                                     EigenTorqueThrust* torque_thrust) {
  assert(torque_thrust != NULL);

  torque_thrust->torque = vector3FromMsg(msg.torque);
  torque_thrust->thrust = vector3FromMsg(msg.thrust);
}

// 从msg获取eigen类型的变量 滚转角、俯仰角、偏航角速度、推力
inline void eigenRollPitchYawrateThrustFromMsg(
    const RollPitchYawrateThrust& msg,
    EigenRollPitchYawrateThrust* roll_pitch_yawrate_thrust) {
  assert(roll_pitch_yawrate_thrust != NULL);

  roll_pitch_yawrate_thrust->roll = msg.roll;
  roll_pitch_yawrate_thrust->pitch = msg.pitch;
  roll_pitch_yawrate_thrust->yaw_rate = msg.yaw_rate;
  roll_pitch_yawrate_thrust->thrust = vector3FromMsg(msg.thrust);
}

// 将msg转换为eigen类型的变量 // 可用于gv done
inline void eigenOdometryFromMsg(const nav_msgs::Odometry& msg,
                                 EigenOdometry* odometry) {
  assert(odometry != NULL);                            // 不为空指针
  odometry->timestamp_ns = msg.header.stamp.toNSec();  // 时间戳
  odometry->position_W = mgv_msgs::vector3FromPointMsg(
      msg.pose.pose.position);  // 获取位置信息（x，y，z）
  odometry->orientation_W_B =
      mgv_msgs::quaternionFromMsg(msg.pose.pose.orientation);  // 四元数表示方向
  odometry->velocity_B =
      mgv_msgs::vector3FromMsg(msg.twist.twist.linear);  // (x,y,z)速度
  odometry->angular_velocity_B =
      mgv_msgs::vector3FromMsg(msg.twist.twist.angular);  // 角速度
  odometry->pose_covariance_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(
      msg.pose.covariance.data());  // 位姿的协方差矩阵
  odometry->twist_covariance_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(
      msg.twist.covariance.data());  // twis的协方差矩阵
}

// 将时间戳msg转换为eigen类型的变量
inline void eigenTrajectoryPointFromTransformMsg(
    const geometry_msgs::TransformStamped& msg,
    EigenTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  ros::Time timestamp = msg.header.stamp;

  trajectory_point->timestamp_ns = timestamp.toNSec();
  trajectory_point->position_W = vector3FromMsg(msg.transform.translation);
  trajectory_point->orientation_W_B = quaternionFromMsg(msg.transform.rotation);
  trajectory_point->velocity_W.setZero();
  trajectory_point->angular_velocity_W.setZero();
  trajectory_point->acceleration_W.setZero();
  trajectory_point->angular_acceleration_W.setZero();
  trajectory_point->jerk_W.setZero();
  trajectory_point->snap_W.setZero();
}

// 用于mgv done

void EigenMavStateFromEigenTrajectoryPoint(
    const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk,
    const Eigen::Vector3d& snap, double yaw, double yaw_rate,
    double yaw_acceleration, double magnitude_of_gravity,
    Eigen::Quaterniond* orientation, Eigen::Vector3d* acceleration_body,
    Eigen::Vector3d* angular_velocity_body,
    Eigen::Vector3d* angular_acceleration_body);

/// Convenience function with default value for the magnitude of the gravity
/// vector.
inline void EigenMavStateFromEigenTrajectoryPoint(
    const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk,
    const Eigen::Vector3d& snap, double yaw, double yaw_rate,
    double yaw_acceleration, Eigen::Quaterniond* orientation,
    Eigen::Vector3d* acceleration_body, Eigen::Vector3d* angular_velocity_body,
    Eigen::Vector3d* angular_acceleration_body) {
  EigenMavStateFromEigenTrajectoryPoint(
      acceleration, jerk, snap, yaw, yaw_rate, yaw_acceleration, kGravity,
      orientation, acceleration_body, angular_velocity_body,
      angular_acceleration_body);
}

/// Convenience function with EigenTrajectoryPoint as input and EigenMavState as
/// output.
// 从轨迹点中获取飞机的各种状态
inline void EigenMavStateFromEigenTrajectoryPoint(
    const EigenTrajectoryPoint& trajectory_point, double magnitude_of_gravity,
    EigenMavState* mav_state) {
  assert(mav_state != NULL);
  if (trajectory_point.degrees_of_freedom == MavActuation::DOF4) {
    EigenMavStateFromEigenTrajectoryPoint(
        trajectory_point.acceleration_W, trajectory_point.jerk_W,
        trajectory_point.snap_W, trajectory_point.getYaw(),
        trajectory_point.getYawRate(), trajectory_point.getYawAcc(),
        magnitude_of_gravity, &(mav_state->orientation_W_B),
        &(mav_state->acceleration_B), &(mav_state->angular_velocity_B),
        &(mav_state->angular_acceleration_B));
    mav_state->position_W = trajectory_point.position_W;
    mav_state->velocity_W = trajectory_point.velocity_W;
  } else {
    // Assume fully actuated vehicle, can track trajectory point.
    Eigen::Matrix3d R = trajectory_point.orientation_W_B.toRotationMatrix();
    mav_state->position_W = trajectory_point.position_W;
    mav_state->velocity_W = trajectory_point.velocity_W;
    mav_state->acceleration_B =
        R.transpose() * (trajectory_point.acceleration_W +
                         Eigen::Vector3d(0.0, 0.0, magnitude_of_gravity));
    mav_state->orientation_W_B = trajectory_point.orientation_W_B;
    mav_state->angular_velocity_B =
        R.transpose() * trajectory_point.angular_velocity_W;
    mav_state->angular_acceleration_B =
        R.transpose() * trajectory_point.angular_acceleration_W;
  }
}

/**
 * \brief Convenience function with EigenTrajectoryPoint as input and
 * EigenMavState as output
 *        with default value for the magnitude of the gravity vector.
 */
// 调用上个函数
inline void EigenMavStateFromEigenTrajectoryPoint(
    const EigenTrajectoryPoint& flat_state, EigenMavState* mav_state) {
  EigenMavStateFromEigenTrajectoryPoint(flat_state, kGravity, mav_state);
}

// 从轨迹点中获取飞机的各种状态
inline void EigenMavStateFromEigenTrajectoryPoint(
    const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk,
    const Eigen::Vector3d& snap, double yaw, double yaw_rate,
    double yaw_acceleration, double magnitude_of_gravity,
    Eigen::Quaterniond* orientation, Eigen::Vector3d* acceleration_body,
    Eigen::Vector3d* angular_velocity_body,
    Eigen::Vector3d* angular_acceleration_body) {
  // Mapping from flat state to full state following to Mellinger [1]:
  // See [1]: Mellinger, Daniel Warren. "Trajectory generation and control for
  //          quadrotors." (2012), Phd-thesis, p. 15ff.
  // http://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations
  //
  //  zb = acc+[0 0 magnitude_of_gravity]';
  //  thrust =  norm(zb);
  //  zb = zb / thrust;
  //
  //  xc = [cos(yaw) sin(yaw) 0]';
  //
  //  yb = cross(zb, xc);
  //  yb = yb/norm(yb);
  //
  //  xb = cross(yb, zb);
  //
  //  q(:,i) = rot2quat([xb yb zb]);
  //
  //  h_w = 1/thrust*(acc_dot - zb' * acc_dot * zb;
  //
  //  w(1,i) = -h_w'*yb;
  //  w(2,i) = h_w'*xb;
  //  w(3,i) = yaw_dot*[0 0 1]*zb;

  assert(orientation != nullptr);
  assert(acceleration_body != nullptr);
  assert(angular_velocity_body != nullptr);
  assert(angular_acceleration_body != nullptr);

  Eigen::Vector3d xb;
  Eigen::Vector3d yb;
  Eigen::Vector3d zb(acceleration);

  zb[2] += magnitude_of_gravity;
  const double thrust = zb.norm();
  const double inv_thrust = 1.0 / thrust;
  zb = zb * inv_thrust;

  yb = zb.cross(Eigen::Vector3d(cos(yaw), sin(yaw), 0.0));
  yb.normalize();

  xb = yb.cross(zb);

  const Eigen::Matrix3d R((Eigen::Matrix3d() << xb, yb, zb).finished());

  const Eigen::Vector3d h_w =
      inv_thrust * (jerk - double(zb.transpose() * jerk) * zb);

  *orientation = Eigen::Quaterniond(R);
  *acceleration_body = R.transpose() * zb * thrust;
  (*angular_velocity_body)[0] = -h_w.transpose() * yb;
  (*angular_velocity_body)[1] = h_w.transpose() * xb;
  (*angular_velocity_body)[2] = yaw_rate * zb[2];

  // Calculate angular accelerations.
  const Eigen::Vector3d wcrossz = (*angular_velocity_body).cross(zb);
  const Eigen::Vector3d wcrosswcrossz = (*angular_velocity_body).cross(wcrossz);
  const Eigen::Vector3d h_a =
      inv_thrust * (snap - double(zb.transpose() * snap) * zb) - 2 * wcrossz -
      wcrosswcrossz + double(zb.transpose() * wcrosswcrossz) * zb;

  (*angular_acceleration_body)[0] = -h_a.transpose() * yb;
  (*angular_acceleration_body)[1] = h_a.transpose() * xb;
  (*angular_acceleration_body)[2] = yaw_acceleration * zb[2];
}

// 从msg中获取轨迹点信息
inline void eigenTrajectoryPointFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg,
    EigenTrajectoryPoint* trajectory_point) {
  assert(trajectory_point != NULL);

  if (msg.transforms.empty()) {
    ROS_ERROR("MultiDofJointTrajectoryPoint is empty.");
    return;
  }

  if (msg.transforms.size() > 1) {
    ROS_WARN(
        "MultiDofJointTrajectoryPoint message should have one joint, but has "
        "%lu. Using first joint.",
        msg.transforms.size());
  }

  trajectory_point->time_from_start_ns = msg.time_from_start.toNSec();
  trajectory_point->position_W = vector3FromMsg(msg.transforms[0].translation);
  trajectory_point->orientation_W_B =
      quaternionFromMsg(msg.transforms[0].rotation);
  if (msg.velocities.size() > 0) {
    trajectory_point->velocity_W = vector3FromMsg(msg.velocities[0].linear);
    trajectory_point->angular_velocity_W =
        vector3FromMsg(msg.velocities[0].angular);
  } else {
    trajectory_point->velocity_W.setZero();
    trajectory_point->angular_velocity_W.setZero();
  }
  if (msg.accelerations.size() > 0) {
    trajectory_point->acceleration_W =
        vector3FromMsg(msg.accelerations[0].linear);
    trajectory_point->angular_acceleration_W =
        vector3FromMsg(msg.accelerations[0].angular);
  } else {
    trajectory_point->acceleration_W.setZero();
    trajectory_point->angular_acceleration_W.setZero();
  }
  trajectory_point->jerk_W.setZero();
  trajectory_point->snap_W.setZero();
}

// 从msg中获取一系列的轨迹信息
inline void eigenTrajectoryPointVectorFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectory& msg,
    EigenTrajectoryPointVector* trajectory) {
  assert(trajectory != NULL);
  trajectory->clear();
  for (const auto& msg_point : msg.points) {
    EigenTrajectoryPoint point;
    eigenTrajectoryPointFromMsg(msg_point, &point);
    trajectory->push_back(point);
  }
}

// EigenTrajectoryPointDeque？？？ 感觉和上面功能函数一样
inline void eigenTrajectoryPointDequeFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectory& msg,
    EigenTrajectoryPointDeque* trajectory) {
  assert(trajectory != NULL);
  trajectory->clear();
  for (const auto& msg_point : msg.points) {
    EigenTrajectoryPoint point;
    eigenTrajectoryPointFromMsg(msg_point, &point);
    trajectory->push_back(point);
  }
}

// In all these conversions, client is responsible for filling in message
// header.
// 构建msg Actuators
inline void msgActuatorsFromEigen(const EigenActuators& actuators,
                                  Actuators* msg) {
  assert(msg != NULL);

  msg->angles.resize(actuators.angles.size());
  for (unsigned int i = 0; i < actuators.angles.size(); ++i) {
    msg->angles[i] = actuators.angles[i];
  }

  msg->angular_velocities.resize(actuators.angular_velocities.size());
  for (unsigned int i = 0; i < actuators.angular_velocities.size(); ++i) {
    msg->angular_velocities[i] = actuators.angular_velocities[i];
  }

  msg->normalized.resize(actuators.normalized.size());
  for (unsigned int i = 0; i < actuators.normalized.size(); ++i) {
    msg->normalized[i] = actuators.normalized[i];
  }
}

// 构建msg AttitudeThrust
inline void msgAttitudeThrustFromEigen(
    const EigenAttitudeThrust& attitude_thrust, AttitudeThrust* msg) {
  assert(msg != NULL);
  quaternionEigenToMsg(attitude_thrust.attitude, &msg->attitude);
  vectorEigenToMsg(attitude_thrust.thrust, &msg->thrust);
}

// 构建msg RateThrust
inline void msgRateThrustFromEigen(EigenRateThrust& rate_thrust,
                                   RateThrust* msg) {
  assert(msg != NULL);
  vectorEigenToMsg(rate_thrust.angular_rates, &msg->angular_rates);
  vectorEigenToMsg(rate_thrust.thrust, &msg->thrust);
}

// 构建msg TorqueThrust
inline void msgTorqueThrustFromEigen(EigenTorqueThrust& torque_thrust,
                                     TorqueThrust* msg) {
  assert(msg != NULL);
  vectorEigenToMsg(torque_thrust.torque, &msg->torque);
  vectorEigenToMsg(torque_thrust.thrust, &msg->thrust);
}

// 构建msg RollPitchYawrateThrust
inline void msgRollPitchYawrateThrustFromEigen(
    const EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust,
    RollPitchYawrateThrust* msg) {
  assert(msg != NULL);
  msg->roll = roll_pitch_yawrate_thrust.roll;
  msg->pitch = roll_pitch_yawrate_thrust.pitch;
  msg->yaw_rate = roll_pitch_yawrate_thrust.yaw_rate;
  vectorEigenToMsg(roll_pitch_yawrate_thrust.thrust, &msg->thrust);
}

// 构建msg Odometry
inline void msgOdometryFromEigen(const EigenOdometry& odometry,
                                 nav_msgs::Odometry* msg) {
  assert(msg != NULL);

  if (odometry.timestamp_ns >= 0) {
    msg->header.stamp.fromNSec(odometry.timestamp_ns);
  }
  pointEigenToMsg(odometry.position_W, &msg->pose.pose.position);
  quaternionEigenToMsg(odometry.orientation_W_B, &msg->pose.pose.orientation);

  vectorEigenToMsg(odometry.velocity_B, &msg->twist.twist.linear);
  vectorEigenToMsg(odometry.angular_velocity_B, &msg->twist.twist.angular);
}

// WARNING: discards all derivatives, etc.
// 从轨迹点信息生成PoseStamped的msg
inline void msgPoseStampedFromEigenTrajectoryPoint(
    const EigenTrajectoryPoint& trajectory_point,
    geometry_msgs::PoseStamped* msg) {
  if (trajectory_point.timestamp_ns >= 0) {
    msg->header.stamp.fromNSec(trajectory_point.timestamp_ns);
  }
  pointEigenToMsg(trajectory_point.position_W, &msg->pose.position);
  quaternionEigenToMsg(trajectory_point.orientation_W_B,
                       &msg->pose.orientation);
}

// 从轨迹点信息生成MultiDOFJointTrajectoryPoint msg， 多传递了一个string参数
inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point, const std::string& link_name,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);
  trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
  msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);

  msg->joint_names.clear();
  msg->points.clear();
  msg->joint_names.push_back(link_name);
  msg->points.push_back(point_msg);
}

// 调用上面函数
inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", msg);
}

// Convenience method to quickly create a trajectory from a single waypoint.
// 从位姿和偏航信息生成MultiDOFJointTrajectoryPoint msg
inline void msgMultiDofJointTrajectoryFromPositionYaw(
    const Eigen::Vector3d& position, double yaw,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  EigenTrajectoryPoint point;
  point.position_W = position;
  point.setFromYaw(yaw);

  msgMultiDofJointTrajectoryFromEigen(point, msg);
}

// 从轨迹信息生成 多个MultiDOFJointTrajectoryPoint msg
inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPointDeque& trajectory, const std::string& link_name,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  if (trajectory.empty()) {
    ROS_ERROR("EigenTrajectoryPointVector is empty.");
    return;
  }

  msg->joint_names.clear();
  msg->joint_names.push_back(link_name);
  msg->points.clear();

  for (const auto& trajectory_point : trajectory) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);
    msg->points.push_back(point_msg);
  }
}

// 调用上个函数
inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPointDeque& trajectory,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  msgMultiDofJointTrajectoryFromEigen(trajectory, "base_link", msg);
}

// =========================================================================================

inline void eigenTrajectoryPointFromPoseMSgMgv(
    const geometry_msgs::PoseStamped& msg,
    EigenTrajectoryPointMgv* trajectory_point) {
  assert(trajectory_point != NULL);
  ros::Time timestamp = msg.header.stamp;
  trajectory_point->timestamp_ns = timestamp.toNSec();

  trajectory_point->position_W = vector3FromPointMsg(msg.pose.position);
  trajectory_point->orientation_W_B = quaternionFromMsg(msg.pose.orientation);
  trajectory_point->velocity_W.setZero();
  trajectory_point->angular_velocity_W.setZero();
  trajectory_point->acceleration_W.setZero();
  trajectory_point->angular_acceleration_W.setZero();
}

// 调用上个函数 // 11111111111111111111111
inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPointVector& trajectory,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  // 为什么不直接传三个参数....
  msgMultiDofJointTrajectoryFromEigen(trajectory, "base_link", msg);
}

// 222222222222222222222
inline void msgMultiDofJointTrajectoryFromEigen(
    const EigenTrajectoryPointVector& trajectory, const std::string& link_name,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);  // assert检验msg是否为空

  if (trajectory.empty()) {
    ROS_ERROR("EigenTrajectoryPointVector is empty.");
    return;
  }  // 检验轨迹是否为空

  msg->joint_names.clear();
  msg->joint_names.push_back(link_name);
  msg->points.clear();

  for (const auto& trajectory_point : trajectory) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &point_msg);
    msg->points.push_back(point_msg);
  }
}

// 从轨迹点信息生成MultiDOFJointTrajectoryPoint msg
// 3333333333333333333
inline void msgMultiDofJointTrajectoryPointFromEigen(
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectoryPoint* msg) {
  assert(msg != NULL);

  msg->time_from_start.fromNSec(trajectory_point.time_from_start_ns);
  msg->transforms.resize(1);
  msg->velocities.resize(1);
  msg->accelerations.resize(1);

  vectorEigenToMsg(trajectory_point.position_W,
                   &msg->transforms[0].translation);
  quaternionEigenToMsg(trajectory_point.orientation_W_B,
                       &msg->transforms[0].rotation);
  vectorEigenToMsg(trajectory_point.velocity_W, &msg->velocities[0].linear);
  vectorEigenToMsg(trajectory_point.angular_velocity_W,
                   &msg->velocities[0].angular);
  vectorEigenToMsg(trajectory_point.acceleration_W,
                   &msg->accelerations[0].linear);
  vectorEigenToMsg(trajectory_point.angular_acceleration_W,
                   &msg->accelerations[0].angular);
}

// mgv使用***************
inline void msgMultiDofJointTrajectoryPointFromEigen_mgv(
    const EigenTrajectoryPointVector& trajectory, const std::string& link_name,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {}

}  // end namespace mgv_msgs

#endif  // MGV_MSGS_CONVERSIONS_H
