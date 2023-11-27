
#ifndef MGV_MSGS_EIGEN_MGV_MSGS_H
#define MGV_MSGS_EIGEN_MGV_MSGS_H

#include <Eigen/Eigen>
#include <deque>
#include <iostream>

#include "mgv_msgs/common.h"

namespace mgv_msgs {

/// Actuated degrees of freedom.
enum MavActuation { DOF4 = 4, DOF6 = 6 };  // 自由度

// 位姿、推力
struct EigenAttitudeThrust {
  EigenAttitudeThrust()
      : attitude(Eigen::Quaterniond::Identity()),
        thrust(Eigen::Vector3d::Zero()) {}
  EigenAttitudeThrust(const Eigen::Quaterniond& _attitude,
                      const Eigen::Vector3d& _thrust) {
    attitude = _attitude;
    thrust = _thrust;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond attitude;
  Eigen::Vector3d thrust;
};

// Actuators，转角、旋转速度
struct EigenActuators {
  // TODO(ffurrer): Find a proper way of initializing :)

  EigenActuators(const Eigen::VectorXd& _angular_velocities) {
    angular_velocities = _angular_velocities;
  }  // just receive angular velocities ???????

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::VectorXd angles;              // In rad.
  Eigen::VectorXd angular_velocities;  // In rad/s.
  Eigen::VectorXd normalized;          // Everything else, normalized [-1 to 1].
};

// 角速度、推力 angular_rates、thrust
struct EigenRateThrust {
  EigenRateThrust()
      : angular_rates(Eigen::Vector3d::Zero()),
        thrust(Eigen::Vector3d::Zero()) {}

  EigenRateThrust(const Eigen::Vector3d& _angular_rates,
                  const Eigen::Vector3d _thrust)
      : angular_rates(_angular_rates), thrust(_thrust) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d angular_rates;
  Eigen::Vector3d thrust;
};

// 扭矩、推力
struct EigenTorqueThrust {
  EigenTorqueThrust()
      : torque(Eigen::Vector3d::Zero()), thrust(Eigen::Vector3d::Zero()) {}

  EigenTorqueThrust(const Eigen::Vector3d& _torque,
                    const Eigen::Vector3d _thrust)
      : torque(_torque), thrust(_thrust) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d torque;
  Eigen::Vector3d thrust;
};

// 滚转角、俯仰角、偏航角速度、推力
struct EigenRollPitchYawrateThrust {
  EigenRollPitchYawrateThrust()
      : roll(0.0), pitch(0.0), yaw_rate(0.0), thrust(Eigen::Vector3d::Zero()) {}

  EigenRollPitchYawrateThrust(double _roll, double _pitch, double _yaw_rate,
                              const Eigen::Vector3d& _thrust)
      : roll(_roll), pitch(_pitch), yaw_rate(_yaw_rate), thrust(_thrust) {}

  double roll;
  double pitch;
  double yaw_rate;
  Eigen::Vector3d thrust;
};

// 包含当前无人机位置、速度、加速度、角速度......信息
class EigenMavState {
 public:
  typedef std::vector<EigenMavState, Eigen::aligned_allocator<EigenMavState>>
      Vector;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Initializes all members to zero / identity.
  // 初始化
  EigenMavState()
      : position_W(Eigen::Vector3d::Zero()),
        velocity_W(Eigen::Vector3d::Zero()),
        acceleration_B(Eigen::Vector3d::Zero()),
        orientation_W_B(Eigen::Quaterniond::Identity()),
        angular_velocity_B(Eigen::Vector3d::Zero()),
        angular_acceleration_B(Eigen::Vector3d::Zero()) {}

  // 初始化成员变量
  EigenMavState(const Eigen::Vector3d& position_W,
                const Eigen::Vector3d& velocity_W,
                const Eigen::Vector3d& acceleration_B,
                const Eigen::Quaterniond& orientation_W_B,
                const Eigen::Vector3d& angular_velocity_B,
                const Eigen::Vector3d& angular_acceleration_B)
      : position_W(position_W),
        velocity_W(velocity_W),
        acceleration_B(acceleration_B),
        orientation_W_B(orientation_W_B),
        angular_velocity_B(angular_velocity_B),
        angular_acceleration_B(angular_acceleration_B) {}

  std::string toString() const {
    std::stringstream ss;
    ss << "position:              " << position_W.transpose() << std::endl
       << "velocity:              " << velocity_W.transpose() << std::endl
       << "acceleration_body:     " << acceleration_B.transpose() << std::endl
       << "orientation (w-x-y-z): " << orientation_W_B.w() << " "
       << orientation_W_B.x() << " " << orientation_W_B.y() << " "
       << orientation_W_B.z() << " " << std::endl
       << "angular_velocity_body: " << angular_velocity_B.transpose()
       << std::endl
       << "angular_acceleration_body: " << angular_acceleration_B.transpose()
       << std::endl;

    return ss.str();
  }

  Eigen::Vector3d position_W;
  Eigen::Vector3d velocity_W;
  Eigen::Vector3d acceleration_B;
  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3d angular_velocity_B;
  Eigen::Vector3d angular_acceleration_B;
};

// 轨迹点
struct EigenTrajectoryPoint {
  typedef std::vector<EigenTrajectoryPoint,
                      Eigen::aligned_allocator<EigenTrajectoryPoint>>
      Vector;
  // 初始化变量
  EigenTrajectoryPoint()
      : timestamp_ns(-1),  // 时间戳
        time_from_start_ns(0),
        position_W(Eigen::Vector3d::Zero()),              // 位置
        velocity_W(Eigen::Vector3d::Zero()),              // 速度
        acceleration_W(Eigen::Vector3d::Zero()),          // 加速度
        jerk_W(Eigen::Vector3d::Zero()),                  // ？
        snap_W(Eigen::Vector3d::Zero()),                  // ？
        orientation_W_B(Eigen::Quaterniond::Identity()),  // 旋转
        angular_velocity_W(Eigen::Vector3d::Zero()),      // 角速度
        angular_acceleration_W(Eigen::Vector3d::Zero()),  // 角速度上的加速度
        degrees_of_freedom(MavActuation::DOF4) {}         // 自由度4

  // 初始化成员变量
  EigenTrajectoryPoint(
      int64_t _time_from_start_ns, const Eigen::Vector3d& _position,
      const Eigen::Vector3d& _velocity, const Eigen::Vector3d& _acceleration,
      const Eigen::Vector3d& _jerk, const Eigen::Vector3d& _snap,
      const Eigen::Quaterniond& _orientation,
      const Eigen::Vector3d& _angular_velocity,
      const Eigen::Vector3d& _angular_acceleration,
      const MavActuation& _degrees_of_freedom = MavActuation::DOF4)
      : time_from_start_ns(_time_from_start_ns),
        position_W(_position),
        velocity_W(_velocity),
        acceleration_W(_acceleration),
        jerk_W(_jerk),
        snap_W(_snap),
        orientation_W_B(_orientation),
        angular_velocity_W(_angular_velocity),
        angular_acceleration_W(_angular_acceleration),
        degrees_of_freedom(_degrees_of_freedom) {}

  // 调用上个EigenTrajectoryPoint初始化成员变量
  EigenTrajectoryPoint(
      int64_t _time_from_start_ns, const Eigen::Vector3d& _position,
      const Eigen::Vector3d& _velocity, const Eigen::Vector3d& _acceleration,
      const Eigen::Vector3d& _jerk, const Eigen::Vector3d& _snap,
      const Eigen::Quaterniond& _orientation,
      const Eigen::Vector3d& _angular_velocity,
      const MavActuation& _degrees_of_freedom = MavActuation::DOF4)
      : EigenTrajectoryPoint(_time_from_start_ns, _position, _velocity,
                             _acceleration, _jerk, _snap, _orientation,
                             _angular_velocity, Eigen::Vector3d::Zero(),
                             _degrees_of_freedom) {}

  // Eigen 库中的一个宏，作用就是为特定的类提供重载的 new 和 delete
  // 操作符，以便使用 aligned 内存分配器进行内存分配
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 定义变量
  int64_t
      timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
  int64_t time_from_start_ns;
  Eigen::Vector3d position_W;
  Eigen::Vector3d velocity_W;
  Eigen::Vector3d acceleration_W;
  Eigen::Vector3d jerk_W;
  Eigen::Vector3d snap_W;

  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3d angular_velocity_W;
  Eigen::Vector3d angular_acceleration_W;
  MavActuation degrees_of_freedom;

  // 变量提取/初始化
  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const {
    return yawFromQuaternion(orientation_W_B);
  }  // 获得偏航角
  inline double getYawRate() const {
    return angular_velocity_W.z();
  }  // z轴上角速度
  inline double getYawAcc() const {
    return angular_acceleration_W.z();
  }  // z轴加速度
  // WARNING: sets roll and pitch to 0.
  inline void setFromYaw(double yaw) {
    orientation_W_B = quaternionFromYaw(yaw);
  }
  inline void setFromYawRate(double yaw_rate) {
    angular_velocity_W.x() = 0.0;
    angular_velocity_W.y() = 0.0;
    angular_velocity_W.z() = yaw_rate;
  }
  inline void setFromYawAcc(double yaw_acc) {
    angular_acceleration_W.x() = 0.0;
    angular_acceleration_W.y() = 0.0;
    angular_acceleration_W.z() = yaw_acc;
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "position:          " << position_W.transpose() << std::endl
       << "velocity:          " << velocity_W.transpose() << std::endl
       << "acceleration:      " << acceleration_W.transpose() << std::endl
       << "jerk:              " << jerk_W.transpose() << std::endl
       << "snap:              " << snap_W.transpose() << std::endl
       << "yaw:               " << getYaw() << std::endl
       << "yaw_rate:          " << getYawRate() << std::endl
       << "yaw_acc:           " << getYawAcc() << std::endl;

    return ss.str();
  }
};

// 用于mgv的轨迹点
struct EigenTrajectoryPointMgv {
  typedef std::vector<EigenTrajectoryPointMgv,
                      Eigen::aligned_allocator<EigenTrajectoryPointMgv>>
      Vector;
   // 初始化变量
  EigenTrajectoryPointMgv()
      : timestamp_ns(-1),  // 时间戳
        time_from_start_ns(0),
        position_W(Eigen::Vector3d::Zero()),              // 位置
        velocity_W(Eigen::Vector3d::Zero()),              // 速度
        acceleration_W(Eigen::Vector3d::Zero()),          // 加速度
        jerk_W(Eigen::Vector3d::Zero()),                  // ？
        snap_W(Eigen::Vector3d::Zero()),                  // ？
        orientation_W_B(Eigen::Quaterniond::Identity()),  // 旋转
        angular_velocity_W(Eigen::Vector3d::Zero()),      // 角速度
        angular_acceleration_W(Eigen::Vector3d::Zero()),  // 角速度上的加速度
        degrees_of_freedom(MavActuation::DOF4) {}         // 自由度4

  // 初始化成员变量
  EigenTrajectoryPointMgv(
      int64_t _time_from_start_ns, const Eigen::Vector3d& _position,
      const Eigen::Vector3d& _velocity, const Eigen::Vector3d& _acceleration,
      const Eigen::Vector3d& _jerk, const Eigen::Vector3d& _snap,
      const Eigen::Quaterniond& _orientation,
      const Eigen::Vector3d& _angular_velocity,
      const Eigen::Vector3d& _angular_acceleration,
      const MavActuation& _degrees_of_freedom = MavActuation::DOF4)
      : time_from_start_ns(_time_from_start_ns),
        position_W(_position),
        velocity_W(_velocity),
        acceleration_W(_acceleration),
        jerk_W(_jerk),
        snap_W(_snap),
        orientation_W_B(_orientation),
        angular_velocity_W(_angular_velocity),
        angular_acceleration_W(_angular_acceleration),
        degrees_of_freedom(_degrees_of_freedom) {}

  // 调用上个EigenTrajectoryPoint初始化成员变量
  EigenTrajectoryPointMgv(
      int64_t _time_from_start_ns, const Eigen::Vector3d& _position,
      const Eigen::Vector3d& _velocity, const Eigen::Vector3d& _acceleration,
      const Eigen::Vector3d& _jerk, const Eigen::Vector3d& _snap,
      const Eigen::Quaterniond& _orientation,
      const Eigen::Vector3d& _angular_velocity,
      const MavActuation& _degrees_of_freedom = MavActuation::DOF4)
      : EigenTrajectoryPointMgv(_time_from_start_ns, _position, _velocity,
                             _acceleration, _jerk, _snap, _orientation,
                             _angular_velocity, Eigen::Vector3d::Zero(),
                             _degrees_of_freedom) {}

  // Eigen 库中的一个宏，作用就是为特定的类提供重载的 new 和 delete
  // 操作符，以便使用 aligned 内存分配器进行内存分配
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // 定义变量
  int64_t
      timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
  int64_t time_from_start_ns;
  Eigen::Vector3d position_W;
  Eigen::Vector3d velocity_W;
  Eigen::Vector3d acceleration_W;
  Eigen::Vector3d jerk_W;
  Eigen::Vector3d snap_W;

  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3d angular_velocity_W;
  Eigen::Vector3d angular_acceleration_W;
  MavActuation degrees_of_freedom;

  // 变量提取/初始化
  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const {
    return yawFromQuaternion(orientation_W_B);
  }  // 获得偏航角
  inline double getYawRate() const {
    return angular_velocity_W.z();
  }  // z轴上角速度
  inline double getYawAcc() const {
    return angular_acceleration_W.z();
  }  // z轴加速度
  // WARNING: sets roll and pitch to 0.
  inline void setFromYaw(double yaw) {
    orientation_W_B = quaternionFromYaw(yaw);
  }
  inline void setFromYawRate(double yaw_rate) {
    angular_velocity_W.x() = 0.0;
    angular_velocity_W.y() = 0.0;
    angular_velocity_W.z() = yaw_rate;
  }
  inline void setFromYawAcc(double yaw_acc) {
    angular_acceleration_W.x() = 0.0;
    angular_acceleration_W.y() = 0.0;
    angular_acceleration_W.z() = yaw_acc;
  }
};

// 对轨迹点进行仿射变换
// Operator overload to transform Trajectory Points according to the Eigen
// interfaces (uses operator* for this).
// Has to be outside of class.
// Example:
// Eigen::Affine3d transform; EigenTrajectoryPoint point;
// EigenTrajectoryPoint transformed = transform * point;
inline EigenTrajectoryPoint operator*(const Eigen::Affine3d& lhs,
                                      const EigenTrajectoryPoint& rhs) {
  EigenTrajectoryPoint transformed(rhs);
  transformed.position_W = lhs * rhs.position_W;
  transformed.velocity_W = lhs.rotation() * rhs.velocity_W;
  transformed.acceleration_W = lhs.rotation() * rhs.acceleration_W;
  transformed.jerk_W = lhs.rotation() * rhs.jerk_W;
  transformed.snap_W = lhs.rotation() * rhs.snap_W;
  transformed.orientation_W_B = lhs.rotation() * rhs.orientation_W_B;
  transformed.angular_velocity_W = lhs.rotation() * rhs.angular_velocity_W;
  transformed.angular_acceleration_W =
      lhs.rotation() * rhs.angular_acceleration_W;
  return transformed;
}

// VO数据
struct EigenOdometry {
  // 初始化
  EigenOdometry()
      : timestamp_ns(-1),                                 // 时间戳
        position_W(Eigen::Vector3d::Zero()),              // 位置
        orientation_W_B(Eigen::Quaterniond::Identity()),  // 旋转
        velocity_B(Eigen::Vector3d::Zero()),              // 速度
        angular_velocity_B(Eigen::Vector3d::Zero()) {}    // 角速度

  // 初始化成员变量
  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity_body,
                const Eigen::Vector3d& _angular_velocity)
      : position_W(_position),
        orientation_W_B(_orientation),
        velocity_B(_velocity_body),
        angular_velocity_B(_angular_velocity) {}

  // 定义变量
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t
      timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
  Eigen::Vector3d position_W;
  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3d velocity_B;  // Velocity in expressed in the Body frame!
  Eigen::Vector3d angular_velocity_B;
  Eigen::Matrix<double, 6, 6> pose_covariance_;
  Eigen::Matrix<double, 6, 6> twist_covariance_;

  // 变量提取/初始化
  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const { return yawFromQuaternion(orientation_W_B); }
  inline void getEulerAngles(Eigen::Vector3d* euler_angles) const {
    getEulerAnglesFromQuaternion(orientation_W_B, euler_angles);
  }
  inline double getYawRate() const { return angular_velocity_B.z(); }
  // WARNING: sets roll and pitch to 0.
  inline void setFromYaw(double yaw) {
    orientation_W_B = quaternionFromYaw(yaw);
  }
  inline void setFromYawRate(double yaw_rate) {
    angular_velocity_B.x() = 0.0;
    angular_velocity_B.y() = 0.0;
    angular_velocity_B.z() = yaw_rate;
  }

  inline Eigen::Vector3d getVelocityWorld() const {
    return orientation_W_B * velocity_B;
  }
  inline void setVelocityWorld(const Eigen::Vector3d& velocity_world) {
    velocity_B = orientation_W_B.inverse() * velocity_world;
  }
};

// 创建针对Eigen类型数据对齐的一个容器
//  TODO(helenol): replaced with aligned allocator headers from Simon.
#define MGV_MSGS_CONCATENATE(x, y) x##y                         // 连接xy
#define MGV_MSGS_CONCATENATE2(x, y) MGV_MSGS_CONCATENATE(x, y)  //
#define MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE)                    \
  typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE>> \
      MGV_MSGS_CONCATENATE2(EIGEN_TYPE, Vector);                        \
  typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE>>  \
      MGV_MSGS_CONCATENATE2(EIGEN_TYPE, Deque);

MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenAttitudeThrust)
MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenActuators)
MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenRateThrust)
MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenRollPitchYawrateThrust)
MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenTrajectoryPoint)
MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenOdometry)
MGV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenTrajectoryPointMgv)

}  // namespace mgv_msgs

#endif  // MAV_MSGS_EIGEN_MAV_MSGS_H
