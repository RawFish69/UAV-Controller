#ifndef CONTROLLERS_PID__PID_HPP_
#define CONTROLLERS_PID__PID_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "common_msgs/msg/body_rate_thrust.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controllers_pid {

struct PIDParams {
  double kp, ki, kd, kff;
  double tau_d;       // Derivative filter time constant
  double i_clamp;     // Integral windup limit
  double rate_limit;  // Output rate limit
};

class PIDAxis {
public:
  PIDAxis();
  void setParams(const PIDParams& params);
  double update(double setpoint, double measured, double dt);
  void reset();

private:
  PIDParams params_;
  double integral_;
  double derivative_filtered_;
  double last_measured_;
  bool initialized_;
};

class PIDCore {
public:
  PIDCore();
  void setParams(const PIDParams& roll, const PIDParams& pitch, const PIDParams& yaw,
                 double hover_thrust, double k_aw, bool use_attitude_outer, double k_att);
  Eigen::Vector4d computeControl(const Eigen::Vector3d& rate_setpoint,
                                  const Eigen::Vector3d& rate_measured, double dt);
  Eigen::Vector4d computeControlWithAttitude(const Eigen::Vector4d& attitude_setpoint,
                                              const Eigen::Vector4d& attitude_measured,
                                              const Eigen::Vector3d& rate_measured, double dt);
  void reset();

private:
  Eigen::Vector3d attitudeError(const Eigen::Vector4d& desired, const Eigen::Vector4d& current);

  PIDAxis roll_pid_;
  PIDAxis pitch_pid_;
  PIDAxis yaw_pid_;
  double hover_thrust_;
  double k_aw_;  // Anti-windup gain
  bool use_attitude_outer_;
  double k_att_;  // Attitude P gain
};

class PIDNode : public rclcpp::Node {
public:
  PIDNode();

private:
  void attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void controlLoop();
  void loadParams();

  std::shared_ptr<PIDCore> pid_core_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Publishers
  rclcpp::Publisher<common_msgs::msg::BodyRateThrust>::SharedPtr pub_cmd_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_attitude_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_angular_velocity_;

  // State
  bool has_attitude_;
  bool has_angular_velocity_;
  geometry_msgs::msg::Quaternion current_attitude_;
  geometry_msgs::msg::Vector3 current_angular_velocity_;
  rclcpp::Time last_control_time_;

  // Parameters
  double control_rate_;
  bool use_attitude_outer_;
};

}  // namespace controllers_pid

#endif  // CONTROLLERS_PID__PID_HPP_

