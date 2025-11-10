#ifndef CONTROLLERS_LQR__LQR_HPP_
#define CONTROLLERS_LQR__LQR_HPP_

#include <Eigen/Dense>
#include <memory>
#include <string>

#include "common_msgs/msg/body_rate_thrust.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controllers_lqr {

// State: x = [phi, theta, psi, p, q, r] (roll, pitch, yaw, body rates)
using State = Eigen::Matrix<double, 6, 1>;
using Control = Eigen::Matrix<double, 4, 1>;  // [p_cmd, q_cmd, r_cmd, thrust]
using GainMatrix = Eigen::Matrix<double, 4, 6>;

class LQRCore {
public:
  LQRCore();
  void setParameters(double mass, double Jx, double Jy, double Jz, double hover_thrust,
                     const GainMatrix& K);
  Control computeControl(const State& current_state, const State& desired_state);
  State getCurrentState() const { return current_state_; }
  bool isStable() const;

private:
  double mass_;
  double Jx_, Jy_, Jz_;
  double hover_thrust_;
  GainMatrix K_;
  State current_state_;
};

class LQRNode : public rclcpp::Node {
public:
  LQRNode();

private:
  void attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void controlLoop();
  void loadGainMatrix();
  Eigen::Vector3d quaternionToEuler(const geometry_msgs::msg::Quaternion& q);

  std::shared_ptr<LQRCore> lqr_core_;
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

  // Parameters
  double control_rate_;
  double mass_;
  double Jx_, Jy_, Jz_;
  double hover_thrust_;
  GainMatrix K_;
};

}  // namespace controllers_lqr

#endif  // CONTROLLERS_LQR__LQR_HPP_

