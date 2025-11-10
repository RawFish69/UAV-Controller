#ifndef SAFETY_GATE__SAFETY_GATE_HPP_
#define SAFETY_GATE__SAFETY_GATE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "common_msgs/msg/attitude_thrust.hpp"
#include "common_msgs/msg/body_rate_thrust.hpp"
#include "common_msgs/msg/virtual_rc.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"

namespace safety_gate {

class SafetyGate : public rclcpp::Node {
public:
  SafetyGate();

private:
  void bodyRateThrustCallback(const common_msgs::msg::BodyRateThrust::SharedPtr msg);
  void attitudeThrustCallback(const common_msgs::msg::AttitudeThrust::SharedPtr msg);
  void checkTimeout();

  // Safety checks
  bool checkBodyRates(const geometry_msgs::msg::Vector3& rates);
  bool checkThrust(float thrust);
  float clampThrust(float thrust);
  float applySlewRate(float current, float target, float dt, float slew_rate);

  // Conversion utilities
  common_msgs::msg::VirtualRC bodyRateToRC(const common_msgs::msg::BodyRateThrust& brt);
  common_msgs::msg::VirtualRC getSafeIdleRC();
  common_msgs::msg::BodyRateThrust getSafeIdleBodyRate();

  // Parameters
  std::string mode_;
  double max_tilt_deg_;
  double max_body_rate_;
  double thrust_min_;
  double thrust_max_;
  double slew_rate_thrust_;
  double slew_rate_yaw_;
  int timeout_ms_;

  // State
  rclcpp::Time last_command_time_;
  float last_thrust_;
  float last_yaw_rate_;
  bool initialized_;

  // Publishers
  rclcpp::Publisher<common_msgs::msg::BodyRateThrust>::SharedPtr pub_body_rate_thrust_;
  rclcpp::Publisher<common_msgs::msg::VirtualRC>::SharedPtr pub_rc_;

  // Subscribers
  rclcpp::Subscription<common_msgs::msg::BodyRateThrust>::SharedPtr sub_body_rate_thrust_;
  rclcpp::Subscription<common_msgs::msg::AttitudeThrust>::SharedPtr sub_attitude_thrust_;

  // Timer
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

}  // namespace safety_gate

#endif  // SAFETY_GATE__SAFETY_GATE_HPP_

