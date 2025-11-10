#include "safety_gate/safety_gate.hpp"

#include <algorithm>
#include <cmath>

namespace safety_gate {

SafetyGate::SafetyGate() : Node("safety_gate"), initialized_(false), last_thrust_(0.0f), last_yaw_rate_(0.0f) {
  // Declare parameters
  this->declare_parameter<std::string>("mode", "sim");
  this->declare_parameter<double>("max_tilt_deg", 35.0);
  this->declare_parameter<double>("max_body_rate", 10.0);
  this->declare_parameter<double>("thrust_min", 0.05);
  this->declare_parameter<double>("thrust_max", 0.9);
  this->declare_parameter<double>("slew_rate_thrust", 3.0);
  this->declare_parameter<double>("slew_rate_yaw", 5.0);
  this->declare_parameter<int>("timeout_ms", 200);

  // Get parameters
  mode_ = this->get_parameter("mode").as_string();
  max_tilt_deg_ = this->get_parameter("max_tilt_deg").as_double();
  max_body_rate_ = this->get_parameter("max_body_rate").as_double();
  thrust_min_ = this->get_parameter("thrust_min").as_double();
  thrust_max_ = this->get_parameter("thrust_max").as_double();
  slew_rate_thrust_ = this->get_parameter("slew_rate_thrust").as_double();
  slew_rate_yaw_ = this->get_parameter("slew_rate_yaw").as_double();
  timeout_ms_ = this->get_parameter("timeout_ms").as_int();

  RCLCPP_INFO(this->get_logger(), "Safety Gate initialized in mode: %s", mode_.c_str());

  // Create publishers based on mode
  if (mode_ == "sim") {
    pub_body_rate_thrust_ = this->create_publisher<common_msgs::msg::BodyRateThrust>(
        "/cmd/final/body_rate_thrust", 10);
  } else if (mode_ == "crsf") {
    pub_rc_ = this->create_publisher<common_msgs::msg::VirtualRC>("/cmd/final/rc", 10);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s. Must be 'sim' or 'crsf'", mode_.c_str());
  }

  // Create subscribers
  sub_body_rate_thrust_ = this->create_subscription<common_msgs::msg::BodyRateThrust>(
      "/cmd/body_rate_thrust", 10,
      std::bind(&SafetyGate::bodyRateThrustCallback, this, std::placeholders::_1));

  sub_attitude_thrust_ = this->create_subscription<common_msgs::msg::AttitudeThrust>(
      "/cmd/attitude_thrust", 10,
      std::bind(&SafetyGate::attitudeThrustCallback, this, std::placeholders::_1));

  // Create timeout timer
  timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timeout_ms_ / 2),
      std::bind(&SafetyGate::checkTimeout, this));

  last_command_time_ = this->now();
}

void SafetyGate::bodyRateThrustCallback(const common_msgs::msg::BodyRateThrust::SharedPtr msg) {
  last_command_time_ = this->now();

  // Safety checks
  if (!checkBodyRates(msg->body_rates)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Body rates exceed limits, clamping");
  }

  // Create output message
  auto out_msg = *msg;

  // Clamp body rates
  out_msg.body_rates.x = std::clamp(msg->body_rates.x, -max_body_rate_, max_body_rate_);
  out_msg.body_rates.y = std::clamp(msg->body_rates.y, -max_body_rate_, max_body_rate_);
  out_msg.body_rates.z = std::clamp(msg->body_rates.z, -max_body_rate_, max_body_rate_);

  // Clamp and apply slew rate to thrust
  float clamped_thrust = clampThrust(msg->thrust);
  double dt = initialized_ ? 0.01 : 0.0;  // Assume ~100Hz if initialized
  out_msg.thrust = applySlewRate(last_thrust_, clamped_thrust, dt, slew_rate_thrust_);
  last_thrust_ = out_msg.thrust;

  // Apply slew rate to yaw
  out_msg.body_rates.z = applySlewRate(last_yaw_rate_, out_msg.body_rates.z, dt, slew_rate_yaw_);
  last_yaw_rate_ = out_msg.body_rates.z;

  initialized_ = true;

  // Publish based on mode
  if (mode_ == "sim") {
    pub_body_rate_thrust_->publish(out_msg);
  } else if (mode_ == "crsf") {
    auto rc_msg = bodyRateToRC(out_msg);
    pub_rc_->publish(rc_msg);
  }
}

void SafetyGate::attitudeThrustCallback(const common_msgs::msg::AttitudeThrust::SharedPtr msg) {
  last_command_time_ = this->now();

  // Convert attitude to body rates using simple P controller
  // This is a simplified approach - proper conversion would need current attitude
  // For now, just pass through with safe defaults
  auto brt_msg = std::make_shared<common_msgs::msg::BodyRateThrust>();
  brt_msg->header = msg->header;
  brt_msg->body_rates.x = 0.0;  // Placeholder
  brt_msg->body_rates.y = 0.0;
  brt_msg->body_rates.z = 0.0;
  brt_msg->thrust = msg->thrust;

  bodyRateThrustCallback(brt_msg);
}

void SafetyGate::checkTimeout() {
  auto elapsed = (this->now() - last_command_time_).seconds() * 1000.0;  // Convert to ms
  if (elapsed > timeout_ms_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Command timeout (%.0f ms), sending safe idle", elapsed);

    if (mode_ == "sim") {
      pub_body_rate_thrust_->publish(getSafeIdleBodyRate());
    } else if (mode_ == "crsf") {
      pub_rc_->publish(getSafeIdleRC());
    }

    last_thrust_ = 0.0f;
    last_yaw_rate_ = 0.0f;
  }
}

bool SafetyGate::checkBodyRates(const geometry_msgs::msg::Vector3& rates) {
  return std::abs(rates.x) <= max_body_rate_ &&
         std::abs(rates.y) <= max_body_rate_ &&
         std::abs(rates.z) <= max_body_rate_;
}

bool SafetyGate::checkThrust(float thrust) {
  return thrust >= thrust_min_ && thrust <= thrust_max_;
}

float SafetyGate::clampThrust(float thrust) {
  return std::clamp(thrust, static_cast<float>(thrust_min_), static_cast<float>(thrust_max_));
}

float SafetyGate::applySlewRate(float current, float target, float dt, float slew_rate) {
  if (dt <= 0.0f) {
    return target;
  }
  float max_change = slew_rate * dt;
  float error = target - current;
  return current + std::clamp(error, -max_change, max_change);
}

common_msgs::msg::VirtualRC SafetyGate::bodyRateToRC(
    const common_msgs::msg::BodyRateThrust& brt) {
  common_msgs::msg::VirtualRC rc;
  rc.header = brt.header;

  // Map body rates to RC channels [-1, 1]
  // Scale by max_body_rate to get normalized values
  rc.roll = std::clamp(static_cast<float>(brt.body_rates.x / max_body_rate_), -1.0f, 1.0f);
  rc.pitch = std::clamp(static_cast<float>(brt.body_rates.y / max_body_rate_), -1.0f, 1.0f);
  rc.yaw = std::clamp(static_cast<float>(brt.body_rates.z / max_body_rate_), -1.0f, 1.0f);
  rc.throttle = brt.thrust;  // Already in [0, 1]

  // Initialize aux channels to safe defaults
  for (int i = 0; i < 12; i++) {
    rc.aux[i] = 0.0f;  // Centered or disarmed
  }

  return rc;
}

common_msgs::msg::VirtualRC SafetyGate::getSafeIdleRC() {
  common_msgs::msg::VirtualRC rc;
  rc.header.stamp = this->now();
  rc.roll = 0.0f;
  rc.pitch = 0.0f;
  rc.yaw = 0.0f;
  rc.throttle = 0.0f;

  for (int i = 0; i < 12; i++) {
    rc.aux[i] = 0.0f;  // Disarmed
  }

  return rc;
}

common_msgs::msg::BodyRateThrust SafetyGate::getSafeIdleBodyRate() {
  common_msgs::msg::BodyRateThrust brt;
  brt.header.stamp = this->now();
  brt.body_rates.x = 0.0;
  brt.body_rates.y = 0.0;
  brt.body_rates.z = 0.0;
  brt.thrust = 0.0f;
  return brt;
}

}  // namespace safety_gate

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_gate::SafetyGate>());
  rclcpp::shutdown();
  return 0;
}

