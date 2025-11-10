#include "controllers_pid/pid.hpp"

#include <cmath>

namespace controllers_pid {

PIDNode::PIDNode()
    : Node("pid_controller"), has_attitude_(false), has_angular_velocity_(false), 
      has_attitude_setpoint_(false), desired_thrust_(0.5) {
  
  // Set default desired attitude (hover)
  desired_attitude_.w = 1.0;
  desired_attitude_.x = 0.0;
  desired_attitude_.y = 0.0;
  desired_attitude_.z = 0.0;
  // Declare parameters
  this->declare_parameter<double>("control_rate", 100.0);
  this->declare_parameter<bool>("use_attitude_outer", true);
  this->declare_parameter<double>("hover_thrust", 0.5);
  this->declare_parameter<double>("k_aw", 0.3);
  this->declare_parameter<double>("k_att", 6.0);

  // Per-axis PID parameters
  this->declare_parameter<double>("kp.x", 8.0);
  this->declare_parameter<double>("kp.y", 8.0);
  this->declare_parameter<double>("kp.z", 3.0);
  this->declare_parameter<double>("ki.x", 0.6);
  this->declare_parameter<double>("ki.y", 0.6);
  this->declare_parameter<double>("ki.z", 0.3);
  this->declare_parameter<double>("kd.x", 0.05);
  this->declare_parameter<double>("kd.y", 0.05);
  this->declare_parameter<double>("kd.z", 0.02);
  this->declare_parameter<double>("kff.x", 0.0);
  this->declare_parameter<double>("kff.y", 0.0);
  this->declare_parameter<double>("kff.z", 0.0);
  this->declare_parameter<double>("tau_d.x", 0.02);
  this->declare_parameter<double>("tau_d.y", 0.02);
  this->declare_parameter<double>("tau_d.z", 0.02);
  this->declare_parameter<double>("i_clamp.x", 2.0);
  this->declare_parameter<double>("i_clamp.y", 2.0);
  this->declare_parameter<double>("i_clamp.z", 1.0);
  this->declare_parameter<double>("rate_limits.x", 10.0);
  this->declare_parameter<double>("rate_limits.y", 10.0);
  this->declare_parameter<double>("rate_limits.z", 6.0);

  // Load parameters
  loadParams();

  // Initialize PID core
  pid_core_ = std::make_shared<PIDCore>();

  PIDParams roll_params, pitch_params, yaw_params;

  roll_params.kp = this->get_parameter("kp.x").as_double();
  roll_params.ki = this->get_parameter("ki.x").as_double();
  roll_params.kd = this->get_parameter("kd.x").as_double();
  roll_params.kff = this->get_parameter("kff.x").as_double();
  roll_params.tau_d = this->get_parameter("tau_d.x").as_double();
  roll_params.i_clamp = this->get_parameter("i_clamp.x").as_double();
  roll_params.rate_limit = this->get_parameter("rate_limits.x").as_double();

  pitch_params.kp = this->get_parameter("kp.y").as_double();
  pitch_params.ki = this->get_parameter("ki.y").as_double();
  pitch_params.kd = this->get_parameter("kd.y").as_double();
  pitch_params.kff = this->get_parameter("kff.y").as_double();
  pitch_params.tau_d = this->get_parameter("tau_d.y").as_double();
  pitch_params.i_clamp = this->get_parameter("i_clamp.y").as_double();
  pitch_params.rate_limit = this->get_parameter("rate_limits.y").as_double();

  yaw_params.kp = this->get_parameter("kp.z").as_double();
  yaw_params.ki = this->get_parameter("ki.z").as_double();
  yaw_params.kd = this->get_parameter("kd.z").as_double();
  yaw_params.kff = this->get_parameter("kff.z").as_double();
  yaw_params.tau_d = this->get_parameter("tau_d.z").as_double();
  yaw_params.i_clamp = this->get_parameter("i_clamp.z").as_double();
  yaw_params.rate_limit = this->get_parameter("rate_limits.z").as_double();

  double hover_thrust = this->get_parameter("hover_thrust").as_double();
  double k_aw = this->get_parameter("k_aw").as_double();
  use_attitude_outer_ = this->get_parameter("use_attitude_outer").as_bool();
  double k_att = this->get_parameter("k_att").as_double();

  pid_core_->setParams(roll_params, pitch_params, yaw_params, hover_thrust, k_aw,
                       use_attitude_outer_, k_att);

  // Create publisher
  pub_cmd_ = this->create_publisher<common_msgs::msg::BodyRateThrust>("/cmd/body_rate_thrust", 10);

  // Create subscribers
  sub_attitude_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/state/attitude", 10,
      std::bind(&PIDNode::attitudeCallback, this, std::placeholders::_1));

  sub_angular_velocity_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/state/angular_velocity", 10,
      std::bind(&PIDNode::angularVelocityCallback, this, std::placeholders::_1));

  // Subscribe to attitude setpoints (for manual IMU control)
  sub_attitude_setpoint_ = this->create_subscription<common_msgs::msg::AttitudeThrust>(
      "/cmd/attitude_thrust", 10,
      [this](const common_msgs::msg::AttitudeThrust::SharedPtr msg) {
        desired_attitude_ = msg->attitude;
        desired_thrust_ = msg->thrust;
        has_attitude_setpoint_ = true;
      });

  // Create control timer
  control_rate_ = this->get_parameter("control_rate").as_double();
  auto period_ms = static_cast<int>(1000.0 / control_rate_);
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                            std::bind(&PIDNode::controlLoop, this));

  last_control_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "PID Controller initialized at %.1f Hz (attitude outer: %s)",
              control_rate_, use_attitude_outer_ ? "enabled" : "disabled");
}

void PIDNode::attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
  current_attitude_ = msg->quaternion;
  has_attitude_ = true;
}

void PIDNode::angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  current_angular_velocity_ = msg->vector;
  has_angular_velocity_ = true;
}

void PIDNode::controlLoop() {
  if (!has_angular_velocity_) {
    return;
  }

  // If using attitude outer loop, need attitude feedback
  if (use_attitude_outer_ && !has_attitude_) {
    return;
  }

  // Compute dt
  auto now = this->now();
  double dt = (now - last_control_time_).seconds();
  last_control_time_ = now;

  if (dt <= 0.0 || dt > 1.0) {
    dt = 1.0 / control_rate_;  // Use nominal dt on first iteration or invalid dt
  }

  Eigen::Vector3d rate_measured;
  rate_measured << current_angular_velocity_.x, current_angular_velocity_.y,
      current_angular_velocity_.z;

  Eigen::Vector4d control_output;

  if (use_attitude_outer_) {
    // Cascaded attitude + rate control
    Eigen::Vector4d attitude_setpoint;
    attitude_setpoint << desired_attitude_.w, desired_attitude_.x, 
                         desired_attitude_.y, desired_attitude_.z;

    Eigen::Vector4d attitude_measured;
    attitude_measured << current_attitude_.w, current_attitude_.x, current_attitude_.y,
        current_attitude_.z;

    control_output =
        pid_core_->computeControlWithAttitude(attitude_setpoint, attitude_measured, rate_measured, dt);
    
    // Use commanded thrust (from manual controller or hover default)
    if (has_attitude_setpoint_) {
      control_output(3) = desired_thrust_;
    }
  } else {
    // Rate-only control (setpoint = 0)
    Eigen::Vector3d rate_setpoint;
    rate_setpoint.setZero();

    control_output = pid_core_->computeControl(rate_setpoint, rate_measured, dt);
  }

  // Publish command
  auto cmd_msg = std::make_shared<common_msgs::msg::BodyRateThrust>();
  cmd_msg->header.stamp = now;
  cmd_msg->header.frame_id = "body";
  cmd_msg->body_rates.x = control_output(0);
  cmd_msg->body_rates.y = control_output(1);
  cmd_msg->body_rates.z = control_output(2);
  cmd_msg->thrust = control_output(3);

  pub_cmd_->publish(*cmd_msg);
}

void PIDNode::loadParams() {
  control_rate_ = this->get_parameter("control_rate").as_double();
}

}  // namespace controllers_pid

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controllers_pid::PIDNode>());
  rclcpp::shutdown();
  return 0;
}

