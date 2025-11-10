#include "controllers_lqr/lqr.hpp"

#include <cmath>

namespace controllers_lqr {

LQRNode::LQRNode()
    : Node("lqr_controller"), has_attitude_(false), has_angular_velocity_(false) {
  // Declare parameters
  this->declare_parameter<double>("control_rate", 100.0);
  this->declare_parameter<double>("mass", 1.0);
  this->declare_parameter<double>("Jx", 0.01);
  this->declare_parameter<double>("Jy", 0.01);
  this->declare_parameter<double>("Jz", 0.02);
  this->declare_parameter<double>("hover_thrust", 0.5);

  // Get parameters
  control_rate_ = this->get_parameter("control_rate").as_double();
  mass_ = this->get_parameter("mass").as_double();
  Jx_ = this->get_parameter("Jx").as_double();
  Jy_ = this->get_parameter("Jy").as_double();
  Jz_ = this->get_parameter("Jz").as_double();
  hover_thrust_ = this->get_parameter("hover_thrust").as_double();

  // Load gain matrix
  loadGainMatrix();

  // Initialize LQR core
  lqr_core_ = std::make_shared<LQRCore>();
  lqr_core_->setParameters(mass_, Jx_, Jy_, Jz_, hover_thrust_, K_);

  // Create publisher
  pub_cmd_ = this->create_publisher<common_msgs::msg::BodyRateThrust>("/cmd/body_rate_thrust", 10);

  // Create subscribers
  sub_attitude_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/state/attitude", 10,
      std::bind(&LQRNode::attitudeCallback, this, std::placeholders::_1));

  sub_angular_velocity_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/state/angular_velocity", 10,
      std::bind(&LQRNode::angularVelocityCallback, this, std::placeholders::_1));

  // Create control timer
  auto period_ms = static_cast<int>(1000.0 / control_rate_);
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                            std::bind(&LQRNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "LQR Controller initialized at %.1f Hz", control_rate_);
}

void LQRNode::attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
  current_attitude_ = msg->quaternion;
  has_attitude_ = true;
}

void LQRNode::angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  current_angular_velocity_ = msg->vector;
  has_angular_velocity_ = true;
}

void LQRNode::controlLoop() {
  if (!has_attitude_ || !has_angular_velocity_) {
    return;
  }

  // Convert quaternion to Euler angles (FRD convention)
  Eigen::Vector3d euler = quaternionToEuler(current_attitude_);

  // Build current state
  State current_state;
  current_state << euler(0), euler(1), euler(2),
      current_angular_velocity_.x, current_angular_velocity_.y, current_angular_velocity_.z;

  // Desired state (hover at zero attitude)
  State desired_state;
  desired_state.setZero();

  // Compute control
  Control u = lqr_core_->computeControl(current_state, desired_state);

  // Publish command
  auto cmd_msg = std::make_shared<common_msgs::msg::BodyRateThrust>();
  cmd_msg->header.stamp = this->now();
  cmd_msg->header.frame_id = "body";
  cmd_msg->body_rates.x = u(0);
  cmd_msg->body_rates.y = u(1);
  cmd_msg->body_rates.z = u(2);
  cmd_msg->thrust = u(3);

  pub_cmd_->publish(*cmd_msg);
}

void LQRNode::loadGainMatrix() {
  // Declare gain matrix parameters
  this->declare_parameter<std::vector<double>>("K", std::vector<double>(24, 0.0));

  auto K_vec = this->get_parameter("K").as_double_array();

  if (K_vec.size() == 24) {
    // K is 4x6, stored row-major
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 6; j++) {
        K_(i, j) = K_vec[i * 6 + j];
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded custom K matrix");
  } else {
    // Use default conservative gains
    K_.setZero();
    // Roll attitude -> roll rate
    K_(0, 0) = 6.0;  // p from phi
    K_(0, 3) = 1.0;  // p from p
    // Pitch attitude -> pitch rate
    K_(1, 1) = 6.0;  // q from theta
    K_(1, 4) = 1.0;  // q from q
    // Yaw attitude -> yaw rate
    K_(2, 2) = 3.0;  // r from psi
    K_(2, 5) = 0.8;  // r from r
    // Thrust (no feedback for this simple version)
    K_(3, 0) = 0.0;
    K_(3, 1) = 0.0;

    RCLCPP_WARN(this->get_logger(), "Using default K matrix (not from LQR solution)");
  }
}

Eigen::Vector3d LQRNode::quaternionToEuler(const geometry_msgs::msg::Quaternion& q) {
  // Convert quaternion to Euler angles (roll, pitch, yaw) in FRD convention
  double w = q.w;
  double x = q.x;
  double y = q.y;
  double z = q.z;

  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace controllers_lqr

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controllers_lqr::LQRNode>());
  rclcpp::shutdown();
  return 0;
}

