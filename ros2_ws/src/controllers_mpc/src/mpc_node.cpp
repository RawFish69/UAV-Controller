#include "controllers_mpc/mpc.hpp"

#include <cmath>
#include <vector>

namespace controllers_mpc {

MPCNode::MPCNode()
    : Node("mpc_controller"), has_attitude_(false), has_angular_velocity_(false),
      omega_total_(0.0) {
  loadParameters();
  
  // Initialize MPC core
  mpc_core_ = std::make_shared<MPCCore>();
  mpc_core_->setParameters(mass_, Jx_, Jy_, Jz_, hover_thrust_, Ts_, horizon_, Q_, S_, R_);
  
  // Initialize current control
  current_U_.setZero();
  
  // Create publisher
  pub_cmd_ = this->create_publisher<common_msgs::msg::BodyRateThrust>("/cmd/body_rate_thrust", 10);
  
  // Create subscribers
  sub_attitude_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/state/attitude", 10,
      std::bind(&MPCNode::attitudeCallback, this, std::placeholders::_1));
  
  sub_angular_velocity_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/state/angular_velocity", 10,
      std::bind(&MPCNode::angularVelocityCallback, this, std::placeholders::_1));
  
  // Create control timer
  auto period_ms = static_cast<int>(1000.0 / control_rate_);
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                            std::bind(&MPCNode::controlLoop, this));
  
  RCLCPP_INFO(this->get_logger(), "MPC Controller initialized at %.1f Hz", control_rate_);
  RCLCPP_INFO(this->get_logger(), "Horizon: %d, Ts: %.3f s", horizon_, Ts_);
}

void MPCNode::attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
  current_attitude_ = msg->quaternion;
  has_attitude_ = true;
}

void MPCNode::angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  current_angular_velocity_ = msg->vector;
  has_angular_velocity_ = true;
}

void MPCNode::controlLoop() {
  if (!has_attitude_ || !has_angular_velocity_) {
    return;
  }
  
  // Convert quaternion to Euler angles (FRD convention)
  Eigen::Vector3d euler = quaternionToEuler(current_attitude_);
  
  // Get angular velocity
  Eigen::Vector3d angular_velocity(
      current_angular_velocity_.x,
      current_angular_velocity_.y,
      current_angular_velocity_.z
  );
  
  // Update LPV model
  mpc_core_->updateModel(euler, angular_velocity, omega_total_);
  
  // Build augmented state: [phi, phi_dot, theta, theta_dot, psi, psi_dot, U2, U3, U4]
  AugmentedState x_aug;
  
  // Euler angles
  x_aug(0) = euler(0);  // phi
  x_aug(2) = euler(1);  // theta
  x_aug(4) = euler(2);  // psi
  
  // Compute Euler angle rates using transformation matrix
  double phi = euler(0);
  double theta = euler(1);
  
  Eigen::Matrix3d T;
  T << 1.0, sin(phi) * tan(theta), cos(phi) * tan(theta),
       0.0, cos(phi), -sin(phi),
       0.0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  
  Eigen::Vector3d euler_dot = T * angular_velocity;
  x_aug(1) = euler_dot(0);  // phi_dot
  x_aug(3) = euler_dot(1);  // theta_dot
  x_aug(5) = euler_dot(2);  // psi_dot
  
  // Current control values (from previous iteration)
  x_aug(6) = current_U_(0);  // U2
  x_aug(7) = current_U_(1);  // U3
  x_aug(8) = current_U_(2);  // U4
  
  // Desired reference (hover at zero attitude for now)
  Reference ref;
  ref.setZero();
  
  // Compute MPC control
  ControlIncrement du = mpc_core_->computeControl(x_aug, ref);
  
  // Update control (incremental MPC)
  current_U_ += du;
  
  // Clamp control values
  current_U_(0) = std::clamp(current_U_(0), -50.0, 50.0);  // U2
  current_U_(1) = std::clamp(current_U_(1), -50.0, 50.0);  // U3
  current_U_(2) = std::clamp(current_U_(2), -20.0, 20.0);  // U4
  
  // Convert to body rates and thrust
  // For now, map U2, U3, U4 directly to body rates (simplified)
  // In full implementation, would use inverse dynamics
  double p_cmd = current_U_(0) / Jx_;  // Approximate mapping
  double q_cmd = current_U_(1) / Jy_;
  double r_cmd = current_U_(2) / Jz_;
  
  // Publish command
  auto cmd_msg = std::make_shared<common_msgs::msg::BodyRateThrust>();
  cmd_msg->header.stamp = this->now();
  cmd_msg->header.frame_id = "body";
  cmd_msg->body_rates.x = p_cmd;
  cmd_msg->body_rates.y = q_cmd;
  cmd_msg->body_rates.z = r_cmd;
  cmd_msg->thrust = hover_thrust_;  // U1 handled separately in position controller
  
  pub_cmd_->publish(*cmd_msg);
}

void MPCNode::loadParameters() {
  // Declare parameters
  this->declare_parameter<double>("control_rate", 100.0);
  this->declare_parameter<double>("mass", 1.0);
  this->declare_parameter<double>("Jx", 0.01);
  this->declare_parameter<double>("Jy", 0.01);
  this->declare_parameter<double>("Jz", 0.02);
  this->declare_parameter<double>("Jtp", 1.302e-6);
  this->declare_parameter<double>("hover_thrust", 0.5);
  this->declare_parameter<double>("Ts", 0.1);
  this->declare_parameter<int>("horizon", 4);
  
  // Cost matrices (3x3)
  this->declare_parameter<std::vector<double>>("Q", std::vector<double>(9, 0.0));
  this->declare_parameter<std::vector<double>>("S", std::vector<double>(9, 0.0));
  this->declare_parameter<std::vector<double>>("R", std::vector<double>(9, 0.0));
  
  // Get parameters
  control_rate_ = this->get_parameter("control_rate").as_double();
  mass_ = this->get_parameter("mass").as_double();
  Jx_ = this->get_parameter("Jx").as_double();
  Jy_ = this->get_parameter("Jy").as_double();
  Jz_ = this->get_parameter("Jz").as_double();
  Jtp_ = this->get_parameter("Jtp").as_double();
  hover_thrust_ = this->get_parameter("hover_thrust").as_double();
  Ts_ = this->get_parameter("Ts").as_double();
  horizon_ = this->get_parameter("horizon").as_int();
  
  // Load cost matrices
  auto Q_vec = this->get_parameter("Q").as_double_array();
  auto S_vec = this->get_parameter("S").as_double_array();
  auto R_vec = this->get_parameter("R").as_double_array();
  
  if (Q_vec.size() == 9) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Q_(i, j) = Q_vec[i * 3 + j];
      }
    }
  } else {
    Q_.setIdentity();
    Q_ *= 10.0;
    RCLCPP_WARN(this->get_logger(), "Using default Q matrix");
  }
  
  if (S_vec.size() == 9) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        S_(i, j) = S_vec[i * 3 + j];
      }
    }
  } else {
    S_.setIdentity();
    S_ *= 10.0;
    RCLCPP_WARN(this->get_logger(), "Using default S matrix");
  }
  
  if (R_vec.size() == 9) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R_(i, j) = R_vec[i * 3 + j];
      }
    }
  } else {
    R_.setIdentity();
    R_ *= 10.0;
    RCLCPP_WARN(this->get_logger(), "Using default R matrix");
  }
}

Eigen::Vector3d MPCNode::quaternionToEuler(const geometry_msgs::msg::Quaternion& q) {
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
    pitch = std::copysign(M_PI / 2, sinp);
  else
    pitch = std::asin(sinp);
  
  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  
  return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace controllers_mpc

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controllers_mpc::MPCNode>());
  rclcpp::shutdown();
  return 0;
}
