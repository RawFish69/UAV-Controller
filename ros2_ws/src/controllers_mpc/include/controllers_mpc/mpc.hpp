#ifndef CONTROLLERS_MPC__MPC_HPP_
#define CONTROLLERS_MPC__MPC_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "common_msgs/msg/body_rate_thrust.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controllers_mpc {

// State: x = [phi, phi_dot, theta, theta_dot, psi, psi_dot, U2, U3, U4] (augmented state)
using AugmentedState = Eigen::Matrix<double, 9, 1>;
// Control: u = [dU2, dU3, dU4] (control increments)
using ControlIncrement = Eigen::Matrix<double, 3, 1>;
// Full control: [U1, U2, U3, U4]
using Control = Eigen::Matrix<double, 4, 1>;
// Reference: [phi_ref, theta_ref, psi_ref]
using Reference = Eigen::Matrix<double, 3, 1>;

class MPCCore {
public:
  MPCCore();
  void setParameters(double mass, double Jx, double Jy, double Jz, double hover_thrust,
                     double Ts, int horizon, const Eigen::Matrix3d& Q, 
                     const Eigen::Matrix3d& S, const Eigen::Matrix3d& R);
  
  // Compute MPC control given current state and reference
  ControlIncrement computeControl(const AugmentedState& current_state, 
                                   const Reference& reference);
  
  // Update the linearized model matrices (LPV approach)
  void updateModel(const Eigen::Vector3d& euler_angles,
                   const Eigen::Vector3d& angular_velocity,
                   double omega_total);
  
  AugmentedState getCurrentState() const { return current_state_; }
  void setCurrentState(const AugmentedState& state) { current_state_ = state; }

private:
  // Build MPC matrices (Hdb, Fdbt) from linearized model
  void buildMPCMatrices();
  
  // Solve QP problem: min 0.5 * du^T * Hdb * du + ft^T * du
  ControlIncrement solveQP(const Eigen::MatrixXd& Hdb, const Eigen::VectorXd& ft);
  
  // Model parameters
  double mass_;
  double Jx_, Jy_, Jz_;
  double Jtp_;  // Propeller moment of inertia
  double hover_thrust_;
  double Ts_;   // Sampling time
  int horizon_; // Prediction horizon
  
  // Cost matrices
  Eigen::Matrix3d Q_;  // State cost
  Eigen::Matrix3d S_;  // Terminal state cost
  Eigen::Matrix3d R_;  // Control cost
  
  // Linearized model matrices (discrete-time)
  Eigen::Matrix<double, 6, 6> Ad_;  // State matrix (6x6 for attitude dynamics)
  Eigen::Matrix<double, 6, 3> Bd_;  // Input matrix
  Eigen::Matrix<double, 3, 6> Cd_;  // Output matrix
  
  // Augmented model matrices
  Eigen::Matrix<double, 9, 9> A_aug_;  // Augmented state matrix
  Eigen::Matrix<double, 9, 3> B_aug_;  // Augmented input matrix
  Eigen::Matrix<double, 3, 9> C_aug_;  // Augmented output matrix
  
  // MPC matrices
  Eigen::MatrixXd Hdb_;   // Hessian matrix
  Eigen::MatrixXd Fdbt_;  // Gradient matrix
  
  // Current state
  AugmentedState current_state_;
  
  // Model update flag
  bool model_updated_;
};

class MPCNode : public rclcpp::Node {
public:
  MPCNode();

private:
  void attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
  void angularVelocityCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void controlLoop();
  void loadParameters();
  Eigen::Vector3d quaternionToEuler(const geometry_msgs::msg::Quaternion& q);
  
  std::shared_ptr<MPCCore> mpc_core_;
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
  double Jtp_;
  double hover_thrust_;
  double Ts_;
  int horizon_;
  Eigen::Matrix3d Q_, S_, R_;
  
  // Current control values (for incremental MPC)
  Eigen::Vector3d current_U_;  // [U2, U3, U4]
  double omega_total_;  // For LPV model update
};

}  // namespace controllers_mpc

#endif  // CONTROLLERS_MPC__MPC_HPP_
