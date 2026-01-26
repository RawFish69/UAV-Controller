#include "controllers_mpc/mpc.hpp"

#include <cmath>
#include <iostream>

namespace controllers_mpc {

MPCCore::MPCCore()
    : mass_(1.0), Jx_(0.01), Jy_(0.01), Jz_(0.02), Jtp_(1.302e-6),
      hover_thrust_(0.5), Ts_(0.1), horizon_(4), model_updated_(false) {
  Q_.setIdentity();
  S_.setIdentity();
  R_.setIdentity();
  current_state_.setZero();
  Ad_.setIdentity();
  Bd_.setZero();
  Cd_.setZero();
  A_aug_.setIdentity();
  B_aug_.setZero();
  C_aug_.setZero();
}

void MPCCore::setParameters(double mass, double Jx, double Jy, double Jz, 
                             double hover_thrust, double Ts, int horizon,
                             const Eigen::Matrix3d& Q, const Eigen::Matrix3d& S,
                             const Eigen::Matrix3d& R) {
  mass_ = mass;
  Jx_ = Jx;
  Jy_ = Jy;
  Jz_ = Jz;
  hover_thrust_ = hover_thrust;
  Ts_ = Ts;
  horizon_ = horizon;
  Q_ = Q;
  S_ = S;
  R_ = R;
}

void MPCCore::updateModel(const Eigen::Vector3d& euler_angles,
                           const Eigen::Vector3d& angular_velocity,
                           double omega_total) {
  double phi = euler_angles(0);
  double theta = euler_angles(1);
  
  double p = angular_velocity(0);
  double q = angular_velocity(1);
  double r = angular_velocity(2);
  
  // Compute transformation matrix T for Euler angle rates
  Eigen::Matrix3d T;
  T << 1.0, sin(phi) * tan(theta), cos(phi) * tan(theta),
       0.0, cos(phi), -sin(phi),
       0.0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  
  Eigen::Vector3d pqr(p, q, r);
  Eigen::Vector3d euler_dot = T * pqr;
  
  double phi_dot = euler_dot(0);
  double theta_dot = euler_dot(1);
  
  // Build continuous-time A matrix (LPV model)
  Eigen::Matrix<double, 6, 6> A;
  A.setZero();
  
  // State: [phi, phi_dot, theta, theta_dot, psi, psi_dot]
  A(0, 1) = 1.0;  // phi_dot = phi_dot
  A(1, 1) = -omega_total * Jtp_ / Jx_;  // phi_dot_dot
  A(1, 5) = theta_dot * (Jy_ - Jz_) / Jx_;
  A(2, 3) = 1.0;  // theta_dot = theta_dot
  A(3, 1) = omega_total * Jtp_ / Jy_;  // theta_dot_dot
  A(3, 5) = phi_dot * (Jz_ - Jx_) / Jy_;
  A(4, 5) = 1.0;  // psi_dot = psi_dot
  A(5, 1) = (theta_dot / 2.0) * (Jx_ - Jy_) / Jz_;
  A(5, 3) = (phi_dot / 2.0) * (Jx_ - Jy_) / Jz_;
  
  // B matrix
  Eigen::Matrix<double, 6, 3> B;
  B.setZero();
  B(1, 0) = 1.0 / Jx_;  // U2 -> phi_dot_dot
  B(3, 1) = 1.0 / Jy_;  // U3 -> theta_dot_dot
  B(5, 2) = 1.0 / Jz_;  // U4 -> psi_dot_dot
  
  // C matrix (output: phi, theta, psi)
  Eigen::Matrix<double, 3, 6> C;
  C.setZero();
  C(0, 0) = 1.0;  // phi
  C(1, 2) = 1.0;  // theta
  C(2, 4) = 1.0;  // psi
  
  // Discretize using forward Euler
  Ad_ = Eigen::Matrix<double, 6, 6>::Identity() + Ts_ * A;
  Bd_ = Ts_ * B;
  Cd_ = C;
  
  // Build augmented system (with integrator for incremental control)
  // Augmented state: [phi, phi_dot, theta, theta_dot, psi, psi_dot, U2, U3, U4]
  A_aug_.setZero();
  A_aug_.block<6, 6>(0, 0) = Ad_;
  A_aug_.block<6, 3>(0, 6) = Bd_;
  A_aug_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  
  B_aug_.setZero();
  B_aug_.block<6, 3>(0, 0) = Bd_;
  B_aug_.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();
  
  C_aug_.setZero();
  C_aug_.block<3, 6>(0, 0) = Cd_;
  
  model_updated_ = true;
}

void MPCCore::buildMPCMatrices() {
  if (!model_updated_) {
    return;
  }
  
  int nx = 9;  // Augmented state dimension
  int nu = 3;  // Control dimension
  int ny = 3;  // Output dimension
  int hz = horizon_;
  
  // Compute intermediate matrices
  Eigen::Matrix<double, 9, 9> CQC = C_aug_.transpose() * Q_ * C_aug_;
  Eigen::Matrix<double, 9, 9> CSC = C_aug_.transpose() * S_ * C_aug_;
  Eigen::Matrix<double, 3, 9> QC = Q_ * C_aug_;
  Eigen::Matrix<double, 3, 9> SC = S_ * C_aug_;
  
  // Initialize block matrices
  Eigen::MatrixXd Qdb = Eigen::MatrixXd::Zero(nx * hz, nx * hz);
  Eigen::MatrixXd Tdb = Eigen::MatrixXd::Zero(ny * hz, nx * hz);
  Eigen::MatrixXd Rdb = Eigen::MatrixXd::Zero(nu * hz, nu * hz);
  Eigen::MatrixXd Cdb = Eigen::MatrixXd::Zero(nx * hz, nu * hz);
  Eigen::MatrixXd Adc = Eigen::MatrixXd::Zero(nx * hz, nx);
  
  // Build block matrices
  for (int i = 0; i < hz; i++) {
    int Q_row_start = i * nx;
    int Q_col_start = i * nx;
    
    int T_row_start = i * ny;
    int T_col_start = i * nx;
    
    int R_row_start = i * nu;
    int R_col_start = i * nu;
    
    // Fill Qdb and Tdb
    if (i == hz - 1) {
      Qdb.block<9, 9>(Q_row_start, Q_col_start) = CSC;
      Tdb.block<3, 9>(T_row_start, T_col_start) = SC;
    } else {
      Qdb.block<9, 9>(Q_row_start, Q_col_start) = CQC;
      Tdb.block<3, 9>(T_row_start, T_col_start) = QC;
    }
    
    // Fill Rdb
    Rdb.block<3, 3>(R_row_start, R_col_start) = R_;
    
    // Fill Adc: A_aug^(i+1)
    Eigen::Matrix<double, 9, 9> A_aug_power = A_aug_;
    for (int pow = 0; pow < i; pow++) {
      A_aug_power = A_aug_power * A_aug_;
    }
    Adc.block(Q_row_start, 0, 9, 9) = A_aug_power;
    
    // Fill Cdb
    for (int j = 0; j <= i; j++) {
      Eigen::Matrix<double, 9, 9> A_aug_power_j = Eigen::Matrix<double, 9, 9>::Identity();
      for (int pow = 0; pow < (i - j); pow++) {
        A_aug_power_j = A_aug_power_j * A_aug_;
      }
      Cdb.block(Q_row_start, j * nu, 9, 3) = A_aug_power_j * B_aug_;
    }
  }
  
  // Compute Hdb and Fdbt
  Hdb_ = Cdb.transpose() * Qdb * Cdb + Rdb;
  // Fdbt is (nx + nu + ny * hz) x (nu * hz) = (9 + 3 * hz) x (3 * hz)
  Fdbt_ = Eigen::MatrixXd::Zero(9 + ny * hz, nu * hz);
  Fdbt_.block(0, 0, 9, nu * hz) = Adc.transpose() * Qdb * Cdb;
  Fdbt_.block(9, 0, ny * hz, nu * hz) = -Tdb * Cdb;
}

ControlIncrement MPCCore::computeControl(const AugmentedState& current_state,
                                          const Reference& reference) {
  current_state_ = current_state;
  
  // Build MPC matrices if model was updated
  if (model_updated_) {
    buildMPCMatrices();
    model_updated_ = false;
  }
  
  // Build reference vector for horizon
  Eigen::VectorXd r = Eigen::VectorXd::Zero(3 * horizon_);
  for (int i = 0; i < horizon_; i++) {
    r.segment<3>(i * 3) = reference;
  }
  
  // Build extended state vector [x_aug_t, r]
  Eigen::VectorXd x_aug_t_r(9 + 3 * horizon_);
  x_aug_t_r.head<9>() = current_state_;
  x_aug_t_r.tail(3 * horizon_) = r;
  
  // Compute gradient: ft = Fdbt^T * [x_aug_t; r]
  Eigen::VectorXd ft = Fdbt_.transpose() * x_aug_t_r;
  
  // Solve QP
  ControlIncrement du = solveQP(Hdb_, ft);
  
  return du;
}

ControlIncrement MPCCore::solveQP(const Eigen::MatrixXd& Hdb, 
                                    const Eigen::VectorXd& ft) {
  // Simple gradient descent solution (can be replaced with proper QP solver)
  // For now, use a simple iterative method
  // In production, use OSQP, qpOASES, or similar
  
  ControlIncrement du = Eigen::Vector3d::Zero();
  
  // Simple unconstrained solution: du = -Hdb^-1 * ft^T
  // For constrained case, would need proper QP solver
  try {
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Hdb);
    if (ldlt.info() == Eigen::Success) {
      Eigen::VectorXd du_full = -ldlt.solve(ft.transpose());
      du = du_full.head<3>();  // Take first control increment
    } else {
      // Fallback: use pseudo-inverse
      Eigen::VectorXd du_full = -(Hdb.completeOrthogonalDecomposition().solve(ft.transpose()));
      du = du_full.head<3>();
    }
  } catch (...) {
    // If solve fails, return zero
    du.setZero();
  }
  
  // Clamp to reasonable values
  du(0) = std::clamp(du(0), -10.0, 10.0);  // dU2
  du(1) = std::clamp(du(1), -10.0, 10.0);  // dU3
  du(2) = std::clamp(du(2), -6.0, 6.0);   // dU4
  
  return du;
}

}  // namespace controllers_mpc
