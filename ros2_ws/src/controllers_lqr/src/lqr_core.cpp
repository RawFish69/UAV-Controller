#include "controllers_lqr/lqr.hpp"

#include <cmath>

namespace controllers_lqr {

LQRCore::LQRCore()
    : mass_(1.0), Jx_(0.01), Jy_(0.01), Jz_(0.02), hover_thrust_(0.5) {
  K_.setZero();
  current_state_.setZero();
}

void LQRCore::setParameters(double mass, double Jx, double Jy, double Jz, double hover_thrust,
                             const GainMatrix& K) {
  mass_ = mass;
  Jx_ = Jx;
  Jy_ = Jy;
  Jz_ = Jz;
  hover_thrust_ = hover_thrust;
  K_ = K;
}

Control LQRCore::computeControl(const State& current_state, const State& desired_state) {
  current_state_ = current_state;

  // Compute state error
  State error = desired_state - current_state;

  // Wrap angle errors to [-pi, pi]
  for (int i = 0; i < 3; i++) {
    while (error(i) > M_PI) error(i) -= 2 * M_PI;
    while (error(i) < -M_PI) error(i) += 2 * M_PI;
  }

  // Compute control: u = -K * error
  Control u = -K_ * error;

  // Add hover thrust offset
  u(3) += hover_thrust_;

  // Clamp outputs
  u(0) = std::clamp(u(0), -10.0, 10.0);  // p (roll rate)
  u(1) = std::clamp(u(1), -10.0, 10.0);  // q (pitch rate)
  u(2) = std::clamp(u(2), -6.0, 6.0);    // r (yaw rate)
  u(3) = std::clamp(u(3), 0.0, 1.0);     // thrust

  return u;
}

bool LQRCore::isStable() const {
  // Simple stability check: bounded state
  return current_state_.norm() < 100.0;  // Arbitrary but reasonable bound
}

}  // namespace controllers_lqr

