#include "controllers_pid/pid.hpp"

#include <algorithm>
#include <cmath>

namespace controllers_pid {

PIDAxis::PIDAxis() : integral_(0.0), derivative_filtered_(0.0), last_measured_(0.0), initialized_(false) {}

void PIDAxis::setParams(const PIDParams& params) {
  params_ = params;
}

double PIDAxis::update(double setpoint, double measured, double dt) {
  if (dt <= 0.0) {
    return 0.0;
  }

  double error = setpoint - measured;

  // Proportional term
  double p_term = params_.kp * error;

  // Integral term with clamping
  integral_ += error * dt;
  integral_ = std::clamp(integral_, -params_.i_clamp, params_.i_clamp);
  double i_term = params_.ki * integral_;

  // Derivative term (on measurement, not error, to avoid derivative kick)
  double derivative = 0.0;
  if (initialized_) {
    derivative = (measured - last_measured_) / dt;
  }
  last_measured_ = measured;
  initialized_ = true;

  // Low-pass filter on derivative
  double alpha = dt / (params_.tau_d + dt);
  derivative_filtered_ = alpha * derivative + (1.0 - alpha) * derivative_filtered_;
  double d_term = -params_.kd * derivative_filtered_;  // Negative because derivative of measurement

  // Feed-forward term
  double ff_term = params_.kff * setpoint;

  // Total output
  double output = p_term + i_term + d_term + ff_term;

  // Apply rate limit
  output = std::clamp(output, -params_.rate_limit, params_.rate_limit);

  // Anti-windup: back-calculation
  // If output is saturated, reduce integral
  double output_unsaturated = p_term + i_term + d_term + ff_term;
  if (std::abs(output_unsaturated) > params_.rate_limit) {
    double excess = output_unsaturated - output;
    integral_ -= excess * 0.3 * dt;  // Back-calculation with fixed k_aw for simplicity
  }

  return output;
}

void PIDAxis::reset() {
  integral_ = 0.0;
  derivative_filtered_ = 0.0;
  last_measured_ = 0.0;
  initialized_ = false;
}

PIDCore::PIDCore() : hover_thrust_(0.5), k_aw_(0.3), use_attitude_outer_(false), k_att_(6.0) {}

void PIDCore::setParams(const PIDParams& roll, const PIDParams& pitch, const PIDParams& yaw,
                         double hover_thrust, double k_aw, bool use_attitude_outer, double k_att) {
  roll_pid_.setParams(roll);
  pitch_pid_.setParams(pitch);
  yaw_pid_.setParams(yaw);
  hover_thrust_ = hover_thrust;
  k_aw_ = k_aw;
  use_attitude_outer_ = use_attitude_outer;
  k_att_ = k_att;
}

Eigen::Vector4d PIDCore::computeControl(const Eigen::Vector3d& rate_setpoint,
                                         const Eigen::Vector3d& rate_measured, double dt) {
  Eigen::Vector4d output;

  output(0) = roll_pid_.update(rate_setpoint(0), rate_measured(0), dt);
  output(1) = pitch_pid_.update(rate_setpoint(1), rate_measured(1), dt);
  output(2) = yaw_pid_.update(rate_setpoint(2), rate_measured(2), dt);
  output(3) = hover_thrust_;

  return output;
}

Eigen::Vector4d PIDCore::computeControlWithAttitude(const Eigen::Vector4d& attitude_setpoint,
                                                     const Eigen::Vector4d& attitude_measured,
                                                     const Eigen::Vector3d& rate_measured,
                                                     double dt) {
  // Compute attitude error
  Eigen::Vector3d att_error = attitudeError(attitude_setpoint, attitude_measured);

  // Outer loop: attitude P controller generates rate setpoint
  Eigen::Vector3d rate_setpoint = k_att_ * att_error;

  // Inner loop: rate PID
  return computeControl(rate_setpoint, rate_measured, dt);
}

void PIDCore::reset() {
  roll_pid_.reset();
  pitch_pid_.reset();
  yaw_pid_.reset();
}

Eigen::Vector3d PIDCore::attitudeError(const Eigen::Vector4d& desired,
                                        const Eigen::Vector4d& current) {
  // Quaternion error: q_error = q_desired * q_current^-1
  // Then convert to small-angle approximation

  // q_current^-1 (conjugate for unit quaternion)
  Eigen::Vector4d current_inv;
  current_inv << current(0), -current(1), -current(2), -current(3);  // [w, -x, -y, -z]

  // q_error = desired * current_inv
  double w1 = desired(0), x1 = desired(1), y1 = desired(2), z1 = desired(3);
  double w2 = current_inv(0), x2 = current_inv(1), y2 = current_inv(2), z2 = current_inv(3);

  double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  double y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  double z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

  // Small-angle approximation: error ≈ 2 * [x, y, z] (vector part)
  Eigen::Vector3d error;
  error << 2.0 * x, 2.0 * y, 2.0 * z;

  return error;
}

}  // namespace controllers_pid

