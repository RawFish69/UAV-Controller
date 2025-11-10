#include <gtest/gtest.h>

#include "controllers_pid/pid.hpp"

using namespace controllers_pid;

TEST(PIDTest, AntiWindup) {
  PIDAxis pid;

  PIDParams params;
  params.kp = 5.0;
  params.ki = 1.0;
  params.kd = 0.1;
  params.kff = 0.0;
  params.tau_d = 0.01;
  params.i_clamp = 2.0;
  params.rate_limit = 10.0;

  pid.setParams(params);

  // Apply large sustained error (should saturate)
  double setpoint = 100.0;
  double measured = 0.0;
  double dt = 0.01;

  for (int i = 0; i < 200; i++) {
    double output = pid.update(setpoint, measured, dt);
    
    // Output should be clamped
    EXPECT_LE(output, params.rate_limit);
    EXPECT_GE(output, -params.rate_limit);
  }

  // Now remove error - output should return to reasonable values quickly
  // (anti-windup should prevent excessive integral buildup)
  setpoint = 0.0;
  measured = 0.0;

  for (int i = 0; i < 50; i++) {
    double output = pid.update(setpoint, measured, dt);
    
    // Output should be small
    if (i > 10) {
      EXPECT_LT(std::abs(output), 5.0);  // Should settle reasonably fast
    }
  }
}

TEST(PIDTest, DerivativeFilter) {
  PIDAxis pid;

  PIDParams params;
  params.kp = 1.0;
  params.ki = 0.0;
  params.kd = 1.0;
  params.kff = 0.0;
  params.tau_d = 0.05;  // 50ms filter
  params.i_clamp = 10.0;
  params.rate_limit = 100.0;

  pid.setParams(params);

  // Apply step in measurement (should see filtered derivative)
  double setpoint = 0.0;
  double measured = 0.0;
  double dt = 0.01;

  // Step
  measured = 1.0;
  double output1 = pid.update(setpoint, measured, dt);

  // Continue
  for (int i = 0; i < 10; i++) {
    pid.update(setpoint, measured, dt);
  }

  // Another step
  measured = 2.0;
  double output2 = pid.update(setpoint, measured, dt);

  // Both outputs should be finite and reasonable
  EXPECT_TRUE(std::isfinite(output1));
  EXPECT_TRUE(std::isfinite(output2));
  EXPECT_LT(std::abs(output1), 50.0);
  EXPECT_LT(std::abs(output2), 50.0);
}

TEST(PIDTest, OutputLimits) {
  PIDCore pid;

  PIDParams roll_params;
  roll_params.kp = 8.0;
  roll_params.ki = 0.6;
  roll_params.kd = 0.05;
  roll_params.kff = 0.0;
  roll_params.tau_d = 0.02;
  roll_params.i_clamp = 2.0;
  roll_params.rate_limit = 10.0;

  pid.setParams(roll_params, roll_params, roll_params, 0.5, 0.3, false, 6.0);

  Eigen::Vector3d rate_setpoint;
  rate_setpoint << 20.0, 20.0, 10.0;  // Excessive setpoints

  Eigen::Vector3d rate_measured;
  rate_measured.setZero();

  double dt = 0.01;

  for (int i = 0; i < 100; i++) {
    Eigen::Vector4d output = pid.computeControl(rate_setpoint, rate_measured, dt);

    // Check outputs are within limits
    EXPECT_LE(output(0), 10.0);
    EXPECT_GE(output(0), -10.0);
    EXPECT_LE(output(1), 10.0);
    EXPECT_GE(output(1), -10.0);
    EXPECT_LE(output(2), 10.0);
    EXPECT_GE(output(2), -10.0);
    EXPECT_LE(output(3), 1.0);
    EXPECT_GE(output(3), 0.0);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

