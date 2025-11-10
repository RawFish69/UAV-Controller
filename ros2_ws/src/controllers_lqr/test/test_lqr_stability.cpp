#include <gtest/gtest.h>

#include "controllers_lqr/lqr.hpp"

using namespace controllers_lqr;

TEST(LQRTest, ClosedLoopStability) {
  LQRCore lqr;

  // Set up with default conservative gains
  GainMatrix K;
  K.setZero();
  K(0, 0) = 6.0;
  K(0, 3) = 1.0;
  K(1, 1) = 6.0;
  K(1, 4) = 1.0;
  K(2, 2) = 3.0;
  K(2, 5) = 0.8;

  lqr.setParameters(1.0, 0.01, 0.01, 0.02, 0.5, K);

  // Initial perturbed state
  State current_state;
  current_state << 0.1, 0.1, 0.0, 0.0, 0.0, 0.0;  // 0.1 rad tilt

  State desired_state;
  desired_state.setZero();

  // Run a few control iterations
  for (int i = 0; i < 100; i++) {
    Control u = lqr.computeControl(current_state, desired_state);

    // Verify outputs are bounded
    EXPECT_GE(u(0), -10.0);
    EXPECT_LE(u(0), 10.0);
    EXPECT_GE(u(1), -10.0);
    EXPECT_LE(u(1), 10.0);
    EXPECT_GE(u(2), -6.0);
    EXPECT_LE(u(2), 6.0);
    EXPECT_GE(u(3), 0.0);
    EXPECT_LE(u(3), 1.0);

    // Simple Euler integration (for test only)
    double dt = 0.01;
    current_state(0) += current_state(3) * dt;  // phi += p * dt
    current_state(1) += current_state(4) * dt;  // theta += q * dt
    current_state(2) += current_state(5) * dt;  // psi += r * dt
    current_state(3) += (u(0) - current_state(3)) * dt * 5.0;  // p dynamics
    current_state(4) += (u(1) - current_state(4)) * dt * 5.0;  // q dynamics
    current_state(5) += (u(2) - current_state(5)) * dt * 5.0;  // r dynamics
  }

  // After 100 iterations, state should be smaller (converging)
  EXPECT_LT(current_state.head<3>().norm(), 0.2);  // Angles reduced
  EXPECT_TRUE(lqr.isStable());
}

TEST(LQRTest, StepResponse) {
  LQRCore lqr;

  GainMatrix K;
  K.setZero();
  K(0, 0) = 6.0;
  K(0, 3) = 1.0;

  lqr.setParameters(1.0, 0.01, 0.01, 0.02, 0.5, K);

  State current_state;
  current_state.setZero();

  State desired_state;
  desired_state << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0;  // Step in roll

  Control u = lqr.computeControl(current_state, desired_state);

  // Should command positive roll rate to track step
  EXPECT_GT(u(0), 0.0);
  EXPECT_LT(u(0), 10.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

