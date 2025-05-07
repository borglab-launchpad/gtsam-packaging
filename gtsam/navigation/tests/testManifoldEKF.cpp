/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

 /**
  * @file testManifoldEKF.cpp
  * @brief Unit test for the ManifoldEKF base class using Unit3.
  * @date April 26, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ManifoldEKF.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace gtsam;

// Define simple dynamics for Unit3: constant velocity in the tangent space
namespace exampleUnit3 {

  // Predicts the next state given current state, tangent velocity, and dt
  Unit3 predictNextState(const Unit3& p, const Vector2& v, double dt) {
    return p.retract(v * dt);
  }

  // Define a measurement model: measure the z-component of the Unit3 direction
  // H is the Jacobian dh/d(local(p))
  Vector1 measureZ(const Unit3& p, OptionalJacobian<1, 2> H) {
    if (H) {
      // H = d(p.point3().z()) / d(local(p))
      // Calculate numerically for simplicity in test
      auto h = [](const Unit3& p_) { return Vector1(p_.point3().z()); };
      *H = numericalDerivative11<Vector1, Unit3, 2>(h, p);
    }
    return Vector1(p.point3().z());
  }

} // namespace exampleUnit3

// Test fixture for ManifoldEKF with Unit3
struct Unit3EKFTest {
  Unit3 p0;
  Matrix2 P0;
  Vector2 velocity;
  double dt;
  Matrix2 Q; // Process noise
  Matrix1 R; // Measurement noise

  Unit3EKFTest() :
    p0(Unit3(Point3(1, 0, 0))), // Start pointing along X-axis
    P0(I_2x2 * 0.01),
    velocity((Vector2() << 0.0, M_PI / 4.0).finished()), // Rotate towards +Z axis
    dt(0.1),
    Q(I_2x2 * 0.001),
    R(Matrix1::Identity() * 0.01) {
  }
};


TEST(ManifoldEKF_Unit3, Predict) {
  Unit3EKFTest data;

  // Initialize the EKF
  ManifoldEKF<Unit3> ekf(data.p0, data.P0);

  // --- Prepare inputs for ManifoldEKF::predict ---
  // 1. Compute expected next state
  Unit3 p_next_expected = exampleUnit3::predictNextState(data.p0, data.velocity, data.dt);

  // 2. Compute state transition Jacobian F = d(local(p_next)) / d(local(p))
  //    We can compute this numerically using the predictNextState function.
  //    GTSAM's numericalDerivative handles derivatives *between* manifolds.
  auto predict_wrapper = [&](const Unit3& p) -> Unit3 {
    return exampleUnit3::predictNextState(p, data.velocity, data.dt);
    };
  Matrix2 F = numericalDerivative11<Unit3, Unit3>(predict_wrapper, data.p0);

  // --- Perform EKF prediction ---
  ekf.predict(p_next_expected, F, data.Q);

  // --- Verification ---
  // Check state
  EXPECT(assert_equal(p_next_expected, ekf.state(), 1e-8));

  // Check covariance
  Matrix2 P_expected = F * data.P0 * F.transpose() + data.Q;
  EXPECT(assert_equal(P_expected, ekf.covariance(), 1e-8));

  // Check F manually for a simple case (e.g., zero velocity should give Identity)
  Vector2 zero_velocity = Vector2::Zero();
  auto predict_wrapper_zero = [&](const Unit3& p) -> Unit3 {
    return exampleUnit3::predictNextState(p, zero_velocity, data.dt);
    };
  Matrix2 F_zero = numericalDerivative11<Unit3, Unit3>(predict_wrapper_zero, data.p0);
  EXPECT(assert_equal<Matrix2>(I_2x2, F_zero, 1e-8));

}

TEST(ManifoldEKF_Unit3, Update) {
  Unit3EKFTest data;

  // Use a slightly different starting point and covariance for variety
  Unit3 p_start = Unit3(Point3(0, 1, 0)).retract((Vector2() << 0.1, 0).finished()); // Perturb pointing along Y
  Matrix2 P_start = I_2x2 * 0.05;
  ManifoldEKF<Unit3> ekf(p_start, P_start);

  // Simulate a measurement (e.g., true value + noise)
  Vector1 z_true = exampleUnit3::measureZ(p_start, {});
  Vector1 z_observed = z_true + Vector1(0.02); // Add some noise

  // --- Perform EKF update ---
  ekf.update(exampleUnit3::measureZ, z_observed, data.R);

  // --- Verification (Manual Kalman Update Steps) ---
  // 1. Predict measurement and get Jacobian H
  Matrix12 H; // Note: Jacobian is 1x2 for Unit3
  Vector1 z_pred = exampleUnit3::measureZ(p_start, H);

  // 2. Innovation and Covariance
  Vector1 y = z_pred - z_observed; // Innovation (using vector subtraction for z)
  Matrix1 S = H * P_start * H.transpose() + data.R; // 1x1 matrix

  // 3. Kalman Gain K
  Matrix K = P_start * H.transpose() * S.inverse(); // 2x1 matrix

  // 4. State Correction (in tangent space)
  Vector2 delta_xi = -K * y; // 2x1 vector

  // 5. Expected Updated State and Covariance
  Unit3 p_updated_expected = p_start.retract(delta_xi);
  Matrix2 I_KH = I_2x2 - K * H;
  Matrix2 P_updated_expected = I_KH * P_start;

  // --- Compare EKF result with manual calculation ---
  EXPECT(assert_equal(p_updated_expected, ekf.state(), 1e-8));
  EXPECT(assert_equal(P_updated_expected, ekf.covariance(), 1e-8));
}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}