/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

 /**
  * @file testInvariantEKF_Pose2.cpp
  * @brief Unit test for the InvariantEKF using Pose2 state.
  *        Based on the logic from IEKF_SE2Example.cpp
  * @date April 26, 2025
  * @authors Frank Dellaert (adapted from example by Scott Baker, Matt Kielo)
  */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/navigation/InvariantEKF.h>

#include <iostream>

using namespace std;
using namespace gtsam;

// Create a 2D GPS measurement function that returns the predicted measurement h
// and Jacobian H. The predicted measurement h is the translation of the state X.
// (Copied from IEKF_SE2Example.cpp)
Vector2 h_gps(const Pose2& X, OptionalJacobian<2, 3> H = {}) {
  return X.translation(H);
}


TEST(IEKF_Pose2, PredictUpdateSequence) {
  // GIVEN: Initial state, covariance, noises, motions, measurements
  // (from IEKF_SE2Example.cpp)
  Pose2 X0(0.0, 0.0, 0.0);
  Matrix3 P0 = Matrix3::Identity() * 0.1;

  Matrix3 Q = (Vector3(0.05, 0.05, 0.001)).asDiagonal();
  Matrix2 R = I_2x2 * 0.01;

  Pose2 U1(1.0, 1.0, 0.5), U2(1.0, 1.0, 0.0);

  Vector2 z1, z2;
  z1 << 1.0, 0.0;
  z2 << 1.0, 1.0;

  // Create the filter
  InvariantEKF<Pose2> ekf(X0, P0);
  EXPECT(assert_equal(X0, ekf.state()));
  EXPECT(assert_equal(P0, ekf.covariance()));

  // --- First Prediction ---
  ekf.predict(U1, Q);

  // Calculate expected state and covariance
  Pose2 X1_expected = X0.compose(U1);
  Matrix3 Ad_U1_inv = U1.inverse().AdjointMap();
  Matrix3 P1_expected = Ad_U1_inv * P0 * Ad_U1_inv.transpose() + Q;

  // Verify
  EXPECT(assert_equal(X1_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P1_expected, ekf.covariance(), 1e-9));


  // --- First Update ---
  ekf.update(h_gps, z1, R);

  // Calculate expected state and covariance (manual Kalman steps)
  Matrix H1; // H = dh/dlocal(X) -> 2x3
  Vector2 z_pred1 = h_gps(X1_expected, H1);
  Vector2 y1 = z_pred1 - z1; // Innovation
  Matrix S1 = H1 * P1_expected * H1.transpose() + R;
  Matrix K1 = P1_expected * H1.transpose() * S1.inverse(); // Gain (3x2)
  Vector3 delta_xi1 = -K1 * y1; // Correction (tangent space)
  Pose2 X1_updated_expected = X1_expected.retract(delta_xi1);
  Matrix3 I_KH1 = Matrix3::Identity() - K1 * H1;
  Matrix3 P1_updated_expected = I_KH1 * P1_expected; // Standard form P = (I-KH)P

  // Verify
  EXPECT(assert_equal(X1_updated_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P1_updated_expected, ekf.covariance(), 1e-9));


  // --- Second Prediction ---
  ekf.predict(U2, Q);

  // Calculate expected state and covariance
  Pose2 X2_expected = X1_updated_expected.compose(U2);
  Matrix3 Ad_U2_inv = U2.inverse().AdjointMap();
  Matrix3 P2_expected = Ad_U2_inv * P1_updated_expected * Ad_U2_inv.transpose() + Q;

  // Verify
  EXPECT(assert_equal(X2_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P2_expected, ekf.covariance(), 1e-9));


  // --- Second Update ---
  ekf.update(h_gps, z2, R);

  // Calculate expected state and covariance (manual Kalman steps)
  Matrix H2; // 2x3
  Vector2 z_pred2 = h_gps(X2_expected, H2);
  Vector2 y2 = z_pred2 - z2; // Innovation
  Matrix S2 = H2 * P2_expected * H2.transpose() + R;
  Matrix K2 = P2_expected * H2.transpose() * S2.inverse(); // Gain (3x2)
  Vector3 delta_xi2 = -K2 * y2; // Correction (tangent space)
  Pose2 X2_updated_expected = X2_expected.retract(delta_xi2);
  Matrix3 I_KH2 = Matrix3::Identity() - K2 * H2;
  Matrix3 P2_updated_expected = I_KH2 * P2_expected; // Standard form

  // Verify
  EXPECT(assert_equal(X2_updated_expected, ekf.state(), 1e-9));
  EXPECT(assert_equal(P2_updated_expected, ekf.covariance(), 1e-9));

}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}