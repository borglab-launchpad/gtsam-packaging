/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

 /**
  * @file testGroupEKF.cpp
  * @brief Unit test for LieGroupEKF, as well as dynamics used in Rot3 example.
  * @date April 26, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/LieGroupEKF.h>
#include <gtsam/navigation/NavState.h>

using namespace gtsam;

// Duplicate the dynamics function in GEKF_Rot3Example
namespace exampleSO3 {
  static constexpr double k = 0.5;
  Vector3 dynamics(const Rot3& X, OptionalJacobian<3, 3> H = {}) {
    // φ = Logmap(R), Dφ = ∂φ/∂δR
    Matrix3 D_phi;
    Vector3 phi = Rot3::Logmap(X, D_phi);
    // zero out yaw
    phi[2] = 0.0;
    D_phi.row(2).setZero();

    if (H) *H = -k * D_phi;  // ∂(–kφ)/∂δR
    return -k * phi;         // xi ∈ 𝔰𝔬(3)
  }
}  // namespace exampleSO3

TEST(GroupeEKF, DynamicsJacobian) {
  // Construct a nontrivial state and IMU input
  Rot3 R = Rot3::RzRyRx(0.1, -0.2, 0.3);

  // Analytic Jacobian
  Matrix3 actualH;
  exampleSO3::dynamics(R, actualH);

  // Numeric Jacobian w.r.t. the state X
  auto f = [&](const Rot3& X_) { return exampleSO3::dynamics(X_); };
  Matrix3 expectedH = numericalDerivative11<Vector3, Rot3>(f, R);

  // Compare
  EXPECT(assert_equal(expectedH, actualH));
}

TEST(GroupeEKF, PredictNumericState) {
  // GIVEN
  Rot3 R0 = Rot3::RzRyRx(0.2, -0.1, 0.3);
  Matrix3 P0 = Matrix3::Identity() * 0.2;
  double dt = 0.1;

  // Analytic Jacobian
  Matrix3 actualH;
  LieGroupEKF<Rot3> ekf0(R0, P0);
  ekf0.predictMean(exampleSO3::dynamics, dt, actualH);

  // wrap predict into a state->state functor (mapping on SO(3))
  auto g = [&](const Rot3& R) -> Rot3 {
    LieGroupEKF<Rot3> ekf(R, P0);
    return ekf.predictMean(exampleSO3::dynamics, dt);
    };

  // numeric Jacobian of g at R0
  Matrix3 expectedH = numericalDerivative11<Rot3, Rot3>(g, R0);

  EXPECT(assert_equal(expectedH, actualH));
}

TEST(GroupeEKF, StateAndControl) {
  auto f = [](const Rot3& X, const Vector2& dummy_u,
    OptionalJacobian<3, 3> H = {}) {
      return exampleSO3::dynamics(X, H);
    };

  // GIVEN
  Rot3 R0 = Rot3::RzRyRx(0.2, -0.1, 0.3);
  Matrix3 P0 = Matrix3::Identity() * 0.2;
  Vector2 dummy_u(1, 2);
  double dt = 0.1;
  Matrix3 Q = Matrix3::Zero();

  // Analytic Jacobian
  Matrix3 actualH;
  LieGroupEKF<Rot3> ekf0(R0, P0);
  ekf0.predictMean(f, dummy_u, dt, actualH);

  // wrap predict into a state->state functor (mapping on SO(3))
  auto g = [&](const Rot3& R) -> Rot3 {
    LieGroupEKF<Rot3> ekf(R, P0);
    return ekf.predictMean(f, dummy_u, dt, Q);
    };

  // numeric Jacobian of g at R0
  Matrix3 expectedH = numericalDerivative11<Rot3, Rot3>(g, R0);

  EXPECT(assert_equal(expectedH, actualH));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
