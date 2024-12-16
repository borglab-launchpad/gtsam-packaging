/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.h
 * @brief   3*3 matrix representation of SO(3)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @author  Duy Nguyen Ta
 * @date    December 2014
 */

#pragma once

#include <gtsam/geometry/SOn.h>

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/dllexport.h>

#include <vector>

namespace gtsam {

using SO3 = SO<3>;

// Below are all declarations of SO<3> specializations.
// They are *defined* in SO3.cpp.

template <>
GTSAM_EXPORT
SO3 SO3::AxisAngle(const Vector3& axis, double theta);

template <>
GTSAM_EXPORT
SO3 SO3::ClosestTo(const Matrix3& M);

template <>
GTSAM_EXPORT
SO3 SO3::ChordalMean(const std::vector<SO3>& rotations);

template <>
GTSAM_EXPORT
Matrix3 SO3::Hat(const Vector3& xi);  ///< make skew symmetric matrix

template <>
GTSAM_EXPORT
Vector3 SO3::Vee(const Matrix3& X);  ///< inverse of Hat

/// Adjoint map
template <>
Matrix3 SO3::AdjointMap() const;

/**
 * Exponential map at identity - create a rotation from canonical coordinates
 * \f$ [R_x,R_y,R_z] \f$ using Rodrigues' formula
 */
template <>
GTSAM_EXPORT
SO3 SO3::Expmap(const Vector3& omega, ChartJacobian H);

/// Derivative of Expmap
template <>
GTSAM_EXPORT
Matrix3 SO3::ExpmapDerivative(const Vector3& omega);

/**
 * Log map at identity - returns the canonical coordinates
 * \f$ [R_x,R_y,R_z] \f$ of this rotation
 */
template <>
GTSAM_EXPORT
Vector3 SO3::Logmap(const SO3& R, ChartJacobian H);

/// Derivative of Logmap
template <>
GTSAM_EXPORT
Matrix3 SO3::LogmapDerivative(const Vector3& omega);

// Chart at origin for SO3 is *not* Cayley but actual Expmap/Logmap
template <>
GTSAM_EXPORT
SO3 SO3::ChartAtOrigin::Retract(const Vector3& omega, ChartJacobian H);

template <>
GTSAM_EXPORT
Vector3 SO3::ChartAtOrigin::Local(const SO3& R, ChartJacobian H);

template <>
GTSAM_EXPORT
Vector9 SO3::vec(OptionalJacobian<9, 3> H) const;

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
template <class Archive>
/** Serialization function */
void serialize(Archive& ar, SO3& R, const unsigned int /*version*/) {
  Matrix3& M = R.matrix_;
  ar& boost::serialization::make_nvp("R11", M(0, 0));
  ar& boost::serialization::make_nvp("R12", M(0, 1));
  ar& boost::serialization::make_nvp("R13", M(0, 2));
  ar& boost::serialization::make_nvp("R21", M(1, 0));
  ar& boost::serialization::make_nvp("R22", M(1, 1));
  ar& boost::serialization::make_nvp("R23", M(1, 2));
  ar& boost::serialization::make_nvp("R31", M(2, 0));
  ar& boost::serialization::make_nvp("R32", M(2, 1));
  ar& boost::serialization::make_nvp("R33", M(2, 2));
}
#endif

namespace so3 {

/**
 * Compose general matrix with an SO(3) element.
 * We only provide the 9*9 derivative in the first argument M.
 */
GTSAM_EXPORT Matrix3 compose(const Matrix3& M, const SO3& R,
                OptionalJacobian<9, 9> H = {});

/// (constant) Jacobian of compose wrpt M
GTSAM_EXPORT Matrix99 Dcompose(const SO3& R);

// Below are two functors that allow for saving computation when exponential map
// and its derivatives are needed at the same location in so<3>. The second
// functor also implements dedicated methods to apply dexp and/or inv(dexp).

/// Functor implementing Exponential map
struct GTSAM_EXPORT ExpmapFunctor {
  const double theta2, theta;
  const Matrix3 W, WW;
  bool nearZero;

  // Ethan Eade's constants:
  double A;  // A = sin(theta) / theta or 1 for theta->0
  double B;  // B = (1 - cos(theta)) / theta^2 or 0.5 for theta->0

  /// Constructor with element of Lie algebra so(3)
  explicit ExpmapFunctor(const Vector3& omega, bool nearZeroApprox = false);

  /// Constructor with axis-angle
  ExpmapFunctor(const Vector3& axis, double angle, bool nearZeroApprox = false);

  /// Rodrigues formula
  SO3 expmap() const;

 protected:
  void init(bool nearZeroApprox = false);
};

/// Functor that implements Exponential map *and* its derivatives
struct GTSAM_EXPORT DexpFunctor : public ExpmapFunctor {
  const Vector3 omega;
  double C;  // Ethan's C constant: (1 - A) / theta^2 or 1/6 for theta->0
  // Constants used in cross and doubleCross
  double D;  // (A - 2.0 * B) / theta2 or -1/12 for theta->0
  double E;  // (B - 3.0 * C) / theta2 or -1/60 for theta->0

  /// Constructor with element of Lie algebra so(3)
  explicit DexpFunctor(const Vector3& omega, bool nearZeroApprox = false);

  // NOTE(luca): Right Jacobian for Exponential map in SO(3) - equation
  // (10.86) and following equations in G.S. Chirikjian, "Stochastic Models,
  // Information Theory, and Lie Groups", Volume 2, 2008.
  //   Expmap(xi + dxi) \approx Expmap(xi) * Expmap(dexp * dxi)
  // This maps a perturbation dxi=(w,v) in the tangent space to
  // a perturbation on the manifold Expmap(dexp * xi)
  Matrix3 rightJacobian() const { return I_3x3 - B * W + C * WW; }

  // Compute the left Jacobian for Exponential map in SO(3)
  Matrix3 leftJacobian() const { return I_3x3 + B * W + C * WW; }

  /// Differential of expmap == right Jacobian
  inline Matrix3 dexp() const { return rightJacobian(); }

  /// Computes B * (omega x v).
  Vector3 crossB(const Vector3& v, OptionalJacobian<3, 3> H = {}) const;

  /// Computes C * (omega x (omega x v)).
  Vector3 doubleCrossC(const Vector3& v, OptionalJacobian<3, 3> H = {}) const;

  /// Multiplies with dexp(), with optional derivatives
  Vector3 applyDexp(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                    OptionalJacobian<3, 3> H2 = {}) const;

  /// Multiplies with dexp().inverse(), with optional derivatives
  Vector3 applyInvDexp(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                       OptionalJacobian<3, 3> H2 = {}) const;

  /// Multiplies with leftJacobian(), with optional derivatives
  Vector3 applyLeftJacobian(const Vector3& v, OptionalJacobian<3, 3> H1 = {},
                            OptionalJacobian<3, 3> H2 = {}) const;

  static constexpr double one_sixth = 1.0 / 6.0;
  static constexpr double _one_twelfth = -1.0 / 12.0;
  static constexpr double _one_sixtieth = -1.0 / 60.0;
};
}  //  namespace so3

/*
 * Define the traits. internal::LieGroup provides both Lie group and Testable
 */

template <>
struct traits<SO3> : public internal::LieGroup<SO3> {};

template <>
struct traits<const SO3> : public internal::LieGroup<SO3> {};

}  // end namespace gtsam
