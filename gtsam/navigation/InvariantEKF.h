/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file    InvariantEKF.h
  * @brief   Left-Invariant Extended Kalman Filter implementation.
  *
  * This file defines the InvariantEKF class template, inheriting from LieGroupEKF,
  * which specifically implements the Left-Invariant EKF formulation. It restricts
  * prediction methods to only those based on group composition (state-independent
  * motion models), hiding the state-dependent prediction variants from LieGroupEKF.
  *
  * @date    April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/navigation/LieGroupEKF.h> // Include the base class

namespace gtsam {

  /**
   * @class InvariantEKF
   * @brief Left-Invariant Extended Kalman Filter on a Lie group G.
   *
   * @tparam G Lie group type (must satisfy LieGroup concept).
   *
   * This filter inherits from LieGroupEKF but restricts the prediction interface
   * to only the left-invariant prediction methods:
   * 1. Prediction via group composition: `predict(const G& U, const Matrix& Q)`
   * 2. Prediction via tangent control vector: `predict(const TangentVector& u, double dt, const Matrix& Q)`
   *
   * The state-dependent prediction methods from LieGroupEKF are hidden.
   * The update step remains the same as in ManifoldEKF/LieGroupEKF.
   */
  template <typename G>
  class InvariantEKF : public LieGroupEKF<G> {
  public:
    using Base = LieGroupEKF<G>; ///< Base class type
    using TangentVector = typename Base::TangentVector; ///< Tangent vector type
    using MatrixN = typename Base::MatrixN; ///< Square matrix type for covariance etc.
    using Jacobian = typename Base::Jacobian; ///< Jacobian matrix type specific to the group G

    /// Constructor: forwards to LieGroupEKF constructor
    InvariantEKF(const G& X0, const MatrixN& P0) : Base(X0, P0) {}

    /**
     * Predict step via group composition (Left-Invariant):
     *   X_{k+1} = X_k * U
     *   P_{k+1} = Ad_{U^{-1}} P_k Ad_{U^{-1}}^T + Q
     * where Ad_{U^{-1}} is the Adjoint map of U^{-1}. This corresponds to
     * F = Ad_{U^{-1}} in the base class predict method.
     *
     * @param U Lie group element representing the motion increment.
     * @param Q Process noise covariance in the tangent space (size nxn).
     */
    void predict(const G& U, const Matrix& Q) {
      this->X_ = this->X_.compose(U);
      // TODO(dellaert): traits<G>::AdjointMap should exist
      Jacobian A = traits<G>::Inverse(U).AdjointMap();
      this->P_ = A * this->P_ * A.transpose() + Q;
    }

    /**
     * Predict step via tangent control vector:
     *   U = Expmap(u * dt)
     * Then calls predict(U, Q).
     *
     * @param u Tangent space control vector.
     * @param dt Time interval.
     * @param Q Process noise covariance matrix (size nxn).
     */
    void predict(const TangentVector& u, double dt, const Matrix& Q) {
      G U = traits<G>::Expmap(u * dt);
      predict(U, Q); // Call the group composition predict
    }

  }; // InvariantEKF

} // namespace gtsam