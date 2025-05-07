/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file  LieGroupEKF.h
  * @brief   Extended Kalman Filter derived class for Lie groups G.
  *
  * This file defines the LieGroupEKF class template, inheriting from ManifoldEKF,
  * for performing EKF steps specifically on states residing in a Lie group.
  * It provides predict methods with state-dependent dynamics functions.
  * Please use the InvariantEKF class for prediction via group composition.
  *
  * @date  April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/navigation/ManifoldEKF.h> // Include the base class
#include <gtsam/base/Lie.h> // Include for Lie group traits and operations

#include <Eigen/Dense>
#include <type_traits>
#include <functional> // For std::function

namespace gtsam {

  /**
   * @class LieGroupEKF
   * @brief Extended Kalman Filter on a Lie group G, derived from ManifoldEKF
   *
   * @tparam G Lie group type providing group operations and Expmap/AdjointMap.
   *       Must satisfy LieGroup concept (`gtsam::IsLieGroup<G>::value`).
   *
   * This filter specializes ManifoldEKF for Lie groups, offering predict methods
   * with state-dependent dynamics functions.
   * Use the InvariantEKF class for prediction via group composition.
   */
  template <typename G>
  class LieGroupEKF : public ManifoldEKF<G> {
  public:
    using Base = ManifoldEKF<G>; ///< Base class type
    static constexpr int n = Base::n; ///< Group dimension (tangent space dimension)
    using TangentVector = typename Base::TangentVector; ///< Tangent vector type for the group G
    using MatrixN = typename Base::MatrixN; ///< Square matrix of size n for covariance and Jacobians
    using Jacobian = Eigen::Matrix<double, n, n>; ///< Jacobian matrix type specific to the group G

    /// Constructor: initialize with state and covariance
    LieGroupEKF(const G& X0, const MatrixN& P0) : Base(X0, P0) {
      static_assert(IsLieGroup<G>::value, "Template parameter G must be a GTSAM Lie Group");
    }

    /**
     * SFINAE check for correctly typed state-dependent dynamics function.
     * Signature: TangentVector f(const G& X, OptionalJacobian<n, n> Df)
     * Df = d(xi)/d(local(X))
     */
    template <typename Dynamics>
    using enable_if_dynamics = std::enable_if_t<
      !std::is_convertible_v<Dynamics, TangentVector>&&
      std::is_invocable_r_v<TangentVector, Dynamics, const G&,
      OptionalJacobian<n, n>&>>;

    /**
     * Predict mean and Jacobian A with state-dependent dynamics:
     *   xi = f(X_k, Df)       (Compute tangent vector dynamics and Jacobian Df)
     *   U = Expmap(xi * dt, Dexp) (Compute motion increment U and Expmap Jacobian Dexp)
     *   X_{k+1} = X_k * U     (Predict next state)
     *   F = Ad_{U^{-1}} + Dexp * Df * dt (Compute full state transition Jacobian)
     *
     * @tparam Dynamics Functor signature: TangentVector f(const G&, OptionalJacobian<n,n>&)
     * @param f Dynamics functor returning tangent vector xi and its Jacobian Df w.r.t. local(X).
     * @param dt Time step.
     * @param A Optional pointer to store the computed state transition Jacobian A.
     * @return Predicted state X_{k+1}.
     */
    template <typename Dynamics, typename = enable_if_dynamics<Dynamics>>
    G predictMean(Dynamics&& f, double dt, OptionalJacobian<n, n> A = {}) const {
      Jacobian Df, Dexp;
      TangentVector xi = f(this->X_, Df); // xi and Df = d(xi)/d(localX)
      G U = traits<G>::Expmap(xi * dt, Dexp); // U and Dexp = d(Log(Exp(v)))/dv | v=xi*dt
      G X_next = this->X_.compose(U);

      if (A) {
        // Full state transition Jacobian for left-invariant EKF:
        *A = traits<G>::Inverse(U).AdjointMap() + Dexp * Df * dt;
      }
      return X_next;
    }


    /**
     * Predict step with state-dependent dynamics:
     * Uses predictMean to compute X_{k+1} and F, then calls base predict.
     *   X_{k+1}, F = predictMean(f, dt)
     *   P_{k+1} = F P_k F^T + Q
     *
     * @tparam Dynamics Functor signature: TangentVector f(const G&, OptionalJacobian<n,n>&)
     * @param f Dynamics functor.
     * @param dt Time step.
     * @param Q Process noise covariance (size nxn).
     */
    template <typename Dynamics, typename = enable_if_dynamics<Dynamics>>
    void predict(Dynamics&& f, double dt, const Matrix& Q) {
      Jacobian A;
      this->X_ = predictMean(std::forward<Dynamics>(f), dt, A);
      this->P_ = A * this->P_ * A.transpose() + Q;
    }

    /**
     * SFINAE check for state- and control-dependent dynamics function.
     * Signature: TangentVector f(const G& X, const Control& u, OptionalJacobian<n, n> Df)
     */
    template<typename Control, typename Dynamics>
    using enable_if_full_dynamics = std::enable_if_t<
      std::is_invocable_r_v<TangentVector, Dynamics, const G&, const Control&, OptionalJacobian<n, n>&>
    >;

    /**
     * Predict mean and Jacobian A with state and control input dynamics:
     * Wraps the dynamics function and calls the state-only predictMean.
     *   xi = f(X_k, u, Df)
     *
     * @tparam Control Control input type.
     * @tparam Dynamics Functor signature: TangentVector f(const G&, const Control&, OptionalJacobian<n,n>&)
     * @param f Dynamics functor.
     * @param u Control input.
     * @param dt Time step.
     * @param A Optional pointer to store the computed state transition Jacobian A.
     * @return Predicted state X_{k+1}.
     */
    template <typename Control, typename Dynamics, typename = enable_if_full_dynamics<Control, Dynamics>>
    G predictMean(Dynamics&& f, const Control& u, double dt, OptionalJacobian<n, n> A = {}) const {
      return predictMean([&](const G& X, OptionalJacobian<n, n> Df) { return f(X, u, Df); }, dt, A);
    }


    /**
     * Predict step with state and control input dynamics:
     * Wraps the dynamics function and calls the state-only predict.
     *   xi = f(X_k, u, Df)
     *
     * @tparam Control Control input type.
     * @tparam Dynamics Functor signature: TangentVector f(const G&, const Control&, OptionalJacobian<n,n>&)
     * @param f Dynamics functor.
     * @param u Control input.
     * @param dt Time step.
     * @param Q Process noise covariance (size nxn).
     */
    template <typename Control, typename Dynamics, typename = enable_if_full_dynamics<Control, Dynamics>>
    void predict(Dynamics&& f, const Control& u, double dt, const Matrix& Q) {
      return predict([&](const G& X, OptionalJacobian<n, n> Df) { return f(X, u, Df); }, dt, Q);
    }

  }; // LieGroupEKF

}  // namespace gtsam