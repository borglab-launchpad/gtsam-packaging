/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file    ManifoldEKF.h
  * @brief   Extended Kalman Filter base class on a generic manifold M
  *
  * This file defines the ManifoldEKF class template for performing prediction
  * and update steps of an Extended Kalman Filter on states residing in a
  * differentiable manifold. It relies on the manifold's retract and
  * localCoordinates operations.
  *
  * @date    April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h> // Include for traits

#include <Eigen/Dense>
#include <type_traits>

namespace gtsam {

  /**
     * @class ManifoldEKF
     * @brief Extended Kalman Filter on a generic manifold M
     *
     * @tparam M  Manifold type providing:
     *           - static int dimension = tangent dimension
     *           - using TangentVector = Eigen::Vector...
     *           - A `retract(const TangentVector&)` method (member or static)
     *           - A `localCoordinates(const M&)` method (member or static)
     *           - `gtsam::traits<M>` specialization must exist.
     *
     * This filter maintains a state X in the manifold M and covariance P in the
     * tangent space at X. Prediction requires providing the predicted next state
     * and the state transition Jacobian F. Updates apply a measurement function h
     * and correct the state using the tangent space error.
     */
  template <typename M>
  class ManifoldEKF {
  public:
    /// Manifold dimension (tangent space dimension)
    static constexpr int n = traits<M>::dimension;

    /// Tangent vector type for the manifold M
    using TangentVector = typename traits<M>::TangentVector;

    /// Square matrix of size n for covariance and Jacobians
    using MatrixN = Eigen::Matrix<double, n, n>;

    /// Constructor: initialize with state and covariance
    ManifoldEKF(const M& X0, const MatrixN& P0) : X_(X0), P_(P0) {
      static_assert(IsManifold<M>::value, "Template parameter M must be a GTSAM Manifold");
    }

    virtual ~ManifoldEKF() = default; // Add virtual destructor for base class

    /// @return current state estimate
    const M& state() const { return X_; }

    /// @return current covariance estimate
    const MatrixN& covariance() const { return P_; }

    /**
     * Basic predict step: Updates state and covariance given the predicted
     * next state and the state transition Jacobian F.
     *   X_{k+1} = X_next
     *   P_{k+1} = F P_k F^T + Q
     * where F = d(local(X_{k+1})) / d(local(X_k)) is the Jacobian of the
     * state transition in local coordinates around X_k.
     *
     * @param X_next The predicted state at time k+1.
     * @param F The state transition Jacobian (size nxn).
     * @param Q Process noise covariance matrix in the tangent space (size nxn).
     */
    void predict(const M& X_next, const MatrixN& F, const Matrix& Q) {
      X_ = X_next;
      P_ = F * P_ * F.transpose() + Q;
    }

    /**
     * Measurement update: Corrects the state and covariance using a measurement.
     *   z_pred, H = h(X)
     *   y = z - z_pred   (innovation, or z_pred - z depending on convention)
     *   S = H P H^T + R  (innovation covariance)
     *   K = P H^T S^{-1} (Kalman gain)
     *   delta_xi = -K * y (correction in tangent space)
     *   X <- X.retract(delta_xi)
     *   P <- (I - K H) P
     *
     * @tparam Measurement Type of the measurement vector (e.g., VectorN<m>)
     * @tparam Prediction Functor signature: Measurement h(const M&,
     * OptionalJacobian<m,n>&)
     *                    where m is the measurement dimension.
     *
     * @param h Measurement model functor returning predicted measurement z_pred
     *          and its Jacobian H = d(h)/d(local(X)).
     * @param z Observed measurement.
     * @param R Measurement noise covariance (size m x m).
     */
    template <typename Measurement, typename Prediction>
    void update(Prediction&& h, const Measurement& z, const Matrix& R) {
      constexpr int m = traits<Measurement>::dimension;
      Eigen::Matrix<double, m, n> H;

      // Predict measurement and get Jacobian H = dh/dlocal(X)
      Measurement z_pred = h(X_, H);

      // Innovation
      // Ensure consistent subtraction for manifold types if Measurement is one
      Vector innovation = traits<Measurement>::Local(z, z_pred); // y = z_pred (-) z (in tangent space)

      // Innovation covariance and Kalman Gain
      auto S = H * P_ * H.transpose() + R;
      Matrix K = P_ * H.transpose() * S.inverse(); // K = P H^T S^-1 (size n x m)

      // Correction vector in tangent space
      TangentVector delta_xi = -K * innovation; // delta_xi = - K * y

      // Update state using retract
      X_ = traits<M>::Retract(X_, delta_xi); // X <- X.retract(delta_xi)

      // Update covariance using Joseph form:
      MatrixN I_KH = I_n - K * H;
      P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    }

  protected:
    M X_;      ///< manifold state estimate
    MatrixN P_; ///< covariance in tangent space at X_

  private:
    /// Identity matrix of size n
    static const MatrixN I_n;
  };

  // Define static identity I_n
  template <typename M>
  const typename ManifoldEKF<M>::MatrixN ManifoldEKF<M>::I_n = ManifoldEKF<M>::MatrixN::Identity();

}  // namespace gtsam