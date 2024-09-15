/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactor.h
 * @brief  Nonlinear Mixture factor of continuous and discrete.
 * @author Kevin Doherty, kdoherty@mit.edu
 * @author Varun Agrawal
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace gtsam {

/**
 * @brief Implementation of a discrete conditional mixture factor.
 *
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a mixture component corresponding to a NonlinearFactor
 * type of measurement.
 *
 * This class stores all factors as HybridFactors which can then be typecast to
 * one of (NonlinearFactor, GaussianFactor) which can then be checked to perform
 * the correct operation.
 */
class HybridNonlinearFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = HybridNonlinearFactor;
  using shared_ptr = std::shared_ptr<HybridNonlinearFactor>;
  using sharedFactor = std::shared_ptr<NonlinearFactor>;

  /**
   * @brief typedef for DecisionTree which has Keys as node labels and
   * pairs of NonlinearFactor & an arbitrary scalar as leaf nodes.
   */
  using Factors = DecisionTree<Key, std::pair<sharedFactor, double>>;

 private:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  Factors factors_;
  bool normalized_;

 public:
  HybridNonlinearFactor() = default;

  /**
   * @brief Construct from Decision tree.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Decision tree with of shared factors.
   * @param normalized Flag indicating if the factor error is already
   * normalized.
   */
  HybridNonlinearFactor(const KeyVector& keys, const DiscreteKeys& discreteKeys,
                        const Factors& factors, bool normalized = false)
      : Base(keys, discreteKeys), factors_(factors), normalized_(normalized) {}

  /**
   * @brief Convenience constructor that generates the underlying factor
   * decision tree for us.
   *
   * Here it is important that the vector of factors has the correct number of
   * elements based on the number of discrete keys and the cardinality of the
   * keys, so that the decision tree is constructed appropriately.
   *
   * @tparam FACTOR The type of the factor shared pointers being passed in.
   * Will be typecast to NonlinearFactor shared pointers.
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Vector of nonlinear factor and scalar pairs.
   * @param normalized Flag indicating if the factor error is already
   * normalized.
   */
  template <typename FACTOR>
  HybridNonlinearFactor(
      const KeyVector& keys, const DiscreteKeys& discreteKeys,
      const std::vector<std::pair<std::shared_ptr<FACTOR>, double>>& factors,
      bool normalized = false)
      : Base(keys, discreteKeys), normalized_(normalized) {
    std::vector<std::pair<NonlinearFactor::shared_ptr, double>>
        nonlinear_factors;
    KeySet continuous_keys_set(keys.begin(), keys.end());
    KeySet factor_keys_set;
    for (auto&& [f, val] : factors) {
      // Insert all factor continuous keys in the continuous keys set.
      std::copy(f->keys().begin(), f->keys().end(),
                std::inserter(factor_keys_set, factor_keys_set.end()));

      if (auto nf = std::dynamic_pointer_cast<NonlinearFactor>(f)) {
        nonlinear_factors.emplace_back(nf, val);
      } else {
        throw std::runtime_error(
            "Factors passed into HybridNonlinearFactor need to be nonlinear!");
      }
    }
    factors_ = Factors(discreteKeys, nonlinear_factors);

    if (continuous_keys_set != factor_keys_set) {
      throw std::runtime_error(
          "The specified continuous keys and the keys in the factors don't "
          "match!");
    }
  }

  /**
   * @brief Compute error of the HybridNonlinearFactor as a tree.
   *
   * @param continuousValues The continuous values for which to compute the
   * error.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factor, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> errorTree(const Values& continuousValues) const {
    // functor to convert from sharedFactor to double error value.
    auto errorFunc =
        [continuousValues](const std::pair<sharedFactor, double>& f) {
          auto [factor, val] = f;
          return factor->error(continuousValues) + (0.5 * val * val);
        };
    DecisionTree<Key, double> result(factors_, errorFunc);
    return result;
  }

  /**
   * @brief Compute error of factor given both continuous and discrete values.
   *
   * @param continuousValues The continuous Values.
   * @param discreteValues The discrete Values.
   * @return double The error of this factor.
   */
  double error(const Values& continuousValues,
               const DiscreteValues& discreteValues) const {
    // Retrieve the factor corresponding to the assignment in discreteValues.
    auto [factor, val] = factors_(discreteValues);
    // Compute the error for the selected factor
    const double factorError = factor->error(continuousValues);
    return factorError + (0.5 * val * val);
  }

  /**
   * @brief Compute error of factor given hybrid values.
   *
   * @param values The continuous Values and the discrete assignment.
   * @return double The error of this factor.
   */
  double error(const HybridValues& values) const override {
    return error(values.nonlinear(), values.discrete());
  }

  /**
   * @brief Get the dimension of the factor (number of rows on linearization).
   * Returns the dimension of the first component factor.
   * @return size_t
   */
  size_t dim() const {
    const auto assignments = DiscreteValues::CartesianProduct(discreteKeys_);
    auto [factor, val] = factors_(assignments.at(0));
    return factor->dim();
  }

  /// Testable
  /// @{

  /// print to stdout
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ");
    Base::print("", keyFormatter);
    std::cout << "\nHybridNonlinearFactor\n";
    auto valueFormatter = [](const std::pair<sharedFactor, double>& v) {
      auto [factor, val] = v;
      if (factor) {
        return "Nonlinear factor on " + std::to_string(factor->size()) +
               " keys";
      } else {
        return std::string("nullptr");
      }
    };
    factors_.print("", keyFormatter, valueFormatter);
  }

  /// Check equality
  bool equals(const HybridFactor& other, double tol = 1e-9) const override {
    // We attempt a dynamic cast from HybridFactor to HybridNonlinearFactor. If
    // it fails, return false.
    if (!dynamic_cast<const HybridNonlinearFactor*>(&other)) return false;

    // If the cast is successful, we'll properly construct a
    // HybridNonlinearFactor object from `other`
    const HybridNonlinearFactor& f(
        static_cast<const HybridNonlinearFactor&>(other));

    // Ensure that this HybridNonlinearFactor and `f` have the same `factors_`.
    auto compare = [tol](const std::pair<sharedFactor, double>& a,
                         const std::pair<sharedFactor, double>& b) {
      return traits<NonlinearFactor>::Equals(*a.first, *b.first, tol) &&
             (a.second == b.second);
    };
    if (!factors_.equals(f.factors_, compare)) return false;

    // If everything above passes, and the keys_, discreteKeys_ and normalized_
    // member variables are identical, return true.
    return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
            (discreteKeys_ == f.discreteKeys_) &&
            (normalized_ == f.normalized_));
  }

  /// @}

  /// Linearize specific nonlinear factors based on the assignment in
  /// discreteValues.
  GaussianFactor::shared_ptr linearize(
      const Values& continuousValues,
      const DiscreteValues& discreteValues) const {
    auto factor = factors_(discreteValues).first;
    return factor->linearize(continuousValues);
  }

  /// Linearize all the continuous factors to get a HybridGaussianFactor.
  std::shared_ptr<HybridGaussianFactor> linearize(
      const Values& continuousValues) const {
    // functional to linearize each factor in the decision tree
    auto linearizeDT =
        [continuousValues](const std::pair<sharedFactor, double>& f)
        -> GaussianFactorValuePair {
      auto [factor, val] = f;
      return {factor->linearize(continuousValues), val};
    };

    DecisionTree<Key, std::pair<GaussianFactor::shared_ptr, double>>
        linearized_factors(factors_, linearizeDT);

    return std::make_shared<HybridGaussianFactor>(
        continuousKeys_, discreteKeys_, linearized_factors);
  }

  /**
   * If the component factors are not already normalized, we want to compute
   * their normalizing constants so that the resulting joint distribution is
   * appropriately computed. Remember, this is the _negative_ normalizing
   * constant for the measurement likelihood (since we are minimizing the
   * _negative_ log-likelihood).
   */
  double nonlinearFactorLogNormalizingConstant(const sharedFactor& factor,
                                               const Values& values) const {
    // Information matrix (inverse covariance matrix) for the factor.
    Matrix infoMat;

    // If this is a NoiseModelFactor, we'll use its noiseModel to
    // otherwise noiseModelFactor will be nullptr
    if (auto noiseModelFactor =
            std::dynamic_pointer_cast<NoiseModelFactor>(factor)) {
      // If dynamic cast to NoiseModelFactor succeeded, see if the noise model
      // is Gaussian
      auto noiseModel = noiseModelFactor->noiseModel();

      auto gaussianNoiseModel =
          std::dynamic_pointer_cast<noiseModel::Gaussian>(noiseModel);
      if (gaussianNoiseModel) {
        // If the noise model is Gaussian, retrieve the information matrix
        infoMat = gaussianNoiseModel->information();
      } else {
        // If the factor is not a Gaussian factor, we'll linearize it to get
        // something with a normalized noise model
        // TODO(kevin): does this make sense to do? I think maybe not in
        // general? Should we just yell at the user?
        auto gaussianFactor = factor->linearize(values);
        infoMat = gaussianFactor->information();
      }
    }

    // Compute the (negative) log of the normalizing constant
    return -(factor->dim() * log(2.0 * M_PI) / 2.0) -
           (log(infoMat.determinant()) / 2.0);
  }
};

}  // namespace gtsam
