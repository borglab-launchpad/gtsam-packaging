/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianFactor.cpp
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(const KeyVector &continuousKeys,
                                           const DiscreteKeys &discreteKeys,
                                           const Factors &factors)
    : Base(continuousKeys, discreteKeys), factors_(factors) {}

/* *******************************************************************************/
bool HybridGaussianFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr) return false;

  // This will return false if either factors_ is empty or e->factors_ is empty,
  // but not if both are empty or both are not empty:
  if (factors_.empty() ^ e->factors_.empty()) return false;

  // Check the base and the factors:
  return Base::equals(*e, tol) &&
         factors_.equals(e->factors_,
                         [tol](const std::pair<sharedFactor, double> &f1,
                               const std::pair<sharedFactor, double> &f2) {
                           return f1.first->equals(*f2.first, tol) &&
                                  (f1.second == f2.second);
                         });
}

/* *******************************************************************************/
void HybridGaussianFactor::print(const std::string &s,
                                 const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  std::cout << "HybridGaussianFactor" << std::endl;
  HybridFactor::print("", formatter);
  std::cout << "{\n";
  if (factors_.empty()) {
    std::cout << "  empty" << std::endl;
  } else {
    factors_.print(
        "", [&](Key k) { return formatter(k); },
        [&](const std::pair<sharedFactor, double> &gfv) -> std::string {
          auto [gf, val] = gfv;
          RedirectCout rd;
          std::cout << ":\n";
          if (gf) {
            gf->print("", formatter);
            std::cout << "value: " << val << std::endl;
            return rd.str();
          } else {
            return "nullptr";
          }
        });
  }
  std::cout << "}" << std::endl;
}

/* *******************************************************************************/
std::pair<HybridGaussianFactor::sharedFactor, double>
HybridGaussianFactor::operator()(const DiscreteValues &assignment) const {
  return factors_(assignment);
}

/* *******************************************************************************/
GaussianFactorGraphTree HybridGaussianFactor::add(
    const GaussianFactorGraphTree &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const auto tree = asGaussianFactorGraphTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianFactorGraphTree HybridGaussianFactor::asGaussianFactorGraphTree()
    const {
  auto wrap = [](const std::pair<sharedFactor, double> &gfv) {
    return GaussianFactorGraph{gfv.first};
  };
  return {factors_, wrap};
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> HybridGaussianFactor::errorTree(
    const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc =
      [&continuousValues](const std::pair<sharedFactor, double> &gfv) {
        auto [gf, val] = gfv;
        return gf->error(continuousValues) + val;
      };
  DecisionTree<Key, double> error_tree(factors_, errorFunc);
  return error_tree;
}

/* *******************************************************************************/
double HybridGaussianFactor::error(const HybridValues &values) const {
  auto &&[gf, val] = factors_(values.discrete());
  return gf->error(values.continuous()) + val;
}

}  // namespace gtsam
