/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridSmoother.cpp
 * @brief   An incremental smoother for hybrid factor graphs
 * @author  Varun Agrawal
 * @date    October 2022
 */

#include <gtsam/hybrid/HybridSmoother.h>

#include <algorithm>
#include <unordered_set>

namespace gtsam {

/* ************************************************************************* */
Ordering HybridSmoother::getOrdering(const HybridGaussianFactorGraph &factors,
                                     const KeySet &newFactorKeys) {
  // Get all the discrete keys from the factors
  KeySet allDiscrete = factors.discreteKeySet();

  // Create KeyVector with continuous keys followed by discrete keys.
  KeyVector newKeysDiscreteLast;

  // Insert continuous keys first.
  for (auto &k : newFactorKeys) {
    if (!allDiscrete.exists(k)) {
      newKeysDiscreteLast.push_back(k);
    }
  }

  // Insert discrete keys at the end
  std::copy(allDiscrete.begin(), allDiscrete.end(),
            std::back_inserter(newKeysDiscreteLast));

  const VariableIndex index(factors);

  // Get an ordering where the new keys are eliminated last
  Ordering ordering = Ordering::ColamdConstrainedLast(
      index, KeyVector(newKeysDiscreteLast.begin(), newKeysDiscreteLast.end()),
      true);
  return ordering;
}

/* ************************************************************************* */
void HybridSmoother::update(const HybridGaussianFactorGraph &graph,
                            std::optional<size_t> maxNrLeaves,
                            const std::optional<Ordering> given_ordering) {
  HybridGaussianFactorGraph updatedGraph;
  // Add the necessary conditionals from the previous timestep(s).
  std::tie(updatedGraph, hybridBayesNet_) =
      addConditionals(graph, hybridBayesNet_);

  Ordering ordering;
  // If no ordering provided, then we compute one
  if (!given_ordering.has_value()) {
    // Get the keys from the new factors
    const KeySet newFactorKeys = graph.keys();

    // Since updatedGraph now has all the connected conditionals,
    // we can get the correct ordering.
    ordering = this->getOrdering(updatedGraph, newFactorKeys);
  } else {
    ordering = *given_ordering;
  }

  // Eliminate.
  HybridBayesNet bayesNetFragment = *updatedGraph.eliminateSequential(ordering);

  /// Prune
  if (maxNrLeaves) {
    // `pruneBayesNet` sets the leaves with 0 in discreteFactor to nullptr in
    // all the conditionals with the same keys in bayesNetFragment.
    bayesNetFragment = bayesNetFragment.prune(*maxNrLeaves, deadModeThreshold_);
  }

  // Add the partial bayes net to the posterior bayes net.
  hybridBayesNet_.add(bayesNetFragment);
}

/* ************************************************************************* */
std::pair<HybridGaussianFactorGraph, HybridBayesNet>
HybridSmoother::addConditionals(const HybridGaussianFactorGraph &originalGraph,
                                const HybridBayesNet &hybridBayesNet) const {
  HybridGaussianFactorGraph graph(originalGraph);
  HybridBayesNet updatedHybridBayesNet(hybridBayesNet);

  KeySet factorKeys = graph.keys();

  // If hybridBayesNet is not empty,
  // it means we have conditionals to add to the factor graph.
  if (!hybridBayesNet.empty()) {
    // We add all relevant hybrid conditionals on the last continuous variable
    // in the previous `hybridBayesNet` to the graph

    // New conditionals to add to the graph
    gtsam::HybridBayesNet newConditionals;

    // NOTE(Varun) Using a for-range loop doesn't work since some of the
    // conditionals are invalid pointers
    for (size_t i = 0; i < hybridBayesNet.size(); i++) {
      auto conditional = hybridBayesNet.at(i);

      for (auto &key : conditional->frontals()) {
        if (std::find(factorKeys.begin(), factorKeys.end(), key) !=
            factorKeys.end()) {
          newConditionals.push_back(conditional);

          // Add the conditional parents to factorKeys
          // so we add those conditionals too.
          // NOTE: This assumes we have a structure where
          // variables depend on those in the future.
          for (auto &&parentKey : conditional->parents()) {
            factorKeys.insert(parentKey);
          }

          // Remove the conditional from the updated Bayes net
          auto it = find(updatedHybridBayesNet.begin(),
                         updatedHybridBayesNet.end(), conditional);
          updatedHybridBayesNet.erase(it);

          break;
        }
      }
    }

    graph.push_back(newConditionals);
  }

  return {graph, updatedHybridBayesNet};
}

/* ************************************************************************* */
HybridGaussianConditional::shared_ptr HybridSmoother::gaussianMixture(
    size_t index) const {
  return hybridBayesNet_.at(index)->asHybrid();
}

/* ************************************************************************* */
const HybridBayesNet &HybridSmoother::hybridBayesNet() const {
  return hybridBayesNet_;
}

}  // namespace gtsam
