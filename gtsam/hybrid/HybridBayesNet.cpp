/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   HybridBayesNet.cpp
 * @brief  A bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Shangjie Xue
 * @date   January 2022
 */

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/hybrid/HybridLookupDAG.h>

namespace gtsam {

/* ************************************************************************* */
/// Return the DiscreteKey vector as a set.
static std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &dkeys) {
  std::set<DiscreteKey> s;
  s.insert(dkeys.begin(), dkeys.end());
  return s;
}

/* ************************************************************************* */
HybridBayesNet HybridBayesNet::prune(
    const DecisionTreeFactor::shared_ptr &discreteFactor) const {
  /* To Prune, we visitWith every leaf in the GaussianMixture.
   * For each leaf, using the assignment we can check the discrete decision tree
   * for 0.0 probability, then just set the leaf to a nullptr.
   *
   * We can later check the GaussianMixture for just nullptrs.
   */

  HybridBayesNet prunedBayesNetFragment;

  // Functional which loops over all assignments and create a set of
  // GaussianConditionals
  auto pruner = [&](const Assignment<Key> &choices,
                    const GaussianConditional::shared_ptr &conditional)
      -> GaussianConditional::shared_ptr {
    // typecast so we can use this to get probability value
    DiscreteValues values(choices);

    if ((*discreteFactor)(values) == 0.0) {
      // empty aka null pointer
      boost::shared_ptr<GaussianConditional> null;
      return null;
    } else {
      return conditional;
    }
  };

  // Go through all the conditionals in the
  // Bayes Net and prune them as per discreteFactor.
  for (size_t i = 0; i < this->size(); i++) {
    HybridConditional::shared_ptr conditional = this->at(i);

    GaussianMixture::shared_ptr gaussianMixture =
        boost::dynamic_pointer_cast<GaussianMixture>(conditional->inner());

    if (gaussianMixture) {
      // We may have mixtures with less discrete keys than discreteFactor so we
      // skip those since the label assignment does not exist.
      auto gmKeySet = DiscreteKeysAsSet(gaussianMixture->discreteKeys());
      auto dfKeySet = DiscreteKeysAsSet(discreteFactor->discreteKeys());
      if (gmKeySet != dfKeySet) {
        // Add the gaussianMixture which doesn't have to be pruned.
        prunedBayesNetFragment.push_back(
            boost::make_shared<HybridConditional>(gaussianMixture));
        continue;
      }

      // Run the pruning to get a new, pruned tree
      GaussianMixture::Conditionals prunedTree =
          gaussianMixture->conditionals().apply(pruner);

      DiscreteKeys discreteKeys = gaussianMixture->discreteKeys();
      // reverse keys to get a natural ordering
      std::reverse(discreteKeys.begin(), discreteKeys.end());

      // Convert from boost::iterator_range to KeyVector
      // so we can pass it to constructor.
      KeyVector frontals(gaussianMixture->frontals().begin(),
                         gaussianMixture->frontals().end()),
          parents(gaussianMixture->parents().begin(),
                  gaussianMixture->parents().end());

      // Create the new gaussian mixture and add it to the bayes net.
      auto prunedGaussianMixture = boost::make_shared<GaussianMixture>(
          frontals, parents, discreteKeys, prunedTree);

      // Type-erase and add to the pruned Bayes Net fragment.
      prunedBayesNetFragment.push_back(
          boost::make_shared<HybridConditional>(prunedGaussianMixture));

    } else {
      // Add the non-GaussianMixture conditional
      prunedBayesNetFragment.push_back(conditional);
    }
  }

  return prunedBayesNetFragment;
}

/* ************************************************************************* */
GaussianMixture::shared_ptr HybridBayesNet::atGaussian(size_t i) const {
  return boost::dynamic_pointer_cast<GaussianMixture>(factors_.at(i)->inner());
}

/* ************************************************************************* */
DiscreteConditional::shared_ptr HybridBayesNet::atDiscrete(size_t i) const {
  return boost::dynamic_pointer_cast<DiscreteConditional>(
      factors_.at(i)->inner());
}

/* ************************************************************************* */
GaussianBayesNet HybridBayesNet::choose(
    const DiscreteValues &assignment) const {
  GaussianBayesNet gbn;
  for (size_t idx = 0; idx < size(); idx++) {
    GaussianMixture gm = *this->atGaussian(idx);
    gbn.push_back(gm(assignment));
  }
  return gbn;
}

/* *******************************************************************************/
HybridValues HybridBayesNet::optimize() const {
  auto dag = HybridLookupDAG::FromBayesNet(*this);
  return dag.argmax();
}

}  // namespace gtsam
