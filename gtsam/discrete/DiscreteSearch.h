/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * DiscreteSearch.cpp
 *
 * @date January, 2025
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteBayesTree.h>

#include <queue>

namespace gtsam {

/**
 * DiscreteSearch: Search for the K best solutions.
 */
class GTSAM_EXPORT DiscreteSearch {
 public:
  /**
   * @brief A solution to a discrete search problem.
   */
  struct Solution {
    double error;
    DiscreteValues assignment;
    Solution(double err, const DiscreteValues& assign)
        : error(err), assignment(assign) {}
    friend std::ostream& operator<<(std::ostream& os, const Solution& sn) {
      os << "[ error=" << sn.error << " assignment={" << sn.assignment << "}]";
      return os;
    }
  };

  /**
   * Construct from a DiscreteBayesNet and K.
   */
  DiscreteSearch(const DiscreteBayesNet& bayesNet);

  /**
   * Construct from a DiscreteBayesTree and K.
   */
  DiscreteSearch(const DiscreteBayesTree& bayesTree);

  /**
   * @brief Search for the K best solutions.
   *
   * This method performs a search to find the K best solutions for the given
   * DiscreteBayesNet. It uses a priority queue to manage the search nodes,
   * expanding nodes with the smallest bound first. The search continues until
   * all possible nodes have been expanded or pruned.
   *
   * @return A vector of the K best solutions found during the search.
   */
  std::vector<Solution> run(size_t K = 1) const;

 private:
  /// Compute the cumulative cost-to-go for each conditional slot.
  static std::vector<double> computeCostToGo(
      const std::vector<DiscreteConditional::shared_ptr>& conditionals);

  /// Expand the next node in the search tree.
  void expandNextNode() const;

  std::vector<DiscreteConditional::shared_ptr> conditionals_;
  std::vector<double> costToGo_;
};
}  // namespace gtsam
