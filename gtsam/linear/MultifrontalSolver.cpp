/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalSolver.cpp
 * @brief Implementation of imperative-style multifrontal solver.
 * @author Frank Dellaert
 * @date   December 2025
 */

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <ostream>
#include <set>
#include <stdexcept>
#include <utility>

namespace gtsam {

namespace {

// Compute variable dimensions from the GaussianFactorGraph
std::map<Key, size_t> computeDims(const GaussianFactorGraph& graph) {
  std::map<Key, size_t> dims;
  for (const auto& factor : graph) {
    if (!factor) continue;
    if (auto jacobianFactor =
            std::dynamic_pointer_cast<JacobianFactor>(factor)) {
      for (auto it = jacobianFactor->begin(); it != jacobianFactor->end();
           ++it) {
        dims[*it] = jacobianFactor->getDim(it);
      }
    } else if (auto hessianFactor =
                   std::dynamic_pointer_cast<HessianFactor>(factor)) {
      throw std::runtime_error(
          "MultifrontalSolver: HessianFactors not supported.");
    }
  }
  return dims;
}

// Build SymbolicFactorGraph from GaussianFactorGraph
SymbolicFactorGraph buildSymbolicGraph(const GaussianFactorGraph& graph) {
  SymbolicFactorGraph symbolicGraph;
  symbolicGraph.reserve(graph.size());
  for (size_t i = 0; i < graph.size(); ++i) {
    if (!graph[i]) continue;
    symbolicGraph.emplace_shared<internal::IndexedSymbolicFactor>(*graph[i], i);
  }
  return symbolicGraph;
}

}  // namespace

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(const GaussianFactorGraph& graph,
                                       const Ordering& ordering) {
  // 0. Pre-compute variable dimensions
  dims_ = computeDims(graph);
  for (Key key : ordering) {
    solution_.insert(key, Vector::Zero(dims_.at(key)));
  }

  // 1. Convert to SymbolicFactorGraph to build the elimination tree
  SymbolicFactorGraph symbolicGraph = buildSymbolicGraph(graph);

  // 2. Build SymbolicEliminationTree and then SymbolicJunctionTree
  SymbolicEliminationTree eliminationTree(symbolicGraph, ordering);
  SymbolicJunctionTree junctionTree(eliminationTree);

  // 3. Recursive function to build Clique hierarchy
  std::function<CliquePtr(const SymbolicJunctionTree::sharedNode&,
                          std::weak_ptr<MultifrontalClique>)>
      buildRecursive =
          [&](const SymbolicJunctionTree::sharedNode& cluster,
              std::weak_ptr<MultifrontalClique> parent) -> CliquePtr {
    if (!cluster) return nullptr;

    // Create Clique
    auto clique = std::make_shared<MultifrontalClique>(cluster);
    clique->setParent(parent);
    cliques_.push_back(clique);

    // Process children
    for (const auto& childCluster : cluster->children) {
      auto childClique = buildRecursive(childCluster, clique);
      clique->addChild(childClique);
    }

    clique->calculateSeparatorKeys();
    clique->cacheValuePointers(&solution_);

    // Initialize matrices
    std::vector<size_t> blockDims = clique->blockDims(dims_);
    size_t vbmRows = clique->countRows(graph);
    clique->initializeMatrices(blockDims, vbmRows);

    // Initial load
    clique->fillAb(graph);

    // Pre-compute parent mapping after separators are finalized.
    clique->assignParentIndicesForChildren();

    postOrderCliques_.push_back(clique);
    return clique;
  };

  // 4. Start traversal from roots
  for (const auto& rootCluster : junctionTree.roots()) {
    if (rootCluster) {
      roots_.push_back(
          buildRecursive(rootCluster, std::weak_ptr<MultifrontalClique>()));
    }
  }
}

/* ************************************************************************* */
void MultifrontalSolver::load(const GaussianFactorGraph& graph) {
  for (auto& clique : cliques_) {
    clique->fillAb(graph);
  }
}

/* ************************************************************************* */
void MultifrontalSolver::eliminate() {
  for (auto& clique : postOrderCliques_) {
    clique->eliminate();
  }
}

/* ************************************************************************* */
const VectorValues& MultifrontalSolver::solve() const {
  for (const auto& clique : cliques_) {
    clique->solve();
  }
  return solution_;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const MultifrontalSolver& solver) {
  os << "MultifrontalSolver(roots=" << solver.roots_.size()
     << ", cliques=" << solver.cliques_.size()
     << ", postOrder=" << solver.postOrderCliques_.size()
     << ", dims=" << solver.dims_.size() << ")\n";

  std::function<void(const MultifrontalSolver::CliquePtr&, int)> dump =
      [&](const MultifrontalSolver::CliquePtr& clique, int depth) {
        if (!clique) return;
        os << std::string(depth * 2, ' ') << *clique << "\n";
        for (const auto& child : clique->children()) {
          dump(child, depth + 1);
        }
      };

  for (const auto& root : solver.roots_) {
    dump(root, 0);
  }
  return os;
}

/* ************************************************************************* */
void MultifrontalSolver::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  if (!s.empty()) std::cout << s;
  std::cout << "MultifrontalSolver matrices (cliques=" << cliques_.size()
            << ")\n";
  for (const auto& clique : cliques_) {
    if (!clique) continue;
    clique->print("", keyFormatter);
  }
}

}  // namespace gtsam
