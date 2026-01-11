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

#include <gtsam/base/FastMap.h>
#include <gtsam/config.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace gtsam {

namespace {

struct StructureStats {
  size_t count = 0;
  size_t maxFrontals = 0;
  size_t maxSeparators = 0;
  size_t maxTotal = 0;
  size_t maxChildren = 0;
  size_t sumFrontals = 0;
  size_t sumSeparators = 0;
  size_t sumTotal = 0;
  size_t sumChildren = 0;

  void update(size_t frontalDim, size_t separatorDim, size_t childCount) {
    const size_t totalDim = frontalDim + separatorDim;
    count += 1;
    maxFrontals = std::max(maxFrontals, frontalDim);
    maxSeparators = std::max(maxSeparators, separatorDim);
    maxTotal = std::max(maxTotal, totalDim);
    maxChildren = std::max(maxChildren, childCount);
    sumFrontals += frontalDim;
    sumSeparators += separatorDim;
    sumTotal += totalDim;
    sumChildren += childCount;
  }

  void report(const std::string& name, std::ostream* os) {
    auto avg = [this](size_t sum) {
      return count ? static_cast<double>(sum) / count : 0.0;
    };
    const double avgF = avg(sumFrontals);
    const double avgS = avg(sumSeparators);
    const double avgT = avg(sumTotal);
    const double avgC = avg(sumChildren);

    *os << name << "\n";
    *os << "  cliques:    " << count << "\n";
    *os << "  frontals:   max=" << maxFrontals << " avg=" << avgF << "\n";
    *os << "  separators: max=" << maxSeparators << " avg=" << avgS << "\n";
    *os << "  total dim:  max=" << maxTotal << " avg=" << avgT << "\n";
    *os << "  children:   max=" << maxChildren << " avg=" << avgC << "\n";
  }
};

constexpr double kConstraintSigmaTol = 1e-12;
constexpr double kConstraintFeasibleTol = 1e-9;

std::unordered_set<Key> collectFixedKeys(const GaussianFactorGraph& graph) {
  std::unordered_set<Key> fixedKeys;
  for (const auto& factor : graph) {
    if (!factor) continue;
    if (!factor->isJacobian()) {
      throw std::runtime_error(
          "MultifrontalSolver: only JacobianFactor inputs are supported.");
    }
    auto jacobianFactor = std::static_pointer_cast<JacobianFactor>(factor);
    auto model = jacobianFactor->get_model();
    if (!model || !model->isConstrained()) continue;
    // Only accept fully constrained unary factors with zero residuals.
    if (jacobianFactor->size() != 1) {
      throw std::runtime_error(
          "MultifrontalSolver: only unary constrained factors are supported.");
    }
    const Vector sigmas = model->sigmas();
    if (!(sigmas.array().abs() <= kConstraintSigmaTol).all()) {
      throw std::runtime_error(
          "MultifrontalSolver: only fully constrained factors are supported.");
    }
    if (jacobianFactor->getb().array().abs().maxCoeff() >
        kConstraintFeasibleTol) {
      throw std::runtime_error(
          "MultifrontalSolver: constrained factor is not feasible.");
    }
    fixedKeys.insert(*jacobianFactor->begin());
  }
  return fixedKeys;
}

// Compute variable dimensions from the GaussianFactorGraph
std::map<Key, size_t> computeDims(const GaussianFactorGraph& graph) {
  std::map<Key, size_t> dims;
  for (const auto& factor : graph) {
    if (!factor) continue;
    if (!factor->isJacobian()) {
      throw std::runtime_error(
          "MultifrontalSolver: only JacobianFactor inputs are supported.");
    }
    auto jacobianFactor = std::static_pointer_cast<JacobianFactor>(factor);
    for (auto it = jacobianFactor->begin(); it != jacobianFactor->end(); ++it) {
      dims[*it] = jacobianFactor->getDim(it);
    }
  }
  return dims;
}

size_t factorRowCount(const GaussianFactorGraph& graph, size_t index) {
  if (index >= graph.size() || !graph[index]) return 0;
  if (!graph[index]->isJacobian()) {
    throw std::runtime_error(
        "MultifrontalSolver: only JacobianFactor inputs are supported.");
  }
  auto jacobianFactor = std::static_pointer_cast<JacobianFactor>(graph[index]);
  return jacobianFactor->rows();
}

// Build SymbolicFactorGraph from GaussianFactorGraph
SymbolicFactorGraph buildSymbolicGraph(
    const GaussianFactorGraph& graph,
    const std::unordered_set<Key>& fixedKeys) {
  SymbolicFactorGraph symbolicGraph;
  symbolicGraph.reserve(graph.size());
  for (size_t i = 0; i < graph.size(); ++i) {
    if (!graph[i]) continue;
    KeyVector keys;
    keys.reserve(graph[i]->size());
    for (Key key : graph[i]->keys()) {
      if (!fixedKeys.count(key)) {
        keys.push_back(key);
      }
    }
    // Skip factors that are fully constrained away.
    if (keys.empty()) continue;
    symbolicGraph.emplace_shared<internal::IndexedSymbolicFactor>(
        keys, i, factorRowCount(graph, i));
  }
  return symbolicGraph;
}

// Sum the dimensions of frontal variables in a symbolic cluster.
size_t frontalDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims) {
  return internal::sumDims(dims, cluster->orderedFrontalKeys);
}

size_t separatorDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    SymbolicJunctionTree::Cluster::KeySetMap* cache) {
  if (!cluster) return 0;
  const KeySet separatorKeys = cluster->separatorKeys(cache);
  return internal::sumDims(dims, separatorKeys);
}

size_t totalDimForSymbolicCluster(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    SymbolicJunctionTree::Cluster::KeySetMap* cache) {
  return frontalDimForSymbolicCluster(cluster, dims) +
         separatorDimForSymbolicCluster(cluster, dims, cache);
}

struct LeafInfo {
  SymbolicJunctionTree::sharedNode child;
  size_t totalDim = 0;
  size_t index = 0;
};

struct LeafGroup {
  std::vector<LeafInfo> leaves;
  size_t firstIndex = 0;
};

struct LeafBatch {
  SymbolicJunctionTree::Cluster::Children leaves;
  size_t firstIndex = 0;
};

std::vector<LeafGroup> collectLeafGroups(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::map<Key, size_t>& dims,
    SymbolicJunctionTree::Cluster::KeySetMap* separatorCache) {
  // Group leaf children by identical separators, keeping first-seen order.
  FastMap<KeySet, size_t> groupIndexBySeparator;
  std::vector<LeafGroup> groups;

  for (size_t i = 0; i < cluster->children.size(); ++i) {
    const auto& child = cluster->children[i];
    if (!child || !child->children.empty()) continue;

    child->separatorKeys(separatorCache);
    const KeySet& separatorKeys = separatorCache->at(child.get());
    auto it = groupIndexBySeparator.find(separatorKeys);
    size_t groupIndex = 0;
    if (it == groupIndexBySeparator.end()) {
      groupIndex = groups.size();
      groupIndexBySeparator.emplace(separatorKeys, groupIndex);
      groups.push_back(LeafGroup{{}, i});
    } else {
      groupIndex = it->second;
    }

    const size_t totalDim =
        totalDimForSymbolicCluster(child, dims, separatorCache);
    groups[groupIndex].leaves.push_back({child, totalDim, i});
  }

  return groups;
}

std::vector<LeafBatch> buildLeafBatches(const std::vector<LeafGroup>& groups,
                                        size_t leafMergeDimCap) {
  // Pack each group into batches capped by total dimension.
  std::vector<LeafBatch> batches;
  for (const auto& group : groups) {
    size_t currentTotalDim = 0;
    LeafBatch batch;
    batch.firstIndex = group.firstIndex;
    for (const auto& leaf : group.leaves) {
      if (!batch.leaves.empty() &&
          currentTotalDim + leaf.totalDim > leafMergeDimCap) {
        batches.push_back(batch);
        batch.leaves.clear();
        currentTotalDim = 0;
        batch.firstIndex = leaf.index;
      }
      if (batch.leaves.empty()) {
        batch.firstIndex = leaf.index;
      }
      batch.leaves.push_back(leaf.child);
      currentTotalDim += leaf.totalDim;
    }
    if (!batch.leaves.empty()) {
      batches.push_back(batch);
    }
  }

  std::sort(batches.begin(), batches.end(),
            [](const LeafBatch& a, const LeafBatch& b) {
              return a.firstIndex < b.firstIndex;
            });
  return batches;
}

bool mergeLeafBatches(const SymbolicJunctionTree::sharedNode& cluster,
                      const std::vector<LeafBatch>& batches) {
  // Merge each batch of siblings into a super-leaf in-place.
  bool anyMerged = false;
  for (const auto& batch : batches) {
    if (batch.leaves.size() <= 1) continue;
    cluster->mergeChildrenSiblings(batch.leaves);
    anyMerged = true;
  }
  return anyMerged;
}

void accumulateSymbolicStats(const SymbolicJunctionTree::sharedNode& cluster,
                             const std::map<Key, size_t>& dims,
                             SymbolicJunctionTree::Cluster::KeySetMap* cache,
                             StructureStats* stats) {
  if (!cluster) return;
  const size_t frontalDim = frontalDimForSymbolicCluster(cluster, dims);
  const size_t separatorDim =
      separatorDimForSymbolicCluster(cluster, dims, cache);
  stats->update(frontalDim, separatorDim, cluster->children.size());
  for (const auto& child : cluster->children) {
    accumulateSymbolicStats(child, dims, cache, stats);
  }
}

/**
 * @brief Recursively groups leaf children into super-leaves capped by total
 * dimension.
 *
 * This function traverses the symbolic junction tree rooted at the given
 * cluster node. For each cluster, it gathers direct leaf children (i.e.,
 * children with no further descendants) and groups those with identical
 * separators into new super-leaves until the running total dimension reaches
 * the specified leafMergeDimCap, then starts a new group.
 *
 * The merging process works as follows:
 * - Recursively process all children clusters first.
 * - Identify all direct leaf children of the current cluster and collect their
 * total dimensions (separator + frontal) and separator keys.
 * - Group leaf children by identical separator keys, preserving first-seen
 * order for each separator.
 * - Build super-leaves by accumulating leaf children until the total dimension
 * hits the cap, then start a new super-leaf.
 * - Replace leaf children with the new super-leaves (non-leaf children keep
 * their relative order and grouped leaves appear at the first leaf location
 * for that separator).
 *
 * @param cluster         The current symbolic junction tree node (cluster) to
 * process.
 * @param dims            A map from variable keys to their dimensions.
 * @param leafMergeDimCap The maximum allowed dimension for a merged cluster.
 */
void mergeLeafChildren(const SymbolicJunctionTree::sharedNode& cluster,
                       const std::map<Key, size_t>& dims,
                       size_t leafMergeDimCap) {
  if (!cluster || cluster->children.empty()) return;
  for (const auto& child : cluster->children) {
    mergeLeafChildren(child, dims, leafMergeDimCap);
  }

  SymbolicJunctionTree::Cluster::KeySetMap separatorCache;
  const std::vector<LeafGroup> groups =
      collectLeafGroups(cluster, dims, &separatorCache);
  const std::vector<LeafBatch> batches =
      buildLeafBatches(groups, leafMergeDimCap);
  if (!mergeLeafBatches(cluster, batches)) return;
}

// Bottom-up merge of child clusters whose merged total dimension stays below
// a threshold.
void mergeSmallClusters(const SymbolicJunctionTree::sharedNode& cluster,
                        const std::map<Key, size_t>& dims, size_t mergeDimCap) {
  if (!cluster) return;
  for (const auto& child : cluster->children) {
    mergeSmallClusters(child, dims, mergeDimCap);
  }
  if (cluster->children.empty()) return;

  const size_t parentTotalDim =
      totalDimForSymbolicCluster(cluster, dims, nullptr);
  std::vector<bool> merge(cluster->children.size(), false);
  bool any = false;
  for (size_t i = 0; i < cluster->children.size(); ++i) {
    const auto& child = cluster->children[i];
    if (!child) continue;
    const size_t childFrontalDim = frontalDimForSymbolicCluster(child, dims);
    if (childFrontalDim + parentTotalDim < mergeDimCap) {
      merge[i] = true;
      any = true;
    }
  }
  if (any) {
    cluster->mergeChildren(merge);
  }
}

void reportStructure(const SymbolicJunctionTree& junctionTree,
                     const std::map<Key, size_t>& dims, const std::string& name,
                     std::ostream* reportStream) {
  if (!reportStream) return;
  // Accumulate stats using cached separator keys.
  StructureStats stats;
  SymbolicJunctionTree::Cluster::KeySetMap separatorCache;
  for (const auto& rootCluster : junctionTree.roots()) {
    accumulateSymbolicStats(rootCluster, dims, &separatorCache, &stats);
  }
  stats.report(name, reportStream);
}

struct BuiltClique {
  MultifrontalSolver::CliquePtr clique;
  KeySet separatorKeys;
};

// Build cliques from a symbolic junction tree and wire parent/child metadata.
struct CliqueBuilder {
  const std::map<Key, size_t>& dims;
  VectorValues* solution;
  std::vector<MultifrontalSolver::CliquePtr>* cliques;
  const std::unordered_set<Key>* fixedKeys;
  SymbolicJunctionTree::Cluster::KeySetMap separatorCache = {};

  BuiltClique build(const SymbolicJunctionTree::sharedNode& cluster,
                    std::weak_ptr<MultifrontalClique> parent) {
    if (!cluster) return {nullptr, KeySet()};

    // Gather symbolic metadata for this clique.
    const KeyVector& frontals = cluster->orderedFrontalKeys;
    const KeySet& separatorKeys = cluster->separatorKeys(&separatorCache);
    std::vector<size_t> factorIndices;
    factorIndices.reserve(cluster->factors.size());
    size_t vbmRows = 0;
    for (const auto& factor : cluster->factors) {
      assert(factor);
      auto indexed =
          std::static_pointer_cast<internal::IndexedSymbolicFactor>(factor);
      factorIndices.push_back(indexed->index_);
      vbmRows += indexed->rows_;
    }

    // Create the clique node and cache static structure.
    auto clique = std::make_shared<MultifrontalClique>(
        std::move(factorIndices), parent, frontals, separatorKeys, dims,
        vbmRows, solution, fixedKeys);

    // Build children and collect separator keys.
    std::vector<MultifrontalClique::ChildInfo> childInfos;
    childInfos.reserve(cluster->children.size());
    for (const auto& childCluster : cluster->children) {
      BuiltClique child = build(childCluster, clique);
      if (child.clique) {
        childInfos.push_back(MultifrontalClique::ChildInfo{
            child.clique, std::move(child.separatorKeys)});
      }
    }

    // Finalize children metadata and register clique.
    clique->finalize(std::move(childInfos));
    cliques->push_back(clique);
    return {clique, std::move(separatorKeys)};
  }
};

size_t resolveThreadCount(size_t requested) {
  if (requested != 0) return requested;
  unsigned int hardwareThreads = std::thread::hardware_concurrency();
  if (hardwareThreads == 0) hardwareThreads = 1;
  size_t threads = (hardwareThreads * 3) / 4;
  return threads == 0 ? 1 : threads;
}

}  // namespace

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(const GaussianFactorGraph& graph,
                                       const Ordering& ordering,
                                       const Parameters& params)
    : MultifrontalSolver(Precompute(graph, ordering), ordering, params) {}

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(const GaussianFactorGraph& graph,
                                       const Ordering& ordering)
    : MultifrontalSolver(graph, ordering, Parameters{}) {}

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(PrecomputedData data,
                                       const Ordering& ordering,
                                       const Parameters& params)
    : ForestTraversal<MultifrontalSolver, MultifrontalClique>(
          resolveThreadCount(params.numThreads)),
      params_(params),
      numThreads_(resolveThreadCount(params.numThreads)) {
  dims_ = std::move(data.dims);
  fixedKeys_ = std::move(data.fixedKeys);

  // Seed the cached solution with zero vectors for all variables.
  for (Key key : ordering) {
    solution_.insert(key, Vector::Zero(dims_.at(key)));
  }
  for (Key key : fixedKeys_) {
    if (!solution_.exists(key)) {
      solution_.insert(key, Vector::Zero(dims_.at(key)));
    }
  }

  // Report the symbolic structure before any merge.
  reportStructure(data.junctionTree, dims_, "Symbolic cluster structure",
                  params_.reportStream);

  // If applicable, merge leaf children by a separate cap first.
  if (params_.leafMergeDimCap > 0) {
    for (const auto& rootCluster : data.junctionTree.roots()) {
      mergeLeafChildren(rootCluster, dims_, params_.leafMergeDimCap);
    }
    reportStructure(data.junctionTree, dims_,
                    "Clique structure after leaf merge", params_.reportStream);
  }

  // If applicable, merge small child cliques bottom-up.
  if (params_.mergeDimCap > 0) {
    for (const auto& rootCluster : data.junctionTree.roots()) {
      mergeSmallClusters(rootCluster, dims_, params_.mergeDimCap);
    }
    reportStructure(data.junctionTree, dims_, "Clique structure after merge",
                    params_.reportStream);
  }

  // Build the actual MultifrontalClique structure.
  CliqueBuilder builder{dims_, &solution_, &cliques_, &fixedKeys_};
  for (const auto& rootCluster : data.junctionTree.roots()) {
    if (rootCluster) {
      roots_.push_back(
          builder.build(rootCluster, std::weak_ptr<MultifrontalClique>())
              .clique);
    }
  }

  // Caller is responsible for loading numerical values via load().
}

/* ************************************************************************* */
MultifrontalSolver::MultifrontalSolver(PrecomputedData data,
                                       const Ordering& ordering)
    : MultifrontalSolver(std::move(data), ordering, Parameters{}) {}

/* ************************************************************************* */
MultifrontalSolver::PrecomputedData MultifrontalSolver::Precompute(
    const GaussianFactorGraph& graph, const Ordering& ordering) {
  auto dims = computeDims(graph);
  auto fixedKeys = collectFixedKeys(graph);

  Ordering reducedOrdering;
  reducedOrdering.reserve(ordering.size());
  for (Key key : ordering) {
    if (!fixedKeys.count(key)) {
      reducedOrdering.push_back(key);
    }
  }

  SymbolicFactorGraph symbolicGraph = buildSymbolicGraph(graph, fixedKeys);
  SymbolicEliminationTree eliminationTree(symbolicGraph, reducedOrdering);
  SymbolicJunctionTree junctionTree(eliminationTree);

  return MultifrontalSolver::PrecomputedData{
      std::move(dims), std::move(fixedKeys), std::move(junctionTree)};
}

/* ************************************************************************* */
void MultifrontalSolver::load(const GaussianFactorGraph& graph) {
  for (auto& clique : cliques_) {
    clique->fillAb(graph);
  }
  loaded_ = true;
  eliminated_ = false;
}

/* ************************************************************************* */
void MultifrontalSolver::eliminateInPlace() {
  if (!loaded_) {
    throw std::runtime_error(
        "MultifrontalSolver::eliminateInPlace: load() must be called before "
        "eliminating.");
  }
  eliminated_ = false;
  runBottomUp([](MultifrontalClique& node) { node.eliminateInPlace(); },
              params_.eliminationParallelThreshold);
  eliminated_ = true;
}

/* ************************************************************************* */
void MultifrontalSolver::eliminateInPlace(const GaussianFactorGraph& graph) {
  // Combine load + eliminate in one post-order traversal to improve locality.
  runBottomUp(
      [&graph](MultifrontalClique& node) {
        node.fillAb(graph);
        node.eliminateInPlace();
      },
      params_.eliminationParallelThreshold);
  loaded_ = true;
  eliminated_ = true;
}

/* ************************************************************************* */
GaussianBayesTree MultifrontalSolver::computeBayesTree() const {
  assert(loaded_ && eliminated_);
  GaussianBayesTree bayesTree;
  using Clique = GaussianBayesTreeClique;
  using BayesCliquePtr = GaussianBayesTree::sharedClique;

  std::function<void(const CliquePtr&, const BayesCliquePtr&)> buildClique =
      [&](const CliquePtr& clique, const BayesCliquePtr& parent) {
        if (!clique) return;
        auto conditional = clique->conditional();
        auto bayesClique = std::make_shared<Clique>(conditional);
        bayesTree.addClique(bayesClique, parent);
        for (const auto& child : clique->children) {
          buildClique(child, bayesClique);
        }
      };

  for (const auto& root : roots_) {
    buildClique(root, BayesCliquePtr());
  }

  if (!fixedKeys_.empty()) {
    for (Key key : fixedKeys_) {
      auto dimIt = dims_.find(key);
      if (dimIt == dims_.end()) {
        throw std::runtime_error(
            "MultifrontalSolver: fixed key has unknown dimension.");
      }
      const size_t dim = dimIt->second;
      Vector d = Vector::Zero(dim);
      Matrix R = Matrix::Identity(dim, dim);
      auto model = noiseModel::Constrained::All(dim);
      auto conditional =
          std::make_shared<GaussianConditional>(key, d, R, model);
      auto bayesClique = std::make_shared<Clique>(conditional);
      bayesTree.addClique(bayesClique, BayesCliquePtr());
    }
  }
  return bayesTree;
}

/* ************************************************************************* */
const VectorValues& MultifrontalSolver::updateSolution() {
  assert(loaded_ && eliminated_);
  runTopDown([](MultifrontalClique& node) { node.updateSolution(); },
             params_.solutionParallelThreshold);

  for (Key key : fixedKeys_) {
    solution_.at(key).setZero();
  }
  return solution_;
}

/* ************************************************************************* */
std::ostream& operator<<(std::ostream& os, const MultifrontalSolver& solver) {
  os << "MultifrontalSolver(roots=" << solver.roots_.size()
     << ", cliques=" << solver.cliques_.size()
     << ", dims=" << solver.dims_.size() << ")\n";

  std::function<void(const MultifrontalSolver::CliquePtr&, int)> dump =
      [&](const MultifrontalSolver::CliquePtr& clique, int depth) {
        if (!clique) return;
        os << std::string(depth * 2, ' ') << *clique << "\n";
        for (const auto& child : clique->children) {
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
