/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalClique.h
 * @brief Imperative multifrontal clique data structure.
 * @author Frank Dellaert
 * @date   December 2025
 */

#pragma once

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gtsam {

namespace internal {

/// Helper class to track original factor indices.
class IndexedSymbolicFactor : public SymbolicFactor {
 public:
  size_t index_;
  IndexedSymbolicFactor(const GaussianFactor& factor, size_t index)
      : SymbolicFactor(factor), index_(index) {}
};

}  // namespace internal

/**
 * Imperative multifrontal clique structure used by MultifrontalSolver.
 */
class GTSAM_EXPORT MultifrontalClique {
 public:
  using shared_ptr = std::shared_ptr<MultifrontalClique>;

  /// Construct a clique from a symbolic junction tree node.
  /// @param cluster The symbolic junction tree node.
  explicit MultifrontalClique(const SymbolicJunctionTree::sharedNode& cluster);

  /// @name Setup (non-const)
  /// @{

  /// Set the parent clique.
  /// @param parent Weak pointer to the parent clique.
  void setParent(const std::weak_ptr<MultifrontalClique>& parent);

  /// Add a child clique.
  /// @param child Shared pointer to the child clique.
  void addChild(const shared_ptr& child);

  /// Compute parent indices for all children after separators are finalized.
  void assignParentIndicesForChildren();

  /// Cache pointers to frontal and separator update vectors.
  void cacheValuePointers(VectorValues* delta);

  /// Calculate separator keys from children's frontals.
  void calculateSeparatorKeys();

  /// Pre-allocate matrices for this clique.
  /// @param blockDims Block dimensions (excluding RHS).
  /// @param verticalBlockMatrixRows Number of rows for the vertical block
  /// matrix.
  void initializeMatrices(const std::vector<size_t>& blockDims,
                          size_t verticalBlockMatrixRows);

  /// Load factor values into the pre-allocated Ab matrix.
  /// @param graph The factor graph with updated values.
  void fillAb(const GaussianFactorGraph& graph);
  /// @}

  /// @name Read-only methods
  /// @{

  /// Get the frontal keys for this clique.
  const KeyVector& frontals() const;

  /// Get the separator keys for this clique.
  const KeyVector& separatorKeys() const;

  /// Get the children of this clique.
  const std::vector<shared_ptr>& children() const;

  /// Get the parent of this clique.
  const std::weak_ptr<MultifrontalClique>& parent() const;

  /// Get the primary key of this clique (first frontal).
  Key key() const;

  /// Get the number of factors in this clique.
  size_t factorCount() const;

  /// Get the vertical block matrix Ab.
  const VerticalBlockMatrix& Ab() const { return Ab_; }

  /// Get the symmetric block matrix (mutable).
  SymmetricBlockMatrix& sbm() { return sbm_; }

  /// Get the symmetric block matrix (const).
  const SymmetricBlockMatrix& sbm() const { return sbm_; }


  /**
   * Compute block dimensions from variable dimensions (excluding RHS).
   * @param dims Variable dimensions.
   * @return Block dimensions for this clique.
   */
  std::vector<size_t> blockDims(const std::map<Key, size_t>& dims) const;

  /**
   * Count rows needed for the vertical block matrix.
   * @param graph The factor graph.
   * @return Total number of rows.
   */
  size_t countRows(const GaussianFactorGraph& graph) const;

  /**
   * Compute parent scatter indices for this clique.
   * @param parent The parent clique.
   * @return Parent indices for separator blocks (excluding RHS).
   */
  std::vector<size_t> parentIndicesFor(const MultifrontalClique& parent) const;

  /**
   * Print this clique.
   * @param s Optional string prefix.
   * @param keyFormatter Key formatter for printing.
   */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}

  /// @name Solve (non-const)
  /// @{

  /**
   * Eliminate this clique and propagate its separator contribution upward.
   *
   * Computes the local normal equations (SBM) from the stacked Jacobian (Ab),
   * performs partial Cholesky on the frontal blocks, and then updates the
   * parent's SBM using only the separator view (plus RHS) of this clique.
   * Requires parent indices to be precomputed.
   */
  void eliminate();

  /**
   * Update this clique using a child's contribution.
   * @param separator Child clique's SBM restricted to its separator blocks,
   * with the RHS block appended as the last block.
   * @param indices Mapping from the child's separator blocks into this
   * parent's SBM block indices (RHS is implicit).
   */
  void updateWith(const SymmetricBlockMatrix& separator,
                  const std::vector<size_t>& indices);

  /**
   * Solve for this clique's frontal variables and write them back to the
   * cached solution vectors.
   *
   * Uses block back-substitution using the upper triangular-part of the
   * Cholesky-stored SBM, solving the triangular system for the frontal blocks.
   */
  void solve() const;
  /// @}

 private:
  void setParentIndices(const std::vector<size_t>& indices) {
    parentIndices_ = indices;
  }
  Key key_;
  std::weak_ptr<MultifrontalClique> parent_;
  std::vector<shared_ptr> children_;

  VerticalBlockMatrix Ab_;
  mutable SymmetricBlockMatrix sbm_;
  mutable Vector rhsScratch_;
  mutable Vector separatorScratch_;

  SymbolicJunctionTree::sharedNode cluster_;
  KeyVector separatorKeys_;
  std::vector<size_t> parentIndices_;
  std::vector<Vector*> frontalPtrs_;
  std::vector<const Vector*> separatorPtrs_;
};

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique);

}  // namespace gtsam
