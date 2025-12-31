/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalClique.cpp
 * @brief Implementation of imperative multifrontal clique data structure.
 * @author Frank Dellaert
 * @date   December 2025
 */

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <set>
#include <stdexcept>

namespace gtsam {

namespace {

// Build a stacked separator vector x_sep in the provided scratch buffer.
Vector& buildSeparatorVector(const std::vector<const Vector*>& separatorPtrs,
                             Vector* scratch) {
  size_t separatorDim = 0;
  for (const Vector* values : separatorPtrs) {
    separatorDim += values->size();
  }
  if (static_cast<size_t>(scratch->size()) != separatorDim) {
    scratch->resize(separatorDim);
  }
  size_t offset = 0;
  for (const Vector* values : separatorPtrs) {
    scratch->segment(offset, values->size()) = *values;
    offset += values->size();
  }
  return *scratch;
}

}  // namespace

MultifrontalClique::MultifrontalClique(
    const SymbolicJunctionTree::sharedNode& cluster) {
  if (!cluster) {
    throw std::runtime_error("MultifrontalSolver: null cluster.");
  }
  cluster_ = cluster;
  const auto& frontals = cluster_->orderedFrontalKeys;
  if (frontals.empty()) {
    throw std::runtime_error(
        "MultifrontalSolver: cluster has no frontal keys.");
  }
  key_ = frontals.front();
}

void MultifrontalClique::setParent(
    const std::weak_ptr<MultifrontalClique>& parent) {
  parent_ = parent;
}

void MultifrontalClique::addChild(const shared_ptr& child) {
  children_.push_back(child);
}

const KeyVector& MultifrontalClique::frontals() const {
  return cluster_->orderedFrontalKeys;
}

const std::vector<MultifrontalClique::shared_ptr>&
MultifrontalClique::children() const {
  return children_;
}

const std::weak_ptr<MultifrontalClique>& MultifrontalClique::parent() const {
  return parent_;
}

void MultifrontalClique::assignParentIndicesForChildren() {
  for (const auto& child : children_) {
    if (!child) continue;
    child->setParentIndices(child->parentIndicesFor(*this));
  }
}

Key MultifrontalClique::key() const { return key_; }

size_t MultifrontalClique::factorCount() const {
  return cluster_->factors.size();
}

void MultifrontalClique::calculateSeparatorKeys() {
  // Separator keys are computed from local factor keys and child separators.
  KeySet allKeys;
  for (const auto& factor : cluster_->factors) {
    if (!factor) continue;
    allKeys.insert(factor->begin(), factor->end());
  }
  for (const auto& child : children_) {
    if (!child) continue;
    allKeys.insert(child->separatorKeys_.begin(), child->separatorKeys_.end());
  }
  for (Key k : frontals()) {
    allKeys.erase(k);
  }
  separatorKeys_.assign(allKeys.begin(), allKeys.end());
}

void MultifrontalClique::cacheValuePointers(VectorValues* values) {
  frontalPtrs_.clear();
  separatorPtrs_.clear();
  frontalPtrs_.reserve(frontals().size());
  separatorPtrs_.reserve(separatorKeys_.size());
  for (Key key : frontals()) {
    frontalPtrs_.push_back(&values->at(key));
  }
  for (Key key : separatorKeys_) {
    separatorPtrs_.push_back(&values->at(key));
  }
}

const KeyVector& MultifrontalClique::separatorKeys() const {
  return separatorKeys_;
}

std::vector<size_t> MultifrontalClique::blockDims(
    const std::map<Key, size_t>& dims) const {
  std::vector<size_t> blockDims;
  for (Key k : frontals()) blockDims.push_back(dims.at(k));
  for (Key k : separatorKeys_) blockDims.push_back(dims.at(k));
  return blockDims;
}

size_t MultifrontalClique::countRows(const GaussianFactorGraph& graph) const {
  size_t vbmRows = 0;
  for (const auto& factor : cluster_->factors) {
    auto indexed =
        std::dynamic_pointer_cast<internal::IndexedSymbolicFactor>(factor);
    if (!indexed) continue;
    if (auto jacobianFactor =
            std::dynamic_pointer_cast<JacobianFactor>(graph[indexed->index_])) {
      vbmRows += jacobianFactor->rows();
    }
  }
  return vbmRows;
}

std::vector<size_t> MultifrontalClique::parentIndicesFor(
    const MultifrontalClique& parent) const {
  std::vector<size_t> indices;
  for (Key k : separatorKeys_) {
    auto fIt = std::find(parent.frontals().begin(), parent.frontals().end(), k);
    if (fIt != parent.frontals().end()) {
      indices.push_back(std::distance(parent.frontals().begin(), fIt));
      continue;
    }
    auto sIt = std::find(parent.separatorKeys_.begin(),
                         parent.separatorKeys_.end(), k);
    if (sIt != parent.separatorKeys_.end()) {
      indices.push_back(parent.frontals().size() +
                        std::distance(parent.separatorKeys_.begin(), sIt));
      continue;
    }
    throw std::runtime_error(
        "MultifrontalSolver: separator key not found in parent clique");
  }
  return indices;
}

void MultifrontalClique::initializeMatrices(
    const std::vector<size_t>& blockDims, size_t verticalBlockMatrixRows) {
  sbm_ = SymmetricBlockMatrix(blockDims, true);
  Ab_ = VerticalBlockMatrix(blockDims, verticalBlockMatrixRows, true);
  Ab_.matrix().setZero();
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  sbm_.setZero();

  // We only overwrite the fixed sparsity pattern, so Ab must be zeroed once in
  // initializeMatrices and then kept consistent across loads.
  size_t rowOffset = 0;
  for (const auto& factor : cluster_->factors) {
    auto indexed =
        std::dynamic_pointer_cast<internal::IndexedSymbolicFactor>(factor);
    if (!indexed) continue;
    auto jacobianFactor =
        std::dynamic_pointer_cast<JacobianFactor>(graph[indexed->index_]);
    if (!jacobianFactor) continue;

    for (auto it = jacobianFactor->begin(); it != jacobianFactor->end(); ++it) {
      Key k = *it;
      auto fIt = std::find(frontals().begin(), frontals().end(), k);
      size_t blockIdx = 0;
      if (fIt != frontals().end()) {
        blockIdx = std::distance(frontals().begin(), fIt);
      } else {
        auto sIt = std::find(separatorKeys_.begin(), separatorKeys_.end(), k);
        if (sIt != separatorKeys_.end()) {
          blockIdx =
              frontals().size() + std::distance(separatorKeys_.begin(), sIt);
        } else
          continue;
      }
      Ab_(blockIdx).middleRows(rowOffset, jacobianFactor->rows()) =
          jacobianFactor->getA(it);
    }
    size_t rhsBlockIdx = Ab_.nBlocks() - 1;  // RHS block is appended by VBM.
    Ab_(rhsBlockIdx).middleRows(rowOffset, jacobianFactor->rows()) =
        jacobianFactor->getb();
    rowOffset += jacobianFactor->rows();
  }
}

void MultifrontalClique::eliminate() {
  // Update SBM with the local factors, Ab^T * Ab
  sbm_.selfadjointView().rankUpdate(Ab_.matrix().transpose());
  
  // Form normal equations and factor the frontal block (Schur complement step).
  sbm_.choleskyPartial(frontals().size());

  auto parent = parent_.lock();
  if (!parent) return;
  // Expose only the separator+RHS view when contributing to the parent.
  sbm_.blockStart() = frontals().size();
  parent->updateWith(sbm_, parentIndices_);
  sbm_.blockStart() = 0;
}

void MultifrontalClique::updateWith(const SymmetricBlockMatrix& separator,
                                    const std::vector<size_t>& indices) {
  const size_t numBlocks = indices.size();
  assert(separator.nBlocks() == numBlocks + 1);
  const size_t rhsBlock = sbm_.nBlocks() - 1;

  for (size_t i = 0; i <= numBlocks; ++i) {
    const size_t p_i = (i < numBlocks) ? indices[i] : rhsBlock;
    sbm_.updateDiagonalBlock(p_i, separator.diagonalBlock(i));
    for (size_t j = i + 1; j <= numBlocks; ++j) {
      const size_t p_j = (j < numBlocks) ? indices[j] : rhsBlock;
      sbm_.updateOffDiagonalBlock(p_i, p_j, separator.aboveDiagonalBlock(i, j));
    }
  }
}

void MultifrontalClique::solve() const {
  // Solve with block back-substitution on the Cholesky-stored SBM, avoiding
  // materializing an explicit R matrix or split representation.
  const size_t nFrontals = frontalPtrs_.size();
  const size_t nSeparators = separatorPtrs_.size();

  const size_t rhsBlock = nFrontals + nSeparators;

  size_t frontalDim = 0;
  for (const Vector* values : frontalPtrs_) {
    frontalDim += values->size();
  }
  if (static_cast<size_t>(rhsScratch_.size()) != frontalDim) {
    rhsScratch_.resize(frontalDim);
  }
  rhsScratch_.noalias() =
      sbm_.aboveDiagonalRange(0, nFrontals, rhsBlock, rhsBlock + 1);

  // Eliminate separator contributions: b -= S * x_sep.
  if (nSeparators > 0) {
    const Vector& xSep =
        buildSeparatorVector(separatorPtrs_, &separatorScratch_);
    rhsScratch_.noalias() -= sbm_.aboveDiagonalRange(0, nFrontals, nFrontals,
                                                     nFrontals + nSeparators) *
                             xSep;
  }

  // Solve the contiguous frontal system in one triangular solve.
  sbm_.triangularView(0, nFrontals).solveInPlace(rhsScratch_);

  // Write solved frontal blocks back into the global solution.
  size_t offset = 0;
  for (Vector* values : frontalPtrs_) {
    const size_t dim = values->size();
    values->noalias() = rhsScratch_.segment(offset, dim);
    offset += dim;
  }
}

void MultifrontalClique::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  if (!s.empty()) std::cout << s;
  std::cout << "Clique(key=" << keyFormatter(key_) << ", frontals=[";
  for (size_t i = 0; i < frontals().size(); ++i) {
    std::cout << keyFormatter(frontals()[i]);
    if (i + 1 < frontals().size()) std::cout << ", ";
  }
  std::cout << "], separators=[";
  for (size_t i = 0; i < separatorKeys_.size(); ++i) {
    std::cout << keyFormatter(separatorKeys_[i]);
    if (i + 1 < separatorKeys_.size()) std::cout << ", ";
  }
  std::cout << "], factors=" << cluster_->factors.size()
            << ", children=" << children_.size()
            << ", sbmBlocks=" << sbm_.nBlocks()
            << ", AbRows=" << Ab_.matrix().rows() << ")\n";

  auto assembleSbm = [](const SymmetricBlockMatrix& sbm) {
    const size_t nBlocks = sbm.nBlocks();
    std::vector<size_t> offsets(nBlocks + 1, 0);
    for (size_t i = 0; i < nBlocks; ++i) {
      offsets[i + 1] = offsets[i] + sbm.getDim(i);
    }
    Matrix full = Matrix::Zero(offsets.back(), offsets.back());
    for (size_t i = 0; i < nBlocks; ++i) {
      for (size_t j = 0; j < nBlocks; ++j) {
        Matrix block = sbm.block(i, j);
        full.block(offsets[i], offsets[j], block.rows(), block.cols()) = block;
      }
    }
    return full;
  };

  std::cout << "  Ab:\n" << Ab_.matrix() << "\n";
  std::cout << "  SBM:\n" << assembleSbm(sbm_) << "\n";
}

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique) {
  const KeyFormatter formatter = DefaultKeyFormatter;
  auto printKeys = [&](const KeyVector& keys) {
    os << "[";
    for (size_t i = 0; i < keys.size(); ++i) {
      os << formatter(keys[i]);
      if (i + 1 < keys.size()) os << ", ";
    }
    os << "]";
  };

  os << "Clique(key=" << formatter(clique.key()) << ", frontals=";
  printKeys(clique.frontals());
  os << ", separators=";
  printKeys(clique.separatorKeys());
  os << ", factors=" << clique.factorCount();
  os << ", children=" << clique.children().size();
  os << ", sbmBlocks=" << clique.sbm().nBlocks();
  os << ", AbRows=" << clique.Ab().matrix().rows() << ")";
  return os;
}

}  // namespace gtsam
