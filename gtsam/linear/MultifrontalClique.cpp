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

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

namespace gtsam {

namespace {

KeyVector orderedKeysFromBlockIndex(const std::map<Key, size_t>& blockIndex) {
  const size_t totalKeys = blockIndex.size();
  KeyVector orderedKeys(totalKeys);
  for (const auto& entry : blockIndex) {
    if (entry.second < totalKeys) {
      orderedKeys[entry.second] = entry.first;
    }
  }
  return orderedKeys;
}

void printKeyRange(std::ostream& os, const KeyVector& keys, size_t start,
                   size_t end, const KeyFormatter& formatter) {
  os << "[";
  for (size_t i = start; i < end; ++i) {
    os << formatter(keys[i]);
    if (i + 1 < end) os << ", ";
  }
  os << "]";
}

// Build a stacked separator vector x_sep in the provided scratch buffer.
Vector& buildSeparatorVector(const std::vector<const Vector*>& separatorPtrs,
                             Vector* scratch) {
  size_t offset = 0;
  for (const Vector* values : separatorPtrs) {
    scratch->segment(offset, values->size()) = *values;
    offset += values->size();
  }
  return *scratch;
}

#ifndef NDEBUG
bool validateFactorKeys(const GaussianFactorGraph& graph,
                        const std::vector<size_t>& factorIndices,
                        const std::map<Key, size_t>& blockIndex,
                        const std::unordered_set<Key>* fixedKeys) {
  for (size_t index : factorIndices) {
    assert(index < graph.size());
    const GaussianFactor::shared_ptr& gf = graph[index];
    if (!gf) continue;
    for (Key key : gf->keys()) {
      if (blockIndex.find(key) != blockIndex.end()) continue;
      if (fixedKeys && fixedKeys->count(key)) continue;
      return false;
    }
  }
  return true;
}
#endif

}  // namespace

MultifrontalClique::MultifrontalClique(
    std::vector<size_t> factorIndices,
    const std::weak_ptr<MultifrontalClique>& parent, const KeyVector& frontals,
    const KeyVector& separatorKeys, const std::map<Key, size_t>& dims,
    const GaussianFactorGraph& graph, VectorValues* solution,
    const std::unordered_set<Key>* fixedKeys) {
  factorIndices_ = std::move(factorIndices);
  this->parent = parent;
  fixedKeys_ = fixedKeys;

  if (frontals.empty()) {
    throw std::runtime_error(
        "MultifrontalSolver: cluster has no frontal keys.");
  }

  // Cache the mapping from key to Ab block index for fast fills.
  blockIndex_.clear();
  size_t blockIdx = 0;
  for (Key key : frontals) {
    blockIndex_[key] = blockIdx;
    ++blockIdx;
  }
  for (Key key : separatorKeys) {
    blockIndex_[key] = blockIdx;
    ++blockIdx;
  }

  size_t dim = 0;
  for (Key key : frontals) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  frontalDim = dim;

  dim = 0;
  for (Key key : separatorKeys) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  separatorDim = dim;

  rhsScratch_.resize(frontalDim);
  separatorScratch_.resize(separatorDim);

  // Cache pointers into the solution for fast back-substitution.
  cacheSolutionPointers(solution, frontals, separatorKeys);

  // Pre-allocate matrices once per structure.
  std::vector<size_t> blockDims =
      this->blockDims(dims, frontals, separatorKeys);
  size_t vbmRows = countRows(graph);
  initializeMatrices(blockDims, vbmRows);
}

void MultifrontalClique::finalize(std::vector<ChildInfo> children) {
  this->children.clear();
  this->children.reserve(children.size());
  for (const auto& child : children) {
    this->children.push_back(child.clique);
  }

  // Compute parent indices for all children.
  for (const auto& child : children) {
    if (!child.clique) continue;
    std::vector<DenseIndex> indices;
    indices.reserve(child.separatorKeys.size() + 1);
    for (Key key : child.separatorKeys) {
      auto it = blockIndex_.find(key);
      if (it == blockIndex_.end()) {
        throw std::runtime_error(
            "MultifrontalSolver: separator key not found in parent clique");
      }
      indices.push_back(static_cast<DenseIndex>(it->second));
    }
    indices.push_back(static_cast<DenseIndex>(blockIndex_.size()));
    child.clique->setParentIndices(indices);
  }
}

void MultifrontalClique::cacheSolutionPointers(VectorValues* solution,
                                               const KeyVector& frontals,
                                               const KeyVector& separatorKeys) {
  frontalPtrs_.clear();
  separatorPtrs_.clear();
  frontalPtrs_.reserve(frontals.size());
  separatorPtrs_.reserve(separatorKeys.size());
  for (Key key : frontals) {
    frontalPtrs_.push_back(&solution->at(key));
  }
  for (Key key : separatorKeys) {
    separatorPtrs_.push_back(&solution->at(key));
  }
}

std::vector<size_t> MultifrontalClique::blockDims(
    const std::map<Key, size_t>& dims, const KeyVector& frontals,
    const KeyVector& separatorKeys) const {
  std::vector<size_t> blockDims;
  for (Key k : frontals) blockDims.push_back(dims.at(k));
  for (Key k : separatorKeys) blockDims.push_back(dims.at(k));
  return blockDims;
}

size_t MultifrontalClique::countRows(const GaussianFactorGraph& graph) const {
  size_t vbmRows = 0;
  for (size_t index : factorIndices_) {
    assert(index < graph.size());
    if (auto jacobianFactor =
            std::dynamic_pointer_cast<JacobianFactor>(graph[index])) {
      vbmRows += jacobianFactor->rows();
    }
  }
  return vbmRows;
}

void MultifrontalClique::initializeMatrices(
    const std::vector<size_t>& blockDims, size_t verticalBlockMatrixRows) {
  sbm_ = SymmetricBlockMatrix(blockDims, true);
  Ab_ = VerticalBlockMatrix(blockDims, verticalBlockMatrixRows, true);
  Ab_.matrix().setZero();
}

size_t MultifrontalClique::addJacobianFactor(
    const JacobianFactor& jacobianFactor, size_t rowOffset) {
  // We only overwrite the fixed sparsity pattern, so Ab must be zeroed once in
  // initializeMatrices and then kept consistent across loads.
  const size_t rows = jacobianFactor.rows();
  const size_t rhsBlockIdx = Ab_.nBlocks() - 1;
  for (auto it = jacobianFactor.begin(); it != jacobianFactor.end(); ++it) {
    Key k = *it;
    if (fixedKeys_ && fixedKeys_->count(k)) continue;
    const size_t blockIdx = blockIndex_.at(k);
    Ab_(blockIdx).middleRows(rowOffset, rows) = jacobianFactor.getA(it);
  }
  Ab_(rhsBlockIdx).middleRows(rowOffset, rows) = jacobianFactor.getb();

  if (auto model = jacobianFactor.get_model()) {
    if (!model->isConstrained()) {
      model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, rows));
    }
  }
  return rows;
}

void MultifrontalClique::addHessianFactor(const HessianFactor& hessianFactor) {
  const SymmetricBlockMatrix& info = hessianFactor.info();
  const DenseIndex factorBlocks = static_cast<DenseIndex>(hessianFactor.size());
  const DenseIndex rhsBlock = static_cast<DenseIndex>(sbm_.nBlocks() - 1);

  std::vector<DenseIndex> blockIndices(factorBlocks + 1, -1);
  DenseIndex slot = 0;
  for (auto it = hessianFactor.begin(); it != hessianFactor.end();
       ++it, ++slot) {
    const Key key = *it;
    if (fixedKeys_ && fixedKeys_->count(key)) continue;
    blockIndices[slot] = static_cast<DenseIndex>(blockIndex_.at(key));
  }
  blockIndices[factorBlocks] = rhsBlock;

  sbm_.updateFromMappedBlocks(info, blockIndices);
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  assert(validateFactorKeys(graph, factorIndices_, blockIndex_, fixedKeys_));
  sbm_.setZero();  // Easily half of the cost !

  size_t rowOffset = 0;
  for (size_t index : factorIndices_) {
    assert(index < graph.size());
    const GaussianFactor::shared_ptr& gf = graph[index];
    if (!gf) continue;
    if (auto jacobianFactor = std::dynamic_pointer_cast<JacobianFactor>(gf)) {
      rowOffset += addJacobianFactor(*jacobianFactor, rowOffset);
    } else if (auto hessianFactor =
                   std::dynamic_pointer_cast<HessianFactor>(gf)) {
      addHessianFactor(*hessianFactor);
    }
  }
}

void MultifrontalClique::eliminateInPlace() {
  // Update SBM with the local factors, Ab^T * Ab
  sbm_.selfadjointView().rankUpdate(Ab_.matrix().transpose());

  for (const auto& child : children) {
    if (!child) continue;
    child->updateParent(*this);
  }

  // Form normal equations and factor the frontal block (Schur complement step).
  sbm_.choleskyPartial(numFrontals());
}

void MultifrontalClique::updateParent(MultifrontalClique& parent) const {
  // Expose only the separator+RHS view when contributing to the parent.
  sbm_.blockStart() = numFrontals();
  assert(sbm_.nBlocks() == parentIndices_.size());
  parent.sbm_.updateFromMappedBlocks(sbm_, parentIndices_);
  sbm_.blockStart() = 0;
}

std::shared_ptr<GaussianConditional> MultifrontalClique::conditional() const {
  const KeyVector keys = orderedKeysFromBlockIndex(blockIndex_);
  SymmetricBlockMatrix& sbm = sbm_;
  VerticalBlockMatrix Ab = sbm.split(numFrontals());
  sbm.blockStart() = 0;  // Split sets it to numFrontals(), reset to 0.
  return std::make_shared<GaussianConditional>(keys, numFrontals(),
                                               std::move(Ab));
}

// Solve with block back-substitution on the Cholesky-stored SBM.
void MultifrontalClique::updateSolution() const {
  const size_t nf = numFrontals();
  const size_t n = sbm_.nBlocks() - 1;  // # frontals + # separators

  // The in-place factorization yields an upper-triangular system [R S d]:
  //   R * x_f + S * x_s = d,
  // with x_f the frontals and x_s the separators.
  const auto R = sbm_.triangularView(0, nf);
  const auto S = sbm_.aboveDiagonalRange(0, nf, nf, n);
  const auto d = sbm_.aboveDiagonalRange(0, nf, n, n + 1);

  // We first solve rhs = d - S * x_s
  rhsScratch_.noalias() = d;
  if (n > nf) {
    const Vector& x_s =
        buildSeparatorVector(separatorPtrs_, &separatorScratch_);
    rhsScratch_.noalias() -= S * x_s;
  }

  // Then solve for x_f, our solution, via R * x_f = rhs
  // We solve the contiguous frontal system in one triangular solve.
  R.solveInPlace(rhsScratch_);
  auto& x_f = rhsScratch_;

  // Write solved frontal blocks back into the global solution.
  size_t offset = 0;
  for (Vector* values : frontalPtrs_) {
    const size_t dim = values->size();
    values->noalias() = x_f.segment(offset, dim);
    offset += dim;
  }
}

void MultifrontalClique::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  if (!s.empty()) std::cout << s;
  const KeyVector orderedKeys = orderedKeysFromBlockIndex(blockIndex_);
  std::cout << "Clique(frontals=[";
  printKeyRange(std::cout, orderedKeys, 0,
                std::min(numFrontals(), orderedKeys.size()), keyFormatter);
  std::cout << "], separators=[";
  printKeyRange(std::cout, orderedKeys,
                std::min(numFrontals(), orderedKeys.size()), orderedKeys.size(),
                keyFormatter);
  std::cout << "], factors=" << factorIndices_.size()
            << ", children=" << children.size()
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
  const KeyVector orderedKeys = orderedKeysFromBlockIndex(clique.blockIndex_);
  const KeyFormatter formatter = DefaultKeyFormatter;
  os << "Clique(frontals=";
  printKeyRange(os, orderedKeys, 0,
                std::min(clique.numFrontals(), orderedKeys.size()), formatter);
  os << ", separators=";
  printKeyRange(os, orderedKeys,
                std::min(clique.numFrontals(), orderedKeys.size()),
                orderedKeys.size(), formatter);
  os << ", factors=" << clique.factorIndices_.size();
  os << ", children=" << clique.children.size();
  os << ", sbmBlocks=" << clique.sbm().nBlocks();
  os << ", AbRows=" << clique.Ab().matrix().rows() << ")";
  return os;
}

}  // namespace gtsam
