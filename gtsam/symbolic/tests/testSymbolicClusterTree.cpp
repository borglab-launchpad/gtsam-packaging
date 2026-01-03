/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicClusterTree.cpp
 * @brief   Unit tests for Cluster Tree
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicEliminationTree.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include "symbolicExampleGraphs.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(ClusterTree, SeparatorKeys) {
  const Ordering order{0, 1, 2, 3};

  SymbolicJunctionTree tree(SymbolicEliminationTree(simpleChain, order));

  auto root = tree.roots().front();
  auto child = root->children.front();

  const KeySet expectedRoot;
  const KeySet expectedChild{2};

  EXPECT(assert_container_equality(expectedRoot, root->separatorKeys()));
  EXPECT(assert_container_equality(expectedChild, child->separatorKeys()));

  SymbolicJunctionTree::Cluster::KeySetMap cache;
  EXPECT(assert_container_equality(expectedRoot, root->separatorKeys(&cache)));
  EXPECT(
      assert_container_equality(expectedChild, child->separatorKeys(&cache)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
