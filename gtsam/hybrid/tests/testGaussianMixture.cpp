/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianMixture.cpp
 * @brief   Unit tests for GaussianMixture class
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>

#include <vector>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/* ************************************************************************* */
/* Check construction of GaussianMixture P(x1 | x2, m1) as well as accessing a
 * specific mode i.e. P(x1 | x2, m1=1).
 */
TEST(GaussianMixture, Equals) {
  // create a conditional gaussian node
  Matrix S1(2, 2);
  S1(0, 0) = 1;
  S1(1, 0) = 2;
  S1(0, 1) = 3;
  S1(1, 1) = 4;

  Matrix S2(2, 2);
  S2(0, 0) = 6;
  S2(1, 0) = 0.2;
  S2(0, 1) = 8;
  S2(1, 1) = 0.4;

  Matrix R1(2, 2);
  R1(0, 0) = 0.1;
  R1(1, 0) = 0.3;
  R1(0, 1) = 0.0;
  R1(1, 1) = 0.34;

  Matrix R2(2, 2);
  R2(0, 0) = 0.1;
  R2(1, 0) = 0.3;
  R2(0, 1) = 0.0;
  R2(1, 1) = 0.34;

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  Vector2 d1(0.2, 0.5), d2(0.5, 0.2);

  auto conditional0 = boost::make_shared<GaussianConditional>(X(1), d1, R1,
                                                              X(2), S1, model),
       conditional1 = boost::make_shared<GaussianConditional>(X(1), d2, R2,
                                                              X(2), S2, model);

  // Create decision tree
  DiscreteKey m1(1, 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  GaussianMixture mixture({X(1)}, {X(2)}, {m1}, conditionals);

  // Let's check that this worked:
  DiscreteValues mode;
  mode[m1.first] = 1;
  auto actual = mixture(mode);
  EXPECT(actual == conditional1);
}

/* ************************************************************************* */
/// Test error method of GaussianMixture.
TEST(GaussianMixture, Error) {
  Matrix22 S1 = Matrix22::Identity();
  Matrix22 S2 = Matrix22::Identity() * 2;
  Matrix22 R1 = Matrix22::Ones();
  Matrix22 R2 = Matrix22::Ones();
  Vector2 d1(1, 2), d2(2, 1);

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  auto conditional0 = boost::make_shared<GaussianConditional>(X(1), d1, R1,
                                                              X(2), S1, model),
       conditional1 = boost::make_shared<GaussianConditional>(X(1), d2, R2,
                                                              X(2), S2, model);

  // Create decision tree
  DiscreteKey m1(M(1), 2);
  GaussianMixture::Conditionals conditionals(
      {m1},
      vector<GaussianConditional::shared_ptr>{conditional0, conditional1});
  GaussianMixture mixture({X(1)}, {X(2)}, {m1}, conditionals);

  VectorValues values;
  values.insert(X(1), Vector2::Ones());
  values.insert(X(2), Vector2::Zero());
  auto error_tree = mixture.error(values);

  // regression
  std::vector<DiscreteKey> discrete_keys = {m1};
  std::vector<double> leaves = {0.5, 4.3252595};
  AlgebraicDecisionTree<Key> expected_error(discrete_keys, leaves);

  EXPECT(assert_equal(expected_error, error_tree, 1e-6));

  // Regression for non-tree version.
  DiscreteValues assignment;
  assignment[M(1)] = 0;
  EXPECT_DOUBLES_EQUAL(0.5, mixture.error(values, assignment), 1e-8);
  assignment[M(1)] = 1;
  EXPECT_DOUBLES_EQUAL(4.3252595155709335, mixture.error(values, assignment), 1e-8);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
