/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeMultifrontalSolver.cpp
 * @brief   Compare MultifrontalSolver against standard elimination
 * @author  Frank Dellaert
 * @date    December 2025
 */

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <tests/smallExample.h>

#include <chrono>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

/// Run standard GTSAM multifrontal elimination and optimization.
static void runStandardSolver(const GaussianFactorGraph& smoother,
                              const Ordering& ordering, size_t iterations) {
  for (size_t i = 0; i < iterations; ++i) {
    GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);
    VectorValues solution = bayesTree.optimize();
  }
}

/// Run new MultifrontalSolver elimination and optimization.
static void runMultifrontalSolver(const GaussianFactorGraph& smoother,
                                  const Ordering& ordering, size_t iterations) {
  MultifrontalSolver solver(smoother, ordering);
  for (size_t i = 0; i < iterations; ++i) {
    solver.load(smoother);
    solver.eliminate();
    const VectorValues& solution = solver.solve();
    (void)solution;
  }
}

int main() {
  const std::vector<size_t> T_values = {10, 50, 100, 500, 1000, 5000};
  const size_t iterations = 1000;

  for (size_t T : T_values) {
    GaussianFactorGraph smoother = createSmoother(T);
    const Ordering ordering = Ordering::Metis(smoother);

    auto start = std::chrono::high_resolution_clock::now();
    runStandardSolver(smoother, ordering, iterations);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_standard = end - start;

    start = std::chrono::high_resolution_clock::now();
    runMultifrontalSolver(smoother, ordering, iterations);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_imperative = end - start;

    cout << "\nBenchmark (T=" << T << ", iterations=" << iterations << "):\n";
    cout << "  Standard GTSAM:     " << t_standard.count() << " s\n";
    cout << "  MultifrontalSolver: " << t_imperative.count() << " s\n";
    cout << "  Speedup:            "
         << t_standard.count() / t_imperative.count() << "x\n";
  }
  return 0;
}
