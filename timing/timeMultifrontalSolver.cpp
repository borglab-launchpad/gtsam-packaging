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

#include "timeSFMBAL.h"

using namespace std;
using namespace gtsam;
using namespace example;

namespace {
// Threshold chosen empirically for these timing experiments: merging small
// frontal cliques tends to improve performance without materially affecting
// numerical behavior for the tested problems.
constexpr size_t kMergeDimCap = 16;
}  // namespace

/// Run standard GTSAM multifrontal elimination and optimization.
static void runStandardSolver(const GaussianFactorGraph& smoother,
                              const Ordering& ordering, size_t iterations) {
  for (size_t i = 0; i < iterations; ++i) {
    GaussianBayesTree bayesTree = *smoother.eliminateMultifrontal(ordering);
    VectorValues solution = bayesTree.optimize();
    (void)solution;
  }
}

/// Run new MultifrontalSolver elimination and optimization.
static void runMultifrontalSolver(MultifrontalSolver& solver,
                                  const GaussianFactorGraph& graph,
                                  size_t iterations) {
  for (size_t i = 0; i < iterations; ++i) {
    solver.eliminateInPlace(graph);
    const VectorValues& solution = solver.updateSolution();
    (void)solution;
  }
}

namespace {
const std::string bal135 = findExampleDataFile("dubrovnik-135-90642-pre.txt");
}

void runBAL135Benchmark() {
  const size_t iterations = 1;
  cout << "\nSingle MFS test: " << bal135 << " (iterations=" << iterations
       << ")" << std::endl;

  const SfmData db = SfmData::FromBalFile(bal135);
  const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, 0.1);
  const Values initial = buildGeneralSfmInitial(db);
  const GaussianFactorGraph linear = *graph.linearize(initial);
  const Ordering ordering = Ordering::Metis(linear);

  MultifrontalSolver solver(linear, ordering, kMergeDimCap, nullptr);
  auto start = std::chrono::high_resolution_clock::now();
  runMultifrontalSolver(solver, linear, iterations);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> t_imperative = end - start;
  cout << "  MultifrontalSolver: " << t_imperative.count() << " s" << std::endl;
  tictoc_print();
}

void runBALBenchmark() {
  const size_t bal_iterations = 2;
  const string bal16 = findExampleDataFile("dubrovnik-16-22106-pre");
  const string bal88 = findExampleDataFile("dubrovnik-88-64298-pre");
  for (const auto& filename : {bal16, bal88, bal135}) {
    cout << "\nProcessing BAL file: " << filename << std::endl;
    const SfmData db = SfmData::FromBalFile(filename);
    const NonlinearFactorGraph graph = buildGeneralSfmGraph(db, 0.1);
    const Values initial = buildGeneralSfmInitial(db);
    const GaussianFactorGraph linear = *graph.linearize(initial);

    auto orderings = createOrderings(db, linear);
    for (const auto& [label, ordering] : orderings) {
      cout << "\nBAL Benchmark (" << label << ", iterations=" << bal_iterations
           << "):" << std::endl;

      MultifrontalSolver solver(linear, ordering, kMergeDimCap, nullptr);
      auto start = std::chrono::high_resolution_clock::now();
      runMultifrontalSolver(solver, linear, bal_iterations);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> t_imperative = end - start;
      cout << "  MultifrontalSolver: " << t_imperative.count() << " s"
           << std::endl;

      start = std::chrono::high_resolution_clock::now();
      runStandardSolver(linear, ordering, bal_iterations);
      end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> t_standard = end - start;
      cout << "  Standard GTSAM:     " << t_standard.count() << " s"
           << std::endl;

      cout << "  Speedup:            "
           << t_standard.count() / t_imperative.count() << "x" << std::endl;
    }
  }
}
void runChainBenchmark() {
  const std::vector<size_t> T_values = {10, 50, 100, 500, 1000, 5000};
  const size_t iterations = 500;

  for (size_t T : T_values) {
    cout << "\nBenchmark (T=" << T << ", iterations=" << iterations
         << "):" << std::endl;
    GaussianFactorGraph smoother = createSmoother(T);
    const Ordering ordering = Ordering::Metis(smoother);

    auto start = std::chrono::high_resolution_clock::now();
    MultifrontalSolver solver(smoother, ordering, kMergeDimCap, &std::cout);
    runMultifrontalSolver(solver, smoother, iterations);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_imperative = end - start;
    cout << "\nTiming results:\n";
    cout << "  MultifrontalSolver: " << t_imperative.count() << " s"
         << std::endl;

    start = std::chrono::high_resolution_clock::now();
    runStandardSolver(smoother, ordering, iterations);
    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_standard = end - start;
    cout << "  Standard GTSAM:     " << t_standard.count() << " s" << std::endl;

    cout << "  Speedup:            "
         << t_standard.count() / t_imperative.count() << "x" << std::endl;
  }
}
int main() {
  cout << "Merging dim cap " << kMergeDimCap << std::endl;

  runBAL135Benchmark();
  runBALBenchmark();
  runChainBenchmark();
  return 0;
}
