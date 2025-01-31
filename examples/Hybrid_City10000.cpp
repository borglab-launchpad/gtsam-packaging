/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Hybrid_City10000.cpp
 * @brief  Example of using hybrid estimation
 *         with multiple odometry measurements.
 * @author Varun Agrawal
 * @date   January 22, 2025
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactorGraph.h>
#include <gtsam/hybrid/HybridSmoother.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <time.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using namespace boost::algorithm;

using symbol_shorthand::L;
using symbol_shorthand::M;
using symbol_shorthand::X;

const size_t kMaxLoopCount = 2000;  // Example default value
const size_t kMaxNrHypotheses = 10;

auto kOpenLoopModel = noiseModel::Diagonal::Sigmas(Vector3::Ones() * 10);

auto kPriorNoiseModel = noiseModel::Diagonal::Sigmas(
    (Vector(3) << 0.0001, 0.0001, 0.0001).finished());

auto kPoseNoiseModel = noiseModel::Diagonal::Sigmas(
    (Vector(3) << 1.0 / 30.0, 1.0 / 30.0, 1.0 / 100.0).finished());

// Experiment Class
class Experiment {
 private:
  std::string filename_;
  HybridSmoother smoother_;
  HybridNonlinearFactorGraph graph_;
  Values initial_;
  Values result_;

  /**
   * @brief Write the result of optimization to file.
   *
   * @param result The Values object with the final result.
   * @param num_poses The number of poses to write to the file.
   * @param filename The file name to save the result to.
   */
  void writeResult(const Values& result, size_t numPoses,
                   const std::string& filename = "Hybrid_city10000.txt") {
    ofstream outfile;
    outfile.open(filename);

    for (size_t i = 0; i < numPoses; ++i) {
      Pose2 outPose = result.at<Pose2>(X(i));
      outfile << outPose.x() << " " << outPose.y() << " " << outPose.theta()
              << std::endl;
    }
    outfile.close();
    std::cout << "Output written to " << filename << std::endl;
  }

  /**
   * @brief Create a hybrid loop closure factor where
   * 0 - loose noise model and 1 - loop noise model.
   */
  HybridNonlinearFactor hybridLoopClosureFactor(size_t loopCounter, size_t keyS,
                                                size_t keyT,
                                                const Pose2& measurement) {
    DiscreteKey l(L(loopCounter), 2);

    auto f0 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), measurement, kOpenLoopModel);
    auto f1 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), measurement, kPoseNoiseModel);

    std::vector<NonlinearFactorValuePair> factors{
        {f0, kOpenLoopModel->negLogConstant()},
        {f1, kPoseNoiseModel->negLogConstant()}};
    HybridNonlinearFactor mixtureFactor(l, factors);
    return mixtureFactor;
  }

  /// @brief Create hybrid odometry factor with discrete measurement choices.
  HybridNonlinearFactor hybridOdometryFactor(
      size_t numMeasurements, size_t keyS, size_t keyT, const DiscreteKey& m,
      const std::vector<Pose2>& poseArray,
      const SharedNoiseModel& poseNoiseModel) {
    auto f0 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), poseArray[0], poseNoiseModel);
    auto f1 = std::make_shared<BetweenFactor<Pose2>>(
        X(keyS), X(keyT), poseArray[1], poseNoiseModel);

    std::vector<NonlinearFactorValuePair> factors{{f0, 0.0}, {f1, 0.0}};
    HybridNonlinearFactor mixtureFactor(m, factors);
    return mixtureFactor;
  }

  /// @brief Perform smoother update and optimize the graph.
  void smootherUpdate(HybridSmoother& smoother,
                      HybridNonlinearFactorGraph& graph, const Values& initial,
                      size_t kMaxNrHypotheses, Values* result) {
    HybridGaussianFactorGraph linearized = *graph.linearize(initial);
    smoother.update(linearized, kMaxNrHypotheses);
    // throw if x0 not in hybridBayesNet_:
    const KeySet& keys = smoother.hybridBayesNet().keys();
    if (keys.find(X(0)) == keys.end()) {
      throw std::runtime_error("x0 not in hybridBayesNet_");
    }
    graph.resize(0);
    // HybridValues delta = smoother.hybridBayesNet().optimize();
    // result->insert_or_assign(initial.retract(delta.continuous()));
  }

 public:
  /// Construct with filename of experiment to run
  explicit Experiment(const std::string& filename)
      : filename_(filename), smoother_(0.99) {}

  /// @brief Run the main experiment with a given maxLoopCount.
  void run(size_t maxLoopCount) {
    // Prepare reading
    ifstream in(filename_);
    if (!in.is_open()) {
      cerr << "Failed to open file: " << filename_ << endl;
      return;
    }

    // Initialize local variables
    size_t discreteCount = 0, index = 0;
    size_t loopCount = 0;

    std::list<double> timeList;

    // Set up initial prior
    double x = 0.0;
    double y = 0.0;
    double rad = 0.0;

    Pose2 priorPose(x, y, rad);
    initial_.insert(X(0), priorPose);
    graph_.push_back(PriorFactor<Pose2>(X(0), priorPose, kPriorNoiseModel));

    // Initial update
    clock_t beforeUpdate = clock();
    smootherUpdate(smoother_, graph_, initial_, kMaxNrHypotheses, &result_);
    clock_t afterUpdate = clock();
    std::vector<std::pair<size_t, double>> smootherUpdateTimes;
    smootherUpdateTimes.push_back({index, afterUpdate - beforeUpdate});

    // Start main loop
    size_t keyS = 0, keyT = 0;
    clock_t startTime = clock();
    std::string line;
    while (getline(in, line) && index < maxLoopCount) {
      std::vector<std::string> parts;
      split(parts, line, is_any_of(" "));

      keyS = stoi(parts[1]);
      keyT = stoi(parts[3]);

      int numMeasurements = stoi(parts[5]);
      std::vector<Pose2> poseArray(numMeasurements);
      for (int i = 0; i < numMeasurements; ++i) {
        x = stod(parts[6 + 3 * i]);
        y = stod(parts[7 + 3 * i]);
        rad = stod(parts[8 + 3 * i]);
        poseArray[i] = Pose2(x, y, rad);
      }

      // Flag to decide whether to run smoother update
      bool doSmootherUpdate = false;

      // Take the first one as the initial estimate
      Pose2 odomPose = poseArray[0];
      if (keyS == keyT - 1) {
        // Odometry factor
        if (numMeasurements > 1) {
          // Add hybrid factor
          DiscreteKey m(M(discreteCount), numMeasurements);
          HybridNonlinearFactor mixtureFactor = hybridOdometryFactor(
              numMeasurements, keyS, keyT, m, poseArray, kPoseNoiseModel);
          graph_.push_back(mixtureFactor);
          discreteCount++;
          doSmootherUpdate = true;
          std::cout << "mixtureFactor: " << keyS << " " << keyT << std::endl;
        } else {
          graph_.add(BetweenFactor<Pose2>(X(keyS), X(keyT), odomPose,
                                          kPoseNoiseModel));
        }
        // Insert next pose initial guess
        initial_.insert(X(keyT), initial_.at<Pose2>(X(keyS)) * odomPose);
      } else {
        // Loop closure
        HybridNonlinearFactor loopFactor =
            hybridLoopClosureFactor(loopCount, keyS, keyT, odomPose);
        // print loop closure event keys:
        std::cout << "Loop closure: " << keyS << " " << keyT << std::endl;
        graph_.add(loopFactor);
        doSmootherUpdate = true;
        loopCount++;
      }

      if (doSmootherUpdate) {
        gttic_(SmootherUpdate);
        beforeUpdate = clock();
        smootherUpdate(smoother_, graph_, initial_, kMaxNrHypotheses, &result_);
        afterUpdate = clock();
        smootherUpdateTimes.push_back({index, afterUpdate - beforeUpdate});
        gttoc_(SmootherUpdate);
        doSmootherUpdate = false;
      }

      // Record timing for odometry edges only
      if (keyS == keyT - 1) {
        clock_t curTime = clock();
        timeList.push_back(curTime - startTime);
      }

      // Print some status every 100 steps
      if (index % 100 == 0) {
        std::cout << "Index: " << index << std::endl;
        if (!timeList.empty()) {
          std::cout << "Acc_time: " << timeList.back() / CLOCKS_PER_SEC
                    << " seconds" << std::endl;
          // delta.discrete().print("The Discrete Assignment");
          tictoc_finishedIteration_();
          tictoc_print_();
        }
      }

      index++;
    }

    // Final update
    beforeUpdate = clock();
    smootherUpdate(smoother_, graph_, initial_, kMaxNrHypotheses, &result_);
    afterUpdate = clock();
    smootherUpdateTimes.push_back({index, afterUpdate - beforeUpdate});

    // Final optimize
    gttic_(HybridSmootherOptimize);
    HybridValues delta = smoother_.optimize();
    gttoc_(HybridSmootherOptimize);

    result_.insert_or_assign(initial_.retract(delta.continuous()));

    std::cout << "Final error: " << smoother_.hybridBayesNet().error(delta)
              << std::endl;

    clock_t endTime = clock();
    clock_t totalTime = endTime - startTime;
    std::cout << "Total time: " << totalTime / CLOCKS_PER_SEC << " seconds"
              << std::endl;

    // Write results to file
    writeResult(result_, keyT + 1, "Hybrid_City10000.txt");

    // TODO Write to file
    //  for (size_t i = 0; i < smoother_update_times.size(); i++) {
    //    auto p = smoother_update_times.at(i);
    //    std::cout << p.first << ", " << p.second / CLOCKS_PER_SEC <<
    //    std::endl;
    //  }

    // Write timing info to file
    ofstream outfileTime;
    std::string timeFileName = "Hybrid_City10000_time.txt";
    outfileTime.open(timeFileName);
    for (auto accTime : timeList) {
      outfileTime << accTime << std::endl;
    }
    outfileTime.close();
    std::cout << "Output " << timeFileName << " file." << std::endl;
  }
};

/* ************************************************************************* */
int main() {
  Experiment experiment(findExampleDataFile("T1_city10000_04.txt"));
  // Experiment experiment("../data/mh_T1_city10000_04.txt"); //Type #1 only
  // Experiment experiment("../data/mh_T3b_city10000_10.txt"); //Type #3 only
  // Experiment experiment("../data/mh_T1_T3_city10000_04.txt"); //Type #1 +
  // Type #3

  // Run the experiment
  experiment.run(kMaxLoopCount);

  return 0;
}