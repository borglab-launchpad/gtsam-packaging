"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Factor Graphs.
Author: Fan Jiang
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.symbol_shorthand import C, M, X, Z
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (DiscreteConditional, DiscreteKeys, GaussianConditional,
                   GaussianMixture, GaussianMixtureFactor,
                   HybridGaussianFactorGraph, JacobianFactor, Ordering,
                   noiseModel)


class TestHybridGaussianFactorGraph(GtsamTestCase):
    """Unit tests for HybridGaussianFactorGraph."""
    def test_create(self):
        """Test construction of hybrid factor graph."""
        model = noiseModel.Unit.Create(3)
        dk = DiscreteKeys()
        dk.push_back((C(0), 2))

        jf1 = JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)), model)
        jf2 = JacobianFactor(X(0), np.eye(3), np.ones((3, 1)), model)

        gmf = GaussianMixtureFactor([X(0)], dk, [jf1, jf2])

        hfg = HybridGaussianFactorGraph()
        hfg.push_back(jf1)
        hfg.push_back(jf2)
        hfg.push_back(gmf)

        hbn = hfg.eliminateSequential(
            Ordering.ColamdConstrainedLastHybridGaussianFactorGraph(
                hfg, [C(0)]))

        self.assertEqual(hbn.size(), 2)

        mixture = hbn.at(0).inner()
        self.assertIsInstance(mixture, GaussianMixture)
        self.assertEqual(len(mixture.keys()), 2)

        discrete_conditional = hbn.at(hbn.size() - 1).inner()
        self.assertIsInstance(discrete_conditional, DiscreteConditional)

    def test_optimize(self):
        """Test construction of hybrid factor graph."""
        model = noiseModel.Unit.Create(3)
        dk = DiscreteKeys()
        dk.push_back((C(0), 2))

        jf1 = JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)), model)
        jf2 = JacobianFactor(X(0), np.eye(3), np.ones((3, 1)), model)

        gmf = GaussianMixtureFactor([X(0)], dk, [jf1, jf2])

        hfg = HybridGaussianFactorGraph()
        hfg.push_back(jf1)
        hfg.push_back(jf2)
        hfg.push_back(gmf)

        dtf = gtsam.DecisionTreeFactor([(C(0), 2)], "0 1")
        hfg.push_back(dtf)

        hbn = hfg.eliminateSequential(
            Ordering.ColamdConstrainedLastHybridGaussianFactorGraph(
                hfg, [C(0)]))

        hv = hbn.optimize()
        self.assertEqual(hv.atDiscrete(C(0)), 1)

    @staticmethod
    def tiny(num_measurements: int = 1):
        """Create a tiny two variable hybrid model."""
        # Create hybrid Bayes net.
        bayesNet = gtsam.HybridBayesNet()

        # Create mode key: 0 is low-noise, 1 is high-noise.
        modeKey = M(0)
        mode = (modeKey, 2)

        # Create Gaussian mixture Z(0) = X(0) + noise for each measurement.
        I = np.eye(1)
        keys = DiscreteKeys()
        keys.push_back(mode)
        for i in range(num_measurements):
            conditional0 = GaussianConditional.FromMeanAndStddev(Z(i),
                                                                 I,
                                                                 X(0), [0],
                                                                 sigma=0.5)
            conditional1 = GaussianConditional.FromMeanAndStddev(Z(i),
                                                                 I,
                                                                 X(0), [0],
                                                                 sigma=3)
            bayesNet.emplaceMixture([Z(i)], [X(0)], keys,
                                    [conditional0, conditional1])

        # Create prior on X(0).
        prior_on_x0 = GaussianConditional.FromMeanAndStddev(X(0), [5.0], 5.0)
        bayesNet.addGaussian(prior_on_x0)

        # Add prior on mode.
        bayesNet.emplaceDiscrete(mode, "1/1")

        return bayesNet

    def test_tiny(self):
        """Test a tiny two variable hybrid model."""
        bayesNet = self.tiny()
        sample = bayesNet.sample()
        # print(sample)

        # Create a factor graph from the Bayes net with sampled measurements.
        fg = HybridGaussianFactorGraph()
        conditional = bayesNet.atMixture(0)
        measurement = gtsam.VectorValues()
        measurement.insert(Z(0), sample.at(Z(0)))
        factor = conditional.likelihood(measurement)
        fg.push_back(factor)
        fg.push_back(bayesNet.atGaussian(1))
        fg.push_back(bayesNet.atDiscrete(2))

        self.assertEqual(fg.size(), 3)

    @staticmethod
    def calculate_ratio(bayesNet, fg, sample):
        """Calculate ratio  between Bayes net probability and the factor graph."""
        continuous = gtsam.VectorValues()
        continuous.insert(X(0), sample.at(X(0)))
        return bayesNet.evaluate(sample) / fg.probPrime(
            continuous, sample.discrete())

    def test_tiny2(self):
        """Test a tiny two variable hybrid model, with 2 measurements."""
        # Create the Bayes net and sample from it.
        bayesNet = self.tiny(num_measurements=2)
        sample = bayesNet.sample()
        # print(sample)

        # Create a factor graph from the Bayes net with sampled measurements.
        fg = HybridGaussianFactorGraph()
        for i in range(2):
            conditional = bayesNet.atMixture(i)
            measurement = gtsam.VectorValues()
            measurement.insert(Z(i), sample.at(Z(i)))
            factor = conditional.likelihood(measurement)
            fg.push_back(factor)
        fg.push_back(bayesNet.atGaussian(2))
        fg.push_back(bayesNet.atDiscrete(3))

        # print(fg)
        self.assertEqual(fg.size(), 4)

        # Calculate ratio between Bayes net probability and the factor graph:
        expected_ratio = self.calculate_ratio(bayesNet, fg, sample)
        # print(f"expected_ratio: {expected_ratio}\n")

        # Create measurements from the sample.
        measurements = gtsam.VectorValues()
        for i in range(2):
            measurements.insert(Z(i), sample.at(Z(i)))

        # Check with a number of other samples.
        for i in range(10):
            other = bayesNet.sample()
            other.update(measurements)
            # print(other)
            # ratio = self.calculate_ratio(bayesNet, fg, other)
            # print(f"Ratio: {ratio}\n")
            # self.assertAlmostEqual(ratio, expected_ratio)


if __name__ == "__main__":
    unittest.main()
