"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for various factors.

Author: Varun Agrawal
"""
import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestNonlinearEquality2Factor(GtsamTestCase):
    """
    Test various instantiations of NonlinearEquality2.
    """

    def test_point3(self):
        """Test for Point3 version."""
        factor = gtsam.NonlinearEquality2Point3(0, 1)
        error = factor.evaluateError(gtsam.Point3(0, 0, 0),
                                     gtsam.Point3(0, 0, 0))

        np.testing.assert_allclose(error, np.zeros(3))


class TestJacobianFactor(GtsamTestCase):
    """Test JacobianFactor"""

    def test_gaussian_factor_graph(self):
        """Test construction from GaussianFactorGraph."""
        gfg = gtsam.GaussianFactorGraph()
        jf = gtsam.JacobianFactor(gfg)
        self.assertIsInstance(jf, gtsam.JacobianFactor)

        nfg = gtsam.NonlinearFactorGraph()
        nfg.push_back(gtsam.PriorFactorDouble(1, 0.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0)))
        values = gtsam.Values()
        values.insert(1, 0.0)
        gfg = nfg.linearize(values)
        jf = gtsam.JacobianFactor(gfg)
        self.assertIsInstance(jf, gtsam.JacobianFactor)

class TestNonlinearFactorGraphIter(GtsamTestCase):
    def test_iter(self):
        nfg = gtsam.NonlinearFactorGraph()
        f1 = gtsam.PriorFactorDouble(1, 0.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0))
        f2 = gtsam.PriorFactorDouble(2, 1.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0))
        nfg.add(f1)
        nfg.add(f2)

        factors = [f1, f2]
        self.assertEqual(len(factors), nfg.size())
        for exp, actual in zip(factors, nfg):
            assert exp == actual

    def test_construction(self):
        f1 = gtsam.PriorFactorDouble(1, 0.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0))
        f2 = gtsam.PriorFactorDouble(2, 1.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0))
        factors = [f1, f2]
        nfg = gtsam.NonlinearFactorGraph(factors)

        factors = [f1, f2]
        nfg2 = gtsam.NonlinearFactorGraph(factors)
        self.assertEqual(nfg.size(), nfg2.size())
        for exp, actual in zip(nfg, nfg2):
            assert exp == actual


if __name__ == "__main__":
    unittest.main()
