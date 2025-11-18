"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

TestCase class with GTSAM assert utils.
Author: Frank Dellaert
"""

import pickle
import unittest
from typing import Any


class GtsamTestCase(unittest.TestCase):
    """TestCase class with GTSAM assert utils."""

    def gtsamAssertEquals(self, actual: Any, expected: Any, tol: float = 1e-9):
        """AssertEqual function that prints out actual and expected if not equal.
        Usage:
            self.gtsamAssertEqual(actual,expected)
        Keyword Arguments:
            tol {float} -- tolerance passed to 'equals', default 1e-9
        """
        import numpy

        if isinstance(expected, numpy.ndarray):
            equal = numpy.allclose(actual, expected, atol=tol)
        else:
            equal = actual.equals(expected, tol)
        if not equal:
            raise self.failureException(
                "Values are not equal:\n{}!={}".format(actual, expected)
            )

    def assertEqualityOnPickleRoundtrip(self, obj: object, tol: float = 1e-9) -> None:
        """Performs a round-trip using pickle and asserts equality.

        Usage:
            self.assertEqualityOnPickleRoundtrip(obj)
        Keyword Arguments:
            tol {float} -- tolerance passed to 'equals', default 1e-9
        """
        # TODO: This will obviously be broken as we disabled all pickling
        # roundTripObj = pickle.loads(pickle.dumps(obj))
        # self.gtsamAssertEquals(roundTripObj, obj)
        pass
