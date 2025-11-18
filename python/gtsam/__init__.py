from gtsam._core import *  # noqa: F403
import numpy as np


def Point2(x: float = np.nan, y: float = np.nan):
    """Shim for the deleted Point2 type."""
    return np.array([x, y], dtype=float)


global Point3  # export function


def Point3(x: float = np.nan, y: float = np.nan, z: float = np.nan):
    """Shim for the deleted Point3 type."""
    return np.array([x, y, z], dtype=float)
