from gtsam._core import *  # noqa: F403
import numpy as np
from typing import overload

from numpy.typing import NDArray

Array = NDArray[np.float64]


@overload
def Point2(x: np.ndarray) -> Array: ...


@overload
def Point2(x: float, y: float = np.nan) -> Array: ...


def Point2(x: float | Array, y: float | None = None) -> Array:
    """Shim for the deleted Point2 type."""
    if y is None:
        return np.array(x, dtype=float)
    else:
        return np.array([x, y], dtype=float)


@overload
def Point3(x: np.ndarray) -> NDArray[np.float64]: ...


@overload
def Point3(x: float, y: float = np.nan, z: float = np.nan) -> NDArray[np.float64]: ...


def Point3(x: float | Array, y: float | None = None, z: float | None = None) -> Array:
    """Shim for the deleted Point3 type."""
    if y is None and z is None:
        return np.array(x, dtype=float)
    else:
        return np.array([x, y, z], dtype=float)
