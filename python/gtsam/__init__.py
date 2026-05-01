# ruff: noqa: F401, F403
from gtsam import _core as _core
from gtsam._core import *
from gtsam.utils import findExampleDataFile  # type: ignore

import numpy as np
from typing import overload

from numpy.typing import NDArray

Array = NDArray[np.float64]


@overload
def Point2(x: None = None) -> Array: ...


@overload
def Point2(x: np.ndarray) -> Array: ...


@overload
def Point2(x: float, y: float = np.nan) -> Array: ...


def Point2(x: float | Array | None = None, y: float | None = None) -> Array:
    """Shim for the deleted Point2 type."""
    if x is None:
        return np.array([np.nan, np.nan], dtype=float)
    elif y is None:
        return np.array(x, dtype=float)
    else:
        return np.array([x, y], dtype=float)


@overload
def Point3(x: None = None) -> NDArray[np.float64]: ...


@overload
def Point3(x: np.ndarray) -> NDArray[np.float64]: ...


@overload
def Point3(x: float, y: float = np.nan, z: float = np.nan) -> NDArray[np.float64]: ...


def Point3(
    x: float | Array | None = None, y: float | None = None, z: float | None = None
) -> Array:
    """Shim for the deleted Point3 type."""
    if x is None:
        return np.array([np.nan, np.nan, np.nan], dtype=float)
    elif y is None and z is None:
        return np.array(x, dtype=float)
    else:
        return np.array([x, y, z], dtype=float)


def _install_iterable_api() -> None:
    # This is a hack to be able to call .value() on all the GenericValue types
    from gtsam import Values

    def _values_items(self: Values):  # type: ignore
        for k, v in self._items():  # type: ignore
            yield k, v.value()  # type: ignore

    def _values_values(self: Values):  # type: ignore
        for v in self._values():  # type: ignore
            yield v.value()  # type: ignore

    def __getitem__(self: Values, key):  # type: ignore
        return self.at(key).value()  # type: ignore

    def __setitem__(self: Values, key, value):  # type: ignore
        self.insert_or_assign(key, value)  # type: ignore

    _core.Values.items = _values_items  # type: ignore
    _core.Values.values = _values_values  # type: ignore
    _core.Values.__getitem__ = __getitem__  # type: ignore
    _core.Values.__setitem__ = __setitem__  # type: ignore


_install_iterable_api()

__version__ = "0.1.2"  # x-release-please-version
