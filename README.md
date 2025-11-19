# gtsam-nb: nanobind bindings for GTSAM

This repository provides Python bindings for [GTSAM](https://gtsam.org/) 4.2 using [nanobind](https://github.com/wjakob/nanobind).

The bindings are all slightly modified versions of the official GTSAM Python bindings, but simply built using nanobind instead of pybind11. Most of the code, tests, data, etc are from the official GTSAM repository, just slightly modified to work with nanobind and clean up type stubs.

## Installation

It's all available via pip!

```bash
pip install gtsam-nb  # if using pip
uv add gtsam-nb       # if using uv
```

Then just use as you would the official gtsam Python bindings:

```python
import gtsam
```

`gtsam-nb` should be a near drop-in replacement for the official `gtsam` python package, with the following exceptions:
- nanobind doesn't accept python lists in place of numpy arrays for function arguments. So wrapping lists in `np.array(...)` is necessary.
- `OrderingType` enum values are now accessed as `gtsam.OrderingType.XXX` instead of `gtsam.Ordering.OrderingType.XXX`.

There are likely other small changes (please let us know if you find any), but they should all be fairly minor. 

## Why?

There's a handful of reasons why I decided to make this (most of which should *hopefully* be resolved when gtsam 4.3 is released):

- As of the writing of this README, `gtsam`'s most recent python release only supports up to python 3.11. This has restricted the python version for a number of my projects.
- In addition, the current stable `gtsam` on pypi are incompatible with numpy>=2.0.  
- The `gtsam` python bindings don't include type stubs, making development in an IDE and type checking more difficult.
- A switch to nanobind over pybind11 for a number of reasons, including 
  - nanobind results in faster compilation, smaller binaries, and a smaller overhead. See [benchmarks](https://nanobind.readthedocs.io/en/latest/benchmark.html#benchmarks) for more details.
  - nanobind has built-in stub generation, make setting up type stubs a cakewalk.
  - Based on my knowledge, nanobind has a more permissive ABI compatibility story than pybind11, which makes it easier to extend `gtsam` with custom C++ code that will just work together. See [here](https://github.com/pybind/pybind11/issues/3793) and [here](https://nanobind.readthedocs.io/en/latest/faq.html#how-can-i-avoid-conflicts-with-other-projects-using-nanobind).
  - Since all my projects now use nanobind, if I want to add new C++ factors to GTSAM from python, I need nanobind bindings.
- I often have found the `gtsam` bindings to be missing a method or two that I need. This then requires me to build the bindings from source. By having my own version, I can easily add in any missing methods I need, and get them merged and released on PyPI quickly. This is more difficult for the official `gtsam` bindings since they are released alongside GTSAM itself.

Hopefully in the future these issues will be resolved in the official `gtsam` bindings, and this repository will no longer be necessary. But for now, this is a useful stopgap.

## Extending
TODO