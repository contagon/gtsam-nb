/**
 * @file    custom.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal
 * @date    Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

// Include relevant boost libraries required by GTSAM
#include <boost/shared_ptr.hpp>

#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h" // for RedirectCout.
#include "gtsam/config.h"
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/function.h>

#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>

// These are the included headers listed in `gtsam.i`
#include "gtsam/nonlinear/CustomFactor.h"

using namespace std;

namespace nb = nanobind;

void custom(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of custom";

  nb::class_<gtsam::CustomFactor, gtsam::NoiseModelFactor>(m_, "CustomFactor")
      .def(nb::init<>())
      .def(nb::init<const gtsam::SharedNoiseModel &, const gtsam::KeyVector &, const gtsam::CustomErrorFunction &>(), nb::arg("noiseModel"), nb::arg("keys"), nb::arg("errorFunction"))
      .def("print", [](gtsam::CustomFactor *self, string s, gtsam::KeyFormatter keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::CustomFactor &self, string s, gtsam::KeyFormatter keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
}
