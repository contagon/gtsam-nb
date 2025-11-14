/**
 * @file    custom.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal
 * @date    Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

#define PYBIND11_DETAILED_ERROR_MESSAGES

// Include relevant boost libraries required by GTSAM
#include <boost/shared_ptr.hpp>

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include "gtsam/config.h"
#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h"  // for RedirectCout.

// These are the included headers listed in `gtsam.i`
#include "gtsam/nonlinear/CustomFactor.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/custom.h"

using namespace std;

namespace py = pybind11;



void custom(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of custom";




    py::class_<gtsam::CustomFactor, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::CustomFactor>>(m_, "CustomFactor")
        .def(py::init<>())
        .def(py::init<const gtsam::SharedNoiseModel&, const gtsam::KeyVector&, const gtsam::CustomErrorFunction&>(), py::arg("noiseModel"), py::arg("keys"), py::arg("errorFunction"))
        .def("print",[](gtsam::CustomFactor* self, string s, gtsam::KeyFormatter keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::CustomFactor& self, string s, gtsam::KeyFormatter keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/custom.h"

}

