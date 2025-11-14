/**
 * @file    base.cpp
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
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/string.h>

// These are the included headers listed in `gtsam.i`
#include "gtsam/base/DSFMap.h"
#include "gtsam/base/GenericValue.h"
#include "gtsam/base/Matrix.h"
#include "gtsam/base/MatrixSerialization.h"
#include "gtsam/base/Value.h"
#include "gtsam/base/debug.h"
#include "gtsam/geometry/Cal3Bundler.h"
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam/geometry/Cal3Fisheye.h"
#include "gtsam/geometry/Cal3Unified.h"
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/EssentialMatrix.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/StereoPoint2.h"
#include "gtsam/navigation/ImuBias.h"

using namespace std;

namespace nb = nanobind;

void base(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of base";

  nb::class_<gtsam::IndexPair>(m_, "IndexPair")
      .def(nb::init<>())
      .def(nb::init<size_t, size_t>(), nb::arg("i"), nb::arg("j"))
      .def("i", [](gtsam::IndexPair *self) { return self->i(); })
      .def("j", [](gtsam::IndexPair *self) { return self->j(); });

  nb::class_<gtsam::DSFMap<gtsam::IndexPair>>(m_, "DSFMapIndexPair")
      .def(nb::init<>())
      .def("find", [](gtsam::DSFMap<gtsam::IndexPair> *self, const gtsam::IndexPair &key) { return self->find(key); }, nb::arg("key"))
      .def("merge", [](gtsam::DSFMap<gtsam::IndexPair> *self, const gtsam::IndexPair &x, const gtsam::IndexPair &y) { self->merge(x, y); }, nb::arg("x"), nb::arg("y"))
      .def("sets", [](gtsam::DSFMap<gtsam::IndexPair> *self) { return self->sets(); });

  nb::class_<gtsam::IndexPairSet>(m_, "IndexPairSet")
      .def(nb::init<>())
      .def("size", [](gtsam::IndexPairSet *self) { return self->size(); })
      .def("empty", [](gtsam::IndexPairSet *self) { return self->empty(); })
      .def("clear", [](gtsam::IndexPairSet *self) { self->clear(); })
      .def("insert", [](gtsam::IndexPairSet *self, gtsam::IndexPair key) { self->insert(key); }, nb::arg("key"))
      .def("erase", [](gtsam::IndexPairSet *self, gtsam::IndexPair key) { return self->erase(key); }, nb::arg("key"))
      .def("count", [](gtsam::IndexPairSet *self, gtsam::IndexPair key) { return self->count(key); }, nb::arg("key"));

  nb::class_<gtsam::Value>(m_, "Value")
      .def("print", [](gtsam::Value *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::Value &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("dim", [](gtsam::Value *self) { return self->dim(); });

  nb::class_<gtsam::GenericValue<gtsam::Vector>, gtsam::Value>(m_, "GenericValueVector");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Vector> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Vector> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Vector> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Vector> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Matrix>, gtsam::Value>(m_, "GenericValueMatrix");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Matrix> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Matrix> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Matrix> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Matrix> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Point2>, gtsam::Value>(m_, "GenericValuePoint2");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Point2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Point3>, gtsam::Value>(m_, "GenericValuePoint3");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Point3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Rot2>, gtsam::Value>(m_, "GenericValueRot2");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Rot2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Rot2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Rot2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Rot3>, gtsam::Value>(m_, "GenericValueRot3");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Rot3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Rot3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Rot3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Rot3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Pose2>, gtsam::Value>(m_, "GenericValuePose2");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Pose2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Pose3>, gtsam::Value>(m_, "GenericValuePose3");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Pose3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::StereoPoint2>, gtsam::Value>(m_, "GenericValueStereoPoint2");
  // .def("serialize", [](gtsam::GenericValue<gtsam::StereoPoint2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::StereoPoint2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::StereoPoint2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::StereoPoint2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Cal3_S2>, gtsam::Value>(m_, "GenericValueCal3_S2");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Cal3DS2>, gtsam::Value>(m_, "GenericValueCal3DS2");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Cal3DS2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Cal3DS2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Cal3DS2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Cal3Bundler>, gtsam::Value>(m_, "GenericValueCal3Bundler");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Cal3Bundler> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Cal3Bundler> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Cal3Bundler> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Cal3Bundler> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Cal3Fisheye>, gtsam::Value>(m_, "GenericValueCal3Fisheye");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Cal3Fisheye> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Cal3Fisheye> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Cal3Fisheye> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::Cal3Unified>, gtsam::Value>(m_, "GenericValueCal3Unified");
  // .def("serialize", [](gtsam::GenericValue<gtsam::Cal3Unified> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::Cal3Unified> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::Cal3Unified> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::EssentialMatrix>, gtsam::Value>(m_, "GenericValueEssentialMatrix");
  // .def("serialize", [](gtsam::GenericValue<gtsam::EssentialMatrix> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::EssentialMatrix> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::EssentialMatrix> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::EssentialMatrix> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::CalibratedCamera>, gtsam::Value>(m_, "GenericValueCalibratedCamera");
  // .def("serialize", [](gtsam::GenericValue<gtsam::CalibratedCamera> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::CalibratedCamera> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::CalibratedCamera> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GenericValue<gtsam::imuBias::ConstantBias>, gtsam::Value>(m_, "GenericValueConstantBias");
  // .def("serialize", [](gtsam::GenericValue<gtsam::imuBias::ConstantBias> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GenericValue<gtsam::imuBias::ConstantBias> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GenericValue<gtsam::imuBias::ConstantBias> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericValue<gtsam::imuBias::ConstantBias> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  m_.def("isDebugVersion", []() { return gtsam::isDebugVersion(); });
  m_.def("IndexPairSetAsArray", [](gtsam::IndexPairSet &set) { return gtsam::IndexPairSetAsArray(set); }, nb::arg("set"));
  m_.def("linear_independent", [](const gtsam::Matrix &A, const gtsam::Matrix &B, double tol) { return gtsam::linear_independent(A, B, tol); }, nb::arg("A"), nb::arg("B"), nb::arg("tol"));
}
