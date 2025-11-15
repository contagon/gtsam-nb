/**
 * @file    sam.cpp
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

#include "utils/boost_shared_ptr.h"

// These are the included headers listed in `gtsam.i`
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/sam/BearingFactor.h"
#include "gtsam/sam/BearingRangeFactor.h"
#include "gtsam/sam/RangeFactor.h"

typedef gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> gtsamRangeFactorgtsamPoint2gtsamPoint2;
typedef gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> gtsamRangeFactorgtsamPoint3gtsamPoint3;
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> gtsamRangeFactorgtsamPose2gtsamPoint2;
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> gtsamRangeFactorgtsamPose2gtsamPose2;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> gtsamRangeFactorgtsamPose3gtsamPoint3;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> gtsamRangeFactorgtsamPose3gtsamPose3;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> gtsamRangeFactorgtsamCalibratedCameragtsamPoint3;
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> gtsamRangeFactorgtsamPinholeCameragtsamCal3_S2gtsamPoint3;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> gtsamRangeFactorgtsamCalibratedCameragtsamCalibratedCamera;
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> gtsamRangeFactorgtsamPinholeCameragtsamCal3_S2gtsamPinholeCameragtsamCal3_S2;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> gtsamRangeFactorWithTransformgtsamPose2gtsamPoint2;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> gtsamRangeFactorWithTransformgtsamPose3gtsamPoint3;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> gtsamRangeFactorWithTransformgtsamPose2gtsamPose2;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> gtsamRangeFactorWithTransformgtsamPose3gtsamPose3;
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> gtsamBearingFactorgtsamPose2gtsamPoint2gtsamRot2;
typedef gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> gtsamBearingFactorgtsamPose3gtsamPoint3gtsamUnit3;
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> gtsamBearingFactorgtsamPose2gtsamPose2gtsamRot2;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> gtsamBearingRangeFactorgtsamPose2gtsamPoint2gtsamRot2double;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> gtsamBearingRangeFactorgtsamPose2gtsamPose2gtsamRot2double;
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> gtsamBearingRangeFactorgtsamPose3gtsamPoint3gtsamUnit3double;
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> gtsamBearingRangeFactorgtsamPose3gtsamPose3gtsamUnit3double;

using namespace std;

namespace nb = nanobind;

void sam(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of sam";

  nb::class_<gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>, gtsam::NoiseModelFactor>(m_, "RangeFactor2")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "RangeFactor3")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>, gtsam::NoiseModelFactor>(m_, "RangeFactor2D")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>, gtsam::NoiseModelFactor>(m_, "RangeFactorPose2")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "RangeFactor3D")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>, gtsam::NoiseModelFactor>(m_, "RangeFactorPose3")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "RangeFactorCalibratedCameraPoint")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "RangeFactorSimpleCameraPoint")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>, gtsam::NoiseModelFactor>(m_, "RangeFactorCalibratedCamera")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor>(m_, "RangeFactorSimpleCamera")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>, gtsam::NoiseModelFactor>(m_, "RangeFactorWithTransform2D")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose2 &>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("body_T_sensor"))
      // .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "RangeFactorWithTransform3D")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose3 &>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("body_T_sensor"))
      // .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>, gtsam::NoiseModelFactor>(m_, "RangeFactorWithTransformPose2")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose2 &>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("body_T_sensor"))
      // .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> *self) { return self->measured(); });

  nb::class_<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>, gtsam::NoiseModelFactor>(m_, "RangeFactorWithTransformPose3")
      .def(nb::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose3 &>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("body_T_sensor"))
      // .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> *self) { return self->measured(); });

  nb::class_<gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>, gtsam::NoiseModelFactor>(m_, "BearingFactor2D")
      .def(nb::init<size_t, size_t, const gtsam::Rot2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> *self) { return self->measured(); });

  nb::class_<gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>, gtsam::NoiseModelFactor>(m_, "BearingFactor3D")
      .def(nb::init<size_t, size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> *self) { return self->measured(); });

  nb::class_<gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>, gtsam::NoiseModelFactor>(m_, "BearingFactorPose2")
      .def(nb::init<size_t, size_t, const gtsam::Rot2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("noiseModel"))
      // .def("serialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("measured", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> *self) { return self->measured(); });

  nb::class_<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>, gtsam::NoiseModelFactor>(m_, "BearingRangeFactor2D")
      .def(nb::init<size_t, size_t, const gtsam::Rot2 &, const double &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("measuredBearing"), nb::arg("measuredRange"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> *self) { return self->measured(); });
  // .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>, gtsam::NoiseModelFactor>(m_, "BearingRangeFactorPose2")
      .def(nb::init<size_t, size_t, const gtsam::Rot2 &, const double &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("measuredBearing"), nb::arg("measuredRange"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> *self) { return self->measured(); });
  // .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>, gtsam::NoiseModelFactor>(m_, "BearingRangeFactor3D")
      .def(nb::init<size_t, size_t, const gtsam::Unit3 &, const double &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("measuredBearing"), nb::arg("measuredRange"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> *self) { return self->measured(); });
  // .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>, gtsam::NoiseModelFactor>(m_, "BearingRangeFactorPose3")
      .def(nb::init<size_t, size_t, const gtsam::Unit3 &, const double &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("measuredBearing"), nb::arg("measuredRange"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> *self) { return self->measured(); });
  // .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));
}
