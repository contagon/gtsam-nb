/**
 * @file    sam.cpp
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
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/sam/RangeFactor.h"
#include "gtsam/sam/RangeFactor.h"
#include "gtsam/sam/BearingFactor.h"
#include "gtsam/sam/BearingRangeFactor.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
typedef gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> gtsamRangeFactorgtsamPoint2gtsamPoint2;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPoint2gtsamPoint2)
typedef gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> gtsamRangeFactorgtsamPoint3gtsamPoint3;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPoint3gtsamPoint3)
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> gtsamRangeFactorgtsamPose2gtsamPoint2;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPose2gtsamPoint2)
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> gtsamRangeFactorgtsamPose2gtsamPose2;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPose2gtsamPose2)
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> gtsamRangeFactorgtsamPose3gtsamPoint3;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPose3gtsamPoint3)
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> gtsamRangeFactorgtsamPose3gtsamPose3;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPose3gtsamPose3)
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> gtsamRangeFactorgtsamCalibratedCameragtsamPoint3;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamCalibratedCameragtsamPoint3)
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> gtsamRangeFactorgtsamPinholeCameragtsamCal3_S2gtsamPoint3;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPinholeCameragtsamCal3_S2gtsamPoint3)
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> gtsamRangeFactorgtsamCalibratedCameragtsamCalibratedCamera;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamCalibratedCameragtsamCalibratedCamera)
typedef gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> gtsamRangeFactorgtsamPinholeCameragtsamCal3_S2gtsamPinholeCameragtsamCal3_S2;
BOOST_CLASS_EXPORT(gtsamRangeFactorgtsamPinholeCameragtsamCal3_S2gtsamPinholeCameragtsamCal3_S2)
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> gtsamRangeFactorWithTransformgtsamPose2gtsamPoint2;
BOOST_CLASS_EXPORT(gtsamRangeFactorWithTransformgtsamPose2gtsamPoint2)
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> gtsamRangeFactorWithTransformgtsamPose3gtsamPoint3;
BOOST_CLASS_EXPORT(gtsamRangeFactorWithTransformgtsamPose3gtsamPoint3)
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> gtsamRangeFactorWithTransformgtsamPose2gtsamPose2;
BOOST_CLASS_EXPORT(gtsamRangeFactorWithTransformgtsamPose2gtsamPose2)
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> gtsamRangeFactorWithTransformgtsamPose3gtsamPose3;
BOOST_CLASS_EXPORT(gtsamRangeFactorWithTransformgtsamPose3gtsamPose3)
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> gtsamBearingFactorgtsamPose2gtsamPoint2gtsamRot2;
BOOST_CLASS_EXPORT(gtsamBearingFactorgtsamPose2gtsamPoint2gtsamRot2)
typedef gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> gtsamBearingFactorgtsamPose3gtsamPoint3gtsamUnit3;
BOOST_CLASS_EXPORT(gtsamBearingFactorgtsamPose3gtsamPoint3gtsamUnit3)
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> gtsamBearingFactorgtsamPose2gtsamPose2gtsamRot2;
BOOST_CLASS_EXPORT(gtsamBearingFactorgtsamPose2gtsamPose2gtsamRot2)
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> gtsamBearingRangeFactorgtsamPose2gtsamPoint2gtsamRot2double;
BOOST_CLASS_EXPORT(gtsamBearingRangeFactorgtsamPose2gtsamPoint2gtsamRot2double)
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> gtsamBearingRangeFactorgtsamPose2gtsamPose2gtsamRot2double;
BOOST_CLASS_EXPORT(gtsamBearingRangeFactorgtsamPose2gtsamPose2gtsamRot2double)
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> gtsamBearingRangeFactorgtsamPose3gtsamPoint3gtsamUnit3double;
BOOST_CLASS_EXPORT(gtsamBearingRangeFactorgtsamPose3gtsamPoint3gtsamUnit3double)
typedef gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> gtsamBearingRangeFactorgtsamPose3gtsamPose3gtsamUnit3double;
BOOST_CLASS_EXPORT(gtsamBearingRangeFactorgtsamPose3gtsamPose3gtsamUnit3double)


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/sam.h"

using namespace std;

namespace py = pybind11;



void sam(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of sam";




    py::class_<gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>>>(m_, "RangeFactor2")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::Point2, gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::Point2, gtsam::Point2>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>>>(m_, "RangeFactor3")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::Point3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::Point3, gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>>>(m_, "RangeFactor2D")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>>>(m_, "RangeFactorPose2")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>>>(m_, "RangeFactor3D")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>>>(m_, "RangeFactorPose3")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>>>(m_, "RangeFactorCalibratedCameraPoint")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>>>(m_, "RangeFactorSimpleCameraPoint")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>>>(m_, "RangeFactorCalibratedCamera")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>* self){return self->measured();});

    py::class_<gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>>>(m_, "RangeFactorSimpleCamera")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::PinholeCamera<gtsam::Cal3_S2>>* self){return self->measured();});

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>>>(m_, "RangeFactorWithTransform2D")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose2&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"))
        .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>* self){return self->measured();});

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>>>(m_, "RangeFactorWithTransform3D")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose3&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"))
        .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>>>(m_, "RangeFactorWithTransformPose2")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose2&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"))
        .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>* self){return self->measured();});

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>>>(m_, "RangeFactorWithTransformPose3")
        .def(py::init<size_t, size_t, double, const boost::shared_ptr<gtsam::noiseModel::Base>, const gtsam::Pose3&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"))
        .def("serialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>* self){return self->measured();});

    py::class_<gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>>>(m_, "BearingFactor2D")
        .def(py::init<size_t, size_t, const gtsam::Rot2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>* self){return self->measured();});

    py::class_<gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>>>(m_, "BearingFactor3D")
        .def(py::init<size_t, size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::BearingFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3>* self){return self->measured();});

    py::class_<gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>>>(m_, "BearingFactorPose2")
        .def(py::init<size_t, size_t, const gtsam::Rot2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("serialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("measured",[](gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>* self){return self->measured();});

    py::class_<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>>>(m_, "BearingRangeFactor2D")
        .def(py::init<size_t, size_t, const gtsam::Rot2&, const double&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measuredBearing"), py::arg("measuredRange"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>* self){return self->measured();})
        .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>>>(m_, "BearingRangeFactorPose2")
        .def(py::init<size_t, size_t, const gtsam::Rot2&, const double&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measuredBearing"), py::arg("measuredRange"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>* self){return self->measured();})
        .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>>>(m_, "BearingRangeFactor3D")
        .def(py::init<size_t, size_t, const gtsam::Unit3&, const double&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measuredBearing"), py::arg("measuredRange"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>* self){return self->measured();})
        .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>>>(m_, "BearingRangeFactorPose3")
        .def(py::init<size_t, size_t, const gtsam::Unit3&, const double&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measuredBearing"), py::arg("measuredRange"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>* self){return self->measured();})
        .def("serialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/sam.h"

}

