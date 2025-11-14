/**
 * @file    slam.cpp
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
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam/geometry/SO4.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/EssentialMatrixConstraint.h"
#include "gtsam/slam/EssentialMatrixFactor.h"
#include "gtsam/slam/FrobeniusFactor.h"
#include "gtsam/slam/GeneralSFMFactor.h"
#include "gtsam/slam/InitializePose3.h"
#include "gtsam/slam/KarcherMeanFactor-inl.h"
#include "gtsam/slam/PoseRotationPrior.h"
#include "gtsam/slam/PoseTranslationPrior.h"
#include "gtsam/slam/ProjectionFactor.h"
#include "gtsam/slam/SmartProjectionFactor.h"
#include "gtsam/slam/SmartProjectionPoseFactor.h"
#include "gtsam/slam/StereoFactor.h"
#include "gtsam/slam/dataset.h"
#include "gtsam/slam/lago.h"

typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> gtsamGenericProjectionFactorgtsamPose3gtsamPoint3gtsamCal3_S2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> gtsamGenericProjectionFactorgtsamPose3gtsamPoint3gtsamCal3DS2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> gtsamGenericProjectionFactorgtsamPose3gtsamPoint3gtsamCal3Fisheye;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> gtsamGenericProjectionFactorgtsamPose3gtsamPoint3gtsamCal3Unified;
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> gtsamGenericStereoFactorgtsamPose3gtsamPoint3;

using namespace std;

namespace nb = nanobind;

void slam(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of slam";

  nb::class_<gtsam::BetweenFactor<double>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<double>>>(m_, "BetweenFactorDouble")
      .def(nb::init<size_t, size_t, const double &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<double> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<double> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<double> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<double> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Vector>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Vector>>>(m_, "BetweenFactorVector")
      .def(nb::init<size_t, size_t, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Vector> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Vector> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Vector> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Vector> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Vector> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Point2>>>(m_, "BetweenFactorPoint2")
      .def(nb::init<size_t, size_t, const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Point2> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Point2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Point3>>>(m_, "BetweenFactorPoint3")
      .def(nb::init<size_t, size_t, const gtsam::Point3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Point3> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Rot2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Rot2>>>(m_, "BetweenFactorRot2")
      .def(nb::init<size_t, size_t, const gtsam::Rot2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Rot2> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Rot2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Rot2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Rot2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::SO3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::SO3>>>(m_, "BetweenFactorSO3")
      .def(nb::init<size_t, size_t, const gtsam::SO3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::SO3> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::SO3> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::SO3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::SO3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::SO3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::SO4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::SO4>>>(m_, "BetweenFactorSO4")
      .def(nb::init<size_t, size_t, const gtsam::SO4 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::SO4> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::SO4> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::SO4> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::SO4> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::SO4> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Rot3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Rot3>>>(m_, "BetweenFactorRot3")
      .def(nb::init<size_t, size_t, const gtsam::Rot3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Rot3> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Rot3> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Rot3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Rot3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Rot3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>>(m_, "BetweenFactorPose2")
      .def(nb::init<size_t, size_t, const gtsam::Pose2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Pose2> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Pose2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>>(m_, "BetweenFactorPose3")
      .def(nb::init<size_t, size_t, const gtsam::Pose3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::Pose3> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::Pose3> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>>(m_, "BetweenFactorConstantBias")
      .def(nb::init<size_t, size_t, const gtsam::imuBias::ConstantBias &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("relativePose"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>>(m_, "GeneralSFMFactor2Cal3_S2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("poseKey"), nb::arg("landmarkKey"), nb::arg("calibKey"))
      .def("measured", [](gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GeneralSFMFactor2<gtsam::Cal3DS2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3DS2>>>(m_, "GeneralSFMFactor2Cal3DS2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("poseKey"), nb::arg("landmarkKey"), nb::arg("calibKey"))
      .def("measured", [](gtsam::GeneralSFMFactor2<gtsam::Cal3DS2> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3DS2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3DS2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GeneralSFMFactor2<gtsam::Cal3DS2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GeneralSFMFactor2<gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler>>>(m_, "GeneralSFMFactor2Cal3Bundler")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("poseKey"), nb::arg("landmarkKey"), nb::arg("calibKey"))
      .def("measured", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GeneralSFMFactor2<gtsam::Cal3Bundler> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye>>>(m_, "GeneralSFMFactor2Cal3Fisheye")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("poseKey"), nb::arg("landmarkKey"), nb::arg("calibKey"))
      .def("measured", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GeneralSFMFactor2<gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GeneralSFMFactor2<gtsam::Cal3Unified>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3Unified>>>(m_, "GeneralSFMFactor2Cal3Unified")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("poseKey"), nb::arg("landmarkKey"), nb::arg("calibKey"))
      .def("measured", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Unified> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Unified> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GeneralSFMFactor2<gtsam::Cal3Unified> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GeneralSFMFactor2<gtsam::Cal3Unified> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GeneralSFMFactor2<gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));
      nb::enum_<gtsam::LinearizationMode>(m_, "LinearizationMode", nb::is_arithmetic())
      .value("HESSIAN", gtsam::LinearizationMode::HESSIAN)
      .value("IMPLICIT_SCHUR", gtsam::LinearizationMode::IMPLICIT_SCHUR)
      .value("JACOBIAN_Q", gtsam::LinearizationMode::JACOBIAN_Q)
      .value("JACOBIAN_SVD", gtsam::LinearizationMode::JACOBIAN_SVD);

  nb::enum_<gtsam::DegeneracyMode>(m_, "DegeneracyMode", nb::is_arithmetic())
      .value("IGNORE_DEGENERACY", gtsam::DegeneracyMode::IGNORE_DEGENERACY)
      .value("ZERO_ON_DEGENERACY", gtsam::DegeneracyMode::ZERO_ON_DEGENERACY)
      .value("HANDLE_INFINITY", gtsam::DegeneracyMode::HANDLE_INFINITY);

  nb::class_<gtsam::SmartProjectionParams>(m_, "SmartProjectionParams")
      .def(nb::init<>())
      .def("setLinearizationMode", [](gtsam::SmartProjectionParams *self, gtsam::LinearizationMode linMode) { self->setLinearizationMode(linMode); }, nb::arg("linMode"))
      .def("setDegeneracyMode", [](gtsam::SmartProjectionParams *self, gtsam::DegeneracyMode degMode) { self->setDegeneracyMode(degMode); }, nb::arg("degMode"))
      .def("setRankTolerance", [](gtsam::SmartProjectionParams *self, double rankTol) { self->setRankTolerance(rankTol); }, nb::arg("rankTol"))
      .def("setEnableEPI", [](gtsam::SmartProjectionParams *self, bool enableEPI) { self->setEnableEPI(enableEPI); }, nb::arg("enableEPI"))
      .def("setLandmarkDistanceThreshold", [](gtsam::SmartProjectionParams *self, bool landmarkDistanceThreshold) { self->setLandmarkDistanceThreshold(landmarkDistanceThreshold); }, nb::arg("landmarkDistanceThreshold"))
      .def("setDynamicOutlierRejectionThreshold", [](gtsam::SmartProjectionParams *self, bool dynOutRejectionThreshold) { self->setDynamicOutlierRejectionThreshold(dynOutRejectionThreshold); }, nb::arg("dynOutRejectionThreshold"));

  nb::class_<gtsam::EssentialMatrixFactor, gtsam::NoiseModelFactor>(m_, "EssentialMatrixFactor")
      .def(nb::init<size_t, const gtsam::Point2 &, const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("pA"), nb::arg("pB"), nb::arg("noiseModel"))
      .def("print", [](gtsam::EssentialMatrixFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::EssentialMatrixFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::EssentialMatrixFactor *self, const gtsam::EssentialMatrixFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("evaluateError", [](gtsam::EssentialMatrixFactor *self, const gtsam::EssentialMatrix &E) { return self->evaluateError(E); }, nb::arg("E"));

  nb::class_<gtsam::EssentialMatrixConstraint, gtsam::NoiseModelFactor>(m_, "EssentialMatrixConstraint")
      .def(nb::init<size_t, size_t, const gtsam::EssentialMatrix &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measuredE"), nb::arg("model"))
      .def("print", [](gtsam::EssentialMatrixConstraint *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::EssentialMatrixConstraint &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::EssentialMatrixConstraint *self, const gtsam::EssentialMatrixConstraint &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("evaluateError", [](gtsam::EssentialMatrixConstraint *self, const gtsam::Pose3 &p1, const gtsam::Pose3 &p2) { return self->evaluateError(p1, p2); }, nb::arg("p1"), nb::arg("p2"))
      .def("measured", [](gtsam::EssentialMatrixConstraint *self) { return self->measured(); });
  nb::enum_<gtsam::NoiseFormat>(m_, "NoiseFormat", nb::is_arithmetic())
      .value("NoiseFormatG2O", gtsam::NoiseFormat::NoiseFormatG2O)
      .value("NoiseFormatTORO", gtsam::NoiseFormat::NoiseFormatTORO)
      .value("NoiseFormatGRAPH", gtsam::NoiseFormat::NoiseFormatGRAPH)
      .value("NoiseFormatCOV", gtsam::NoiseFormat::NoiseFormatCOV)
      .value("NoiseFormatAUTO", gtsam::NoiseFormat::NoiseFormatAUTO);

  nb::enum_<gtsam::KernelFunctionType>(m_, "KernelFunctionType", nb::is_arithmetic())
      .value("KernelFunctionTypeNONE", gtsam::KernelFunctionType::KernelFunctionTypeNONE)
      .value("KernelFunctionTypeHUBER", gtsam::KernelFunctionType::KernelFunctionTypeHUBER)
      .value("KernelFunctionTypeTUKEY", gtsam::KernelFunctionType::KernelFunctionTypeTUKEY);

  nb::class_<gtsam::InitializePose3>(m_, "InitializePose3")
      .def_static("computeOrientationsChordal", [](const gtsam::NonlinearFactorGraph &pose3Graph) { return gtsam::InitializePose3::computeOrientationsChordal(pose3Graph); }, nb::arg("pose3Graph"))
      .def_static("computeOrientationsGradient", [](const gtsam::NonlinearFactorGraph &pose3Graph, const gtsam::Values &givenGuess, size_t maxIter, const bool setRefFrame) { return gtsam::InitializePose3::computeOrientationsGradient(pose3Graph, givenGuess, maxIter, setRefFrame); }, nb::arg("pose3Graph"), nb::arg("givenGuess"), nb::arg("maxIter"), nb::arg("setRefFrame"))
      .def_static("computeOrientationsGradient", [](const gtsam::NonlinearFactorGraph &pose3Graph, const gtsam::Values &givenGuess) { return gtsam::InitializePose3::computeOrientationsGradient(pose3Graph, givenGuess); }, nb::arg("pose3Graph"), nb::arg("givenGuess"))
      .def_static("buildPose3graph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::InitializePose3::buildPose3graph(graph); }, nb::arg("graph"))
      .def_static("initializeOrientations", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::InitializePose3::initializeOrientations(graph); }, nb::arg("graph"))
      .def_static("initialize", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &givenGuess, bool useGradient) { return gtsam::InitializePose3::initialize(graph, givenGuess, useGradient); }, nb::arg("graph"), nb::arg("givenGuess"), nb::arg("useGradient"))
      .def_static("initialize", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::InitializePose3::initialize(graph); }, nb::arg("graph"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::Point2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::Point2>>>(m_, "KarcherMeanFactorPoint2")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::Rot2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::Rot2>>>(m_, "KarcherMeanFactorRot2")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::Pose2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::Pose2>>>(m_, "KarcherMeanFactorPose2")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::Point3>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::Point3>>>(m_, "KarcherMeanFactorPoint3")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::SO3>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::SO3>>>(m_, "KarcherMeanFactorSO3")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::SO4>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::SO4>>>(m_, "KarcherMeanFactorSO4")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::Rot3>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::Rot3>>>(m_, "KarcherMeanFactorRot3")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::KarcherMeanFactor<gtsam::Pose3>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::KarcherMeanFactor<gtsam::Pose3>>>(m_, "KarcherMeanFactorPose3")
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("keys"));

  nb::class_<gtsam::FrobeniusFactor<gtsam::SO3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::FrobeniusFactor<gtsam::SO3>>>(m_, "FrobeniusFactorSO3")
      .def(nb::init<size_t, size_t>(), nb::arg("key1"), nb::arg("key2"))
      .def(nb::init<size_t, size_t, boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("model"))
      .def("evaluateError", [](gtsam::FrobeniusFactor<gtsam::SO3> *self, const gtsam::SO3 &R1, const gtsam::SO3 &R2) { return self->evaluateError(R1, R2); }, nb::arg("R1"), nb::arg("R2"));

  nb::class_<gtsam::FrobeniusFactor<gtsam::SO4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::FrobeniusFactor<gtsam::SO4>>>(m_, "FrobeniusFactorSO4")
      .def(nb::init<size_t, size_t>(), nb::arg("key1"), nb::arg("key2"))
      .def(nb::init<size_t, size_t, boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("model"))
      .def("evaluateError", [](gtsam::FrobeniusFactor<gtsam::SO4> *self, const gtsam::SO4 &R1, const gtsam::SO4 &R2) { return self->evaluateError(R1, R2); }, nb::arg("R1"), nb::arg("R2"));

  nb::class_<gtsam::FrobeniusBetweenFactor<gtsam::SO3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::FrobeniusBetweenFactor<gtsam::SO3>>>(m_, "FrobeniusBetweenFactorSO3")
      .def(nb::init<size_t, size_t, const gtsam::SO3 &>(), nb::arg("key1"), nb::arg("key2"), nb::arg("R12"))
      .def(nb::init<size_t, size_t, const gtsam::SO3 &, boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("R12"), nb::arg("model"))
      .def("evaluateError", [](gtsam::FrobeniusBetweenFactor<gtsam::SO3> *self, const gtsam::SO3 &R1, const gtsam::SO3 &R2) { return self->evaluateError(R1, R2); }, nb::arg("R1"), nb::arg("R2"));

  nb::class_<gtsam::FrobeniusBetweenFactor<gtsam::SO4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::FrobeniusBetweenFactor<gtsam::SO4>>>(m_, "FrobeniusBetweenFactorSO4")
      .def(nb::init<size_t, size_t, const gtsam::SO4 &>(), nb::arg("key1"), nb::arg("key2"), nb::arg("R12"))
      .def(nb::init<size_t, size_t, const gtsam::SO4 &, boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("R12"), nb::arg("model"))
      .def("evaluateError", [](gtsam::FrobeniusBetweenFactor<gtsam::SO4> *self, const gtsam::SO4 &R1, const gtsam::SO4 &R2) { return self->evaluateError(R1, R2); }, nb::arg("R1"), nb::arg("R2"));
  auto m_lago = m_.def_submodule("lago", "lago submodule");

  m_lago.def("initialize", [](const gtsam::NonlinearFactorGraph &graph, bool useOdometricPath) { return gtsam::lago::initialize(graph, useOdometricPath); }, nb::arg("graph"), nb::arg("useOdometricPath") = true);
  m_lago.def("initialize", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &initialGuess) { return gtsam::lago::initialize(graph, initialGuess); }, nb::arg("graph"), nb::arg("initialGuess"));
  nb::class_<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>>(m_, "GenericProjectionFactorCal3_S2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("body_P_sensor"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, bool, bool>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2>, bool, bool, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"), nb::arg("body_P_sensor"))
      .def("measured", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> *self) { return self->measured(); })
      .def("calibration", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> *self) { return self->calibration(); })
      .def("verboseCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> *self) { return self->verboseCheirality(); })
      .def("throwCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> *self) { return self->throwCheirality(); })
      // // .def("serialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>>(m_, "GenericProjectionFactorCal3DS2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3DS2>>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3DS2>, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("body_P_sensor"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3DS2>, bool, bool>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3DS2>, bool, bool, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"), nb::arg("body_P_sensor"))
      .def("measured", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> *self) { return self->measured(); })
      .def("calibration", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> *self) { return self->calibration(); })
      .def("verboseCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> *self) { return self->verboseCheirality(); })
      .def("throwCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> *self) { return self->throwCheirality(); })
      // // .def("serialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye>>>(m_, "GenericProjectionFactorCal3Fisheye")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Fisheye>>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Fisheye>, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("body_P_sensor"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Fisheye>, bool, bool>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Fisheye>, bool, bool, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"), nb::arg("body_P_sensor"))
      .def("measured", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> *self) { return self->measured(); })
      .def("calibration", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> *self) { return self->calibration(); })
      .def("verboseCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> *self) { return self->verboseCheirality(); })
      .def("throwCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> *self) { return self->throwCheirality(); })
      // // .def("serialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified>>>(m_, "GenericProjectionFactorCal3Unified")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Unified>>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Unified>, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("body_P_sensor"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Unified>, bool, bool>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"))
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3Unified>, bool, bool, const gtsam::Pose3 &>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("pointKey"), nb::arg("k"), nb::arg("throwCheirality"), nb::arg("verboseCheirality"), nb::arg("body_P_sensor"))
      .def("measured", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> *self) { return self->measured(); })
      .def("calibration", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> *self) { return self->calibration(); })
      .def("verboseCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> *self) { return self->verboseCheirality(); })
      .def("throwCheirality", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> *self) { return self->throwCheirality(); })
      // // .def("serialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3>>>(m_, "GeneralSFMFactorCal3_S2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3DS2>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3DS2>, gtsam::Point3>>>(m_, "GeneralSFMFactorCal3DS2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3DS2>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>>>(m_, "GeneralSFMFactorCal3Bundler")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>, gtsam::Point3>>>(m_, "GeneralSFMFactorCal3Fisheye")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>, gtsam::Point3>>>(m_, "GeneralSFMFactorCal3Unified")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3_S2>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3_S2>, gtsam::Point3>>>(m_, "GeneralSFMFactorPoseCal3_S2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3_S2>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3DS2>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3DS2>, gtsam::Point3>>>(m_, "GeneralSFMFactorPoseCal3DS2")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3DS2>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Bundler>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Bundler>, gtsam::Point3>>>(m_, "GeneralSFMFactorPoseCal3Bundler")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Bundler>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Fisheye>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Fisheye>, gtsam::Point3>>>(m_, "GeneralSFMFactorPoseCal3Fisheye")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Fisheye>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Unified>, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Unified>, gtsam::Point3>>>(m_, "GeneralSFMFactorPoseCal3Unified")
      .def(nb::init<const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t>(), nb::arg("measured"), nb::arg("model"), nb::arg("cameraKey"), nb::arg("landmarkKey"))
      .def("measured", [](gtsam::GeneralSFMFactor<gtsam::PinholePose<gtsam::Cal3Unified>, gtsam::Point3> *self) { return self->measured(); });

  nb::class_<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>>(m_, "SmartProjectionPose3Factor")
      .def(nb::init<const boost::shared_ptr<gtsam::noiseModel::Base>, const boost::shared_ptr<gtsam::Cal3_S2>>(), nb::arg("noise"), nb::arg("K"))
      .def(nb::init<const boost::shared_ptr<gtsam::noiseModel::Base>, const boost::shared_ptr<gtsam::Cal3_S2>, const gtsam::Pose3 &>(), nb::arg("noise"), nb::arg("K"), nb::arg("body_P_sensor"))
      .def(nb::init<const boost::shared_ptr<gtsam::noiseModel::Base>, const boost::shared_ptr<gtsam::Cal3_S2>, const gtsam::SmartProjectionParams &>(), nb::arg("noise"), nb::arg("K"), nb::arg("params"))
      .def(nb::init<const boost::shared_ptr<gtsam::noiseModel::Base>, const boost::shared_ptr<gtsam::Cal3_S2>, const gtsam::Pose3 &, const gtsam::SmartProjectionParams &>(), nb::arg("noise"), nb::arg("K"), nb::arg("body_P_sensor"), nb::arg("params"))
      .def("add", [](gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> *self, const gtsam::Point2 &measured_i, size_t poseKey_i) { self->add(measured_i, poseKey_i); }, nb::arg("measured_i"), nb::arg("poseKey_i"))
      // // .def("serialize", [](gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>>(m_, "GenericStereoFactor3D")
      .def(nb::init<const gtsam::StereoPoint2 &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t, size_t, const boost::shared_ptr<gtsam::Cal3_S2Stereo>>(), nb::arg("measured"), nb::arg("noiseModel"), nb::arg("poseKey"), nb::arg("landmarkKey"), nb::arg("K"))
      .def("measured", [](gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> *self) { return self->measured(); })
      .def("calibration", [](gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> *self) { return self->calibration(); })
      // // .def("serialize", [](gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::PoseTranslationPrior<gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PoseTranslationPrior<gtsam::Pose2>>>(m_, "PoseTranslationPrior2D")
      .def(nb::init<size_t, const gtsam::Pose2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("pose_z"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::PoseTranslationPrior<gtsam::Pose2> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::PoseTranslationPrior<gtsam::Pose2> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::PoseTranslationPrior<gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::PoseTranslationPrior<gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PoseTranslationPrior<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::PoseTranslationPrior<gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PoseTranslationPrior<gtsam::Pose3>>>(m_, "PoseTranslationPrior3D")
      .def(nb::init<size_t, const gtsam::Pose3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("pose_z"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::PoseTranslationPrior<gtsam::Pose3> *self) { return self->measured(); })
      // // .def("serialize", [](gtsam::PoseTranslationPrior<gtsam::Pose3> *self) { return gtsam::serialize(*self); })
      // // .def("deserialize", [](gtsam::PoseTranslationPrior<gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // // .def(nb::pickle([](const gtsam::PoseTranslationPrior<gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PoseTranslationPrior<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

      nb::class_<gtsam::PoseRotationPrior<gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose2>>>(m_, "PoseRotationPrior2D")
      .def(nb::init<size_t, const gtsam::Pose2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("pose_z"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::PoseRotationPrior<gtsam::Pose2> *self) { return self->measured(); });

  nb::class_<gtsam::PoseRotationPrior<gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose3>>>(m_, "PoseRotationPrior3D")
      .def(nb::init<size_t, const gtsam::Pose3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("pose_z"), nb::arg("noiseModel"))
      .def("measured", [](gtsam::PoseRotationPrior<gtsam::Pose3> *self) { return self->measured(); });

  m_.def("load2D", [](string filename, boost::shared_ptr<gtsam::noiseModel::Diagonal> model, size_t maxIndex, bool addNoise, bool smart, gtsam::NoiseFormat noiseFormat, gtsam::KernelFunctionType kernelFunctionType) { return gtsam::load2D(filename, model, maxIndex, addNoise, smart, noiseFormat, kernelFunctionType); }, nb::arg("filename"), nb::arg("model") = nullptr, nb::arg("maxIndex") = 0, nb::arg("addNoise") = false, nb::arg("smart") = true, nb::arg("noiseFormat") = gtsam::NoiseFormat::NoiseFormatAUTO, nb::arg("kernelFunctionType") = gtsam::KernelFunctionType::KernelFunctionTypeNONE);
  m_.def("save2D", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &config, boost::shared_ptr<gtsam::noiseModel::Diagonal> model, string filename) { gtsam::save2D(graph, config, model, filename); }, nb::arg("graph"), nb::arg("config"), nb::arg("model"), nb::arg("filename"));
  m_.def("parse2DFactors", [](string filename) { return gtsam::parse2DFactors(filename); }, nb::arg("filename"));
  m_.def("parse3DFactors", [](string filename) { return gtsam::parse3DFactors(filename); }, nb::arg("filename"));
  m_.def("load3D", [](string filename) { return gtsam::load3D(filename); }, nb::arg("filename"));
  m_.def("readG2o", [](string filename, const bool is3D, gtsam::KernelFunctionType kernelFunctionType) { return gtsam::readG2o(filename, is3D, kernelFunctionType); }, nb::arg("filename"), nb::arg("is3D") = false, nb::arg("kernelFunctionType") = gtsam::KernelFunctionType::KernelFunctionTypeNONE);
  m_.def("writeG2o", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &estimate, string filename) { gtsam::writeG2o(graph, estimate, filename); }, nb::arg("graph"), nb::arg("estimate"), nb::arg("filename"));
  m_.def("FindKarcherMean", [](const gtsam::Rot3Vector &rotations) { return gtsam::FindKarcherMean(rotations); }, nb::arg("rotations"));
  m_.def("ConvertNoiseModel", [](boost::shared_ptr<gtsam::noiseModel::Base> model, size_t d) { return gtsam::ConvertNoiseModel(model, d); }, nb::arg("model"), nb::arg("d"));
}
