/**
 * @file    navigation.cpp
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

#include "utils/boost_optional.h"
#include "utils/boost_shared_ptr.h"

// These are the included headers listed in `gtsam.i`
#include "gtsam/navigation/AHRSFactor.h"
#include "gtsam/navigation/AttitudeFactor.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/navigation/NavState.h"
#include "gtsam/navigation/PreintegratedRotation.h"
#include "gtsam/navigation/PreintegrationParams.h"
#include "gtsam/navigation/Scenario.h"
#include "gtsam/navigation/ScenarioRunner.h"

using namespace std;

namespace nb = nanobind;

void navigation(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of navigation";

  auto m_imuBias = m_.def_submodule("imuBias", "imuBias submodule");

  nb::class_<gtsam::imuBias::ConstantBias>(m_imuBias, "ConstantBias")
      .def(nb::init<>())
      .def(nb::init<const gtsam::Vector &, const gtsam::Vector &>(), nb::arg("biasAcc"), nb::arg("biasGyro"))
      .def("print", [](gtsam::imuBias::ConstantBias *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::imuBias::ConstantBias &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::imuBias::ConstantBias *self, const gtsam::imuBias::ConstantBias &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("vector", [](gtsam::imuBias::ConstantBias *self) { return self->vector(); })
      .def("accelerometer", [](gtsam::imuBias::ConstantBias *self) { return self->accelerometer(); })
      .def("gyroscope", [](gtsam::imuBias::ConstantBias *self) { return self->gyroscope(); })
      .def("correctAccelerometer", [](gtsam::imuBias::ConstantBias *self, const gtsam::Vector &measurement) { return self->correctAccelerometer(measurement); }, nb::arg("measurement"))
      .def("correctGyroscope", [](gtsam::imuBias::ConstantBias *self, const gtsam::Vector &measurement) { return self->correctGyroscope(measurement); }, nb::arg("measurement"))
      // .def("serialize", [](gtsam::imuBias::ConstantBias *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::imuBias::ConstantBias *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::imuBias::ConstantBias &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::imuBias::ConstantBias obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Identity", []() { return gtsam::imuBias::ConstantBias::Identity(); })
      .def(-nb::self)
      .def(nb::self + nb::self)
      .def(nb::self - nb::self);

  nb::class_<gtsam::NavState>(m_, "NavState")
      .def(nb::init<>())
      .def(nb::init<const gtsam::Rot3 &, const gtsam::Point3 &, const gtsam::Vector &>(), nb::arg("R"), nb::arg("t"), nb::arg("v"))
      .def(nb::init<const gtsam::Pose3 &, const gtsam::Vector &>(), nb::arg("pose"), nb::arg("v"))
      .def("print", [](gtsam::NavState *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::NavState &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::NavState *self, const gtsam::NavState &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("attitude", [](gtsam::NavState *self) { return self->attitude(); })
      .def("position", [](gtsam::NavState *self) { return self->position(); })
      .def("velocity", [](gtsam::NavState *self) { return self->velocity(); })
      .def("pose", [](gtsam::NavState *self) { return self->pose(); })
      .def("retract", [](gtsam::NavState *self, const gtsam::Vector &x) { return self->retract(x); }, nb::arg("x"))
      .def("localCoordinates", [](gtsam::NavState *self, const gtsam::NavState &g) { return self->localCoordinates(g); }, nb::arg("g"));
  // .def("serialize", [](gtsam::NavState *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NavState *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NavState &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NavState obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PreintegratedRotationParams>(m_, "PreintegratedRotationParams")
      .def(nb::init<>())
      .def("print", [](gtsam::PreintegratedRotationParams *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::PreintegratedRotationParams &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::PreintegratedRotationParams *self, const gtsam::PreintegratedRotationParams &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("setGyroscopeCovariance", [](gtsam::PreintegratedRotationParams *self, const gtsam::Matrix &cov) { self->setGyroscopeCovariance(cov); }, nb::arg("cov"))
      .def("setOmegaCoriolis", [](gtsam::PreintegratedRotationParams *self, const gtsam::Vector &omega) { self->setOmegaCoriolis(omega); }, nb::arg("omega"))
      .def("setBodyPSensor", [](gtsam::PreintegratedRotationParams *self, const gtsam::Pose3 &pose) { self->setBodyPSensor(pose); }, nb::arg("pose"))
      .def("getGyroscopeCovariance", [](gtsam::PreintegratedRotationParams *self) { return self->getGyroscopeCovariance(); })
      .def("getOmegaCoriolis", [](gtsam::PreintegratedRotationParams *self) { return self->getOmegaCoriolis(); })
      .def("getBodyPSensor", [](gtsam::PreintegratedRotationParams *self) { return self->getBodyPSensor(); });

  nb::class_<gtsam::PreintegrationParams, gtsam::PreintegratedRotationParams>(m_, "PreintegrationParams")
      .def(nb::init<const gtsam::Vector &>(), nb::arg("n_gravity"))
      .def("print", [](gtsam::PreintegrationParams *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::PreintegrationParams &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::PreintegrationParams *self, const gtsam::PreintegrationParams &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("setAccelerometerCovariance", [](gtsam::PreintegrationParams *self, const gtsam::Matrix &cov) { self->setAccelerometerCovariance(cov); }, nb::arg("cov"))
      .def("setIntegrationCovariance", [](gtsam::PreintegrationParams *self, const gtsam::Matrix &cov) { self->setIntegrationCovariance(cov); }, nb::arg("cov"))
      .def("setUse2ndOrderCoriolis", [](gtsam::PreintegrationParams *self, bool flag) { self->setUse2ndOrderCoriolis(flag); }, nb::arg("flag"))
      .def("getAccelerometerCovariance", [](gtsam::PreintegrationParams *self) { return self->getAccelerometerCovariance(); })
      .def("getIntegrationCovariance", [](gtsam::PreintegrationParams *self) { return self->getIntegrationCovariance(); })
      .def("getUse2ndOrderCoriolis", [](gtsam::PreintegrationParams *self) { return self->getUse2ndOrderCoriolis(); })
      // .def("serialize", [](gtsam::PreintegrationParams *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::PreintegrationParams *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::PreintegrationParams &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PreintegrationParams obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("MakeSharedD", [](double g) { return gtsam::PreintegrationParams::MakeSharedD(g); }, nb::arg("g"))
      .def_static("MakeSharedU", [](double g) { return gtsam::PreintegrationParams::MakeSharedU(g); }, nb::arg("g"))
      .def_static("MakeSharedD", []() { return gtsam::PreintegrationParams::MakeSharedD(); })
      .def_static("MakeSharedU", []() { return gtsam::PreintegrationParams::MakeSharedU(); })
      .def_rw("n_gravity", &gtsam::PreintegrationParams::n_gravity);

  nb::class_<gtsam::PreintegratedImuMeasurements>(m_, "PreintegratedImuMeasurements")
      .def(nb::init<const boost::shared_ptr<gtsam::PreintegrationParams>>(), nb::arg("params"))
      .def(nb::init<const boost::shared_ptr<gtsam::PreintegrationParams>, const gtsam::imuBias::ConstantBias &>(), nb::arg("params"), nb::arg("bias"))
      .def("print", [](gtsam::PreintegratedImuMeasurements *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::PreintegratedImuMeasurements &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::PreintegratedImuMeasurements *self, const gtsam::PreintegratedImuMeasurements &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("integrateMeasurement", [](gtsam::PreintegratedImuMeasurements *self, const gtsam::Vector &measuredAcc, const gtsam::Vector &measuredOmega, double deltaT) { self->integrateMeasurement(measuredAcc, measuredOmega, deltaT); }, nb::arg("measuredAcc"), nb::arg("measuredOmega"), nb::arg("deltaT"))
      .def("resetIntegration", [](gtsam::PreintegratedImuMeasurements *self) { self->resetIntegration(); })
      .def("resetIntegrationAndSetBias", [](gtsam::PreintegratedImuMeasurements *self, const gtsam::imuBias::ConstantBias &biasHat) { self->resetIntegrationAndSetBias(biasHat); }, nb::arg("biasHat"))
      .def("preintMeasCov", [](gtsam::PreintegratedImuMeasurements *self) { return self->preintMeasCov(); })
      .def("preintegrated", [](gtsam::PreintegratedImuMeasurements *self) { return self->preintegrated(); })
      .def("deltaTij", [](gtsam::PreintegratedImuMeasurements *self) { return self->deltaTij(); })
      .def("deltaRij", [](gtsam::PreintegratedImuMeasurements *self) { return self->deltaRij(); })
      .def("deltaPij", [](gtsam::PreintegratedImuMeasurements *self) { return self->deltaPij(); })
      .def("deltaVij", [](gtsam::PreintegratedImuMeasurements *self) { return self->deltaVij(); })
      .def("biasHat", [](gtsam::PreintegratedImuMeasurements *self) { return self->biasHat(); })
      .def("biasHatVector", [](gtsam::PreintegratedImuMeasurements *self) { return self->biasHatVector(); })
      .def("predict", [](gtsam::PreintegratedImuMeasurements *self, const gtsam::NavState &state_i, const gtsam::imuBias::ConstantBias &bias) { return self->predict(state_i, bias); }, nb::arg("state_i"), nb::arg("bias"));
  // .def("serialize", [](gtsam::PreintegratedImuMeasurements *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PreintegratedImuMeasurements *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PreintegratedImuMeasurements &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PreintegratedImuMeasurements obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::ImuFactor, gtsam::NonlinearFactor>(m_, "ImuFactor")
      .def(nb::init<size_t, size_t, size_t, size_t, size_t, const gtsam::PreintegratedImuMeasurements &>(), nb::arg("pose_i"), nb::arg("vel_i"), nb::arg("pose_j"), nb::arg("vel_j"), nb::arg("bias"), nb::arg("preintegratedMeasurements"))
      .def("preintegratedMeasurements", [](gtsam::ImuFactor *self) { return self->preintegratedMeasurements(); })
      .def("evaluateError", [](gtsam::ImuFactor *self, const gtsam::Pose3 &pose_i, const gtsam::Vector &vel_i, const gtsam::Pose3 &pose_j, const gtsam::Vector &vel_j, const gtsam::imuBias::ConstantBias &bias) { return self->evaluateError(pose_i, vel_i, pose_j, vel_j, bias); }, nb::arg("pose_i"), nb::arg("vel_i"), nb::arg("pose_j"), nb::arg("vel_j"), nb::arg("bias"));

  nb::class_<gtsam::PreintegrationCombinedParams, gtsam::PreintegrationParams>(m_, "PreintegrationCombinedParams")
      .def(nb::init<const gtsam::Vector &>(), nb::arg("n_gravity"))
      .def("print", [](gtsam::PreintegrationCombinedParams *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::PreintegrationCombinedParams &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::PreintegrationCombinedParams *self, const gtsam::PreintegrationCombinedParams &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("setBiasAccCovariance", [](gtsam::PreintegrationCombinedParams *self, const gtsam::Matrix &cov) { self->setBiasAccCovariance(cov); }, nb::arg("cov"))
      .def("setBiasOmegaCovariance", [](gtsam::PreintegrationCombinedParams *self, const gtsam::Matrix &cov) { self->setBiasOmegaCovariance(cov); }, nb::arg("cov"))
      .def("setBiasAccOmegaInit", [](gtsam::PreintegrationCombinedParams *self, const gtsam::Matrix &cov) { self->setBiasAccOmegaInit(cov); }, nb::arg("cov"))
      .def("getBiasAccCovariance", [](gtsam::PreintegrationCombinedParams *self) { return self->getBiasAccCovariance(); })
      .def("getBiasOmegaCovariance", [](gtsam::PreintegrationCombinedParams *self) { return self->getBiasOmegaCovariance(); })
      .def("getBiasAccOmegaInit", [](gtsam::PreintegrationCombinedParams *self) { return self->getBiasAccOmegaInit(); })
      .def_static("MakeSharedD", [](double g) { return gtsam::PreintegrationCombinedParams::MakeSharedD(g); }, nb::arg("g"))
      .def_static("MakeSharedU", [](double g) { return gtsam::PreintegrationCombinedParams::MakeSharedU(g); }, nb::arg("g"))
      .def_static("MakeSharedD", []() { return gtsam::PreintegrationCombinedParams::MakeSharedD(); })
      .def_static("MakeSharedU", []() { return gtsam::PreintegrationCombinedParams::MakeSharedU(); });

  nb::class_<gtsam::PreintegratedCombinedMeasurements>(m_, "PreintegratedCombinedMeasurements")
      .def(nb::init<const boost::shared_ptr<gtsam::PreintegrationCombinedParams>>(), nb::arg("params"))
      .def(nb::init<const boost::shared_ptr<gtsam::PreintegrationCombinedParams>, const gtsam::imuBias::ConstantBias &>(), nb::arg("params"), nb::arg("bias"))
      .def("print", [](gtsam::PreintegratedCombinedMeasurements *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "Preintegrated Measurements:")
      .def("__repr__", [](const gtsam::PreintegratedCombinedMeasurements &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "Preintegrated Measurements:")
      .def("equals", [](gtsam::PreintegratedCombinedMeasurements *self, const gtsam::PreintegratedCombinedMeasurements &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("integrateMeasurement", [](gtsam::PreintegratedCombinedMeasurements *self, const gtsam::Vector &measuredAcc, const gtsam::Vector &measuredOmega, double deltaT) { self->integrateMeasurement(measuredAcc, measuredOmega, deltaT); }, nb::arg("measuredAcc"), nb::arg("measuredOmega"), nb::arg("deltaT"))
      .def("resetIntegration", [](gtsam::PreintegratedCombinedMeasurements *self) { self->resetIntegration(); })
      .def("resetIntegrationAndSetBias", [](gtsam::PreintegratedCombinedMeasurements *self, const gtsam::imuBias::ConstantBias &biasHat) { self->resetIntegrationAndSetBias(biasHat); }, nb::arg("biasHat"))
      .def("preintMeasCov", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->preintMeasCov(); })
      .def("deltaTij", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->deltaTij(); })
      .def("deltaRij", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->deltaRij(); })
      .def("deltaPij", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->deltaPij(); })
      .def("deltaVij", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->deltaVij(); })
      .def("biasHat", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->biasHat(); })
      .def("biasCorrectedDelta", [](gtsam::PreintegratedCombinedMeasurements *self, const gtsam::imuBias::ConstantBias &bias) { return self->biasCorrectedDelta(bias); }, nb::arg("bias"))
      .def("preintegrated_H_biasAcc", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->preintegrated_H_biasAcc(); })
      .def("preintegrated_H_biasOmega", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->preintegrated_H_biasOmega(); })
      .def("biasHatVector", [](gtsam::PreintegratedCombinedMeasurements *self) { return self->biasHatVector(); })
      .def("predict", [](gtsam::PreintegratedCombinedMeasurements *self, const gtsam::NavState &state_i, const gtsam::imuBias::ConstantBias &bias) { return self->predict(state_i, bias); }, nb::arg("state_i"), nb::arg("bias"));

  nb::class_<gtsam::CombinedImuFactor, gtsam::NonlinearFactor>(m_, "CombinedImuFactor")
      .def(nb::init<size_t, size_t, size_t, size_t, size_t, size_t, const gtsam::PreintegratedCombinedMeasurements &>(), nb::arg("pose_i"), nb::arg("vel_i"), nb::arg("pose_j"), nb::arg("vel_j"), nb::arg("bias_i"), nb::arg("bias_j"), nb::arg("CombinedPreintegratedMeasurements"))
      .def("preintegratedMeasurements", [](gtsam::CombinedImuFactor *self) { return self->preintegratedMeasurements(); })
      .def("evaluateError", [](gtsam::CombinedImuFactor *self, const gtsam::Pose3 &pose_i, const gtsam::Vector &vel_i, const gtsam::Pose3 &pose_j, const gtsam::Vector &vel_j, const gtsam::imuBias::ConstantBias &bias_i, const gtsam::imuBias::ConstantBias &bias_j) { return self->evaluateError(pose_i, vel_i, pose_j, vel_j, bias_i, bias_j); }, nb::arg("pose_i"), nb::arg("vel_i"), nb::arg("pose_j"), nb::arg("vel_j"), nb::arg("bias_i"), nb::arg("bias_j"));

  nb::class_<gtsam::PreintegratedAhrsMeasurements>(m_, "PreintegratedAhrsMeasurements")
      .def(nb::init<const boost::shared_ptr<gtsam::PreintegrationParams>, const gtsam::Vector &>(), nb::arg("params"), nb::arg("biasHat"))
      .def(nb::init<const boost::shared_ptr<gtsam::PreintegrationParams>, const gtsam::Vector &, double, const gtsam::Rot3 &, const gtsam::Matrix &, const gtsam::Matrix &>(), nb::arg("p"), nb::arg("bias_hat"), nb::arg("deltaTij"), nb::arg("deltaRij"), nb::arg("delRdelBiasOmega"), nb::arg("preint_meas_cov"))
      .def(nb::init<const gtsam::PreintegratedAhrsMeasurements &>(), nb::arg("rhs"))
      .def("print", [](gtsam::PreintegratedAhrsMeasurements *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "Preintegrated Measurements: ")
      .def("__repr__", [](const gtsam::PreintegratedAhrsMeasurements &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "Preintegrated Measurements: ")
      .def("equals", [](gtsam::PreintegratedAhrsMeasurements *self, const gtsam::PreintegratedAhrsMeasurements &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("deltaRij", [](gtsam::PreintegratedAhrsMeasurements *self) { return self->deltaRij(); })
      .def("deltaTij", [](gtsam::PreintegratedAhrsMeasurements *self) { return self->deltaTij(); })
      .def("biasHat", [](gtsam::PreintegratedAhrsMeasurements *self) { return self->biasHat(); })
      .def("integrateMeasurement", [](gtsam::PreintegratedAhrsMeasurements *self, const gtsam::Vector &measuredOmega, double deltaT) { self->integrateMeasurement(measuredOmega, deltaT); }, nb::arg("measuredOmega"), nb::arg("deltaT"))
      .def("resetIntegration", [](gtsam::PreintegratedAhrsMeasurements *self) { self->resetIntegration(); });

  nb::class_<gtsam::AHRSFactor, gtsam::NonlinearFactor>(m_, "AHRSFactor")
      .def(nb::init<size_t, size_t, size_t, const gtsam::PreintegratedAhrsMeasurements &, const gtsam::Vector &>(), nb::arg("rot_i"), nb::arg("rot_j"), nb::arg("bias"), nb::arg("preintegratedMeasurements"), nb::arg("omegaCoriolis"))
      .def(nb::init<size_t, size_t, size_t, const gtsam::PreintegratedAhrsMeasurements &, const gtsam::Vector &, const gtsam::Pose3 &>(), nb::arg("rot_i"), nb::arg("rot_j"), nb::arg("bias"), nb::arg("preintegratedMeasurements"), nb::arg("omegaCoriolis"), nb::arg("body_P_sensor"))
      .def("preintegratedMeasurements", [](gtsam::AHRSFactor *self) { return self->preintegratedMeasurements(); })
      .def("evaluateError", [](gtsam::AHRSFactor *self, const gtsam::Rot3 &rot_i, const gtsam::Rot3 &rot_j, const gtsam::Vector &bias) { return self->evaluateError(rot_i, rot_j, bias); }, nb::arg("rot_i"), nb::arg("rot_j"), nb::arg("bias"))
      .def("predict", [](gtsam::AHRSFactor *self, const gtsam::Rot3 &rot_i, const gtsam::Vector &bias, const gtsam::PreintegratedAhrsMeasurements &preintegratedMeasurements, const gtsam::Vector &omegaCoriolis) { return self->predict(rot_i, bias, preintegratedMeasurements, omegaCoriolis); }, nb::arg("rot_i"), nb::arg("bias"), nb::arg("preintegratedMeasurements"), nb::arg("omegaCoriolis"));

  nb::class_<gtsam::Rot3AttitudeFactor, gtsam::NonlinearFactor>(m_, "Rot3AttitudeFactor")
      .def(nb::init<size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>, const gtsam::Unit3 &>(), nb::arg("key"), nb::arg("nZ"), nb::arg("model"), nb::arg("bRef"))
      .def(nb::init<size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("key"), nb::arg("nZ"), nb::arg("model"))
      .def(nb::init<>())
      .def("print", [](gtsam::Rot3AttitudeFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Rot3AttitudeFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::Rot3AttitudeFactor *self, const gtsam::NonlinearFactor &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("nZ", [](gtsam::Rot3AttitudeFactor *self) { return self->nZ(); })
      .def("bRef", [](gtsam::Rot3AttitudeFactor *self) { return self->bRef(); });

  nb::class_<gtsam::Pose3AttitudeFactor, gtsam::NonlinearFactor>(m_, "Pose3AttitudeFactor")
      .def(nb::init<size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>, const gtsam::Unit3 &>(), nb::arg("key"), nb::arg("nZ"), nb::arg("model"), nb::arg("bRef"))
      .def(nb::init<size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("key"), nb::arg("nZ"), nb::arg("model"))
      .def(nb::init<>())
      .def("print", [](gtsam::Pose3AttitudeFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Pose3AttitudeFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::Pose3AttitudeFactor *self, const gtsam::NonlinearFactor &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("nZ", [](gtsam::Pose3AttitudeFactor *self) { return self->nZ(); })
      .def("bRef", [](gtsam::Pose3AttitudeFactor *self) { return self->bRef(); });

  nb::class_<gtsam::GPSFactor, gtsam::NonlinearFactor>(m_, "GPSFactor")
      .def(nb::init<size_t, const gtsam::Point3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("gpsIn"), nb::arg("model"))
      .def("print", [](gtsam::GPSFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GPSFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GPSFactor *self, const gtsam::GPSFactor &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("measurementIn", [](gtsam::GPSFactor *self) { return self->measurementIn(); });

  nb::class_<gtsam::GPSFactor2, gtsam::NonlinearFactor>(m_, "GPSFactor2")
      .def(nb::init<size_t, const gtsam::Point3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("gpsIn"), nb::arg("model"))
      .def("print", [](gtsam::GPSFactor2 *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GPSFactor2 &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GPSFactor2 *self, const gtsam::GPSFactor2 &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("measurementIn", [](gtsam::GPSFactor2 *self) { return self->measurementIn(); });

  nb::class_<gtsam::Scenario>(m_, "Scenario")
      .def("pose", [](gtsam::Scenario *self, double t) { return self->pose(t); }, nb::arg("t"))
      .def("omega_b", [](gtsam::Scenario *self, double t) { return self->omega_b(t); }, nb::arg("t"))
      .def("velocity_n", [](gtsam::Scenario *self, double t) { return self->velocity_n(t); }, nb::arg("t"))
      .def("acceleration_n", [](gtsam::Scenario *self, double t) { return self->acceleration_n(t); }, nb::arg("t"))
      .def("rotation", [](gtsam::Scenario *self, double t) { return self->rotation(t); }, nb::arg("t"))
      .def("navState", [](gtsam::Scenario *self, double t) { return self->navState(t); }, nb::arg("t"))
      .def("velocity_b", [](gtsam::Scenario *self, double t) { return self->velocity_b(t); }, nb::arg("t"))
      .def("acceleration_b", [](gtsam::Scenario *self, double t) { return self->acceleration_b(t); }, nb::arg("t"));

  nb::class_<gtsam::ConstantTwistScenario, gtsam::Scenario>(m_, "ConstantTwistScenario")
      .def(nb::init<const gtsam::Vector &, const gtsam::Vector &>(), nb::arg("w"), nb::arg("v"))
      .def(nb::init<const gtsam::Vector &, const gtsam::Vector &, const gtsam::Pose3 &>(), nb::arg("w"), nb::arg("v"), nb::arg("nTb0"));

  nb::class_<gtsam::AcceleratingScenario, gtsam::Scenario>(m_, "AcceleratingScenario")
      .def(nb::init<const gtsam::Rot3 &, const gtsam::Point3 &, const gtsam::Vector &, const gtsam::Vector &, const gtsam::Vector &>(), nb::arg("nRb"), nb::arg("p0"), nb::arg("v0"), nb::arg("a_n"), nb::arg("omega_b"));

  nb::class_<gtsam::ScenarioRunner>(m_, "ScenarioRunner")
      .def(nb::init<const gtsam::Scenario &, const boost::shared_ptr<gtsam::PreintegrationParams>, double, const gtsam::imuBias::ConstantBias &>(), nb::arg("scenario"), nb::arg("p"), nb::arg("imuSampleTime"), nb::arg("bias"))
      .def("gravity_n", [](gtsam::ScenarioRunner *self) { return self->gravity_n(); })
      .def("actualAngularVelocity", [](gtsam::ScenarioRunner *self, double t) { return self->actualAngularVelocity(t); }, nb::arg("t"))
      .def("actualSpecificForce", [](gtsam::ScenarioRunner *self, double t) { return self->actualSpecificForce(t); }, nb::arg("t"))
      .def("measuredAngularVelocity", [](gtsam::ScenarioRunner *self, double t) { return self->measuredAngularVelocity(t); }, nb::arg("t"))
      .def("measuredSpecificForce", [](gtsam::ScenarioRunner *self, double t) { return self->measuredSpecificForce(t); }, nb::arg("t"))
      .def("imuSampleTime", [](gtsam::ScenarioRunner *self) { return self->imuSampleTime(); })
      .def("integrate", [](gtsam::ScenarioRunner *self, double T, const gtsam::imuBias::ConstantBias &estimatedBias, bool corrupted) { return self->integrate(T, estimatedBias, corrupted); }, nb::arg("T"), nb::arg("estimatedBias"), nb::arg("corrupted"))
      .def("predict", [](gtsam::ScenarioRunner *self, const gtsam::PreintegratedImuMeasurements &pim, const gtsam::imuBias::ConstantBias &estimatedBias) { return self->predict(pim, estimatedBias); }, nb::arg("pim"), nb::arg("estimatedBias"))
      .def("estimateCovariance", [](gtsam::ScenarioRunner *self, double T, size_t N, const gtsam::imuBias::ConstantBias &estimatedBias) { return self->estimateCovariance(T, N, estimatedBias); }, nb::arg("T"), nb::arg("N"), nb::arg("estimatedBias"))
      .def("estimateNoiseCovariance", [](gtsam::ScenarioRunner *self, size_t N) { return self->estimateNoiseCovariance(N); }, nb::arg("N"));
}
