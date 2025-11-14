/**
 * @file    navigation.cpp
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
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/NavState.h"
#include "gtsam/navigation/PreintegratedRotation.h"
#include "gtsam/navigation/PreintegrationParams.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/AHRSFactor.h"
#include "gtsam/navigation/AttitudeFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/Scenario.h"
#include "gtsam/navigation/ScenarioRunner.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::imuBias::ConstantBias)
BOOST_CLASS_EXPORT(gtsam::NavState)
BOOST_CLASS_EXPORT(gtsam::PreintegrationParams)
BOOST_CLASS_EXPORT(gtsam::PreintegratedImuMeasurements)


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/navigation.h"

using namespace std;

namespace py = pybind11;



void navigation(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of navigation";



    pybind11::module m_imuBias = m_.def_submodule("imuBias", "imuBias submodule");

    py::class_<gtsam::imuBias::ConstantBias, boost::shared_ptr<gtsam::imuBias::ConstantBias>>(m_imuBias, "ConstantBias")
        .def(py::init<>())
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&>(), py::arg("biasAcc"), py::arg("biasGyro"))
        .def("print",[](gtsam::imuBias::ConstantBias* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::imuBias::ConstantBias& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::imuBias::ConstantBias* self, const gtsam::imuBias::ConstantBias& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("vector",[](gtsam::imuBias::ConstantBias* self){return self->vector();})
        .def("accelerometer",[](gtsam::imuBias::ConstantBias* self){return self->accelerometer();})
        .def("gyroscope",[](gtsam::imuBias::ConstantBias* self){return self->gyroscope();})
        .def("correctAccelerometer",[](gtsam::imuBias::ConstantBias* self, const gtsam::Vector& measurement){return self->correctAccelerometer(measurement);}, py::arg("measurement"))
        .def("correctGyroscope",[](gtsam::imuBias::ConstantBias* self, const gtsam::Vector& measurement){return self->correctGyroscope(measurement);}, py::arg("measurement"))
        .def("serialize", [](gtsam::imuBias::ConstantBias* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::imuBias::ConstantBias* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::imuBias::ConstantBias &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::imuBias::ConstantBias obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Identity",[](){return gtsam::imuBias::ConstantBias::Identity();})
        .def(-py::self)
        .def(py::self + py::self)
        .def(py::self - py::self);

    py::class_<gtsam::NavState, boost::shared_ptr<gtsam::NavState>>(m_, "NavState")
        .def(py::init<>())
        .def(py::init<const gtsam::Rot3&, const gtsam::Point3&, const gtsam::Vector&>(), py::arg("R"), py::arg("t"), py::arg("v"))
        .def(py::init<const gtsam::Pose3&, const gtsam::Vector&>(), py::arg("pose"), py::arg("v"))
        .def("print",[](gtsam::NavState* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::NavState& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::NavState* self, const gtsam::NavState& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("attitude",[](gtsam::NavState* self){return self->attitude();})
        .def("position",[](gtsam::NavState* self){return self->position();})
        .def("velocity",[](gtsam::NavState* self){return self->velocity();})
        .def("pose",[](gtsam::NavState* self){return self->pose();})
        .def("retract",[](gtsam::NavState* self, const gtsam::Vector& x){return self->retract(x);}, py::arg("x"))
        .def("localCoordinates",[](gtsam::NavState* self, const gtsam::NavState& g){return self->localCoordinates(g);}, py::arg("g"))
        .def("serialize", [](gtsam::NavState* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NavState* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NavState &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NavState obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PreintegratedRotationParams, boost::shared_ptr<gtsam::PreintegratedRotationParams>>(m_, "PreintegratedRotationParams")
        .def(py::init<>())
        .def("print",[](gtsam::PreintegratedRotationParams* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::PreintegratedRotationParams& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::PreintegratedRotationParams* self, const gtsam::PreintegratedRotationParams& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("setGyroscopeCovariance",[](gtsam::PreintegratedRotationParams* self, const gtsam::Matrix& cov){ self->setGyroscopeCovariance(cov);}, py::arg("cov"))
        .def("setOmegaCoriolis",[](gtsam::PreintegratedRotationParams* self, const gtsam::Vector& omega){ self->setOmegaCoriolis(omega);}, py::arg("omega"))
        .def("setBodyPSensor",[](gtsam::PreintegratedRotationParams* self, const gtsam::Pose3& pose){ self->setBodyPSensor(pose);}, py::arg("pose"))
        .def("getGyroscopeCovariance",[](gtsam::PreintegratedRotationParams* self){return self->getGyroscopeCovariance();})
        .def("getOmegaCoriolis",[](gtsam::PreintegratedRotationParams* self){return self->getOmegaCoriolis();})
        .def("getBodyPSensor",[](gtsam::PreintegratedRotationParams* self){return self->getBodyPSensor();});

    py::class_<gtsam::PreintegrationParams, gtsam::PreintegratedRotationParams, boost::shared_ptr<gtsam::PreintegrationParams>>(m_, "PreintegrationParams")
        .def(py::init<const gtsam::Vector&>(), py::arg("n_gravity"))
        .def("print",[](gtsam::PreintegrationParams* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::PreintegrationParams& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::PreintegrationParams* self, const gtsam::PreintegrationParams& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("setAccelerometerCovariance",[](gtsam::PreintegrationParams* self, const gtsam::Matrix& cov){ self->setAccelerometerCovariance(cov);}, py::arg("cov"))
        .def("setIntegrationCovariance",[](gtsam::PreintegrationParams* self, const gtsam::Matrix& cov){ self->setIntegrationCovariance(cov);}, py::arg("cov"))
        .def("setUse2ndOrderCoriolis",[](gtsam::PreintegrationParams* self, bool flag){ self->setUse2ndOrderCoriolis(flag);}, py::arg("flag"))
        .def("getAccelerometerCovariance",[](gtsam::PreintegrationParams* self){return self->getAccelerometerCovariance();})
        .def("getIntegrationCovariance",[](gtsam::PreintegrationParams* self){return self->getIntegrationCovariance();})
        .def("getUse2ndOrderCoriolis",[](gtsam::PreintegrationParams* self){return self->getUse2ndOrderCoriolis();})
        .def("serialize", [](gtsam::PreintegrationParams* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PreintegrationParams* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PreintegrationParams &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PreintegrationParams obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("MakeSharedD",[](double g){return gtsam::PreintegrationParams::MakeSharedD(g);}, py::arg("g"))
        .def_static("MakeSharedU",[](double g){return gtsam::PreintegrationParams::MakeSharedU(g);}, py::arg("g"))
        .def_static("MakeSharedD",[](){return gtsam::PreintegrationParams::MakeSharedD();})
        .def_static("MakeSharedU",[](){return gtsam::PreintegrationParams::MakeSharedU();})
        .def_readwrite("n_gravity", &gtsam::PreintegrationParams::n_gravity);

    py::class_<gtsam::PreintegratedImuMeasurements, boost::shared_ptr<gtsam::PreintegratedImuMeasurements>>(m_, "PreintegratedImuMeasurements")
        .def(py::init<const boost::shared_ptr<gtsam::PreintegrationParams>>(), py::arg("params"))
        .def(py::init<const boost::shared_ptr<gtsam::PreintegrationParams>, const gtsam::imuBias::ConstantBias&>(), py::arg("params"), py::arg("bias"))
        .def("print",[](gtsam::PreintegratedImuMeasurements* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::PreintegratedImuMeasurements& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::PreintegratedImuMeasurements* self, const gtsam::PreintegratedImuMeasurements& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("integrateMeasurement",[](gtsam::PreintegratedImuMeasurements* self, const gtsam::Vector& measuredAcc, const gtsam::Vector& measuredOmega, double deltaT){ self->integrateMeasurement(measuredAcc, measuredOmega, deltaT);}, py::arg("measuredAcc"), py::arg("measuredOmega"), py::arg("deltaT"))
        .def("resetIntegration",[](gtsam::PreintegratedImuMeasurements* self){ self->resetIntegration();})
        .def("resetIntegrationAndSetBias",[](gtsam::PreintegratedImuMeasurements* self, const gtsam::imuBias::ConstantBias& biasHat){ self->resetIntegrationAndSetBias(biasHat);}, py::arg("biasHat"))
        .def("preintMeasCov",[](gtsam::PreintegratedImuMeasurements* self){return self->preintMeasCov();})
        .def("preintegrated",[](gtsam::PreintegratedImuMeasurements* self){return self->preintegrated();})
        .def("deltaTij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaTij();})
        .def("deltaRij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaRij();})
        .def("deltaPij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaPij();})
        .def("deltaVij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaVij();})
        .def("biasHat",[](gtsam::PreintegratedImuMeasurements* self){return self->biasHat();})
        .def("biasHatVector",[](gtsam::PreintegratedImuMeasurements* self){return self->biasHatVector();})
        .def("predict",[](gtsam::PreintegratedImuMeasurements* self, const gtsam::NavState& state_i, const gtsam::imuBias::ConstantBias& bias){return self->predict(state_i, bias);}, py::arg("state_i"), py::arg("bias"))
        .def("serialize", [](gtsam::PreintegratedImuMeasurements* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PreintegratedImuMeasurements* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PreintegratedImuMeasurements &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PreintegratedImuMeasurements obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::ImuFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::ImuFactor>>(m_, "ImuFactor")
        .def(py::init<size_t, size_t, size_t, size_t, size_t, const gtsam::PreintegratedImuMeasurements&>(), py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias"), py::arg("preintegratedMeasurements"))
        .def("preintegratedMeasurements",[](gtsam::ImuFactor* self){return self->preintegratedMeasurements();})
        .def("evaluateError",[](gtsam::ImuFactor* self, const gtsam::Pose3& pose_i, const gtsam::Vector& vel_i, const gtsam::Pose3& pose_j, const gtsam::Vector& vel_j, const gtsam::imuBias::ConstantBias& bias){return self->evaluateError(pose_i, vel_i, pose_j, vel_j, bias);}, py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias"));

    py::class_<gtsam::PreintegrationCombinedParams, gtsam::PreintegrationParams, boost::shared_ptr<gtsam::PreintegrationCombinedParams>>(m_, "PreintegrationCombinedParams")
        .def(py::init<const gtsam::Vector&>(), py::arg("n_gravity"))
        .def("print",[](gtsam::PreintegrationCombinedParams* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::PreintegrationCombinedParams& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::PreintegrationCombinedParams* self, const gtsam::PreintegrationCombinedParams& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("setBiasAccCovariance",[](gtsam::PreintegrationCombinedParams* self, const gtsam::Matrix& cov){ self->setBiasAccCovariance(cov);}, py::arg("cov"))
        .def("setBiasOmegaCovariance",[](gtsam::PreintegrationCombinedParams* self, const gtsam::Matrix& cov){ self->setBiasOmegaCovariance(cov);}, py::arg("cov"))
        .def("setBiasAccOmegaInit",[](gtsam::PreintegrationCombinedParams* self, const gtsam::Matrix& cov){ self->setBiasAccOmegaInit(cov);}, py::arg("cov"))
        .def("getBiasAccCovariance",[](gtsam::PreintegrationCombinedParams* self){return self->getBiasAccCovariance();})
        .def("getBiasOmegaCovariance",[](gtsam::PreintegrationCombinedParams* self){return self->getBiasOmegaCovariance();})
        .def("getBiasAccOmegaInit",[](gtsam::PreintegrationCombinedParams* self){return self->getBiasAccOmegaInit();})
        .def_static("MakeSharedD",[](double g){return gtsam::PreintegrationCombinedParams::MakeSharedD(g);}, py::arg("g"))
        .def_static("MakeSharedU",[](double g){return gtsam::PreintegrationCombinedParams::MakeSharedU(g);}, py::arg("g"))
        .def_static("MakeSharedD",[](){return gtsam::PreintegrationCombinedParams::MakeSharedD();})
        .def_static("MakeSharedU",[](){return gtsam::PreintegrationCombinedParams::MakeSharedU();});

    py::class_<gtsam::PreintegratedCombinedMeasurements, boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements>>(m_, "PreintegratedCombinedMeasurements")
        .def(py::init<const boost::shared_ptr<gtsam::PreintegrationCombinedParams>>(), py::arg("params"))
        .def(py::init<const boost::shared_ptr<gtsam::PreintegrationCombinedParams>, const gtsam::imuBias::ConstantBias&>(), py::arg("params"), py::arg("bias"))
        .def("print",[](gtsam::PreintegratedCombinedMeasurements* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "Preintegrated Measurements:")
        .def("__repr__",
                    [](const gtsam::PreintegratedCombinedMeasurements& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "Preintegrated Measurements:")
        .def("equals",[](gtsam::PreintegratedCombinedMeasurements* self, const gtsam::PreintegratedCombinedMeasurements& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("integrateMeasurement",[](gtsam::PreintegratedCombinedMeasurements* self, const gtsam::Vector& measuredAcc, const gtsam::Vector& measuredOmega, double deltaT){ self->integrateMeasurement(measuredAcc, measuredOmega, deltaT);}, py::arg("measuredAcc"), py::arg("measuredOmega"), py::arg("deltaT"))
        .def("resetIntegration",[](gtsam::PreintegratedCombinedMeasurements* self){ self->resetIntegration();})
        .def("resetIntegrationAndSetBias",[](gtsam::PreintegratedCombinedMeasurements* self, const gtsam::imuBias::ConstantBias& biasHat){ self->resetIntegrationAndSetBias(biasHat);}, py::arg("biasHat"))
        .def("preintMeasCov",[](gtsam::PreintegratedCombinedMeasurements* self){return self->preintMeasCov();})
        .def("deltaTij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaTij();})
        .def("deltaRij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaRij();})
        .def("deltaPij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaPij();})
        .def("deltaVij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaVij();})
        .def("biasHat",[](gtsam::PreintegratedCombinedMeasurements* self){return self->biasHat();})
        .def("biasHatVector",[](gtsam::PreintegratedCombinedMeasurements* self){return self->biasHatVector();})
        .def("predict",[](gtsam::PreintegratedCombinedMeasurements* self, const gtsam::NavState& state_i, const gtsam::imuBias::ConstantBias& bias){return self->predict(state_i, bias);}, py::arg("state_i"), py::arg("bias"));

    py::class_<gtsam::CombinedImuFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::CombinedImuFactor>>(m_, "CombinedImuFactor")
        .def(py::init<size_t, size_t, size_t, size_t, size_t, size_t, const gtsam::PreintegratedCombinedMeasurements&>(), py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias_i"), py::arg("bias_j"), py::arg("CombinedPreintegratedMeasurements"))
        .def("preintegratedMeasurements",[](gtsam::CombinedImuFactor* self){return self->preintegratedMeasurements();})
        .def("evaluateError",[](gtsam::CombinedImuFactor* self, const gtsam::Pose3& pose_i, const gtsam::Vector& vel_i, const gtsam::Pose3& pose_j, const gtsam::Vector& vel_j, const gtsam::imuBias::ConstantBias& bias_i, const gtsam::imuBias::ConstantBias& bias_j){return self->evaluateError(pose_i, vel_i, pose_j, vel_j, bias_i, bias_j);}, py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias_i"), py::arg("bias_j"));

    py::class_<gtsam::PreintegratedAhrsMeasurements, boost::shared_ptr<gtsam::PreintegratedAhrsMeasurements>>(m_, "PreintegratedAhrsMeasurements")
        .def(py::init<const boost::shared_ptr<gtsam::PreintegrationParams>, const gtsam::Vector&>(), py::arg("params"), py::arg("biasHat"))
        .def(py::init<const boost::shared_ptr<gtsam::PreintegrationParams>, const gtsam::Vector&, double, const gtsam::Rot3&, const gtsam::Matrix&, const gtsam::Matrix&>(), py::arg("p"), py::arg("bias_hat"), py::arg("deltaTij"), py::arg("deltaRij"), py::arg("delRdelBiasOmega"), py::arg("preint_meas_cov"))
        .def(py::init<const gtsam::PreintegratedAhrsMeasurements&>(), py::arg("rhs"))
        .def("print",[](gtsam::PreintegratedAhrsMeasurements* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "Preintegrated Measurements: ")
        .def("__repr__",
                    [](const gtsam::PreintegratedAhrsMeasurements& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "Preintegrated Measurements: ")
        .def("equals",[](gtsam::PreintegratedAhrsMeasurements* self, const gtsam::PreintegratedAhrsMeasurements& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("deltaRij",[](gtsam::PreintegratedAhrsMeasurements* self){return self->deltaRij();})
        .def("deltaTij",[](gtsam::PreintegratedAhrsMeasurements* self){return self->deltaTij();})
        .def("biasHat",[](gtsam::PreintegratedAhrsMeasurements* self){return self->biasHat();})
        .def("integrateMeasurement",[](gtsam::PreintegratedAhrsMeasurements* self, const gtsam::Vector& measuredOmega, double deltaT){ self->integrateMeasurement(measuredOmega, deltaT);}, py::arg("measuredOmega"), py::arg("deltaT"))
        .def("resetIntegration",[](gtsam::PreintegratedAhrsMeasurements* self){ self->resetIntegration();});

    py::class_<gtsam::AHRSFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::AHRSFactor>>(m_, "AHRSFactor")
        .def(py::init<size_t, size_t, size_t, const gtsam::PreintegratedAhrsMeasurements&, const gtsam::Vector&>(), py::arg("rot_i"), py::arg("rot_j"), py::arg("bias"), py::arg("preintegratedMeasurements"), py::arg("omegaCoriolis"))
        .def(py::init<size_t, size_t, size_t, const gtsam::PreintegratedAhrsMeasurements&, const gtsam::Vector&, const gtsam::Pose3&>(), py::arg("rot_i"), py::arg("rot_j"), py::arg("bias"), py::arg("preintegratedMeasurements"), py::arg("omegaCoriolis"), py::arg("body_P_sensor"))
        .def("preintegratedMeasurements",[](gtsam::AHRSFactor* self){return self->preintegratedMeasurements();})
        .def("evaluateError",[](gtsam::AHRSFactor* self, const gtsam::Rot3& rot_i, const gtsam::Rot3& rot_j, const gtsam::Vector& bias){return self->evaluateError(rot_i, rot_j, bias);}, py::arg("rot_i"), py::arg("rot_j"), py::arg("bias"))
        .def("predict",[](gtsam::AHRSFactor* self, const gtsam::Rot3& rot_i, const gtsam::Vector& bias, const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements, const gtsam::Vector& omegaCoriolis){return self->predict(rot_i, bias, preintegratedMeasurements, omegaCoriolis);}, py::arg("rot_i"), py::arg("bias"), py::arg("preintegratedMeasurements"), py::arg("omegaCoriolis"));

    py::class_<gtsam::Rot3AttitudeFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::Rot3AttitudeFactor>>(m_, "Rot3AttitudeFactor")
        .def(py::init<size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>, const gtsam::Unit3&>(), py::arg("key"), py::arg("nZ"), py::arg("model"), py::arg("bRef"))
        .def(py::init<size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("key"), py::arg("nZ"), py::arg("model"))
        .def(py::init<>())
        .def("print",[](gtsam::Rot3AttitudeFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::Rot3AttitudeFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::Rot3AttitudeFactor* self, const gtsam::NonlinearFactor& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("nZ",[](gtsam::Rot3AttitudeFactor* self){return self->nZ();})
        .def("bRef",[](gtsam::Rot3AttitudeFactor* self){return self->bRef();});

    py::class_<gtsam::Pose3AttitudeFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::Pose3AttitudeFactor>>(m_, "Pose3AttitudeFactor")
        .def(py::init<size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>, const gtsam::Unit3&>(), py::arg("key"), py::arg("nZ"), py::arg("model"), py::arg("bRef"))
        .def(py::init<size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("key"), py::arg("nZ"), py::arg("model"))
        .def(py::init<>())
        .def("print",[](gtsam::Pose3AttitudeFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::Pose3AttitudeFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::Pose3AttitudeFactor* self, const gtsam::NonlinearFactor& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("nZ",[](gtsam::Pose3AttitudeFactor* self){return self->nZ();})
        .def("bRef",[](gtsam::Pose3AttitudeFactor* self){return self->bRef();});

    py::class_<gtsam::GPSFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::GPSFactor>>(m_, "GPSFactor")
        .def(py::init<size_t, const gtsam::Point3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("gpsIn"), py::arg("model"))
        .def("print",[](gtsam::GPSFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GPSFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GPSFactor* self, const gtsam::GPSFactor& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("measurementIn",[](gtsam::GPSFactor* self){return self->measurementIn();});

    py::class_<gtsam::GPSFactor2, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::GPSFactor2>>(m_, "GPSFactor2")
        .def(py::init<size_t, const gtsam::Point3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("gpsIn"), py::arg("model"))
        .def("print",[](gtsam::GPSFactor2* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GPSFactor2& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GPSFactor2* self, const gtsam::GPSFactor2& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("measurementIn",[](gtsam::GPSFactor2* self){return self->measurementIn();});

    py::class_<gtsam::Scenario, boost::shared_ptr<gtsam::Scenario>>(m_, "Scenario")
        .def("pose",[](gtsam::Scenario* self, double t){return self->pose(t);}, py::arg("t"))
        .def("omega_b",[](gtsam::Scenario* self, double t){return self->omega_b(t);}, py::arg("t"))
        .def("velocity_n",[](gtsam::Scenario* self, double t){return self->velocity_n(t);}, py::arg("t"))
        .def("acceleration_n",[](gtsam::Scenario* self, double t){return self->acceleration_n(t);}, py::arg("t"))
        .def("rotation",[](gtsam::Scenario* self, double t){return self->rotation(t);}, py::arg("t"))
        .def("navState",[](gtsam::Scenario* self, double t){return self->navState(t);}, py::arg("t"))
        .def("velocity_b",[](gtsam::Scenario* self, double t){return self->velocity_b(t);}, py::arg("t"))
        .def("acceleration_b",[](gtsam::Scenario* self, double t){return self->acceleration_b(t);}, py::arg("t"));

    py::class_<gtsam::ConstantTwistScenario, gtsam::Scenario, boost::shared_ptr<gtsam::ConstantTwistScenario>>(m_, "ConstantTwistScenario")
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&>(), py::arg("w"), py::arg("v"))
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&, const gtsam::Pose3&>(), py::arg("w"), py::arg("v"), py::arg("nTb0"));

    py::class_<gtsam::AcceleratingScenario, gtsam::Scenario, boost::shared_ptr<gtsam::AcceleratingScenario>>(m_, "AcceleratingScenario")
        .def(py::init<const gtsam::Rot3&, const gtsam::Point3&, const gtsam::Vector&, const gtsam::Vector&, const gtsam::Vector&>(), py::arg("nRb"), py::arg("p0"), py::arg("v0"), py::arg("a_n"), py::arg("omega_b"));

    py::class_<gtsam::ScenarioRunner, boost::shared_ptr<gtsam::ScenarioRunner>>(m_, "ScenarioRunner")
        .def(py::init<const gtsam::Scenario&, const boost::shared_ptr<gtsam::PreintegrationParams>, double, const gtsam::imuBias::ConstantBias&>(), py::arg("scenario"), py::arg("p"), py::arg("imuSampleTime"), py::arg("bias"))
        .def("gravity_n",[](gtsam::ScenarioRunner* self){return self->gravity_n();})
        .def("actualAngularVelocity",[](gtsam::ScenarioRunner* self, double t){return self->actualAngularVelocity(t);}, py::arg("t"))
        .def("actualSpecificForce",[](gtsam::ScenarioRunner* self, double t){return self->actualSpecificForce(t);}, py::arg("t"))
        .def("measuredAngularVelocity",[](gtsam::ScenarioRunner* self, double t){return self->measuredAngularVelocity(t);}, py::arg("t"))
        .def("measuredSpecificForce",[](gtsam::ScenarioRunner* self, double t){return self->measuredSpecificForce(t);}, py::arg("t"))
        .def("imuSampleTime",[](gtsam::ScenarioRunner* self){return self->imuSampleTime();})
        .def("integrate",[](gtsam::ScenarioRunner* self, double T, const gtsam::imuBias::ConstantBias& estimatedBias, bool corrupted){return self->integrate(T, estimatedBias, corrupted);}, py::arg("T"), py::arg("estimatedBias"), py::arg("corrupted"))
        .def("predict",[](gtsam::ScenarioRunner* self, const gtsam::PreintegratedImuMeasurements& pim, const gtsam::imuBias::ConstantBias& estimatedBias){return self->predict(pim, estimatedBias);}, py::arg("pim"), py::arg("estimatedBias"))
        .def("estimateCovariance",[](gtsam::ScenarioRunner* self, double T, size_t N, const gtsam::imuBias::ConstantBias& estimatedBias){return self->estimateCovariance(T, N, estimatedBias);}, py::arg("T"), py::arg("N"), py::arg("estimatedBias"))
        .def("estimateNoiseCovariance",[](gtsam::ScenarioRunner* self, size_t N){return self->estimateNoiseCovariance(N);}, py::arg("N"));


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/navigation.h"

}

