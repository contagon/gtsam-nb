/**
 * @file    nonlinear.cpp
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
#include "gtsam/geometry/Cal3Bundler.h"
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam/geometry/Cal3Fisheye.h"
#include "gtsam/geometry/Cal3Unified.h"
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/EssentialMatrix.h"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/SO3.h"
#include "gtsam/geometry/SO4.h"
#include "gtsam/geometry/SOn.h"
#include "gtsam/geometry/StereoPoint2.h"
#include "gtsam/geometry/Unit3.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/NavState.h"
#include "gtsam/basis/ParameterMatrix.h"
#include "gtsam/nonlinear/GraphvizFormatting.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/LinearContainerFactor.h"
#include "gtsam/nonlinear/NonlinearOptimizerParams.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/GncParams.h"
#include "gtsam/nonlinear/NonlinearOptimizer.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/GncOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/NonlinearISAM.h"
#include "gtsam/nonlinear/PriorFactor.h"
#include "gtsam/nonlinear/NonlinearEquality.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::NonlinearFactorGraph)
BOOST_CLASS_EXPORT(gtsam::Values)
BOOST_CLASS_EXPORT(gtsam::LinearContainerFactor)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<double>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Vector>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Point2>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::StereoPoint2>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Point3>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Rot2>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::SO3>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::SO4>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::SOn>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Rot3>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Pose2>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Pose3>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Unit3>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Cal3_S2>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Cal3DS2>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Cal3Bundler>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Cal3Fisheye>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::Cal3Unified>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::CalibratedCamera>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>)
BOOST_CLASS_EXPORT(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Point2>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::StereoPoint2>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Point3>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Rot2>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::SO3>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::SO4>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::SOn>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Rot3>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Pose2>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Pose3>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::Cal3_S2>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::CalibratedCamera>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>>)
BOOST_CLASS_EXPORT(gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>)


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/nonlinear.h"

using namespace std;

namespace py = pybind11;



void nonlinear(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of nonlinear";




    py::class_<gtsam::GraphvizFormatting, gtsam::DotWriter, boost::shared_ptr<gtsam::GraphvizFormatting>> graphvizformatting(m_, "GraphvizFormatting");
    graphvizformatting
        .def(py::init<>())
        .def_readwrite("paperHorizontalAxis", &gtsam::GraphvizFormatting::paperHorizontalAxis)
        .def_readwrite("paperVerticalAxis", &gtsam::GraphvizFormatting::paperVerticalAxis)
        .def_readwrite("scale", &gtsam::GraphvizFormatting::scale)
        .def_readwrite("mergeSimilarFactors", &gtsam::GraphvizFormatting::mergeSimilarFactors);

    py::enum_<gtsam::GraphvizFormatting::Axis>(graphvizformatting, "Axis", py::arithmetic())
        .value("X", gtsam::GraphvizFormatting::Axis::X)
        .value("Y", gtsam::GraphvizFormatting::Axis::Y)
        .value("Z", gtsam::GraphvizFormatting::Axis::Z)
        .value("NEGX", gtsam::GraphvizFormatting::Axis::NEGX)
        .value("NEGY", gtsam::GraphvizFormatting::Axis::NEGY)
        .value("NEGZ", gtsam::GraphvizFormatting::Axis::NEGZ);


    py::class_<gtsam::NonlinearFactorGraph, boost::shared_ptr<gtsam::NonlinearFactorGraph>>(m_, "NonlinearFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::NonlinearFactorGraph&>(), py::arg("graph"))
        .def("print",[](gtsam::NonlinearFactorGraph* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "NonlinearFactorGraph: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::NonlinearFactorGraph& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "NonlinearFactorGraph: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::NonlinearFactorGraph* self, const gtsam::NonlinearFactorGraph& fg, double tol){return self->equals(fg, tol);}, py::arg("fg"), py::arg("tol"))
        .def("size",[](gtsam::NonlinearFactorGraph* self){return self->size();})
        .def("empty",[](gtsam::NonlinearFactorGraph* self){return self->empty();})
        .def("remove",[](gtsam::NonlinearFactorGraph* self, size_t i){ self->remove(i);}, py::arg("i"))
        .def("replace",[](gtsam::NonlinearFactorGraph* self, size_t i, boost::shared_ptr<gtsam::NonlinearFactor> factors){ self->replace(i, factors);}, py::arg("i"), py::arg("factors"))
        .def("resize",[](gtsam::NonlinearFactorGraph* self, size_t size){ self->resize(size);}, py::arg("size"))
        .def("nrFactors",[](gtsam::NonlinearFactorGraph* self){return self->nrFactors();})
        .def("at",[](gtsam::NonlinearFactorGraph* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("push_back",[](gtsam::NonlinearFactorGraph* self, const gtsam::NonlinearFactorGraph& factors){ self->push_back(factors);}, py::arg("factors"))
        .def("push_back",[](gtsam::NonlinearFactorGraph* self, boost::shared_ptr<gtsam::NonlinearFactor> factor){ self->push_back(factor);}, py::arg("factor"))
        .def("add",[](gtsam::NonlinearFactorGraph* self, boost::shared_ptr<gtsam::NonlinearFactor> factor){ self->add(factor);}, py::arg("factor"))
        .def("exists",[](gtsam::NonlinearFactorGraph* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::NonlinearFactorGraph* self){return self->keys();})
        .def("keyVector",[](gtsam::NonlinearFactorGraph* self){return self->keyVector();})
        .def("addPriorDouble",[](gtsam::NonlinearFactorGraph* self, size_t key, const double& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<double>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorVector",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Vector& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Vector>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPoint2",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Point2& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Point2>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorStereoPoint2",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::StereoPoint2& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::StereoPoint2>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPoint3",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Point3& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Point3>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorRot2",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Rot2& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Rot2>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorSO3",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::SO3& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::SO3>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorSO4",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::SO4& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::SO4>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorRot3",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Rot3& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Rot3>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPose2",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Pose2& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Pose2>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPose3",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Pose3& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Pose3>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorCal3_S2",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Cal3_S2& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Cal3_S2>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorCal3Fisheye",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Cal3Fisheye& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Cal3Fisheye>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorCal3Unified",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::Cal3Unified& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::Cal3Unified>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorCalibratedCamera",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::CalibratedCamera& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::CalibratedCamera>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPinholeCameraCal3_S2",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3_S2>& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::PinholeCamera<gtsam::Cal3_S2>>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPinholeCameraCal3Bundler",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPinholeCameraCal3Fisheye",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorPinholeCameraCal3Unified",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Unified>& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Unified>>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("addPriorConstantBias",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::imuBias::ConstantBias& prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::imuBias::ConstantBias>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("printErrors",[](gtsam::NonlinearFactorGraph* self, const gtsam::Values& values){ self->printErrors(values);}, py::arg("values"))
        .def("error",[](gtsam::NonlinearFactorGraph* self, const gtsam::Values& values){return self->error(values);}, py::arg("values"))
        .def("probPrime",[](gtsam::NonlinearFactorGraph* self, const gtsam::Values& values){return self->probPrime(values);}, py::arg("values"))
        .def("orderingCOLAMD",[](gtsam::NonlinearFactorGraph* self){return self->orderingCOLAMD();})
        .def("linearize",[](gtsam::NonlinearFactorGraph* self, const gtsam::Values& values){return self->linearize(values);}, py::arg("values"))
        .def("clone",[](gtsam::NonlinearFactorGraph* self){return self->clone();})
        .def("dot",[](gtsam::NonlinearFactorGraph* self, const gtsam::Values& values, const gtsam::KeyFormatter& keyFormatter, const gtsam::GraphvizFormatting& writer){return self->dot(values, keyFormatter, writer);}, py::arg("values"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::GraphvizFormatting())
        .def("saveGraph",[](gtsam::NonlinearFactorGraph* self, const string& s, const gtsam::Values& values, const gtsam::KeyFormatter& keyFormatter, const gtsam::GraphvizFormatting& writer){ self->saveGraph(s, values, keyFormatter, writer);}, py::arg("s"), py::arg("values"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::GraphvizFormatting())
        .def("serialize", [](gtsam::NonlinearFactorGraph* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearFactorGraph* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearFactorGraph &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearFactorGraph obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearFactor, gtsam::Factor, boost::shared_ptr<gtsam::NonlinearFactor>>(m_, "NonlinearFactor")
        .def("print",[](gtsam::NonlinearFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::NonlinearFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::NonlinearFactor* self, const gtsam::NonlinearFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("error",[](gtsam::NonlinearFactor* self, const gtsam::Values& c){return self->error(c);}, py::arg("c"))
        .def("dim",[](gtsam::NonlinearFactor* self){return self->dim();})
        .def("active",[](gtsam::NonlinearFactor* self, const gtsam::Values& c){return self->active(c);}, py::arg("c"))
        .def("linearize",[](gtsam::NonlinearFactor* self, const gtsam::Values& c){return self->linearize(c);}, py::arg("c"))
        .def("clone",[](gtsam::NonlinearFactor* self){return self->clone();})
        .def("rekey",[](gtsam::NonlinearFactor* self, const gtsam::KeyVector& newKeys){return self->rekey(newKeys);}, py::arg("newKeys"));

    py::class_<gtsam::NoiseModelFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::NoiseModelFactor>>(m_, "NoiseModelFactor")
        .def("equals",[](gtsam::NoiseModelFactor* self, const gtsam::NoiseModelFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("noiseModel",[](gtsam::NoiseModelFactor* self){return self->noiseModel();})
        .def("unwhitenedError",[](gtsam::NoiseModelFactor* self, const gtsam::Values& x){return self->unwhitenedError(x);}, py::arg("x"))
        .def("whitenedError",[](gtsam::NoiseModelFactor* self, const gtsam::Values& x){return self->whitenedError(x);}, py::arg("x"));

    py::class_<gtsam::Values, boost::shared_ptr<gtsam::Values>>(m_, "Values")
        .def(py::init<>())
        .def(py::init<const gtsam::Values&>(), py::arg("other"))
        .def("size",[](gtsam::Values* self){return self->size();})
        .def("empty",[](gtsam::Values* self){return self->empty();})
        .def("clear",[](gtsam::Values* self){ self->clear();})
        .def("dim",[](gtsam::Values* self){return self->dim();})
        .def("print",[](gtsam::Values* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::Values& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::Values* self, const gtsam::Values& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("insert",[](gtsam::Values* self, const gtsam::Values& values){ self->insert(values);}, py::arg("values"))
        .def("update",[](gtsam::Values* self, const gtsam::Values& values){ self->update(values);}, py::arg("values"))
        .def("insert_or_assign",[](gtsam::Values* self, const gtsam::Values& values){ self->insert_or_assign(values);}, py::arg("values"))
        .def("erase",[](gtsam::Values* self, size_t j){ self->erase(j);}, py::arg("j"))
        .def("swap",[](gtsam::Values* self, gtsam::Values& values){ self->swap(values);}, py::arg("values"))
        .def("exists",[](gtsam::Values* self, size_t j){return self->exists(j);}, py::arg("j"))
        .def("keys",[](gtsam::Values* self){return self->keys();})
        .def("zeroVectors",[](gtsam::Values* self){return self->zeroVectors();})
        .def("retract",[](gtsam::Values* self, const gtsam::VectorValues& delta){return self->retract(delta);}, py::arg("delta"))
        .def("localCoordinates",[](gtsam::Values* self, const gtsam::Values& cp){return self->localCoordinates(cp);}, py::arg("cp"))
        .def("serialize", [](gtsam::Values* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::Values* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::Values &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::Values obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("insert_vector",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->insert(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->insert(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("insert_matrix",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->insert(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->insert(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("insert_point2",[](gtsam::Values* self, size_t j, const gtsam::Point2& point2){ self->insert(j, point2);}, py::arg("j"), py::arg("point2"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Point2& point2){ self->insert(j, point2);}, py::arg("j"), py::arg("point2"))
        .def("insert_point3",[](gtsam::Values* self, size_t j, const gtsam::Point3& point3){ self->insert(j, point3);}, py::arg("j"), py::arg("point3"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Point3& point3){ self->insert(j, point3);}, py::arg("j"), py::arg("point3"))
        .def("insert_rot2",[](gtsam::Values* self, size_t j, const gtsam::Rot2& rot2){ self->insert(j, rot2);}, py::arg("j"), py::arg("rot2"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Rot2& rot2){ self->insert(j, rot2);}, py::arg("j"), py::arg("rot2"))
        .def("insert_pose2",[](gtsam::Values* self, size_t j, const gtsam::Pose2& pose2){ self->insert(j, pose2);}, py::arg("j"), py::arg("pose2"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Pose2& pose2){ self->insert(j, pose2);}, py::arg("j"), py::arg("pose2"))
        .def("insert_R",[](gtsam::Values* self, size_t j, const gtsam::SO3& R){ self->insert(j, R);}, py::arg("j"), py::arg("R"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::SO3& R){ self->insert(j, R);}, py::arg("j"), py::arg("R"))
        .def("insert_Q",[](gtsam::Values* self, size_t j, const gtsam::SO4& Q){ self->insert(j, Q);}, py::arg("j"), py::arg("Q"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::SO4& Q){ self->insert(j, Q);}, py::arg("j"), py::arg("Q"))
        .def("insert_P",[](gtsam::Values* self, size_t j, const gtsam::SOn& P){ self->insert(j, P);}, py::arg("j"), py::arg("P"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::SOn& P){ self->insert(j, P);}, py::arg("j"), py::arg("P"))
        .def("insert_rot3",[](gtsam::Values* self, size_t j, const gtsam::Rot3& rot3){ self->insert(j, rot3);}, py::arg("j"), py::arg("rot3"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Rot3& rot3){ self->insert(j, rot3);}, py::arg("j"), py::arg("rot3"))
        .def("insert_pose3",[](gtsam::Values* self, size_t j, const gtsam::Pose3& pose3){ self->insert(j, pose3);}, py::arg("j"), py::arg("pose3"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Pose3& pose3){ self->insert(j, pose3);}, py::arg("j"), py::arg("pose3"))
        .def("insert_unit3",[](gtsam::Values* self, size_t j, const gtsam::Unit3& unit3){ self->insert(j, unit3);}, py::arg("j"), py::arg("unit3"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Unit3& unit3){ self->insert(j, unit3);}, py::arg("j"), py::arg("unit3"))
        .def("insert_cal3_s2",[](gtsam::Values* self, size_t j, const gtsam::Cal3_S2& cal3_s2){ self->insert(j, cal3_s2);}, py::arg("j"), py::arg("cal3_s2"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Cal3_S2& cal3_s2){ self->insert(j, cal3_s2);}, py::arg("j"), py::arg("cal3_s2"))
        .def("insert_cal3ds2",[](gtsam::Values* self, size_t j, const gtsam::Cal3DS2& cal3ds2){ self->insert(j, cal3ds2);}, py::arg("j"), py::arg("cal3ds2"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Cal3DS2& cal3ds2){ self->insert(j, cal3ds2);}, py::arg("j"), py::arg("cal3ds2"))
        .def("insert_cal3bundler",[](gtsam::Values* self, size_t j, const gtsam::Cal3Bundler& cal3bundler){ self->insert(j, cal3bundler);}, py::arg("j"), py::arg("cal3bundler"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Cal3Bundler& cal3bundler){ self->insert(j, cal3bundler);}, py::arg("j"), py::arg("cal3bundler"))
        .def("insert_cal3fisheye",[](gtsam::Values* self, size_t j, const gtsam::Cal3Fisheye& cal3fisheye){ self->insert(j, cal3fisheye);}, py::arg("j"), py::arg("cal3fisheye"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Cal3Fisheye& cal3fisheye){ self->insert(j, cal3fisheye);}, py::arg("j"), py::arg("cal3fisheye"))
        .def("insert_cal3unified",[](gtsam::Values* self, size_t j, const gtsam::Cal3Unified& cal3unified){ self->insert(j, cal3unified);}, py::arg("j"), py::arg("cal3unified"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Cal3Unified& cal3unified){ self->insert(j, cal3unified);}, py::arg("j"), py::arg("cal3unified"))
        .def("insert_essential_matrix",[](gtsam::Values* self, size_t j, const gtsam::EssentialMatrix& essential_matrix){ self->insert(j, essential_matrix);}, py::arg("j"), py::arg("essential_matrix"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::EssentialMatrix& essential_matrix){ self->insert(j, essential_matrix);}, py::arg("j"), py::arg("essential_matrix"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_camera",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera){ self->insert(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_constant_bias",[](gtsam::Values* self, size_t j, const gtsam::imuBias::ConstantBias& constant_bias){ self->insert(j, constant_bias);}, py::arg("j"), py::arg("constant_bias"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::imuBias::ConstantBias& constant_bias){ self->insert(j, constant_bias);}, py::arg("j"), py::arg("constant_bias"))
        .def("insert_nav_state",[](gtsam::Values* self, size_t j, const gtsam::NavState& nav_state){ self->insert(j, nav_state);}, py::arg("j"), py::arg("nav_state"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::NavState& nav_state){ self->insert(j, nav_state);}, py::arg("j"), py::arg("nav_state"))
        .def("insert_c",[](gtsam::Values* self, size_t j, double c){ self->insert(j, c);}, py::arg("j"), py::arg("c"))
        .def("insert",[](gtsam::Values* self, size_t j, double c){ self->insert(j, c);}, py::arg("j"), py::arg("c"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<1>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<1>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<2>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<2>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<3>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<3>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<4>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<4>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<5>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<5>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<6>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<6>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<7>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<7>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<8>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<8>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<9>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<9>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<10>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<10>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<11>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<11>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<12>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<12>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<13>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<13>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<14>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<14>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_X",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<15>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<15>& X){ self->insert(j, X);}, py::arg("j"), py::arg("X"))
        .def("insertPoint2",[](gtsam::Values* self, size_t j, const gtsam::Point2& val){ self->insert<gtsam::Point2>(j, val);}, py::arg("j"), py::arg("val"))
        .def("insertPoint3",[](gtsam::Values* self, size_t j, const gtsam::Point3& val){ self->insert<gtsam::Point3>(j, val);}, py::arg("j"), py::arg("val"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Point2& point2){ self->update(j, point2);}, py::arg("j"), py::arg("point2"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Point3& point3){ self->update(j, point3);}, py::arg("j"), py::arg("point3"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Rot2& rot2){ self->update(j, rot2);}, py::arg("j"), py::arg("rot2"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Pose2& pose2){ self->update(j, pose2);}, py::arg("j"), py::arg("pose2"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::SO3& R){ self->update(j, R);}, py::arg("j"), py::arg("R"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::SO4& Q){ self->update(j, Q);}, py::arg("j"), py::arg("Q"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::SOn& P){ self->update(j, P);}, py::arg("j"), py::arg("P"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Rot3& rot3){ self->update(j, rot3);}, py::arg("j"), py::arg("rot3"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Pose3& pose3){ self->update(j, pose3);}, py::arg("j"), py::arg("pose3"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Unit3& unit3){ self->update(j, unit3);}, py::arg("j"), py::arg("unit3"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Cal3_S2& cal3_s2){ self->update(j, cal3_s2);}, py::arg("j"), py::arg("cal3_s2"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Cal3DS2& cal3ds2){ self->update(j, cal3ds2);}, py::arg("j"), py::arg("cal3ds2"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Cal3Bundler& cal3bundler){ self->update(j, cal3bundler);}, py::arg("j"), py::arg("cal3bundler"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Cal3Fisheye& cal3fisheye){ self->update(j, cal3fisheye);}, py::arg("j"), py::arg("cal3fisheye"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Cal3Unified& cal3unified){ self->update(j, cal3unified);}, py::arg("j"), py::arg("cal3unified"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::EssentialMatrix& essential_matrix){ self->update(j, essential_matrix);}, py::arg("j"), py::arg("essential_matrix"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera){ self->update(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::imuBias::ConstantBias& constant_bias){ self->update(j, constant_bias);}, py::arg("j"), py::arg("constant_bias"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::NavState& nav_state){ self->update(j, nav_state);}, py::arg("j"), py::arg("nav_state"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->update(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->update(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("update",[](gtsam::Values* self, size_t j, double c){ self->update(j, c);}, py::arg("j"), py::arg("c"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<1>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<2>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<3>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<4>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<5>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<6>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<7>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<8>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<9>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<10>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<11>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<12>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<13>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<14>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("update",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<15>& X){ self->update(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Point2& point2){ self->insert_or_assign(j, point2);}, py::arg("j"), py::arg("point2"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Point3& point3){ self->insert_or_assign(j, point3);}, py::arg("j"), py::arg("point3"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Rot2& rot2){ self->insert_or_assign(j, rot2);}, py::arg("j"), py::arg("rot2"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Pose2& pose2){ self->insert_or_assign(j, pose2);}, py::arg("j"), py::arg("pose2"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::SO3& R){ self->insert_or_assign(j, R);}, py::arg("j"), py::arg("R"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::SO4& Q){ self->insert_or_assign(j, Q);}, py::arg("j"), py::arg("Q"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::SOn& P){ self->insert_or_assign(j, P);}, py::arg("j"), py::arg("P"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Rot3& rot3){ self->insert_or_assign(j, rot3);}, py::arg("j"), py::arg("rot3"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Pose3& pose3){ self->insert_or_assign(j, pose3);}, py::arg("j"), py::arg("pose3"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Unit3& unit3){ self->insert_or_assign(j, unit3);}, py::arg("j"), py::arg("unit3"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Cal3_S2& cal3_s2){ self->insert_or_assign(j, cal3_s2);}, py::arg("j"), py::arg("cal3_s2"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Cal3DS2& cal3ds2){ self->insert_or_assign(j, cal3ds2);}, py::arg("j"), py::arg("cal3ds2"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Cal3Bundler& cal3bundler){ self->insert_or_assign(j, cal3bundler);}, py::arg("j"), py::arg("cal3bundler"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Cal3Fisheye& cal3fisheye){ self->insert_or_assign(j, cal3fisheye);}, py::arg("j"), py::arg("cal3fisheye"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Cal3Unified& cal3unified){ self->insert_or_assign(j, cal3unified);}, py::arg("j"), py::arg("cal3unified"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::EssentialMatrix& essential_matrix){ self->insert_or_assign(j, essential_matrix);}, py::arg("j"), py::arg("essential_matrix"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera){ self->insert_or_assign(j, camera);}, py::arg("j"), py::arg("camera"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::imuBias::ConstantBias& constant_bias){ self->insert_or_assign(j, constant_bias);}, py::arg("j"), py::arg("constant_bias"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::NavState& nav_state){ self->insert_or_assign(j, nav_state);}, py::arg("j"), py::arg("nav_state"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->insert_or_assign(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->insert_or_assign(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, double c){ self->insert_or_assign(j, c);}, py::arg("j"), py::arg("c"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<1>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<2>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<3>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<4>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<5>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<6>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<7>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<8>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<9>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<10>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<11>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<12>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<13>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<14>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("insert_or_assign",[](gtsam::Values* self, size_t j, const gtsam::ParameterMatrix<15>& X){ self->insert_or_assign(j, X);}, py::arg("j"), py::arg("X"))
        .def("atPoint2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Point2>(j);}, py::arg("j"))
        .def("atPoint3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Point3>(j);}, py::arg("j"))
        .def("atRot2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Rot2>(j);}, py::arg("j"))
        .def("atPose2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Pose2>(j);}, py::arg("j"))
        .def("atSO3",[](gtsam::Values* self, size_t j){return self->at<gtsam::SO3>(j);}, py::arg("j"))
        .def("atSO4",[](gtsam::Values* self, size_t j){return self->at<gtsam::SO4>(j);}, py::arg("j"))
        .def("atSOn",[](gtsam::Values* self, size_t j){return self->at<gtsam::SOn>(j);}, py::arg("j"))
        .def("atRot3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Rot3>(j);}, py::arg("j"))
        .def("atPose3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Pose3>(j);}, py::arg("j"))
        .def("atUnit3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Unit3>(j);}, py::arg("j"))
        .def("atCal3_S2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3_S2>(j);}, py::arg("j"))
        .def("atCal3DS2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3DS2>(j);}, py::arg("j"))
        .def("atCal3Bundler",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3Bundler>(j);}, py::arg("j"))
        .def("atCal3Fisheye",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3Fisheye>(j);}, py::arg("j"))
        .def("atCal3Unified",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3Unified>(j);}, py::arg("j"))
        .def("atEssentialMatrix",[](gtsam::Values* self, size_t j){return self->at<gtsam::EssentialMatrix>(j);}, py::arg("j"))
        .def("atPinholeCameraCal3_S2",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholeCamera<gtsam::Cal3_S2>>(j);}, py::arg("j"))
        .def("atPinholeCameraCal3Bundler",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(j);}, py::arg("j"))
        .def("atPinholeCameraCal3Fisheye",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(j);}, py::arg("j"))
        .def("atPinholeCameraCal3Unified",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholeCamera<gtsam::Cal3Unified>>(j);}, py::arg("j"))
        .def("atPinholePoseCal3_S2",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholePose<gtsam::Cal3_S2>>(j);}, py::arg("j"))
        .def("atPinholePoseCal3Bundler",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholePose<gtsam::Cal3Bundler>>(j);}, py::arg("j"))
        .def("atPinholePoseCal3Fisheye",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholePose<gtsam::Cal3Fisheye>>(j);}, py::arg("j"))
        .def("atPinholePoseCal3Unified",[](gtsam::Values* self, size_t j){return self->at<gtsam::PinholePose<gtsam::Cal3Unified>>(j);}, py::arg("j"))
        .def("atConstantBias",[](gtsam::Values* self, size_t j){return self->at<gtsam::imuBias::ConstantBias>(j);}, py::arg("j"))
        .def("atNavState",[](gtsam::Values* self, size_t j){return self->at<gtsam::NavState>(j);}, py::arg("j"))
        .def("atVector",[](gtsam::Values* self, size_t j){return self->at<gtsam::Vector>(j);}, py::arg("j"))
        .def("atMatrix",[](gtsam::Values* self, size_t j){return self->at<gtsam::Matrix>(j);}, py::arg("j"))
        .def("atDouble",[](gtsam::Values* self, size_t j){return self->at<double>(j);}, py::arg("j"))
        .def("atParameterMatrix1",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<1>>(j);}, py::arg("j"))
        .def("atParameterMatrix2",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<2>>(j);}, py::arg("j"))
        .def("atParameterMatrix3",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<3>>(j);}, py::arg("j"))
        .def("atParameterMatrix4",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<4>>(j);}, py::arg("j"))
        .def("atParameterMatrix5",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<5>>(j);}, py::arg("j"))
        .def("atParameterMatrix6",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<6>>(j);}, py::arg("j"))
        .def("atParameterMatrix7",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<7>>(j);}, py::arg("j"))
        .def("atParameterMatrix8",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<8>>(j);}, py::arg("j"))
        .def("atParameterMatrix9",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<9>>(j);}, py::arg("j"))
        .def("atParameterMatrix10",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<10>>(j);}, py::arg("j"))
        .def("atParameterMatrix11",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<11>>(j);}, py::arg("j"))
        .def("atParameterMatrix12",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<12>>(j);}, py::arg("j"))
        .def("atParameterMatrix13",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<13>>(j);}, py::arg("j"))
        .def("atParameterMatrix14",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<14>>(j);}, py::arg("j"))
        .def("atParameterMatrix15",[](gtsam::Values* self, size_t j){return self->at<gtsam::ParameterMatrix<15>>(j);}, py::arg("j"));

    py::class_<gtsam::Marginals, boost::shared_ptr<gtsam::Marginals>>(m_, "Marginals")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("solution"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::Values&>(), py::arg("gfgraph"), py::arg("solution"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::VectorValues&>(), py::arg("gfgraph"), py::arg("solutionvec"))
        .def("print",[](gtsam::Marginals* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "Marginals: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::Marginals& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "Marginals: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("marginalCovariance",[](gtsam::Marginals* self, size_t variable){return self->marginalCovariance(variable);}, py::arg("variable"))
        .def("marginalInformation",[](gtsam::Marginals* self, size_t variable){return self->marginalInformation(variable);}, py::arg("variable"))
        .def("jointMarginalCovariance",[](gtsam::Marginals* self, const gtsam::KeyVector& variables){return self->jointMarginalCovariance(variables);}, py::arg("variables"))
        .def("jointMarginalInformation",[](gtsam::Marginals* self, const gtsam::KeyVector& variables){return self->jointMarginalInformation(variables);}, py::arg("variables"));

    py::class_<gtsam::JointMarginal, boost::shared_ptr<gtsam::JointMarginal>>(m_, "JointMarginal")
        .def("at",[](gtsam::JointMarginal* self, size_t iVariable, size_t jVariable){return self->at(iVariable, jVariable);}, py::arg("iVariable"), py::arg("jVariable"))
        .def("fullMatrix",[](gtsam::JointMarginal* self){return self->fullMatrix();})
        .def("print",[](gtsam::JointMarginal* self, string s, gtsam::KeyFormatter keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::JointMarginal& self, string s, gtsam::KeyFormatter keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

    py::class_<gtsam::LinearContainerFactor, gtsam::NonlinearFactor, boost::shared_ptr<gtsam::LinearContainerFactor>>(m_, "LinearContainerFactor")
        .def(py::init<boost::shared_ptr<gtsam::GaussianFactor>, const gtsam::Values&>(), py::arg("factor"), py::arg("linearizationPoint"))
        .def(py::init<boost::shared_ptr<gtsam::GaussianFactor>>(), py::arg("factor"))
        .def("factor",[](gtsam::LinearContainerFactor* self){return self->factor();})
        .def("isJacobian",[](gtsam::LinearContainerFactor* self){return self->isJacobian();})
        .def("toJacobian",[](gtsam::LinearContainerFactor* self){return self->toJacobian();})
        .def("toHessian",[](gtsam::LinearContainerFactor* self){return self->toHessian();})
        .def("serialize", [](gtsam::LinearContainerFactor* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::LinearContainerFactor* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::LinearContainerFactor &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::LinearContainerFactor obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("ConvertLinearGraph",[](const gtsam::GaussianFactorGraph& linear_graph, const gtsam::Values& linearizationPoint){return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph, linearizationPoint);}, py::arg("linear_graph"), py::arg("linearizationPoint"))
        .def_static("ConvertLinearGraph",[](const gtsam::GaussianFactorGraph& linear_graph){return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph);}, py::arg("linear_graph"));

    py::class_<gtsam::NonlinearOptimizerParams, boost::shared_ptr<gtsam::NonlinearOptimizerParams>>(m_, "NonlinearOptimizerParams")
        .def(py::init<>())
        .def("print",[](gtsam::NonlinearOptimizerParams* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::NonlinearOptimizerParams& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("getMaxIterations",[](gtsam::NonlinearOptimizerParams* self){return self->getMaxIterations();})
        .def("getRelativeErrorTol",[](gtsam::NonlinearOptimizerParams* self){return self->getRelativeErrorTol();})
        .def("getAbsoluteErrorTol",[](gtsam::NonlinearOptimizerParams* self){return self->getAbsoluteErrorTol();})
        .def("getErrorTol",[](gtsam::NonlinearOptimizerParams* self){return self->getErrorTol();})
        .def("getVerbosity",[](gtsam::NonlinearOptimizerParams* self){return self->getVerbosity();})
        .def("setMaxIterations",[](gtsam::NonlinearOptimizerParams* self, int value){ self->setMaxIterations(value);}, py::arg("value"))
        .def("setRelativeErrorTol",[](gtsam::NonlinearOptimizerParams* self, double value){ self->setRelativeErrorTol(value);}, py::arg("value"))
        .def("setAbsoluteErrorTol",[](gtsam::NonlinearOptimizerParams* self, double value){ self->setAbsoluteErrorTol(value);}, py::arg("value"))
        .def("setErrorTol",[](gtsam::NonlinearOptimizerParams* self, double value){ self->setErrorTol(value);}, py::arg("value"))
        .def("setVerbosity",[](gtsam::NonlinearOptimizerParams* self, string s){ self->setVerbosity(s);}, py::arg("s"))
        .def("getLinearSolverType",[](gtsam::NonlinearOptimizerParams* self){return self->getLinearSolverType();})
        .def("setLinearSolverType",[](gtsam::NonlinearOptimizerParams* self, string solver){ self->setLinearSolverType(solver);}, py::arg("solver"))
        .def("setIterativeParams",[](gtsam::NonlinearOptimizerParams* self, boost::shared_ptr<gtsam::IterativeOptimizationParameters> params){ self->setIterativeParams(params);}, py::arg("params"))
        .def("setOrdering",[](gtsam::NonlinearOptimizerParams* self, const gtsam::Ordering& ordering){ self->setOrdering(ordering);}, py::arg("ordering"))
        .def("getOrderingType",[](gtsam::NonlinearOptimizerParams* self){return self->getOrderingType();})
        .def("setOrderingType",[](gtsam::NonlinearOptimizerParams* self, string ordering){ self->setOrderingType(ordering);}, py::arg("ordering"))
        .def("isMultifrontal",[](gtsam::NonlinearOptimizerParams* self){return self->isMultifrontal();})
        .def("isSequential",[](gtsam::NonlinearOptimizerParams* self){return self->isSequential();})
        .def("isCholmod",[](gtsam::NonlinearOptimizerParams* self){return self->isCholmod();})
        .def("isIterative",[](gtsam::NonlinearOptimizerParams* self){return self->isIterative();})
        .def_readwrite("iterationHook", &gtsam::NonlinearOptimizerParams::iterationHook);

    py::class_<gtsam::GaussNewtonParams, gtsam::NonlinearOptimizerParams, boost::shared_ptr<gtsam::GaussNewtonParams>>(m_, "GaussNewtonParams")
        .def(py::init<>());

    py::class_<gtsam::LevenbergMarquardtParams, gtsam::NonlinearOptimizerParams, boost::shared_ptr<gtsam::LevenbergMarquardtParams>>(m_, "LevenbergMarquardtParams")
        .def(py::init<>())
        .def("getDiagonalDamping",[](gtsam::LevenbergMarquardtParams* self){return self->getDiagonalDamping();})
        .def("getlambdaFactor",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaFactor();})
        .def("getlambdaInitial",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaInitial();})
        .def("getlambdaLowerBound",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaLowerBound();})
        .def("getlambdaUpperBound",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaUpperBound();})
        .def("getUseFixedLambdaFactor",[](gtsam::LevenbergMarquardtParams* self){return self->getUseFixedLambdaFactor();})
        .def("getLogFile",[](gtsam::LevenbergMarquardtParams* self){return self->getLogFile();})
        .def("getVerbosityLM",[](gtsam::LevenbergMarquardtParams* self){return self->getVerbosityLM();})
        .def("setDiagonalDamping",[](gtsam::LevenbergMarquardtParams* self, bool flag){ self->setDiagonalDamping(flag);}, py::arg("flag"))
        .def("setlambdaFactor",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaFactor(value);}, py::arg("value"))
        .def("setlambdaInitial",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaInitial(value);}, py::arg("value"))
        .def("setlambdaLowerBound",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaLowerBound(value);}, py::arg("value"))
        .def("setlambdaUpperBound",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaUpperBound(value);}, py::arg("value"))
        .def("setUseFixedLambdaFactor",[](gtsam::LevenbergMarquardtParams* self, bool flag){ self->setUseFixedLambdaFactor(flag);}, py::arg("flag"))
        .def("setLogFile",[](gtsam::LevenbergMarquardtParams* self, string s){ self->setLogFile(s);}, py::arg("s"))
        .def("setVerbosityLM",[](gtsam::LevenbergMarquardtParams* self, string s){ self->setVerbosityLM(s);}, py::arg("s"))
        .def_static("LegacyDefaults",[](){return gtsam::LevenbergMarquardtParams::LegacyDefaults();})
        .def_static("CeresDefaults",[](){return gtsam::LevenbergMarquardtParams::CeresDefaults();})
        .def_static("EnsureHasOrdering",[](gtsam::LevenbergMarquardtParams params, const gtsam::NonlinearFactorGraph& graph){return gtsam::LevenbergMarquardtParams::EnsureHasOrdering(params, graph);}, py::arg("params"), py::arg("graph"))
        .def_static("ReplaceOrdering",[](gtsam::LevenbergMarquardtParams params, const gtsam::Ordering& ordering){return gtsam::LevenbergMarquardtParams::ReplaceOrdering(params, ordering);}, py::arg("params"), py::arg("ordering"));

    py::class_<gtsam::DoglegParams, gtsam::NonlinearOptimizerParams, boost::shared_ptr<gtsam::DoglegParams>>(m_, "DoglegParams")
        .def(py::init<>())
        .def("getDeltaInitial",[](gtsam::DoglegParams* self){return self->getDeltaInitial();})
        .def("getVerbosityDL",[](gtsam::DoglegParams* self){return self->getVerbosityDL();})
        .def("setDeltaInitial",[](gtsam::DoglegParams* self, double deltaInitial){ self->setDeltaInitial(deltaInitial);}, py::arg("deltaInitial"))
        .def("setVerbosityDL",[](gtsam::DoglegParams* self, string verbosityDL){ self->setVerbosityDL(verbosityDL);}, py::arg("verbosityDL"));
    py::enum_<gtsam::GncLossType>(m_, "GncLossType", py::arithmetic())
        .value("GM", gtsam::GncLossType::GM)
        .value("TLS", gtsam::GncLossType::TLS);


    py::class_<gtsam::NonlinearOptimizer, boost::shared_ptr<gtsam::NonlinearOptimizer>>(m_, "NonlinearOptimizer")
        .def("optimize",[](gtsam::NonlinearOptimizer* self){return self->optimize();})
        .def("optimizeSafely",[](gtsam::NonlinearOptimizer* self){return self->optimizeSafely();})
        .def("error",[](gtsam::NonlinearOptimizer* self){return self->error();})
        .def("iterations",[](gtsam::NonlinearOptimizer* self){return self->iterations();})
        .def("values",[](gtsam::NonlinearOptimizer* self){return self->values();})
        .def("graph",[](gtsam::NonlinearOptimizer* self){return self->graph();})
        .def("iterate",[](gtsam::NonlinearOptimizer* self){return self->iterate();});

    py::class_<gtsam::GaussNewtonOptimizer, gtsam::NonlinearOptimizer, boost::shared_ptr<gtsam::GaussNewtonOptimizer>>(m_, "GaussNewtonOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("initialValues"))
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::GaussNewtonParams&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"));

    py::class_<gtsam::DoglegOptimizer, gtsam::NonlinearOptimizer, boost::shared_ptr<gtsam::DoglegOptimizer>>(m_, "DoglegOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("initialValues"))
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::DoglegParams&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"))
        .def("getDelta",[](gtsam::DoglegOptimizer* self){return self->getDelta();});

    py::class_<gtsam::LevenbergMarquardtOptimizer, gtsam::NonlinearOptimizer, boost::shared_ptr<gtsam::LevenbergMarquardtOptimizer>>(m_, "LevenbergMarquardtOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("initialValues"))
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::LevenbergMarquardtParams&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"))
        .def("lambda_",[](gtsam::LevenbergMarquardtOptimizer* self){return self->lambda();})
        .def("print",[](gtsam::LevenbergMarquardtOptimizer* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::LevenbergMarquardtOptimizer& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ISAM2GaussNewtonParams, boost::shared_ptr<gtsam::ISAM2GaussNewtonParams>>(m_, "ISAM2GaussNewtonParams")
        .def(py::init<>())
        .def("print",[](gtsam::ISAM2GaussNewtonParams* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ISAM2GaussNewtonParams& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("getWildfireThreshold",[](gtsam::ISAM2GaussNewtonParams* self){return self->getWildfireThreshold();})
        .def("setWildfireThreshold",[](gtsam::ISAM2GaussNewtonParams* self, double wildfireThreshold){ self->setWildfireThreshold(wildfireThreshold);}, py::arg("wildfireThreshold"));

    py::class_<gtsam::ISAM2DoglegParams, boost::shared_ptr<gtsam::ISAM2DoglegParams>>(m_, "ISAM2DoglegParams")
        .def(py::init<>())
        .def("print",[](gtsam::ISAM2DoglegParams* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ISAM2DoglegParams& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("getWildfireThreshold",[](gtsam::ISAM2DoglegParams* self){return self->getWildfireThreshold();})
        .def("setWildfireThreshold",[](gtsam::ISAM2DoglegParams* self, double wildfireThreshold){ self->setWildfireThreshold(wildfireThreshold);}, py::arg("wildfireThreshold"))
        .def("getInitialDelta",[](gtsam::ISAM2DoglegParams* self){return self->getInitialDelta();})
        .def("setInitialDelta",[](gtsam::ISAM2DoglegParams* self, double initialDelta){ self->setInitialDelta(initialDelta);}, py::arg("initialDelta"))
        .def("getAdaptationMode",[](gtsam::ISAM2DoglegParams* self){return self->getAdaptationMode();})
        .def("setAdaptationMode",[](gtsam::ISAM2DoglegParams* self, string adaptationMode){ self->setAdaptationMode(adaptationMode);}, py::arg("adaptationMode"))
        .def("isVerbose",[](gtsam::ISAM2DoglegParams* self){return self->isVerbose();})
        .def("setVerbose",[](gtsam::ISAM2DoglegParams* self, bool verbose){ self->setVerbose(verbose);}, py::arg("verbose"));

    py::class_<gtsam::ISAM2ThresholdMap, boost::shared_ptr<gtsam::ISAM2ThresholdMap>>(m_, "ISAM2ThresholdMap")
        .def(py::init<>())
        .def(py::init<const gtsam::ISAM2ThresholdMap&>(), py::arg("other"))
        .def("size",[](gtsam::ISAM2ThresholdMap* self){return self->size();})
        .def("empty",[](gtsam::ISAM2ThresholdMap* self){return self->empty();})
        .def("clear",[](gtsam::ISAM2ThresholdMap* self){ self->clear();})
        .def("insert",[](gtsam::ISAM2ThresholdMap* self, const gtsam::ISAM2ThresholdMapValue& value){ self->insert(value);}, py::arg("value"));

    py::class_<gtsam::ISAM2Params, boost::shared_ptr<gtsam::ISAM2Params>> isam2params(m_, "ISAM2Params");
    isam2params
        .def(py::init<>())
        .def("print",[](gtsam::ISAM2Params* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ISAM2Params& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("setOptimizationParams",[](gtsam::ISAM2Params* self, const gtsam::ISAM2GaussNewtonParams& gauss_newton__params){ self->setOptimizationParams(gauss_newton__params);}, py::arg("gauss_newton__params"))
        .def("setOptimizationParams",[](gtsam::ISAM2Params* self, const gtsam::ISAM2DoglegParams& dogleg_params){ self->setOptimizationParams(dogleg_params);}, py::arg("dogleg_params"))
        .def("setRelinearizeThreshold",[](gtsam::ISAM2Params* self, double threshold){ self->setRelinearizeThreshold(threshold);}, py::arg("threshold"))
        .def("setRelinearizeThreshold",[](gtsam::ISAM2Params* self, const gtsam::ISAM2ThresholdMap& threshold_map){ self->setRelinearizeThreshold(threshold_map);}, py::arg("threshold_map"))
        .def("getFactorization",[](gtsam::ISAM2Params* self){return self->getFactorization();})
        .def("setFactorization",[](gtsam::ISAM2Params* self, string factorization){ self->setFactorization(factorization);}, py::arg("factorization"))
        .def_readwrite("relinearizeSkip", &gtsam::ISAM2Params::relinearizeSkip)
        .def_readwrite("enableRelinearization", &gtsam::ISAM2Params::enableRelinearization)
        .def_readwrite("evaluateNonlinearError", &gtsam::ISAM2Params::evaluateNonlinearError)
        .def_readwrite("cacheLinearizedFactors", &gtsam::ISAM2Params::cacheLinearizedFactors)
        .def_readwrite("enableDetailedResults", &gtsam::ISAM2Params::enableDetailedResults)
        .def_readwrite("enablePartialRelinearizationCheck", &gtsam::ISAM2Params::enablePartialRelinearizationCheck)
        .def_readwrite("findUnusedFactorSlots", &gtsam::ISAM2Params::findUnusedFactorSlots)
        .def_readwrite("factorization", &gtsam::ISAM2Params::factorization);

    py::enum_<gtsam::ISAM2Params::Factorization>(isam2params, "Factorization", py::arithmetic())
        .value("CHOLESKY", gtsam::ISAM2Params::Factorization::CHOLESKY)
        .value("QR", gtsam::ISAM2Params::Factorization::QR);


    py::class_<gtsam::ISAM2Clique, boost::shared_ptr<gtsam::ISAM2Clique>>(m_, "ISAM2Clique")
        .def(py::init<>())
        .def("gradientContribution",[](gtsam::ISAM2Clique* self){return self->gradientContribution();})
        .def("print",[](gtsam::ISAM2Clique* self, string s, gtsam::KeyFormatter keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::ISAM2Clique& self, string s, gtsam::KeyFormatter keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

    py::class_<gtsam::ISAM2Result, boost::shared_ptr<gtsam::ISAM2Result>>(m_, "ISAM2Result")
        .def(py::init<>())
        .def("print",[](gtsam::ISAM2Result* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ISAM2Result& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("getVariablesRelinearized",[](gtsam::ISAM2Result* self){return self->getVariablesRelinearized();})
        .def("getVariablesReeliminated",[](gtsam::ISAM2Result* self){return self->getVariablesReeliminated();})
        .def("getNewFactorsIndices",[](gtsam::ISAM2Result* self){return self->getNewFactorsIndices();})
        .def("getCliques",[](gtsam::ISAM2Result* self){return self->getCliques();})
        .def("getErrorBefore",[](gtsam::ISAM2Result* self){return self->getErrorBefore();})
        .def("getErrorAfter",[](gtsam::ISAM2Result* self){return self->getErrorAfter();});

    py::class_<gtsam::ISAM2, boost::shared_ptr<gtsam::ISAM2>>(m_, "ISAM2")
        .def(py::init<>())
        .def(py::init<const gtsam::ISAM2Params&>(), py::arg("params"))
        .def(py::init<const gtsam::ISAM2&>(), py::arg("other"))
        .def("equals",[](gtsam::ISAM2* self, const gtsam::ISAM2& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print",[](gtsam::ISAM2* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::ISAM2& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("printStats",[](gtsam::ISAM2* self){ self->printStats();})
        .def("saveGraph",[](gtsam::ISAM2* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("update",[](gtsam::ISAM2* self){return self->update();})
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta){return self->update(newFactors, newTheta);}, py::arg("newFactors"), py::arg("newTheta"))
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices){return self->update(newFactors, newTheta, removeFactorIndices);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"))
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices, const gtsam::KeyGroupMap& constrainedKeys){return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"), py::arg("constrainedKeys"))
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices, gtsam::KeyGroupMap& constrainedKeys, const gtsam::KeyList& noRelinKeys){return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys, noRelinKeys);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"), py::arg("constrainedKeys"), py::arg("noRelinKeys"))
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices, gtsam::KeyGroupMap& constrainedKeys, const gtsam::KeyList& noRelinKeys, const gtsam::KeyList& extraReelimKeys){return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys, noRelinKeys, extraReelimKeys);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"), py::arg("constrainedKeys"), py::arg("noRelinKeys"), py::arg("extraReelimKeys"))
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices, gtsam::KeyGroupMap& constrainedKeys, const gtsam::KeyList& noRelinKeys, const gtsam::KeyList& extraReelimKeys, bool force_relinearize){return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys, noRelinKeys, extraReelimKeys, force_relinearize);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"), py::arg("constrainedKeys"), py::arg("noRelinKeys"), py::arg("extraReelimKeys"), py::arg("force_relinearize"))
        .def("update",[](gtsam::ISAM2* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::ISAM2UpdateParams& updateParams){return self->update(newFactors, newTheta, updateParams);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("updateParams"))
        .def("getLinearizationPoint",[](gtsam::ISAM2* self){return self->getLinearizationPoint();})
        .def("valueExists",[](gtsam::ISAM2* self, gtsam::Key key){return self->valueExists(key);}, py::arg("key"))
        .def("calculateEstimate",[](gtsam::ISAM2* self){return self->calculateEstimate();})
        .def("calculateEstimatePoint2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Point2>(key);}, py::arg("key"))
        .def("calculateEstimateRot2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Rot2>(key);}, py::arg("key"))
        .def("calculateEstimatePose2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Pose2>(key);}, py::arg("key"))
        .def("calculateEstimatePoint3",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Point3>(key);}, py::arg("key"))
        .def("calculateEstimateRot3",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Rot3>(key);}, py::arg("key"))
        .def("calculateEstimatePose3",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Pose3>(key);}, py::arg("key"))
        .def("calculateEstimateCal3_S2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Cal3_S2>(key);}, py::arg("key"))
        .def("calculateEstimateCal3DS2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Cal3DS2>(key);}, py::arg("key"))
        .def("calculateEstimateCal3Bundler",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Cal3Bundler>(key);}, py::arg("key"))
        .def("calculateEstimateEssentialMatrix",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::EssentialMatrix>(key);}, py::arg("key"))
        .def("calculateEstimatePinholeCameraCal3_S2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3_S2>>(key);}, py::arg("key"))
        .def("calculateEstimatePinholeCameraCal3Bundler",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key);}, py::arg("key"))
        .def("calculateEstimatePinholeCameraCal3Fisheye",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(key);}, py::arg("key"))
        .def("calculateEstimatePinholeCameraCal3Unified",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3Unified>>(key);}, py::arg("key"))
        .def("calculateEstimateVector",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Vector>(key);}, py::arg("key"))
        .def("calculateEstimateMatrix",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Matrix>(key);}, py::arg("key"))
        .def("marginalCovariance",[](gtsam::ISAM2* self, size_t key){return self->marginalCovariance(key);}, py::arg("key"))
        .def("calculateBestEstimate",[](gtsam::ISAM2* self){return self->calculateBestEstimate();})
        .def("getDelta",[](gtsam::ISAM2* self){return self->getDelta();})
        .def("error",[](gtsam::ISAM2* self, const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("getFactorsUnsafe",[](gtsam::ISAM2* self){return self->getFactorsUnsafe();})
        .def("getVariableIndex",[](gtsam::ISAM2* self){return self->getVariableIndex();})
        .def("getFixedVariables",[](gtsam::ISAM2* self){return self->getFixedVariables();})
        .def("params",[](gtsam::ISAM2* self){return self->params();})
        .def("printStats",[](gtsam::ISAM2* self){ self->printStats();})
        .def("gradientAtZero",[](gtsam::ISAM2* self){return self->gradientAtZero();})
        .def("dot",[](gtsam::ISAM2* self, const gtsam::KeyFormatter& keyFormatter){return self->dot(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("saveGraph",[](gtsam::ISAM2* self, string s, const gtsam::KeyFormatter& keyFormatter){ self->saveGraph(s, keyFormatter);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

    py::class_<gtsam::NonlinearISAM, boost::shared_ptr<gtsam::NonlinearISAM>>(m_, "NonlinearISAM")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("reorderInterval"))
        .def("print",[](gtsam::NonlinearISAM* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::NonlinearISAM& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("printStats",[](gtsam::NonlinearISAM* self){ self->printStats();})
        .def("saveGraph",[](gtsam::NonlinearISAM* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("estimate",[](gtsam::NonlinearISAM* self){return self->estimate();})
        .def("marginalCovariance",[](gtsam::NonlinearISAM* self, size_t key){return self->marginalCovariance(key);}, py::arg("key"))
        .def("reorderInterval",[](gtsam::NonlinearISAM* self){return self->reorderInterval();})
        .def("reorderCounter",[](gtsam::NonlinearISAM* self){return self->reorderCounter();})
        .def("update",[](gtsam::NonlinearISAM* self, const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& initialValues){ self->update(newFactors, initialValues);}, py::arg("newFactors"), py::arg("initialValues"))
        .def("reorder_relinearize",[](gtsam::NonlinearISAM* self){ self->reorder_relinearize();})
        .def("bayesTree",[](gtsam::NonlinearISAM* self){return self->bayesTree();})
        .def("getLinearizationPoint",[](gtsam::NonlinearISAM* self){return self->getLinearizationPoint();})
        .def("getFactorsUnsafe",[](gtsam::NonlinearISAM* self){return self->getFactorsUnsafe();});

    py::class_<gtsam::PriorFactor<double>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<double>>>(m_, "PriorFactorDouble")
        .def(py::init<size_t, const double&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<double>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<double>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<double>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<double> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Vector>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Vector>>>(m_, "PriorFactorVector")
        .def(py::init<size_t, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Vector>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Vector>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Vector>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Vector> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Vector> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Point2>>>(m_, "PriorFactorPoint2")
        .def(py::init<size_t, const gtsam::Point2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Point2>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Point2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Point2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Point2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::StereoPoint2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::StereoPoint2>>>(m_, "PriorFactorStereoPoint2")
        .def(py::init<size_t, const gtsam::StereoPoint2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::StereoPoint2>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::StereoPoint2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::StereoPoint2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::StereoPoint2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::StereoPoint2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Point3>>>(m_, "PriorFactorPoint3")
        .def(py::init<size_t, const gtsam::Point3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Point3>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Rot2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Rot2>>>(m_, "PriorFactorRot2")
        .def(py::init<size_t, const gtsam::Rot2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Rot2>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Rot2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Rot2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Rot2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::SO3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::SO3>>>(m_, "PriorFactorSO3")
        .def(py::init<size_t, const gtsam::SO3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::SO3>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::SO3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::SO3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::SO3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::SO3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::SO4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::SO4>>>(m_, "PriorFactorSO4")
        .def(py::init<size_t, const gtsam::SO4&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::SO4>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::SO4>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::SO4>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::SO4> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::SO4> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::SOn>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::SOn>>>(m_, "PriorFactorSOn")
        .def(py::init<size_t, const gtsam::SOn&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::SOn>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::SOn>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::SOn>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::SOn> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::SOn> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Rot3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Rot3>>>(m_, "PriorFactorRot3")
        .def(py::init<size_t, const gtsam::Rot3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Rot3>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Rot3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Rot3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Rot3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Rot3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>>(m_, "PriorFactorPose2")
        .def(py::init<size_t, const gtsam::Pose2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Pose2>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>>(m_, "PriorFactorPose3")
        .def(py::init<size_t, const gtsam::Pose3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Pose3>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Pose3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Pose3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Pose3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Unit3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Unit3>>>(m_, "PriorFactorUnit3")
        .def(py::init<size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Unit3>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Unit3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Unit3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Unit3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Unit3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Cal3_S2>>>(m_, "PriorFactorCal3_S2")
        .def(py::init<size_t, const gtsam::Cal3_S2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Cal3_S2>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3_S2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3_S2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Cal3_S2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Cal3DS2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Cal3DS2>>>(m_, "PriorFactorCal3DS2")
        .def(py::init<size_t, const gtsam::Cal3DS2&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Cal3DS2>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3DS2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3DS2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Cal3DS2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Cal3Bundler>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Cal3Bundler>>>(m_, "PriorFactorCal3Bundler")
        .def(py::init<size_t, const gtsam::Cal3Bundler&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Cal3Bundler>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3Bundler>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3Bundler>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Cal3Bundler> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3Bundler> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Cal3Fisheye>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Cal3Fisheye>>>(m_, "PriorFactorCal3Fisheye")
        .def(py::init<size_t, const gtsam::Cal3Fisheye&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Cal3Fisheye>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3Fisheye>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3Fisheye>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Cal3Fisheye> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::Cal3Unified>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::Cal3Unified>>>(m_, "PriorFactorCal3Unified")
        .def(py::init<size_t, const gtsam::Cal3Unified&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Cal3Unified>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3Unified>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3Unified>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::Cal3Unified> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::CalibratedCamera>>>(m_, "PriorFactorCalibratedCamera")
        .def(py::init<size_t, const gtsam::CalibratedCamera&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::CalibratedCamera>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::CalibratedCamera>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::CalibratedCamera>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::CalibratedCamera> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>>>(m_, "PriorFactorPinholeCameraCal3_S2")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3_S2>&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>>(m_, "PriorFactorPinholeCameraCal3Bundler")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Bundler>&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>>(m_, "PriorFactorPinholeCameraCal3Fisheye")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>>>(m_, "PriorFactorPinholeCameraCal3Unified")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Unified>&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>>(m_, "PriorFactorConstantBias")
        .def(py::init<size_t, const gtsam::imuBias::ConstantBias&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::imuBias::ConstantBias>* self){return self->prior();})
        .def("serialize", [](gtsam::PriorFactor<gtsam::imuBias::ConstantBias>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::PriorFactor<gtsam::imuBias::ConstantBias>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::PriorFactor<gtsam::imuBias::ConstantBias> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::PriorFactor<gtsam::imuBias::ConstantBias> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Point2>>>(m_, "NonlinearEqualityPoint2")
        .def(py::init<size_t, const gtsam::Point2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Point2&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Point2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Point2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Point2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::StereoPoint2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::StereoPoint2>>>(m_, "NonlinearEqualityStereoPoint2")
        .def(py::init<size_t, const gtsam::StereoPoint2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::StereoPoint2&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::StereoPoint2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::StereoPoint2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::StereoPoint2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::StereoPoint2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Point3>>>(m_, "NonlinearEqualityPoint3")
        .def(py::init<size_t, const gtsam::Point3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Point3&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Point3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Point3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Point3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Rot2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Rot2>>>(m_, "NonlinearEqualityRot2")
        .def(py::init<size_t, const gtsam::Rot2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Rot2&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Rot2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Rot2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Rot2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::SO3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::SO3>>>(m_, "NonlinearEqualitySO3")
        .def(py::init<size_t, const gtsam::SO3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::SO3&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::SO3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::SO3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::SO3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::SO3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::SO4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::SO4>>>(m_, "NonlinearEqualitySO4")
        .def(py::init<size_t, const gtsam::SO4&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::SO4&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::SO4>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::SO4>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::SO4> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::SO4> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::SOn>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::SOn>>>(m_, "NonlinearEqualitySOn")
        .def(py::init<size_t, const gtsam::SOn&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::SOn&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::SOn>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::SOn>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::SOn> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::SOn> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Rot3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Rot3>>>(m_, "NonlinearEqualityRot3")
        .def(py::init<size_t, const gtsam::Rot3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Rot3&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Rot3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Rot3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Rot3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Rot3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Pose2>>>(m_, "NonlinearEqualityPose2")
        .def(py::init<size_t, const gtsam::Pose2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Pose2&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Pose2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Pose2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Pose2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Pose3>>>(m_, "NonlinearEqualityPose3")
        .def(py::init<size_t, const gtsam::Pose3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Pose3&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Pose3>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Pose3>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Pose3> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::Cal3_S2>>>(m_, "NonlinearEqualityCal3_S2")
        .def(py::init<size_t, const gtsam::Cal3_S2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::Cal3_S2&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::Cal3_S2>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Cal3_S2>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::Cal3_S2> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::CalibratedCamera>>>(m_, "NonlinearEqualityCalibratedCamera")
        .def(py::init<size_t, const gtsam::CalibratedCamera&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::CalibratedCamera&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::CalibratedCamera>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::CalibratedCamera>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::CalibratedCamera> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>>>>(m_, "NonlinearEqualityPinholeCameraCal3_S2")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3_S2>&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3_S2>&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>>(m_, "NonlinearEqualityPinholeCameraCal3Bundler")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Bundler>&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Bundler>&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>>(m_, "NonlinearEqualityPinholeCameraCal3Fisheye")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>>>>(m_, "NonlinearEqualityPinholeCameraCal3Unified")
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Unified>&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Unified>&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>>>(m_, "NonlinearEqualityConstantBias")
        .def(py::init<size_t, const gtsam::imuBias::ConstantBias&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init<size_t, const gtsam::imuBias::ConstantBias&, double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"))
        .def("serialize", [](gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias> &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::NonlinearEquality2<gtsam::Point2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Point2>>>(m_, "NonlinearEquality2Point2")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Point2>* self, const gtsam::Point2& x1, const gtsam::Point2& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::StereoPoint2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::StereoPoint2>>>(m_, "NonlinearEquality2StereoPoint2")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::StereoPoint2>* self, const gtsam::StereoPoint2& x1, const gtsam::StereoPoint2& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::Point3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Point3>>>(m_, "NonlinearEquality2Point3")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Point3>* self, const gtsam::Point3& x1, const gtsam::Point3& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::Rot2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Rot2>>>(m_, "NonlinearEquality2Rot2")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Rot2>* self, const gtsam::Rot2& x1, const gtsam::Rot2& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::SO3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::SO3>>>(m_, "NonlinearEquality2SO3")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::SO3>* self, const gtsam::SO3& x1, const gtsam::SO3& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::SO4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::SO4>>>(m_, "NonlinearEquality2SO4")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::SO4>* self, const gtsam::SO4& x1, const gtsam::SO4& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::SOn>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::SOn>>>(m_, "NonlinearEquality2SOn")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::SOn>* self, const gtsam::SOn& x1, const gtsam::SOn& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::Rot3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Rot3>>>(m_, "NonlinearEquality2Rot3")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Rot3>* self, const gtsam::Rot3& x1, const gtsam::Rot3& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::Pose2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Pose2>>>(m_, "NonlinearEquality2Pose2")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Pose2>* self, const gtsam::Pose2& x1, const gtsam::Pose2& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::Pose3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Pose3>>>(m_, "NonlinearEquality2Pose3")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Pose3>* self, const gtsam::Pose3& x1, const gtsam::Pose3& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::Cal3_S2>>>(m_, "NonlinearEquality2Cal3_S2")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::Cal3_S2>* self, const gtsam::Cal3_S2& x1, const gtsam::Cal3_S2& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::CalibratedCamera>>>(m_, "NonlinearEquality2CalibratedCamera")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::CalibratedCamera>* self, const gtsam::CalibratedCamera& x1, const gtsam::CalibratedCamera& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3_S2>>>>(m_, "NonlinearEquality2PinholeCameraCal3_S2")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self, const gtsam::PinholeCamera<gtsam::Cal3_S2>& x1, const gtsam::PinholeCamera<gtsam::Cal3_S2>& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Bundler>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>>(m_, "NonlinearEquality2PinholeCameraCal3Bundler")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Bundler>>* self, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& x1, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>>(m_, "NonlinearEquality2PinholeCameraCal3Fisheye")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>* self, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& x1, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Unified>>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Unified>>>>(m_, "NonlinearEquality2PinholeCameraCal3Unified")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Unified>>* self, const gtsam::PinholeCamera<gtsam::Cal3Unified>& x1, const gtsam::PinholeCamera<gtsam::Cal3Unified>& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::NonlinearEquality2<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::NonlinearEquality2<gtsam::imuBias::ConstantBias>>>(m_, "NonlinearEquality2ConstantBias")
        .def(py::init<gtsam::Key, gtsam::Key, double>(), py::arg("key1"), py::arg("key2"), py::arg("mu") = 1e4)
        .def("evaluateError",[](gtsam::NonlinearEquality2<gtsam::imuBias::ConstantBias>* self, const gtsam::imuBias::ConstantBias& x1, const gtsam::imuBias::ConstantBias& x2){return self->evaluateError(x1, x2);}, py::arg("x1"), py::arg("x2"));

    py::class_<gtsam::GncParams<gtsam::GaussNewtonParams>, boost::shared_ptr<gtsam::GncParams<gtsam::GaussNewtonParams>>> gncgaussnewtonparams(m_, "GncGaussNewtonParams");
    gncgaussnewtonparams
        .def(py::init<const gtsam::GaussNewtonParams&>(), py::arg("baseOptimizerParams"))
        .def(py::init<>())
        .def("setLossType",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const gtsam::GncLossType type){ self->setLossType(type);}, py::arg("type"))
        .def("setMaxIterations",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const size_t maxIter){ self->setMaxIterations(maxIter);}, py::arg("maxIter"))
        .def("setMuStep",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const double step){ self->setMuStep(step);}, py::arg("step"))
        .def("setRelativeCostTol",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, double value){ self->setRelativeCostTol(value);}, py::arg("value"))
        .def("setWeightsTol",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, double value){ self->setWeightsTol(value);}, py::arg("value"))
        .def("setVerbosityGNC",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity value){ self->setVerbosityGNC(value);}, py::arg("value"))
        .def("setKnownInliers",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const gtsam::KeyVector& knownIn){ self->setKnownInliers(knownIn);}, py::arg("knownIn"))
        .def("setKnownOutliers",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const gtsam::KeyVector& knownOut){ self->setKnownOutliers(knownOut);}, py::arg("knownOut"))
        .def("print",[](gtsam::GncParams<gtsam::GaussNewtonParams>* self, const string& str){ py::scoped_ostream_redirect output; self->print(str);}, py::arg("str") = "GncParams: ")
        .def("__repr__",
                    [](const gtsam::GncParams<gtsam::GaussNewtonParams>& self, const string& str){
                        gtsam::RedirectCout redirect;
                        self.print(str);
                        return redirect.str();
                    }, py::arg("str") = "GncParams: ")
        .def_readwrite("baseOptimizerParams", &gtsam::GncParams<gtsam::GaussNewtonParams>::baseOptimizerParams)
        .def_readwrite("lossType", &gtsam::GncParams<gtsam::GaussNewtonParams>::lossType)
        .def_readwrite("maxIterations", &gtsam::GncParams<gtsam::GaussNewtonParams>::maxIterations)
        .def_readwrite("muStep", &gtsam::GncParams<gtsam::GaussNewtonParams>::muStep)
        .def_readwrite("relativeCostTol", &gtsam::GncParams<gtsam::GaussNewtonParams>::relativeCostTol)
        .def_readwrite("weightsTol", &gtsam::GncParams<gtsam::GaussNewtonParams>::weightsTol)
        .def_readwrite("verbosity", &gtsam::GncParams<gtsam::GaussNewtonParams>::verbosity)
        .def_readwrite("knownInliers", &gtsam::GncParams<gtsam::GaussNewtonParams>::knownInliers)
        .def_readwrite("knownOutliers", &gtsam::GncParams<gtsam::GaussNewtonParams>::knownOutliers);

    py::enum_<gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity>(gncgaussnewtonparams, "Verbosity", py::arithmetic())
        .value("SILENT", gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity::SILENT)
        .value("SUMMARY", gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity::SUMMARY)
        .value("VALUES", gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity::VALUES);


    py::class_<gtsam::GncParams<gtsam::LevenbergMarquardtParams>, boost::shared_ptr<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>> gnclmparams(m_, "GncLMParams");
    gnclmparams
        .def(py::init<const gtsam::LevenbergMarquardtParams&>(), py::arg("baseOptimizerParams"))
        .def(py::init<>())
        .def("setLossType",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const gtsam::GncLossType type){ self->setLossType(type);}, py::arg("type"))
        .def("setMaxIterations",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const size_t maxIter){ self->setMaxIterations(maxIter);}, py::arg("maxIter"))
        .def("setMuStep",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const double step){ self->setMuStep(step);}, py::arg("step"))
        .def("setRelativeCostTol",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, double value){ self->setRelativeCostTol(value);}, py::arg("value"))
        .def("setWeightsTol",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, double value){ self->setWeightsTol(value);}, py::arg("value"))
        .def("setVerbosityGNC",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity value){ self->setVerbosityGNC(value);}, py::arg("value"))
        .def("setKnownInliers",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const gtsam::KeyVector& knownIn){ self->setKnownInliers(knownIn);}, py::arg("knownIn"))
        .def("setKnownOutliers",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const gtsam::KeyVector& knownOut){ self->setKnownOutliers(knownOut);}, py::arg("knownOut"))
        .def("print",[](gtsam::GncParams<gtsam::LevenbergMarquardtParams>* self, const string& str){ py::scoped_ostream_redirect output; self->print(str);}, py::arg("str") = "GncParams: ")
        .def("__repr__",
                    [](const gtsam::GncParams<gtsam::LevenbergMarquardtParams>& self, const string& str){
                        gtsam::RedirectCout redirect;
                        self.print(str);
                        return redirect.str();
                    }, py::arg("str") = "GncParams: ")
        .def_readwrite("baseOptimizerParams", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::baseOptimizerParams)
        .def_readwrite("lossType", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::lossType)
        .def_readwrite("maxIterations", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::maxIterations)
        .def_readwrite("muStep", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::muStep)
        .def_readwrite("relativeCostTol", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::relativeCostTol)
        .def_readwrite("weightsTol", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::weightsTol)
        .def_readwrite("verbosity", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::verbosity)
        .def_readwrite("knownInliers", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::knownInliers)
        .def_readwrite("knownOutliers", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::knownOutliers);

    py::enum_<gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity>(gnclmparams, "Verbosity", py::arithmetic())
        .value("SILENT", gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SILENT)
        .value("SUMMARY", gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SUMMARY)
        .value("VALUES", gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::VALUES);


    py::class_<gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>, boost::shared_ptr<gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>>>(m_, "GncGaussNewtonOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::GncParams<gtsam::GaussNewtonParams>&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"))
        .def("setInlierCostThresholds",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>* self, const double inth){ self->setInlierCostThresholds(inth);}, py::arg("inth"))
        .def("getInlierCostThresholds",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>* self){return self->getInlierCostThresholds();})
        .def("setInlierCostThresholdsAtProbability",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>* self, const double alpha){ self->setInlierCostThresholdsAtProbability(alpha);}, py::arg("alpha"))
        .def("setWeights",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>* self, const gtsam::Vector& w){ self->setWeights(w);}, py::arg("w"))
        .def("getWeights",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>* self){return self->getWeights();})
        .def("optimize",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>* self){return self->optimize();});

    py::class_<gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>, boost::shared_ptr<gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>>>(m_, "GncLMOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::GncParams<gtsam::LevenbergMarquardtParams>&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"))
        .def("setInlierCostThresholds",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>* self, const double inth){ self->setInlierCostThresholds(inth);}, py::arg("inth"))
        .def("getInlierCostThresholds",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>* self){return self->getInlierCostThresholds();})
        .def("setInlierCostThresholdsAtProbability",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>* self, const double alpha){ self->setInlierCostThresholdsAtProbability(alpha);}, py::arg("alpha"))
        .def("setWeights",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>* self, const gtsam::Vector& w){ self->setWeights(w);}, py::arg("w"))
        .def("getWeights",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>* self){return self->getWeights();})
        .def("optimize",[](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>* self){return self->optimize();});

    m_.def("checkConvergence",[](double relativeErrorTreshold, double absoluteErrorTreshold, double errorThreshold, double currentError, double newError){return gtsam::checkConvergence(relativeErrorTreshold, absoluteErrorTreshold, errorThreshold, currentError, newError);}, py::arg("relativeErrorTreshold"), py::arg("absoluteErrorTreshold"), py::arg("errorThreshold"), py::arg("currentError"), py::arg("newError"));
    m_.def("checkConvergence",[](const gtsam::NonlinearOptimizerParams& params, double currentError, double newError){return gtsam::checkConvergence(params, currentError, newError);}, py::arg("params"), py::arg("currentError"), py::arg("newError"));

// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/nonlinear.h"

}

