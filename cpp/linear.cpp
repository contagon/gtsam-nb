/**
 * @file    linear.cpp
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
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/linear/Sampler.h"
#include "gtsam/linear/VectorValues.h"
#include "gtsam/linear/GaussianFactor.h"
#include "gtsam/linear/JacobianFactor.h"
#include "gtsam/linear/HessianFactor.h"
#include "gtsam/linear/GaussianFactorGraph.h"
#include "gtsam/linear/GaussianConditional.h"
#include "gtsam/hybrid/HybridValues.h"
#include "gtsam/linear/GaussianDensity.h"
#include "gtsam/linear/GaussianBayesNet.h"
#include "gtsam/linear/GaussianBayesTree.h"
#include "gtsam/linear/GaussianISAM.h"
#include "gtsam/linear/IterativeSolver.h"
#include "gtsam/linear/ConjugateGradientSolver.h"
#include "gtsam/linear/Preconditioner.h"
#include "gtsam/linear/PCGSolver.h"
#include "gtsam/linear/SubgraphSolver.h"
#include "gtsam/linear/KalmanFilter.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::noiseModel::Gaussian)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Diagonal)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Constrained)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Isotropic)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Unit)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Null)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Fair)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Huber)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Cauchy)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Tukey)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::Welsch)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::GemanMcClure)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::DCS)
BOOST_CLASS_EXPORT(gtsam::noiseModel::mEstimator::L2WithDeadZone)
BOOST_CLASS_EXPORT(gtsam::noiseModel::Robust)
BOOST_CLASS_EXPORT(gtsam::VectorValues)
BOOST_CLASS_EXPORT(gtsam::JacobianFactor)
BOOST_CLASS_EXPORT(gtsam::HessianFactor)
BOOST_CLASS_EXPORT(gtsam::GaussianFactorGraph)
BOOST_CLASS_EXPORT(gtsam::GaussianConditional)


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/linear.h"

using namespace std;

namespace py = pybind11;



void linear(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of linear";



    pybind11::module m_noiseModel = m_.def_submodule("noiseModel", "noiseModel submodule");

    py::class_<gtsam::noiseModel::Base, boost::shared_ptr<gtsam::noiseModel::Base>>(m_noiseModel, "Base")
        .def("print",[](gtsam::noiseModel::Base* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::noiseModel::Base& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::noiseModel::Gaussian, gtsam::noiseModel::Base, boost::shared_ptr<gtsam::noiseModel::Gaussian>>(m_noiseModel, "Gaussian")
        .def("equals",[](gtsam::noiseModel::Gaussian* self, gtsam::noiseModel::Base& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("R",[](gtsam::noiseModel::Gaussian* self){return self->R();})
        .def("information",[](gtsam::noiseModel::Gaussian* self){return self->information();})
        .def("covariance",[](gtsam::noiseModel::Gaussian* self){return self->covariance();})
        .def("whiten",[](gtsam::noiseModel::Gaussian* self, const gtsam::Vector& v){return self->whiten(v);}, py::arg("v"))
        .def("unwhiten",[](gtsam::noiseModel::Gaussian* self, const gtsam::Vector& v){return self->unwhiten(v);}, py::arg("v"))
        .def("Whiten",[](gtsam::noiseModel::Gaussian* self, const gtsam::Matrix& H){return self->Whiten(H);}, py::arg("H"))
        .def("serialize", [](gtsam::noiseModel::Gaussian* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::Gaussian* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::Gaussian &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::Gaussian obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Information",[](const gtsam::Matrix& R, bool smart){return gtsam::noiseModel::Gaussian::Information(R, smart);}, py::arg("R"), py::arg("smart") = true)
        .def_static("SqrtInformation",[](const gtsam::Matrix& R, bool smart){return gtsam::noiseModel::Gaussian::SqrtInformation(R, smart);}, py::arg("R"), py::arg("smart") = true)
        .def_static("Covariance",[](const gtsam::Matrix& R, bool smart){return gtsam::noiseModel::Gaussian::Covariance(R, smart);}, py::arg("R"), py::arg("smart") = true);

    py::class_<gtsam::noiseModel::Diagonal, gtsam::noiseModel::Gaussian, boost::shared_ptr<gtsam::noiseModel::Diagonal>>(m_noiseModel, "Diagonal")
        .def("R",[](gtsam::noiseModel::Diagonal* self){return self->R();})
        .def("sigmas",[](gtsam::noiseModel::Diagonal* self){return self->sigmas();})
        .def("invsigmas",[](gtsam::noiseModel::Diagonal* self){return self->invsigmas();})
        .def("precisions",[](gtsam::noiseModel::Diagonal* self){return self->precisions();})
        .def("serialize", [](gtsam::noiseModel::Diagonal* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::Diagonal* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::Diagonal &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::Diagonal obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Sigmas",[](const gtsam::Vector& sigmas, bool smart){return gtsam::noiseModel::Diagonal::Sigmas(sigmas, smart);}, py::arg("sigmas"), py::arg("smart") = true)
        .def_static("Variances",[](const gtsam::Vector& variances, bool smart){return gtsam::noiseModel::Diagonal::Variances(variances, smart);}, py::arg("variances"), py::arg("smart") = true)
        .def_static("Precisions",[](const gtsam::Vector& precisions, bool smart){return gtsam::noiseModel::Diagonal::Precisions(precisions, smart);}, py::arg("precisions"), py::arg("smart") = true);

    py::class_<gtsam::noiseModel::Constrained, gtsam::noiseModel::Diagonal, boost::shared_ptr<gtsam::noiseModel::Constrained>>(m_noiseModel, "Constrained")
        .def("unit",[](gtsam::noiseModel::Constrained* self){return self->unit();})
        .def("serialize", [](gtsam::noiseModel::Constrained* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::Constrained* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::Constrained &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::Constrained obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("MixedSigmas",[](const gtsam::Vector& mu, const gtsam::Vector& sigmas){return gtsam::noiseModel::Constrained::MixedSigmas(mu, sigmas);}, py::arg("mu"), py::arg("sigmas"))
        .def_static("MixedSigmas",[](double m, const gtsam::Vector& sigmas){return gtsam::noiseModel::Constrained::MixedSigmas(m, sigmas);}, py::arg("m"), py::arg("sigmas"))
        .def_static("MixedVariances",[](const gtsam::Vector& mu, const gtsam::Vector& variances){return gtsam::noiseModel::Constrained::MixedVariances(mu, variances);}, py::arg("mu"), py::arg("variances"))
        .def_static("MixedVariances",[](const gtsam::Vector& variances){return gtsam::noiseModel::Constrained::MixedVariances(variances);}, py::arg("variances"))
        .def_static("MixedPrecisions",[](const gtsam::Vector& mu, const gtsam::Vector& precisions){return gtsam::noiseModel::Constrained::MixedPrecisions(mu, precisions);}, py::arg("mu"), py::arg("precisions"))
        .def_static("MixedPrecisions",[](const gtsam::Vector& precisions){return gtsam::noiseModel::Constrained::MixedPrecisions(precisions);}, py::arg("precisions"))
        .def_static("All",[](size_t dim){return gtsam::noiseModel::Constrained::All(dim);}, py::arg("dim"))
        .def_static("All",[](size_t dim, double mu){return gtsam::noiseModel::Constrained::All(dim, mu);}, py::arg("dim"), py::arg("mu"));

    py::class_<gtsam::noiseModel::Isotropic, gtsam::noiseModel::Diagonal, boost::shared_ptr<gtsam::noiseModel::Isotropic>>(m_noiseModel, "Isotropic")
        .def("sigma",[](gtsam::noiseModel::Isotropic* self){return self->sigma();})
        .def("serialize", [](gtsam::noiseModel::Isotropic* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::Isotropic* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::Isotropic &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::Isotropic obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Sigma",[](size_t dim, double sigma, bool smart){return gtsam::noiseModel::Isotropic::Sigma(dim, sigma, smart);}, py::arg("dim"), py::arg("sigma"), py::arg("smart") = true)
        .def_static("Variance",[](size_t dim, double varianace, bool smart){return gtsam::noiseModel::Isotropic::Variance(dim, varianace, smart);}, py::arg("dim"), py::arg("varianace"), py::arg("smart") = true)
        .def_static("Precision",[](size_t dim, double precision, bool smart){return gtsam::noiseModel::Isotropic::Precision(dim, precision, smart);}, py::arg("dim"), py::arg("precision"), py::arg("smart") = true);

    py::class_<gtsam::noiseModel::Unit, gtsam::noiseModel::Isotropic, boost::shared_ptr<gtsam::noiseModel::Unit>>(m_noiseModel, "Unit")
        .def("serialize", [](gtsam::noiseModel::Unit* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::Unit* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::Unit &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::Unit obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Create",[](size_t dim){return gtsam::noiseModel::Unit::Create(dim);}, py::arg("dim"));
    pybind11::module m_noiseModel_mEstimator = m_noiseModel.def_submodule("mEstimator", "mEstimator submodule");

    py::class_<gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>>(m_noiseModel_mEstimator, "Base")
        .def("print",[](gtsam::noiseModel::mEstimator::Base* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::noiseModel::mEstimator::Base& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::noiseModel::mEstimator::Null, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Null>>(m_noiseModel_mEstimator, "Null")
        .def(py::init<>())
        .def("serialize", [](gtsam::noiseModel::mEstimator::Null* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::Null* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::Null &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::Null obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::Null* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::Null* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](){return gtsam::noiseModel::mEstimator::Null::Create();});

    py::class_<gtsam::noiseModel::mEstimator::Fair, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Fair>>(m_noiseModel_mEstimator, "Fair")
        .def(py::init<double>(), py::arg("c"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::Fair* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::Fair* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::Fair &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::Fair obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::Fair* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::Fair* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double c){return gtsam::noiseModel::mEstimator::Fair::Create(c);}, py::arg("c"));

    py::class_<gtsam::noiseModel::mEstimator::Huber, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Huber>>(m_noiseModel_mEstimator, "Huber")
        .def(py::init<double>(), py::arg("k"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::Huber* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::Huber* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::Huber &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::Huber obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::Huber* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::Huber* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double k){return gtsam::noiseModel::mEstimator::Huber::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::mEstimator::Cauchy, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Cauchy>>(m_noiseModel_mEstimator, "Cauchy")
        .def(py::init<double>(), py::arg("k"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::Cauchy* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::Cauchy* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::Cauchy &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::Cauchy obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::Cauchy* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::Cauchy* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double k){return gtsam::noiseModel::mEstimator::Cauchy::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::mEstimator::Tukey, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Tukey>>(m_noiseModel_mEstimator, "Tukey")
        .def(py::init<double>(), py::arg("k"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::Tukey* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::Tukey* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::Tukey &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::Tukey obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::Tukey* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::Tukey* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double k){return gtsam::noiseModel::mEstimator::Tukey::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::mEstimator::Welsch, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::Welsch>>(m_noiseModel_mEstimator, "Welsch")
        .def(py::init<double>(), py::arg("k"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::Welsch* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::Welsch* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::Welsch &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::Welsch obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::Welsch* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::Welsch* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double k){return gtsam::noiseModel::mEstimator::Welsch::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::mEstimator::GemanMcClure, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::GemanMcClure>>(m_noiseModel_mEstimator, "GemanMcClure")
        .def(py::init<double>(), py::arg("c"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::GemanMcClure* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::GemanMcClure* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::GemanMcClure &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::GemanMcClure obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::GemanMcClure* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::GemanMcClure* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double c){return gtsam::noiseModel::mEstimator::GemanMcClure::Create(c);}, py::arg("c"));

    py::class_<gtsam::noiseModel::mEstimator::DCS, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::DCS>>(m_noiseModel_mEstimator, "DCS")
        .def(py::init<double>(), py::arg("c"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::DCS* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::DCS* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::DCS &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::DCS obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::DCS* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::DCS* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double c){return gtsam::noiseModel::mEstimator::DCS::Create(c);}, py::arg("c"));

    py::class_<gtsam::noiseModel::mEstimator::L2WithDeadZone, gtsam::noiseModel::mEstimator::Base, boost::shared_ptr<gtsam::noiseModel::mEstimator::L2WithDeadZone>>(m_noiseModel_mEstimator, "L2WithDeadZone")
        .def(py::init<double>(), py::arg("k"))
        .def("serialize", [](gtsam::noiseModel::mEstimator::L2WithDeadZone* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::mEstimator::L2WithDeadZone* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::mEstimator::L2WithDeadZone &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::mEstimator::L2WithDeadZone obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("weight",[](gtsam::noiseModel::mEstimator::L2WithDeadZone* self, double error){return self->weight(error);}, py::arg("error"))
        .def("loss",[](gtsam::noiseModel::mEstimator::L2WithDeadZone* self, double error){return self->loss(error);}, py::arg("error"))
        .def_static("Create",[](double k){return gtsam::noiseModel::mEstimator::L2WithDeadZone::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::Robust, gtsam::noiseModel::Base, boost::shared_ptr<gtsam::noiseModel::Robust>>(m_noiseModel, "Robust")
        .def(py::init<const boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("robust"), py::arg("noise"))
        .def("serialize", [](gtsam::noiseModel::Robust* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::noiseModel::Robust* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::noiseModel::Robust &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::noiseModel::Robust obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("Create",[](const boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> robust, const boost::shared_ptr<gtsam::noiseModel::Base> noise){return gtsam::noiseModel::Robust::Create(robust, noise);}, py::arg("robust"), py::arg("noise"));

    py::class_<gtsam::Sampler, boost::shared_ptr<gtsam::Sampler>>(m_, "Sampler")
        .def(py::init<boost::shared_ptr<gtsam::noiseModel::Diagonal>, int>(), py::arg("model"), py::arg("seed"))
        .def(py::init<const gtsam::Vector&, int>(), py::arg("sigmas"), py::arg("seed"))
        .def("dim",[](gtsam::Sampler* self){return self->dim();})
        .def("sigmas",[](gtsam::Sampler* self){return self->sigmas();})
        .def("model",[](gtsam::Sampler* self){return self->model();})
        .def("sample",[](gtsam::Sampler* self){return self->sample();});

    py::class_<gtsam::VectorValues, boost::shared_ptr<gtsam::VectorValues>>(m_, "VectorValues")
        .def(py::init<>())
        .def(py::init<const gtsam::VectorValues&>(), py::arg("other"))
        .def(py::init<const gtsam::VectorValues&, const gtsam::VectorValues&>(), py::arg("first"), py::arg("second"))
        .def("size",[](gtsam::VectorValues* self){return self->size();})
        .def("dim",[](gtsam::VectorValues* self, size_t j){return self->dim(j);}, py::arg("j"))
        .def("exists",[](gtsam::VectorValues* self, size_t j){return self->exists(j);}, py::arg("j"))
        .def("print",[](gtsam::VectorValues* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "VectorValues", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::VectorValues& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "VectorValues", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::VectorValues* self, const gtsam::VectorValues& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("insert",[](gtsam::VectorValues* self, size_t j, const gtsam::Vector& value){ self->insert(j, value);}, py::arg("j"), py::arg("value"))
        .def("vector",[](gtsam::VectorValues* self){return self->vector();})
        .def("vector",[](gtsam::VectorValues* self, const gtsam::KeyVector& keys){return self->vector(keys);}, py::arg("keys"))
        .def("at",[](gtsam::VectorValues* self, size_t j){return self->at(j);}, py::arg("j"))
        .def("insert",[](gtsam::VectorValues* self, const gtsam::VectorValues& values){ self->insert(values);}, py::arg("values"))
        .def("update",[](gtsam::VectorValues* self, const gtsam::VectorValues& values){ self->update(values);}, py::arg("values"))
        .def("setZero",[](gtsam::VectorValues* self){ self->setZero();})
        .def("add",[](gtsam::VectorValues* self, const gtsam::VectorValues& c){return self->add(c);}, py::arg("c"))
        .def("addInPlace",[](gtsam::VectorValues* self, const gtsam::VectorValues& c){ self->addInPlace(c);}, py::arg("c"))
        .def("subtract",[](gtsam::VectorValues* self, const gtsam::VectorValues& c){return self->subtract(c);}, py::arg("c"))
        .def("scale",[](gtsam::VectorValues* self, double a){return self->scale(a);}, py::arg("a"))
        .def("scaleInPlace",[](gtsam::VectorValues* self, double a){ self->scaleInPlace(a);}, py::arg("a"))
        .def("hasSameStructure",[](gtsam::VectorValues* self, const gtsam::VectorValues& other){return self->hasSameStructure(other);}, py::arg("other"))
        .def("dot",[](gtsam::VectorValues* self, const gtsam::VectorValues& V){return self->dot(V);}, py::arg("V"))
        .def("norm",[](gtsam::VectorValues* self){return self->norm();})
        .def("squaredNorm",[](gtsam::VectorValues* self){return self->squaredNorm();})
        .def("serialize", [](gtsam::VectorValues* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::VectorValues* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::VectorValues &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::VectorValues obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("_repr_html_",[](gtsam::VectorValues* self){return self->html();})
        .def_static("Zero",[](const gtsam::VectorValues& model){return gtsam::VectorValues::Zero(model);}, py::arg("model"));

    py::class_<gtsam::GaussianFactor, gtsam::Factor, boost::shared_ptr<gtsam::GaussianFactor>>(m_, "GaussianFactor")
        .def("print",[](gtsam::GaussianFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GaussianFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GaussianFactor* self, const gtsam::GaussianFactor& lf, double tol){return self->equals(lf, tol);}, py::arg("lf"), py::arg("tol"))
        .def("error",[](gtsam::GaussianFactor* self, const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("clone",[](gtsam::GaussianFactor* self){return self->clone();})
        .def("negate",[](gtsam::GaussianFactor* self){return self->negate();})
        .def("augmentedInformation",[](gtsam::GaussianFactor* self){return self->augmentedInformation();})
        .def("information",[](gtsam::GaussianFactor* self){return self->information();})
        .def("augmentedJacobian",[](gtsam::GaussianFactor* self){return self->augmentedJacobian();})
        .def("jacobian",[](gtsam::GaussianFactor* self){return self->jacobian();});

    py::class_<gtsam::JacobianFactor, gtsam::GaussianFactor, boost::shared_ptr<gtsam::JacobianFactor>>(m_, "JacobianFactor")
        .def(py::init<>())
        .def(py::init<const gtsam::Vector&>(), py::arg("b_in"))
        .def(py::init<size_t, const gtsam::Matrix&, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("i1"), py::arg("A1"), py::arg("b"), py::arg("model"))
        .def(py::init<size_t, const gtsam::Matrix&, size_t, const gtsam::Matrix&, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("i1"), py::arg("A1"), py::arg("i2"), py::arg("A2"), py::arg("b"), py::arg("model"))
        .def(py::init<size_t, const gtsam::Matrix&, size_t, const gtsam::Matrix&, size_t, const gtsam::Matrix&, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("i1"), py::arg("A1"), py::arg("i2"), py::arg("A2"), py::arg("i3"), py::arg("A3"), py::arg("b"), py::arg("model"))
        .def(py::init<const gtsam::GaussianFactorGraph&>(), py::arg("graph"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::VariableSlots&>(), py::arg("graph"), py::arg("p_variableSlots"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::Ordering&>(), py::arg("graph"), py::arg("ordering"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::Ordering&, const gtsam::VariableSlots&>(), py::arg("graph"), py::arg("ordering"), py::arg("p_variableSlots"))
        .def(py::init<const gtsam::GaussianFactor&>(), py::arg("factor"))
        .def("print",[](gtsam::JacobianFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::JacobianFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::JacobianFactor* self, const gtsam::GaussianFactor& lf, double tol){return self->equals(lf, tol);}, py::arg("lf"), py::arg("tol"))
        .def("unweighted_error",[](gtsam::JacobianFactor* self, const gtsam::VectorValues& c){return self->unweighted_error(c);}, py::arg("c"))
        .def("error_vector",[](gtsam::JacobianFactor* self, const gtsam::VectorValues& c){return self->error_vector(c);}, py::arg("c"))
        .def("error",[](gtsam::JacobianFactor* self, const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("getA",[](gtsam::JacobianFactor* self){return self->getA();})
        .def("getb",[](gtsam::JacobianFactor* self){return self->getb();})
        .def("rows",[](gtsam::JacobianFactor* self){return self->rows();})
        .def("cols",[](gtsam::JacobianFactor* self){return self->cols();})
        .def("isConstrained",[](gtsam::JacobianFactor* self){return self->isConstrained();})
        .def("jacobianUnweighted",[](gtsam::JacobianFactor* self){return self->jacobianUnweighted();})
        .def("augmentedJacobianUnweighted",[](gtsam::JacobianFactor* self){return self->augmentedJacobianUnweighted();})
        .def("transposeMultiplyAdd",[](gtsam::JacobianFactor* self, double alpha, const gtsam::Vector& e, gtsam::VectorValues& x){ self->transposeMultiplyAdd(alpha, e, x);}, py::arg("alpha"), py::arg("e"), py::arg("x"))
        .def("whiten",[](gtsam::JacobianFactor* self){return self->whiten();})
        .def("eliminate",[](gtsam::JacobianFactor* self, const gtsam::Ordering& keys){return self->eliminate(keys);}, py::arg("keys"))
        .def("setModel",[](gtsam::JacobianFactor* self, bool anyConstrained, const gtsam::Vector& sigmas){ self->setModel(anyConstrained, sigmas);}, py::arg("anyConstrained"), py::arg("sigmas"))
        .def("get_model",[](gtsam::JacobianFactor* self){return self->get_model();})
        .def("serialize", [](gtsam::JacobianFactor* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::JacobianFactor* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::JacobianFactor &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::JacobianFactor obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::HessianFactor, gtsam::GaussianFactor, boost::shared_ptr<gtsam::HessianFactor>>(m_, "HessianFactor")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianFactor&>(), py::arg("factor"))
        .def(py::init<size_t, const gtsam::Matrix&, const gtsam::Vector&, double>(), py::arg("j"), py::arg("G"), py::arg("g"), py::arg("f"))
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&>(), py::arg("j"), py::arg("mu"), py::arg("Sigma"))
        .def(py::init<size_t, size_t, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Vector&, const gtsam::Matrix&, const gtsam::Vector&, double>(), py::arg("j1"), py::arg("j2"), py::arg("G11"), py::arg("G12"), py::arg("g1"), py::arg("G22"), py::arg("g2"), py::arg("f"))
        .def(py::init<size_t, size_t, size_t, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Vector&, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Vector&, const gtsam::Matrix&, const gtsam::Vector&, double>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("G11"), py::arg("G12"), py::arg("G13"), py::arg("g1"), py::arg("G22"), py::arg("G23"), py::arg("g2"), py::arg("G33"), py::arg("g3"), py::arg("f"))
        .def(py::init<const gtsam::GaussianFactorGraph&>(), py::arg("factors"))
        .def("print",[](gtsam::HessianFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::HessianFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::HessianFactor* self, const gtsam::GaussianFactor& lf, double tol){return self->equals(lf, tol);}, py::arg("lf"), py::arg("tol"))
        .def("error",[](gtsam::HessianFactor* self, const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("rows",[](gtsam::HessianFactor* self){return self->rows();})
        .def("information",[](gtsam::HessianFactor* self){return self->information();})
        .def("constantTerm",[](gtsam::HessianFactor* self){return self->constantTerm();})
        .def("linearTerm",[](gtsam::HessianFactor* self){return self->linearTerm();})
        .def("serialize", [](gtsam::HessianFactor* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::HessianFactor* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::HessianFactor &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::HessianFactor obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::GaussianFactorGraph, boost::shared_ptr<gtsam::GaussianFactorGraph>>(m_, "GaussianFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianBayesNet&>(), py::arg("bayesNet"))
        .def(py::init<const gtsam::GaussianBayesTree&>(), py::arg("bayesTree"))
        .def("print",[](gtsam::GaussianFactorGraph* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GaussianFactorGraph& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GaussianFactorGraph* self, const gtsam::GaussianFactorGraph& lfgraph, double tol){return self->equals(lfgraph, tol);}, py::arg("lfgraph"), py::arg("tol"))
        .def("size",[](gtsam::GaussianFactorGraph* self){return self->size();})
        .def("at",[](gtsam::GaussianFactorGraph* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::GaussianFactorGraph* self){return self->keys();})
        .def("keyVector",[](gtsam::GaussianFactorGraph* self){return self->keyVector();})
        .def("exists",[](gtsam::GaussianFactorGraph* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self, const boost::shared_ptr<gtsam::GaussianFactor> factor){ self->push_back(factor);}, py::arg("factor"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self, const boost::shared_ptr<gtsam::GaussianConditional> conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self, const gtsam::GaussianFactorGraph& graph){ self->push_back(graph);}, py::arg("graph"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self, const gtsam::GaussianBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self, const gtsam::GaussianBayesTree& bayesTree){ self->push_back(bayesTree);}, py::arg("bayesTree"))
        .def("add",[](gtsam::GaussianFactorGraph* self, const gtsam::GaussianFactor& factor){ self->add(factor);}, py::arg("factor"))
        .def("add",[](gtsam::GaussianFactorGraph* self, const gtsam::Vector& b){ self->add(b);}, py::arg("b"))
        .def("add",[](gtsam::GaussianFactorGraph* self, size_t key1, const gtsam::Matrix& A1, const gtsam::Vector& b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model){ self->add(key1, A1, b, model);}, py::arg("key1"), py::arg("A1"), py::arg("b"), py::arg("model"))
        .def("add",[](gtsam::GaussianFactorGraph* self, size_t key1, const gtsam::Matrix& A1, size_t key2, const gtsam::Matrix& A2, const gtsam::Vector& b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model){ self->add(key1, A1, key2, A2, b, model);}, py::arg("key1"), py::arg("A1"), py::arg("key2"), py::arg("A2"), py::arg("b"), py::arg("model"))
        .def("add",[](gtsam::GaussianFactorGraph* self, size_t key1, const gtsam::Matrix& A1, size_t key2, const gtsam::Matrix& A2, size_t key3, const gtsam::Matrix& A3, const gtsam::Vector& b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model){ self->add(key1, A1, key2, A2, key3, A3, b, model);}, py::arg("key1"), py::arg("A1"), py::arg("key2"), py::arg("A2"), py::arg("key3"), py::arg("A3"), py::arg("b"), py::arg("model"))
        .def("error",[](gtsam::GaussianFactorGraph* self, const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("probPrime",[](gtsam::GaussianFactorGraph* self, const gtsam::VectorValues& c){return self->probPrime(c);}, py::arg("c"))
        .def("printErrors",[](gtsam::GaussianFactorGraph* self, const gtsam::VectorValues& c, string str, const gtsam::KeyFormatter& keyFormatter){ self->printErrors(c, str, keyFormatter);}, py::arg("c"), py::arg("str") = "GaussianFactorGraph: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("clone",[](gtsam::GaussianFactorGraph* self){return self->clone();})
        .def("negate",[](gtsam::GaussianFactorGraph* self){return self->negate();})
        .def("optimize",[](gtsam::GaussianFactorGraph* self){return self->optimize();})
        .def("optimizeDensely",[](gtsam::GaussianFactorGraph* self){return self->optimizeDensely();})
        .def("optimize",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->optimize(ordering);}, py::arg("ordering"))
        .def("optimizeGradientSearch",[](gtsam::GaussianFactorGraph* self){return self->optimizeGradientSearch();})
        .def("gradient",[](gtsam::GaussianFactorGraph* self, const gtsam::VectorValues& x0){return self->gradient(x0);}, py::arg("x0"))
        .def("gradientAtZero",[](gtsam::GaussianFactorGraph* self){return self->gradientAtZero();})
        .def("eliminateSequential",[](gtsam::GaussianFactorGraph* self){return self->eliminateSequential();})
        .def("eliminateSequential",[](gtsam::GaussianFactorGraph* self, gtsam::Ordering::OrderingType type){return self->eliminateSequential(type);}, py::arg("type"))
        .def("eliminateSequential",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminateSequential(ordering);}, py::arg("ordering"))
        .def("eliminateMultifrontal",[](gtsam::GaussianFactorGraph* self){return self->eliminateMultifrontal();})
        .def("eliminateMultifrontal",[](gtsam::GaussianFactorGraph* self, gtsam::Ordering::OrderingType type){return self->eliminateMultifrontal(type);}, py::arg("type"))
        .def("eliminateMultifrontal",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminateMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminatePartialSequential(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::GaussianFactorGraph* self, const gtsam::KeyVector& keys){return self->eliminatePartialSequential(keys);}, py::arg("keys"))
        .def("eliminatePartialMultifrontal",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminatePartialMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialMultifrontal",[](gtsam::GaussianFactorGraph* self, const gtsam::KeyVector& keys){return self->eliminatePartialMultifrontal(keys);}, py::arg("keys"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->marginalMultifrontalBayesNet(ordering);}, py::arg("ordering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self, const gtsam::KeyVector& key_vector){return self->marginalMultifrontalBayesNet(key_vector);}, py::arg("key_vector"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering, const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(ordering, marginalizedVariableOrdering);}, py::arg("ordering"), py::arg("marginalizedVariableOrdering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self, const gtsam::KeyVector& key_vector, const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(key_vector, marginalizedVariableOrdering);}, py::arg("key_vector"), py::arg("marginalizedVariableOrdering"))
        .def("marginal",[](gtsam::GaussianFactorGraph* self, const gtsam::KeyVector& key_vector){return self->marginal(key_vector);}, py::arg("key_vector"))
        .def("sparseJacobian_",[](gtsam::GaussianFactorGraph* self){return self->sparseJacobian_();})
        .def("augmentedJacobian",[](gtsam::GaussianFactorGraph* self){return self->augmentedJacobian();})
        .def("augmentedJacobian",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->augmentedJacobian(ordering);}, py::arg("ordering"))
        .def("jacobian",[](gtsam::GaussianFactorGraph* self){return self->jacobian();})
        .def("jacobian",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->jacobian(ordering);}, py::arg("ordering"))
        .def("augmentedHessian",[](gtsam::GaussianFactorGraph* self){return self->augmentedHessian();})
        .def("augmentedHessian",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->augmentedHessian(ordering);}, py::arg("ordering"))
        .def("hessian",[](gtsam::GaussianFactorGraph* self){return self->hessian();})
        .def("hessian",[](gtsam::GaussianFactorGraph* self, const gtsam::Ordering& ordering){return self->hessian(ordering);}, py::arg("ordering"))
        .def("dot",[](gtsam::GaussianFactorGraph* self, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){return self->dot(keyFormatter, writer);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("saveGraph",[](gtsam::GaussianFactorGraph* self, string s, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){ self->saveGraph(s, keyFormatter, writer);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("serialize", [](gtsam::GaussianFactorGraph* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::GaussianFactorGraph* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::GaussianFactorGraph &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::GaussianFactorGraph obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::GaussianConditional, gtsam::JacobianFactor, boost::shared_ptr<gtsam::GaussianConditional>>(m_, "GaussianConditional")
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("sigmas"))
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&, size_t, const gtsam::Matrix&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"), py::arg("sigmas"))
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&, size_t, const gtsam::Matrix&, size_t, const gtsam::Matrix&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"), py::arg("name2"), py::arg("T"), py::arg("sigmas"))
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&>(), py::arg("key"), py::arg("d"), py::arg("R"))
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&, size_t, const gtsam::Matrix&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"))
        .def(py::init<size_t, const gtsam::Vector&, const gtsam::Matrix&, size_t, const gtsam::Matrix&, size_t, const gtsam::Matrix&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"), py::arg("name2"), py::arg("T"))
        .def("print",[](gtsam::GaussianConditional* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "GaussianConditional", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GaussianConditional& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "GaussianConditional", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GaussianConditional* self, const gtsam::GaussianConditional& cg, double tol){return self->equals(cg, tol);}, py::arg("cg"), py::arg("tol"))
        .def("logNormalizationConstant",[](gtsam::GaussianConditional* self){return self->logNormalizationConstant();})
        .def("logProbability",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& x){return self->logProbability(x);}, py::arg("x"))
        .def("evaluate",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& x){return self->evaluate(x);}, py::arg("x"))
        .def("error",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("firstFrontalKey",[](gtsam::GaussianConditional* self){return self->firstFrontalKey();})
        .def("solve",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& parents){return self->solve(parents);}, py::arg("parents"))
        .def("likelihood",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& frontalValues){return self->likelihood(frontalValues);}, py::arg("frontalValues"))
        .def("likelihood",[](gtsam::GaussianConditional* self, const gtsam::Vector& frontal){return self->likelihood(frontal);}, py::arg("frontal"))
        .def("sample",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& parents){return self->sample(parents);}, py::arg("parents"))
        .def("sample",[](gtsam::GaussianConditional* self){return self->sample();})
        .def("solveOtherRHS",[](gtsam::GaussianConditional* self, const gtsam::VectorValues& parents, const gtsam::VectorValues& rhs){return self->solveOtherRHS(parents, rhs);}, py::arg("parents"), py::arg("rhs"))
        .def("solveTransposeInPlace",[](gtsam::GaussianConditional* self, gtsam::VectorValues& gy){ self->solveTransposeInPlace(gy);}, py::arg("gy"))
        .def("R",[](gtsam::GaussianConditional* self){return self->R();})
        .def("S",[](gtsam::GaussianConditional* self){return self->S();})
        .def("d",[](gtsam::GaussianConditional* self){return self->d();})
        .def("serialize", [](gtsam::GaussianConditional* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::GaussianConditional* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::GaussianConditional &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::GaussianConditional obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("logProbability",[](gtsam::GaussianConditional* self, const gtsam::HybridValues& x){return self->logProbability(x);}, py::arg("x"))
        .def("evaluate",[](gtsam::GaussianConditional* self, const gtsam::HybridValues& x){return self->evaluate(x);}, py::arg("x"))
        .def("error",[](gtsam::GaussianConditional* self, const gtsam::HybridValues& x){return self->error(x);}, py::arg("x"))
        .def_static("FromMeanAndStddev",[](gtsam::Key key, const gtsam::Vector& mu, double sigma){return gtsam::GaussianConditional::FromMeanAndStddev(key, mu, sigma);}, py::arg("key"), py::arg("mu"), py::arg("sigma"))
        .def_static("FromMeanAndStddev",[](gtsam::Key key, const gtsam::Matrix& A, gtsam::Key parent, const gtsam::Vector& b, double sigma){return gtsam::GaussianConditional::FromMeanAndStddev(key, A, parent, b, sigma);}, py::arg("key"), py::arg("A"), py::arg("parent"), py::arg("b"), py::arg("sigma"))
        .def_static("FromMeanAndStddev",[](gtsam::Key key, const gtsam::Matrix& A1, gtsam::Key parent1, const gtsam::Matrix& A2, gtsam::Key parent2, const gtsam::Vector& b, double sigma){return gtsam::GaussianConditional::FromMeanAndStddev(key, A1, parent1, A2, parent2, b, sigma);}, py::arg("key"), py::arg("A1"), py::arg("parent1"), py::arg("A2"), py::arg("parent2"), py::arg("b"), py::arg("sigma"));

    py::class_<gtsam::GaussianDensity, gtsam::GaussianConditional, boost::shared_ptr<gtsam::GaussianDensity>>(m_, "GaussianDensity")
        .def(py::init<gtsam::Key, const gtsam::Vector&, const gtsam::Matrix&, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("sigmas"))
        .def("print",[](gtsam::GaussianDensity* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "GaussianDensity", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GaussianDensity& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "GaussianDensity", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GaussianDensity* self, const gtsam::GaussianDensity& cg, double tol){return self->equals(cg, tol);}, py::arg("cg"), py::arg("tol"))
        .def("mean",[](gtsam::GaussianDensity* self){return self->mean();})
        .def("covariance",[](gtsam::GaussianDensity* self){return self->covariance();})
        .def_static("FromMeanAndStddev",[](gtsam::Key key, const gtsam::Vector& mean, double sigma){return gtsam::GaussianDensity::FromMeanAndStddev(key, mean, sigma);}, py::arg("key"), py::arg("mean"), py::arg("sigma"));

    py::class_<gtsam::GaussianBayesNet, boost::shared_ptr<gtsam::GaussianBayesNet>>(m_, "GaussianBayesNet")
        .def(py::init<>())
        .def(py::init<const boost::shared_ptr<gtsam::GaussianConditional>>(), py::arg("conditional"))
        .def("print",[](gtsam::GaussianBayesNet* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GaussianBayesNet& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::GaussianBayesNet* self, const gtsam::GaussianBayesNet& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("size",[](gtsam::GaussianBayesNet* self){return self->size();})
        .def("push_back",[](gtsam::GaussianBayesNet* self, boost::shared_ptr<gtsam::GaussianConditional> conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::GaussianBayesNet* self, const gtsam::GaussianBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("front",[](gtsam::GaussianBayesNet* self){return self->front();})
        .def("back",[](gtsam::GaussianBayesNet* self){return self->back();})
        .def("logProbability",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& x){return self->logProbability(x);}, py::arg("x"))
        .def("evaluate",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& x){return self->evaluate(x);}, py::arg("x"))
        .def("error",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("optimize",[](gtsam::GaussianBayesNet* self){return self->optimize();})
        .def("optimize",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& given){return self->optimize(given);}, py::arg("given"))
        .def("optimizeGradientSearch",[](gtsam::GaussianBayesNet* self){return self->optimizeGradientSearch();})
        .def("sample",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& given){return self->sample(given);}, py::arg("given"))
        .def("sample",[](gtsam::GaussianBayesNet* self){return self->sample();})
        .def("backSubstitute",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& gx){return self->backSubstitute(gx);}, py::arg("gx"))
        .def("backSubstituteTranspose",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& gx){return self->backSubstituteTranspose(gx);}, py::arg("gx"))
        .def("at",[](gtsam::GaussianBayesNet* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::GaussianBayesNet* self){return self->keys();})
        .def("keyVector",[](gtsam::GaussianBayesNet* self){return self->keyVector();})
        .def("exists",[](gtsam::GaussianBayesNet* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("saveGraph",[](gtsam::GaussianBayesNet* self, const string& s){ self->saveGraph(s);}, py::arg("s"))
        .def("matrix",[](gtsam::GaussianBayesNet* self){return self->matrix();})
        .def("gradient",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& x0){return self->gradient(x0);}, py::arg("x0"))
        .def("gradientAtZero",[](gtsam::GaussianBayesNet* self){return self->gradientAtZero();})
        .def("error",[](gtsam::GaussianBayesNet* self, const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("determinant",[](gtsam::GaussianBayesNet* self){return self->determinant();})
        .def("logDeterminant",[](gtsam::GaussianBayesNet* self){return self->logDeterminant();})
        .def("dot",[](gtsam::GaussianBayesNet* self, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){return self->dot(keyFormatter, writer);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("saveGraph",[](gtsam::GaussianBayesNet* self, string s, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){ self->saveGraph(s, keyFormatter, writer);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter());

    py::class_<gtsam::GaussianBayesTree, boost::shared_ptr<gtsam::GaussianBayesTree>>(m_, "GaussianBayesTree")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianBayesTree&>(), py::arg("other"))
        .def("equals",[](gtsam::GaussianBayesTree* self, const gtsam::GaussianBayesTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print",[](gtsam::GaussianBayesTree* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::GaussianBayesTree& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("size",[](gtsam::GaussianBayesTree* self){return self->size();})
        .def("empty",[](gtsam::GaussianBayesTree* self){return self->empty();})
        .def("numCachedSeparatorMarginals",[](gtsam::GaussianBayesTree* self){return self->numCachedSeparatorMarginals();})
        .def("dot",[](gtsam::GaussianBayesTree* self, const gtsam::KeyFormatter& keyFormatter){return self->dot(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("saveGraph",[](gtsam::GaussianBayesTree* self, string s, const gtsam::KeyFormatter& keyFormatter){ self->saveGraph(s, keyFormatter);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("optimize",[](gtsam::GaussianBayesTree* self){return self->optimize();})
        .def("optimizeGradientSearch",[](gtsam::GaussianBayesTree* self){return self->optimizeGradientSearch();})
        .def("gradient",[](gtsam::GaussianBayesTree* self, const gtsam::VectorValues& x0){return self->gradient(x0);}, py::arg("x0"))
        .def("gradientAtZero",[](gtsam::GaussianBayesTree* self){return self->gradientAtZero();})
        .def("error",[](gtsam::GaussianBayesTree* self, const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("determinant",[](gtsam::GaussianBayesTree* self){return self->determinant();})
        .def("logDeterminant",[](gtsam::GaussianBayesTree* self){return self->logDeterminant();})
        .def("marginalCovariance",[](gtsam::GaussianBayesTree* self, size_t key){return self->marginalCovariance(key);}, py::arg("key"))
        .def("marginalFactor",[](gtsam::GaussianBayesTree* self, size_t key){return self->marginalFactor(key);}, py::arg("key"))
        .def("joint",[](gtsam::GaussianBayesTree* self, size_t key1, size_t key2){return self->joint(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("jointBayesNet",[](gtsam::GaussianBayesTree* self, size_t key1, size_t key2){return self->jointBayesNet(key1, key2);}, py::arg("key1"), py::arg("key2"));

    py::class_<gtsam::GaussianISAM, boost::shared_ptr<gtsam::GaussianISAM>>(m_, "GaussianISAM")
        .def(py::init<>())
        .def("update",[](gtsam::GaussianISAM* self, const gtsam::GaussianFactorGraph& newFactors){ self->update(newFactors);}, py::arg("newFactors"))
        .def("saveGraph",[](gtsam::GaussianISAM* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("clear",[](gtsam::GaussianISAM* self){ self->clear();});

    py::class_<gtsam::IterativeOptimizationParameters, boost::shared_ptr<gtsam::IterativeOptimizationParameters>>(m_, "IterativeOptimizationParameters")
        .def("getVerbosity",[](gtsam::IterativeOptimizationParameters* self){return self->getVerbosity();})
        .def("setVerbosity",[](gtsam::IterativeOptimizationParameters* self, string s){ self->setVerbosity(s);}, py::arg("s"));

    py::class_<gtsam::ConjugateGradientParameters, gtsam::IterativeOptimizationParameters, boost::shared_ptr<gtsam::ConjugateGradientParameters>>(m_, "ConjugateGradientParameters")
        .def(py::init<>())
        .def("getMinIterations",[](gtsam::ConjugateGradientParameters* self){return self->getMinIterations();})
        .def("getMaxIterations",[](gtsam::ConjugateGradientParameters* self){return self->getMaxIterations();})
        .def("getReset",[](gtsam::ConjugateGradientParameters* self){return self->getReset();})
        .def("getEpsilon_rel",[](gtsam::ConjugateGradientParameters* self){return self->getEpsilon_rel();})
        .def("getEpsilon_abs",[](gtsam::ConjugateGradientParameters* self){return self->getEpsilon_abs();})
        .def("setMinIterations",[](gtsam::ConjugateGradientParameters* self, int value){ self->setMinIterations(value);}, py::arg("value"))
        .def("setMaxIterations",[](gtsam::ConjugateGradientParameters* self, int value){ self->setMaxIterations(value);}, py::arg("value"))
        .def("setReset",[](gtsam::ConjugateGradientParameters* self, int value){ self->setReset(value);}, py::arg("value"))
        .def("setEpsilon_rel",[](gtsam::ConjugateGradientParameters* self, double value){ self->setEpsilon_rel(value);}, py::arg("value"))
        .def("setEpsilon_abs",[](gtsam::ConjugateGradientParameters* self, double value){ self->setEpsilon_abs(value);}, py::arg("value"));

    py::class_<gtsam::PreconditionerParameters, boost::shared_ptr<gtsam::PreconditionerParameters>>(m_, "PreconditionerParameters")
        .def(py::init<>());

    py::class_<gtsam::DummyPreconditionerParameters, gtsam::PreconditionerParameters, boost::shared_ptr<gtsam::DummyPreconditionerParameters>>(m_, "DummyPreconditionerParameters")
        .def(py::init<>());

    py::class_<gtsam::BlockJacobiPreconditionerParameters, gtsam::PreconditionerParameters, boost::shared_ptr<gtsam::BlockJacobiPreconditionerParameters>>(m_, "BlockJacobiPreconditionerParameters")
        .def(py::init<>());

    py::class_<gtsam::PCGSolverParameters, gtsam::ConjugateGradientParameters, boost::shared_ptr<gtsam::PCGSolverParameters>>(m_, "PCGSolverParameters")
        .def(py::init<>())
        .def("print",[](gtsam::PCGSolverParameters* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::PCGSolverParameters& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("setPreconditionerParams",[](gtsam::PCGSolverParameters* self, boost::shared_ptr<gtsam::PreconditionerParameters> preconditioner){ self->setPreconditionerParams(preconditioner);}, py::arg("preconditioner"));

    py::class_<gtsam::SubgraphSolverParameters, gtsam::ConjugateGradientParameters, boost::shared_ptr<gtsam::SubgraphSolverParameters>>(m_, "SubgraphSolverParameters")
        .def(py::init<>());

    py::class_<gtsam::SubgraphSolver, boost::shared_ptr<gtsam::SubgraphSolver>>(m_, "SubgraphSolver")
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::SubgraphSolverParameters&, const gtsam::Ordering&>(), py::arg("A"), py::arg("parameters"), py::arg("ordering"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::GaussianFactorGraph&, const gtsam::SubgraphSolverParameters&, const gtsam::Ordering&>(), py::arg("Ab1"), py::arg("Ab2"), py::arg("parameters"), py::arg("ordering"))
        .def("optimize",[](gtsam::SubgraphSolver* self){return self->optimize();});

    py::class_<gtsam::KalmanFilter, boost::shared_ptr<gtsam::KalmanFilter>>(m_, "KalmanFilter")
        .def(py::init<size_t>(), py::arg("n"))
        .def("init",[](gtsam::KalmanFilter* self, const gtsam::Vector& x0, const gtsam::Matrix& P0){return self->init(x0, P0);}, py::arg("x0"), py::arg("P0"))
        .def("print",[](gtsam::KalmanFilter* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::KalmanFilter& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("predict",[](gtsam::KalmanFilter* self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix& F, const gtsam::Matrix& B, const gtsam::Vector& u, const boost::shared_ptr<gtsam::noiseModel::Diagonal> modelQ){return self->predict(p, F, B, u, modelQ);}, py::arg("p"), py::arg("F"), py::arg("B"), py::arg("u"), py::arg("modelQ"))
        .def("predictQ",[](gtsam::KalmanFilter* self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix& F, const gtsam::Matrix& B, const gtsam::Vector& u, const gtsam::Matrix& Q){return self->predictQ(p, F, B, u, Q);}, py::arg("p"), py::arg("F"), py::arg("B"), py::arg("u"), py::arg("Q"))
        .def("predict2",[](gtsam::KalmanFilter* self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix& A0, const gtsam::Matrix& A1, const gtsam::Vector& b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model){return self->predict2(p, A0, A1, b, model);}, py::arg("p"), py::arg("A0"), py::arg("A1"), py::arg("b"), py::arg("model"))
        .def("update",[](gtsam::KalmanFilter* self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix& H, const gtsam::Vector& z, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model){return self->update(p, H, z, model);}, py::arg("p"), py::arg("H"), py::arg("z"), py::arg("model"))
        .def("updateQ",[](gtsam::KalmanFilter* self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix& H, const gtsam::Vector& z, const gtsam::Matrix& Q){return self->updateQ(p, H, z, Q);}, py::arg("p"), py::arg("H"), py::arg("z"), py::arg("Q"))
        .def_static("step",[](boost::shared_ptr<gtsam::GaussianDensity> p){return gtsam::KalmanFilter::step(p);}, py::arg("p"));


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/linear.h"

}

