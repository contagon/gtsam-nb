/**
 * @file    linear.cpp
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

#include "utils/boost_shared_ptr.h"

// These are the included headers listed in `gtsam.i`
#include "gtsam/hybrid/HybridValues.h"
#include "gtsam/linear/ConjugateGradientSolver.h"
#include "gtsam/linear/GaussianBayesNet.h"
#include "gtsam/linear/GaussianBayesTree.h"
#include "gtsam/linear/GaussianConditional.h"
#include "gtsam/linear/GaussianDensity.h"
#include "gtsam/linear/GaussianFactor.h"
#include "gtsam/linear/GaussianFactorGraph.h"
#include "gtsam/linear/GaussianISAM.h"
#include "gtsam/linear/HessianFactor.h"
#include "gtsam/linear/IterativeSolver.h"
#include "gtsam/linear/JacobianFactor.h"
#include "gtsam/linear/KalmanFilter.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/linear/PCGSolver.h"
#include "gtsam/linear/Preconditioner.h"
#include "gtsam/linear/Sampler.h"
#include "gtsam/linear/SubgraphSolver.h"
#include "gtsam/linear/VectorValues.h"

using namespace std;

namespace nb = nanobind;

void linear(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of linear";

  auto m_noiseModel = m_.def_submodule("noiseModel", "noiseModel submodule");

  nb::class_<gtsam::noiseModel::Base>(m_noiseModel, "Base")
      .def("print", [](gtsam::noiseModel::Base *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::noiseModel::Base &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::noiseModel::Gaussian, gtsam::noiseModel::Base>(m_noiseModel, "Gaussian")
      .def("equals", [](gtsam::noiseModel::Gaussian *self, gtsam::noiseModel::Base &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("R", [](gtsam::noiseModel::Gaussian *self) { return self->R(); })
      .def("information", [](gtsam::noiseModel::Gaussian *self) { return self->information(); })
      .def("covariance", [](gtsam::noiseModel::Gaussian *self) { return self->covariance(); })
      .def("whiten", [](gtsam::noiseModel::Gaussian *self, const gtsam::Vector &v) { return self->whiten(v); }, nb::arg("v"))
      .def("unwhiten", [](gtsam::noiseModel::Gaussian *self, const gtsam::Vector &v) { return self->unwhiten(v); }, nb::arg("v"))
      .def("Whiten", [](gtsam::noiseModel::Gaussian *self, const gtsam::Matrix &H) { return self->Whiten(H); }, nb::arg("H"))
      // .def("serialize", [](gtsam::noiseModel::Gaussian *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::Gaussian *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::Gaussian &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::Gaussian obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Information", [](const gtsam::Matrix &R, bool smart) { return gtsam::noiseModel::Gaussian::Information(R, smart); }, nb::arg("R"), nb::arg("smart") = true)
      .def_static("SqrtInformation", [](const gtsam::Matrix &R, bool smart) { return gtsam::noiseModel::Gaussian::SqrtInformation(R, smart); }, nb::arg("R"), nb::arg("smart") = true)
      .def_static("Covariance", [](const gtsam::Matrix &R, bool smart) { return gtsam::noiseModel::Gaussian::Covariance(R, smart); }, nb::arg("R"), nb::arg("smart") = true);

  nb::class_<gtsam::noiseModel::Diagonal, gtsam::noiseModel::Gaussian>(m_noiseModel, "Diagonal")
      .def("R", [](gtsam::noiseModel::Diagonal *self) { return self->R(); })
      .def("sigmas", [](gtsam::noiseModel::Diagonal *self) { return self->sigmas(); })
      .def("invsigmas", [](gtsam::noiseModel::Diagonal *self) { return self->invsigmas(); })
      .def("precisions", [](gtsam::noiseModel::Diagonal *self) { return self->precisions(); })
      // .def("serialize", [](gtsam::noiseModel::Diagonal *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::Diagonal *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::Diagonal &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::Diagonal obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Sigmas", [](const gtsam::Vector &sigmas, bool smart) { return gtsam::noiseModel::Diagonal::Sigmas(sigmas, smart); }, nb::arg("sigmas"), nb::arg("smart") = true)
      .def_static("Variances", [](const gtsam::Vector &variances, bool smart) { return gtsam::noiseModel::Diagonal::Variances(variances, smart); }, nb::arg("variances"), nb::arg("smart") = true)
      .def_static("Precisions", [](const gtsam::Vector &precisions, bool smart) { return gtsam::noiseModel::Diagonal::Precisions(precisions, smart); }, nb::arg("precisions"), nb::arg("smart") = true);

  nb::class_<gtsam::noiseModel::Constrained, gtsam::noiseModel::Diagonal>(m_noiseModel, "Constrained")
      .def("unit", [](gtsam::noiseModel::Constrained *self) { return self->unit(); })
      // .def("serialize", [](gtsam::noiseModel::Constrained *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::Constrained *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::Constrained &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::Constrained obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("MixedSigmas", [](const gtsam::Vector &mu, const gtsam::Vector &sigmas) { return gtsam::noiseModel::Constrained::MixedSigmas(mu, sigmas); }, nb::arg("mu"), nb::arg("sigmas"))
      .def_static("MixedSigmas", [](double m, const gtsam::Vector &sigmas) { return gtsam::noiseModel::Constrained::MixedSigmas(m, sigmas); }, nb::arg("m"), nb::arg("sigmas"))
      .def_static("MixedVariances", [](const gtsam::Vector &mu, const gtsam::Vector &variances) { return gtsam::noiseModel::Constrained::MixedVariances(mu, variances); }, nb::arg("mu"), nb::arg("variances"))
      .def_static("MixedVariances", [](const gtsam::Vector &variances) { return gtsam::noiseModel::Constrained::MixedVariances(variances); }, nb::arg("variances"))
      .def_static("MixedPrecisions", [](const gtsam::Vector &mu, const gtsam::Vector &precisions) { return gtsam::noiseModel::Constrained::MixedPrecisions(mu, precisions); }, nb::arg("mu"), nb::arg("precisions"))
      .def_static("MixedPrecisions", [](const gtsam::Vector &precisions) { return gtsam::noiseModel::Constrained::MixedPrecisions(precisions); }, nb::arg("precisions"))
      .def_static("All", [](size_t dim) { return gtsam::noiseModel::Constrained::All(dim); }, nb::arg("dim"))
      .def_static("All", [](size_t dim, double mu) { return gtsam::noiseModel::Constrained::All(dim, mu); }, nb::arg("dim"), nb::arg("mu"));

  nb::class_<gtsam::noiseModel::Isotropic, gtsam::noiseModel::Diagonal>(m_noiseModel, "Isotropic")
      .def("sigma", [](gtsam::noiseModel::Isotropic *self) { return self->sigma(); })
      // .def("serialize", [](gtsam::noiseModel::Isotropic *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::Isotropic *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::Isotropic &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::Isotropic obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Sigma", [](size_t dim, double sigma, bool smart) { return gtsam::noiseModel::Isotropic::Sigma(dim, sigma, smart); }, nb::arg("dim"), nb::arg("sigma"), nb::arg("smart") = true)
      .def_static("Variance", [](size_t dim, double varianace, bool smart) { return gtsam::noiseModel::Isotropic::Variance(dim, varianace, smart); }, nb::arg("dim"), nb::arg("varianace"), nb::arg("smart") = true)
      .def_static("Precision", [](size_t dim, double precision, bool smart) { return gtsam::noiseModel::Isotropic::Precision(dim, precision, smart); }, nb::arg("dim"), nb::arg("precision"), nb::arg("smart") = true);

  nb::class_<gtsam::noiseModel::Unit, gtsam::noiseModel::Isotropic>(m_noiseModel, "Unit")
      // .def("serialize", [](gtsam::noiseModel::Unit *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::Unit *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::Unit &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::Unit obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Create", [](size_t dim) { return gtsam::noiseModel::Unit::Create(dim); }, nb::arg("dim"));
  auto m_noiseModel_mEstimator = m_noiseModel.def_submodule("mEstimator", "mEstimator submodule");

  nb::class_<gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Base")
      .def("print", [](gtsam::noiseModel::mEstimator::Base *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::noiseModel::mEstimator::Base &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::noiseModel::mEstimator::Null, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Null")
      .def(nb::init<>())
      // .def("serialize", [](gtsam::noiseModel::mEstimator::Null *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::Null *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::Null &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::Null obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::Null *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::Null *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", []() { return gtsam::noiseModel::mEstimator::Null::Create(); });

  nb::class_<gtsam::noiseModel::mEstimator::Fair, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Fair")
      .def(nb::init<double>(), nb::arg("c"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::Fair *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::Fair *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::Fair &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::Fair obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::Fair *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::Fair *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double c) { return gtsam::noiseModel::mEstimator::Fair::Create(c); }, nb::arg("c"));

  nb::class_<gtsam::noiseModel::mEstimator::Huber, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Huber")
      .def(nb::init<double>(), nb::arg("k"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::Huber *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::Huber *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::Huber &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::Huber obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::Huber *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::Huber *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double k) { return gtsam::noiseModel::mEstimator::Huber::Create(k); }, nb::arg("k"));

  nb::class_<gtsam::noiseModel::mEstimator::Cauchy, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Cauchy")
      .def(nb::init<double>(), nb::arg("k"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::Cauchy *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::Cauchy *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::Cauchy &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::Cauchy obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::Cauchy *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::Cauchy *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double k) { return gtsam::noiseModel::mEstimator::Cauchy::Create(k); }, nb::arg("k"));

  nb::class_<gtsam::noiseModel::mEstimator::Tukey, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Tukey")
      .def(nb::init<double>(), nb::arg("k"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::Tukey *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::Tukey *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::Tukey &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::Tukey obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::Tukey *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::Tukey *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double k) { return gtsam::noiseModel::mEstimator::Tukey::Create(k); }, nb::arg("k"));

  nb::class_<gtsam::noiseModel::mEstimator::Welsch, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "Welsch")
      .def(nb::init<double>(), nb::arg("k"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::Welsch *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::Welsch *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::Welsch &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::Welsch obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::Welsch *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::Welsch *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double k) { return gtsam::noiseModel::mEstimator::Welsch::Create(k); }, nb::arg("k"));

  nb::class_<gtsam::noiseModel::mEstimator::GemanMcClure, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "GemanMcClure")
      .def(nb::init<double>(), nb::arg("c"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::GemanMcClure *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::GemanMcClure *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::GemanMcClure &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::GemanMcClure obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::GemanMcClure *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::GemanMcClure *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double c) { return gtsam::noiseModel::mEstimator::GemanMcClure::Create(c); }, nb::arg("c"));

  nb::class_<gtsam::noiseModel::mEstimator::DCS, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "DCS")
      .def(nb::init<double>(), nb::arg("c"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::DCS *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::DCS *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::DCS &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::DCS obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::DCS *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::DCS *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double c) { return gtsam::noiseModel::mEstimator::DCS::Create(c); }, nb::arg("c"));

  nb::class_<gtsam::noiseModel::mEstimator::L2WithDeadZone, gtsam::noiseModel::mEstimator::Base>(m_noiseModel_mEstimator, "L2WithDeadZone")
      .def(nb::init<double>(), nb::arg("k"))
      // .def("serialize", [](gtsam::noiseModel::mEstimator::L2WithDeadZone *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::mEstimator::L2WithDeadZone *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::mEstimator::L2WithDeadZone &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::mEstimator::L2WithDeadZone obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("weight", [](gtsam::noiseModel::mEstimator::L2WithDeadZone *self, double error) { return self->weight(error); }, nb::arg("error"))
      .def("loss", [](gtsam::noiseModel::mEstimator::L2WithDeadZone *self, double error) { return self->loss(error); }, nb::arg("error"))
      .def_static("Create", [](double k) { return gtsam::noiseModel::mEstimator::L2WithDeadZone::Create(k); }, nb::arg("k"));

  nb::class_<gtsam::noiseModel::Robust, gtsam::noiseModel::Base>(m_noiseModel, "Robust")
      .def(nb::init<const boost::shared_ptr<gtsam::noiseModel::mEstimator::Base>, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("robust"), nb::arg("noise"))
      // .def("serialize", [](gtsam::noiseModel::Robust *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::noiseModel::Robust *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::noiseModel::Robust &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::noiseModel::Robust obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Create", [](const boost::shared_ptr<gtsam::noiseModel::mEstimator::Base> robust, const boost::shared_ptr<gtsam::noiseModel::Base> noise) { return gtsam::noiseModel::Robust::Create(robust, noise); }, nb::arg("robust"), nb::arg("noise"));

  nb::class_<gtsam::Sampler>(m_, "Sampler")
      .def(nb::init<boost::shared_ptr<gtsam::noiseModel::Diagonal>, int>(), nb::arg("model"), nb::arg("seed"))
      .def(nb::init<const gtsam::Vector &, int>(), nb::arg("sigmas"), nb::arg("seed"))
      .def("dim", [](gtsam::Sampler *self) { return self->dim(); })
      .def("sigmas", [](gtsam::Sampler *self) { return self->sigmas(); })
      .def("model", [](gtsam::Sampler *self) { return self->model(); })
      .def("sample", [](gtsam::Sampler *self) { return self->sample(); });

  nb::class_<gtsam::VectorValues>(m_, "VectorValues")
      .def(nb::init<>())
      .def(nb::init<const gtsam::VectorValues &>(), nb::arg("other"))
      .def(nb::init<const gtsam::VectorValues &, const gtsam::VectorValues &>(), nb::arg("first"), nb::arg("second"))
      .def("size", [](gtsam::VectorValues *self) { return self->size(); })
      .def("dim", [](gtsam::VectorValues *self, size_t j) { return self->dim(j); }, nb::arg("j"))
      .def("exists", [](gtsam::VectorValues *self, size_t j) { return self->exists(j); }, nb::arg("j"))
      .def("print", [](gtsam::VectorValues *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "VectorValues", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::VectorValues &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "VectorValues", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::VectorValues *self, const gtsam::VectorValues &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("insert", [](gtsam::VectorValues *self, size_t j, const gtsam::Vector &value) { self->insert(j, value); }, nb::arg("j"), nb::arg("value"))
      .def("vector", [](gtsam::VectorValues *self) { return self->vector(); })
      .def("vector", [](gtsam::VectorValues *self, const gtsam::KeyVector &keys) { return self->vector(keys); }, nb::arg("keys"))
      .def("at", [](gtsam::VectorValues *self, size_t j) { return self->at(j); }, nb::arg("j"))
      .def("insert", [](gtsam::VectorValues *self, const gtsam::VectorValues &values) { self->insert(values); }, nb::arg("values"))
      .def("update", [](gtsam::VectorValues *self, const gtsam::VectorValues &values) { self->update(values); }, nb::arg("values"))
      .def("setZero", [](gtsam::VectorValues *self) { self->setZero(); })
      .def("add", [](gtsam::VectorValues *self, const gtsam::VectorValues &c) { return self->add(c); }, nb::arg("c"))
      .def("addInPlace", [](gtsam::VectorValues *self, const gtsam::VectorValues &c) { self->addInPlace(c); }, nb::arg("c"))
      .def("subtract", [](gtsam::VectorValues *self, const gtsam::VectorValues &c) { return self->subtract(c); }, nb::arg("c"))
      .def("scale", [](gtsam::VectorValues *self, double a) { return self->scale(a); }, nb::arg("a"))
      .def("scaleInPlace", [](gtsam::VectorValues *self, double a) { self->scaleInPlace(a); }, nb::arg("a"))
      .def("hasSameStructure", [](gtsam::VectorValues *self, const gtsam::VectorValues &other) { return self->hasSameStructure(other); }, nb::arg("other"))
      .def("dot", [](gtsam::VectorValues *self, const gtsam::VectorValues &V) { return self->dot(V); }, nb::arg("V"))
      .def("norm", [](gtsam::VectorValues *self) { return self->norm(); })
      .def("squaredNorm", [](gtsam::VectorValues *self) { return self->squaredNorm(); })
      // .def("serialize", [](gtsam::VectorValues *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::VectorValues *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::VectorValues &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::VectorValues obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("_repr_html_", [](gtsam::VectorValues *self) { return self->html(); })
      .def_static("Zero", [](const gtsam::VectorValues &model) { return gtsam::VectorValues::Zero(model); }, nb::arg("model"));

  nb::class_<gtsam::GaussianFactor, gtsam::Factor>(m_, "GaussianFactor")
      .def("print", [](gtsam::GaussianFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GaussianFactor *self, const gtsam::GaussianFactor &lf, double tol) { return self->equals(lf, tol); }, nb::arg("lf"), nb::arg("tol"))
      .def("error", [](gtsam::GaussianFactor *self, const gtsam::VectorValues &c) { return self->error(c); }, nb::arg("c"))
      .def("clone", [](gtsam::GaussianFactor *self) { return self->clone(); })
      .def("negate", [](gtsam::GaussianFactor *self) { return self->negate(); })
      .def("augmentedInformation", [](gtsam::GaussianFactor *self) { return self->augmentedInformation(); })
      .def("information", [](gtsam::GaussianFactor *self) { return self->information(); })
      .def("augmentedJacobian", [](gtsam::GaussianFactor *self) { return self->augmentedJacobian(); })
      .def("jacobian", [](gtsam::GaussianFactor *self) { return self->jacobian(); });

  nb::class_<gtsam::JacobianFactor, gtsam::GaussianFactor>(m_, "JacobianFactor")
      .def(nb::init<>())
      .def(nb::init<const gtsam::Vector &>(), nb::arg("b_in"))
      .def(nb::init<size_t, const gtsam::Matrix &, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("i1"), nb::arg("A1"), nb::arg("b"), nb::arg("model"))
      .def(nb::init<size_t, const gtsam::Matrix &, size_t, const gtsam::Matrix &, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("i1"), nb::arg("A1"), nb::arg("i2"), nb::arg("A2"), nb::arg("b"), nb::arg("model"))
      .def(nb::init<size_t, const gtsam::Matrix &, size_t, const gtsam::Matrix &, size_t, const gtsam::Matrix &, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("i1"), nb::arg("A1"), nb::arg("i2"), nb::arg("A2"), nb::arg("i3"), nb::arg("A3"), nb::arg("b"), nb::arg("model"))
      .def(nb::init<const gtsam::GaussianFactorGraph &>(), nb::arg("graph"))
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::VariableSlots &>(), nb::arg("graph"), nb::arg("p_variableSlots"))
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::Ordering &>(), nb::arg("graph"), nb::arg("ordering"))
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::Ordering &, const gtsam::VariableSlots &>(), nb::arg("graph"), nb::arg("ordering"), nb::arg("p_variableSlots"))
      .def(nb::init<const gtsam::GaussianFactor &>(), nb::arg("factor"))
      .def("print", [](gtsam::JacobianFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::JacobianFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::JacobianFactor *self, const gtsam::GaussianFactor &lf, double tol) { return self->equals(lf, tol); }, nb::arg("lf"), nb::arg("tol"))
      .def("unweighted_error", [](gtsam::JacobianFactor *self, const gtsam::VectorValues &c) { return self->unweighted_error(c); }, nb::arg("c"))
      .def("error_vector", [](gtsam::JacobianFactor *self, const gtsam::VectorValues &c) { return self->error_vector(c); }, nb::arg("c"))
      .def("error", [](gtsam::JacobianFactor *self, const gtsam::VectorValues &c) { return self->error(c); }, nb::arg("c"))
      .def("getA", [](gtsam::JacobianFactor *self) { return self->getA(); })
      .def("getb", [](gtsam::JacobianFactor *self) { return self->getb(); })
      .def("rows", [](gtsam::JacobianFactor *self) { return self->rows(); })
      .def("cols", [](gtsam::JacobianFactor *self) { return self->cols(); })
      .def("isConstrained", [](gtsam::JacobianFactor *self) { return self->isConstrained(); })
      .def("jacobianUnweighted", [](gtsam::JacobianFactor *self) { return self->jacobianUnweighted(); })
      .def("augmentedJacobianUnweighted", [](gtsam::JacobianFactor *self) { return self->augmentedJacobianUnweighted(); })
      .def("transposeMultiplyAdd", [](gtsam::JacobianFactor *self, double alpha, const gtsam::Vector &e, gtsam::VectorValues &x) { self->transposeMultiplyAdd(alpha, e, x); }, nb::arg("alpha"), nb::arg("e"), nb::arg("x"))
      .def("whiten", [](gtsam::JacobianFactor *self) { return self->whiten(); })
      .def("eliminate", [](gtsam::JacobianFactor *self, const gtsam::Ordering &keys) { return self->eliminate(keys); }, nb::arg("keys"))
      .def("setModel", [](gtsam::JacobianFactor *self, bool anyConstrained, const gtsam::Vector &sigmas) { self->setModel(anyConstrained, sigmas); }, nb::arg("anyConstrained"), nb::arg("sigmas"))
      .def("get_model", [](gtsam::JacobianFactor *self) { return self->get_model(); });
  // .def("serialize", [](gtsam::JacobianFactor *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::JacobianFactor *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::JacobianFactor &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::JacobianFactor obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::HessianFactor, gtsam::GaussianFactor>(m_, "HessianFactor")
      .def(nb::init<>())
      .def(nb::init<const gtsam::GaussianFactor &>(), nb::arg("factor"))
      .def(nb::init<size_t, const gtsam::Matrix &, const gtsam::Vector &, double>(), nb::arg("j"), nb::arg("G"), nb::arg("g"), nb::arg("f"))
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &>(), nb::arg("j"), nb::arg("mu"), nb::arg("Sigma"))
      .def(nb::init<size_t, size_t, const gtsam::Matrix &, const gtsam::Matrix &, const gtsam::Vector &, const gtsam::Matrix &, const gtsam::Vector &, double>(), nb::arg("j1"), nb::arg("j2"), nb::arg("G11"), nb::arg("G12"), nb::arg("g1"), nb::arg("G22"), nb::arg("g2"), nb::arg("f"))
      .def(nb::init<size_t, size_t, size_t, const gtsam::Matrix &, const gtsam::Matrix &, const gtsam::Matrix &, const gtsam::Vector &, const gtsam::Matrix &, const gtsam::Matrix &, const gtsam::Vector &, const gtsam::Matrix &, const gtsam::Vector &, double>(), nb::arg("j1"), nb::arg("j2"), nb::arg("j3"), nb::arg("G11"), nb::arg("G12"), nb::arg("G13"), nb::arg("g1"), nb::arg("G22"), nb::arg("G23"), nb::arg("g2"), nb::arg("G33"), nb::arg("g3"), nb::arg("f"))
      .def(nb::init<const gtsam::GaussianFactorGraph &>(), nb::arg("factors"))
      .def("print", [](gtsam::HessianFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HessianFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HessianFactor *self, const gtsam::GaussianFactor &lf, double tol) { return self->equals(lf, tol); }, nb::arg("lf"), nb::arg("tol"))
      .def("error", [](gtsam::HessianFactor *self, const gtsam::VectorValues &c) { return self->error(c); }, nb::arg("c"))
      .def("rows", [](gtsam::HessianFactor *self) { return self->rows(); })
      .def("information", [](gtsam::HessianFactor *self) { return self->information(); })
      .def("constantTerm", [](gtsam::HessianFactor *self) { return self->constantTerm(); })
      .def("linearTerm", [](gtsam::HessianFactor *self) { return self->linearTerm(); });
  // .def("serialize", [](gtsam::HessianFactor *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::HessianFactor *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::HessianFactor &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::HessianFactor obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GaussianFactorGraph>(m_, "GaussianFactorGraph")
      .def(nb::init<>())
      .def(nb::init<const gtsam::GaussianBayesNet &>(), nb::arg("bayesNet"))
      .def(nb::init<const gtsam::GaussianBayesTree &>(), nb::arg("bayesTree"))
      .def("print", [](gtsam::GaussianFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianFactorGraph &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GaussianFactorGraph *self, const gtsam::GaussianFactorGraph &lfgraph, double tol) { return self->equals(lfgraph, tol); }, nb::arg("lfgraph"), nb::arg("tol"))
      .def("size", [](gtsam::GaussianFactorGraph *self) { return self->size(); })
      .def("at", [](gtsam::GaussianFactorGraph *self, size_t idx) { return self->at(idx); }, nb::arg("idx"))
      .def("keys", [](gtsam::GaussianFactorGraph *self) { return self->keys(); })
      .def("keyVector", [](gtsam::GaussianFactorGraph *self) { return self->keyVector(); })
      .def("exists", [](gtsam::GaussianFactorGraph *self, size_t idx) { return self->exists(idx); }, nb::arg("idx"))
      .def("push_back", [](gtsam::GaussianFactorGraph *self, const boost::shared_ptr<gtsam::GaussianFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("push_back", [](gtsam::GaussianFactorGraph *self, const boost::shared_ptr<gtsam::GaussianConditional> conditional) { self->push_back(conditional); }, nb::arg("conditional"))
      .def("push_back", [](gtsam::GaussianFactorGraph *self, const gtsam::GaussianFactorGraph &graph) { self->push_back(graph); }, nb::arg("graph"))
      .def("push_back", [](gtsam::GaussianFactorGraph *self, const gtsam::GaussianBayesNet &bayesNet) { self->push_back(bayesNet); }, nb::arg("bayesNet"))
      .def("push_back", [](gtsam::GaussianFactorGraph *self, const gtsam::GaussianBayesTree &bayesTree) { self->push_back(bayesTree); }, nb::arg("bayesTree"))
      .def("add", [](gtsam::GaussianFactorGraph *self, const gtsam::GaussianFactor &factor) { self->add(factor); }, nb::arg("factor"))
      .def("add", [](gtsam::GaussianFactorGraph *self, const gtsam::Vector &b) { self->add(b); }, nb::arg("b"))
      .def("add", [](gtsam::GaussianFactorGraph *self, size_t key1, const gtsam::Matrix &A1, const gtsam::Vector &b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model) { self->add(key1, A1, b, model); }, nb::arg("key1"), nb::arg("A1"), nb::arg("b"), nb::arg("model"))
      .def("add", [](gtsam::GaussianFactorGraph *self, size_t key1, const gtsam::Matrix &A1, size_t key2, const gtsam::Matrix &A2, const gtsam::Vector &b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model) { self->add(key1, A1, key2, A2, b, model); }, nb::arg("key1"), nb::arg("A1"), nb::arg("key2"), nb::arg("A2"), nb::arg("b"), nb::arg("model"))
      .def("add", [](gtsam::GaussianFactorGraph *self, size_t key1, const gtsam::Matrix &A1, size_t key2, const gtsam::Matrix &A2, size_t key3, const gtsam::Matrix &A3, const gtsam::Vector &b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model) { self->add(key1, A1, key2, A2, key3, A3, b, model); }, nb::arg("key1"), nb::arg("A1"), nb::arg("key2"), nb::arg("A2"), nb::arg("key3"), nb::arg("A3"), nb::arg("b"), nb::arg("model"))
      .def("error", [](gtsam::GaussianFactorGraph *self, const gtsam::VectorValues &c) { return self->error(c); }, nb::arg("c"))
      .def("probPrime", [](gtsam::GaussianFactorGraph *self, const gtsam::VectorValues &c) { return self->probPrime(c); }, nb::arg("c"))
      .def("printErrors", [](gtsam::GaussianFactorGraph *self, const gtsam::VectorValues &c, string str, const gtsam::KeyFormatter &keyFormatter) { self->printErrors(c, str, keyFormatter); }, nb::arg("c"), nb::arg("str") = "GaussianFactorGraph: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("clone", [](gtsam::GaussianFactorGraph *self) { return self->clone(); })
      .def("negate", [](gtsam::GaussianFactorGraph *self) { return self->negate(); })
      .def("optimize", [](gtsam::GaussianFactorGraph *self) { return self->optimize(); })
      .def("optimizeDensely", [](gtsam::GaussianFactorGraph *self) { return self->optimizeDensely(); })
      .def("optimize", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->optimize(ordering); }, nb::arg("ordering"))
      .def("optimizeGradientSearch", [](gtsam::GaussianFactorGraph *self) { return self->optimizeGradientSearch(); })
      .def("gradient", [](gtsam::GaussianFactorGraph *self, const gtsam::VectorValues &x0) { return self->gradient(x0); }, nb::arg("x0"))
      .def("gradientAtZero", [](gtsam::GaussianFactorGraph *self) { return self->gradientAtZero(); })
      .def("eliminateSequential", [](gtsam::GaussianFactorGraph *self) { return self->eliminateSequential(); })
      .def("eliminateSequential", [](gtsam::GaussianFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateSequential(type); }, nb::arg("type"))
      .def("eliminateSequential", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateSequential(ordering); }, nb::arg("ordering"))
      .def("eliminateMultifrontal", [](gtsam::GaussianFactorGraph *self) { return self->eliminateMultifrontal(); })
      .def("eliminateMultifrontal", [](gtsam::GaussianFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateMultifrontal(type); }, nb::arg("type"))
      .def("eliminateMultifrontal", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialSequential(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::GaussianFactorGraph *self, const gtsam::KeyVector &keys) { return self->eliminatePartialSequential(keys); }, nb::arg("keys"))
      .def("eliminatePartialMultifrontal", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialMultifrontal", [](gtsam::GaussianFactorGraph *self, const gtsam::KeyVector &keys) { return self->eliminatePartialMultifrontal(keys); }, nb::arg("keys"))
      .def("marginalMultifrontalBayesNet", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->marginalMultifrontalBayesNet(ordering); }, nb::arg("ordering"))
      .def("marginalMultifrontalBayesNet", [](gtsam::GaussianFactorGraph *self, const gtsam::KeyVector &key_vector) { return self->marginalMultifrontalBayesNet(key_vector); }, nb::arg("key_vector"))
      .def("marginalMultifrontalBayesNet", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering, const gtsam::Ordering &marginalizedVariableOrdering) { return self->marginalMultifrontalBayesNet(ordering, marginalizedVariableOrdering); }, nb::arg("ordering"), nb::arg("marginalizedVariableOrdering"))
      .def("marginalMultifrontalBayesNet", [](gtsam::GaussianFactorGraph *self, const gtsam::KeyVector &key_vector, const gtsam::Ordering &marginalizedVariableOrdering) { return self->marginalMultifrontalBayesNet(key_vector, marginalizedVariableOrdering); }, nb::arg("key_vector"), nb::arg("marginalizedVariableOrdering"))
      .def("marginal", [](gtsam::GaussianFactorGraph *self, const gtsam::KeyVector &key_vector) { return self->marginal(key_vector); }, nb::arg("key_vector"))
      .def("sparseJacobian_", [](gtsam::GaussianFactorGraph *self) { return self->sparseJacobian_(); })
      .def("augmentedJacobian", [](gtsam::GaussianFactorGraph *self) { return self->augmentedJacobian(); })
      .def("augmentedJacobian", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->augmentedJacobian(ordering); }, nb::arg("ordering"))
      .def("jacobian", [](gtsam::GaussianFactorGraph *self) { return self->jacobian(); })
      .def("jacobian", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->jacobian(ordering); }, nb::arg("ordering"))
      .def("augmentedHessian", [](gtsam::GaussianFactorGraph *self) { return self->augmentedHessian(); })
      .def("augmentedHessian", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->augmentedHessian(ordering); }, nb::arg("ordering"))
      .def("hessian", [](gtsam::GaussianFactorGraph *self) { return self->hessian(); })
      .def("hessian", [](gtsam::GaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->hessian(ordering); }, nb::arg("ordering"))
      .def("dot", [](gtsam::GaussianFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::GaussianFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter());
  // .def("serialize", [](gtsam::GaussianFactorGraph *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::GaussianFactorGraph *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::GaussianFactorGraph &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GaussianFactorGraph obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::GaussianConditional, gtsam::JacobianFactor>(m_, "GaussianConditional")
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("key"), nb::arg("d"), nb::arg("R"), nb::arg("sigmas"))
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &, size_t, const gtsam::Matrix &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("key"), nb::arg("d"), nb::arg("R"), nb::arg("name1"), nb::arg("S"), nb::arg("sigmas"))
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &, size_t, const gtsam::Matrix &, size_t, const gtsam::Matrix &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("key"), nb::arg("d"), nb::arg("R"), nb::arg("name1"), nb::arg("S"), nb::arg("name2"), nb::arg("T"), nb::arg("sigmas"))
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &>(), nb::arg("key"), nb::arg("d"), nb::arg("R"))
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &, size_t, const gtsam::Matrix &>(), nb::arg("key"), nb::arg("d"), nb::arg("R"), nb::arg("name1"), nb::arg("S"))
      .def(nb::init<size_t, const gtsam::Vector &, const gtsam::Matrix &, size_t, const gtsam::Matrix &, size_t, const gtsam::Matrix &>(), nb::arg("key"), nb::arg("d"), nb::arg("R"), nb::arg("name1"), nb::arg("S"), nb::arg("name2"), nb::arg("T"))
      .def("print", [](gtsam::GaussianConditional *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "GaussianConditional", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianConditional &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "GaussianConditional", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GaussianConditional *self, const gtsam::GaussianConditional &cg, double tol) { return self->equals(cg, tol); }, nb::arg("cg"), nb::arg("tol"))
      .def("logNormalizationConstant", [](gtsam::GaussianConditional *self) { return self->logNormalizationConstant(); })
      .def("logProbability", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &x) { return self->logProbability(x); }, nb::arg("x"))
      .def("evaluate", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &x) { return self->evaluate(x); }, nb::arg("x"))
      .def("error", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &x) { return self->error(x); }, nb::arg("x"))
      .def("firstFrontalKey", [](gtsam::GaussianConditional *self) { return self->firstFrontalKey(); })
      .def("solve", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &parents) { return self->solve(parents); }, nb::arg("parents"))
      .def("likelihood", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &frontalValues) { return self->likelihood(frontalValues); }, nb::arg("frontalValues"))
      .def("likelihood", [](gtsam::GaussianConditional *self, const gtsam::Vector &frontal) { return self->likelihood(frontal); }, nb::arg("frontal"))
      .def("sample", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &parents) { return self->sample(parents); }, nb::arg("parents"))
      .def("sample", [](gtsam::GaussianConditional *self) { return self->sample(); })
      .def("solveOtherRHS", [](gtsam::GaussianConditional *self, const gtsam::VectorValues &parents, const gtsam::VectorValues &rhs) { return self->solveOtherRHS(parents, rhs); }, nb::arg("parents"), nb::arg("rhs"))
      .def("solveTransposeInPlace", [](gtsam::GaussianConditional *self, gtsam::VectorValues &gy) { self->solveTransposeInPlace(gy); }, nb::arg("gy"))
      .def("R", [](gtsam::GaussianConditional *self) { return self->R(); })
      .def("S", [](gtsam::GaussianConditional *self) { return self->S(); })
      .def("d", [](gtsam::GaussianConditional *self) { return self->d(); })
      // .def("serialize", [](gtsam::GaussianConditional *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::GaussianConditional *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::GaussianConditional &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::GaussianConditional obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("logProbability", [](gtsam::GaussianConditional *self, const gtsam::HybridValues &x) { return self->logProbability(x); }, nb::arg("x"))
      .def("evaluate", [](gtsam::GaussianConditional *self, const gtsam::HybridValues &x) { return self->evaluate(x); }, nb::arg("x"))
      .def("error", [](gtsam::GaussianConditional *self, const gtsam::HybridValues &x) { return self->error(x); }, nb::arg("x"))
      .def_static("FromMeanAndStddev", [](gtsam::Key key, const gtsam::Vector &mu, double sigma) { return gtsam::GaussianConditional::FromMeanAndStddev(key, mu, sigma); }, nb::arg("key"), nb::arg("mu"), nb::arg("sigma"))
      .def_static("FromMeanAndStddev", [](gtsam::Key key, const gtsam::Matrix &A, gtsam::Key parent, const gtsam::Vector &b, double sigma) { return gtsam::GaussianConditional::FromMeanAndStddev(key, A, parent, b, sigma); }, nb::arg("key"), nb::arg("A"), nb::arg("parent"), nb::arg("b"), nb::arg("sigma"))
      .def_static("FromMeanAndStddev", [](gtsam::Key key, const gtsam::Matrix &A1, gtsam::Key parent1, const gtsam::Matrix &A2, gtsam::Key parent2, const gtsam::Vector &b, double sigma) { return gtsam::GaussianConditional::FromMeanAndStddev(key, A1, parent1, A2, parent2, b, sigma); }, nb::arg("key"), nb::arg("A1"), nb::arg("parent1"), nb::arg("A2"), nb::arg("parent2"), nb::arg("b"), nb::arg("sigma"));

  nb::class_<gtsam::GaussianDensity, gtsam::GaussianConditional>(m_, "GaussianDensity")
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const gtsam::Matrix &, const boost::shared_ptr<gtsam::noiseModel::Diagonal>>(), nb::arg("key"), nb::arg("d"), nb::arg("R"), nb::arg("sigmas"))
      .def("print", [](gtsam::GaussianDensity *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "GaussianDensity", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianDensity &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "GaussianDensity", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GaussianDensity *self, const gtsam::GaussianDensity &cg, double tol) { return self->equals(cg, tol); }, nb::arg("cg"), nb::arg("tol"))
      .def("mean", [](gtsam::GaussianDensity *self) { return self->mean(); })
      .def("covariance", [](gtsam::GaussianDensity *self) { return self->covariance(); })
      .def_static("FromMeanAndStddev", [](gtsam::Key key, const gtsam::Vector &mean, double sigma) { return gtsam::GaussianDensity::FromMeanAndStddev(key, mean, sigma); }, nb::arg("key"), nb::arg("mean"), nb::arg("sigma"));

  nb::class_<gtsam::GaussianBayesNet>(m_, "GaussianBayesNet")
      .def(nb::init<>())
      .def(nb::init<const boost::shared_ptr<gtsam::GaussianConditional>>(), nb::arg("conditional"))
      .def("print", [](gtsam::GaussianBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianBayesNet &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::GaussianBayesNet *self, const gtsam::GaussianBayesNet &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("size", [](gtsam::GaussianBayesNet *self) { return self->size(); })
      .def("push_back", [](gtsam::GaussianBayesNet *self, boost::shared_ptr<gtsam::GaussianConditional> conditional) { self->push_back(conditional); }, nb::arg("conditional"))
      .def("push_back", [](gtsam::GaussianBayesNet *self, const gtsam::GaussianBayesNet &bayesNet) { self->push_back(bayesNet); }, nb::arg("bayesNet"))
      .def("front", [](gtsam::GaussianBayesNet *self) { return self->front(); })
      .def("back", [](gtsam::GaussianBayesNet *self) { return self->back(); })
      .def("logProbability", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &x) { return self->logProbability(x); }, nb::arg("x"))
      .def("evaluate", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &x) { return self->evaluate(x); }, nb::arg("x"))
      .def("error", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &x) { return self->error(x); }, nb::arg("x"))
      .def("optimize", [](gtsam::GaussianBayesNet *self) { return self->optimize(); })
      .def("optimize", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &given) { return self->optimize(given); }, nb::arg("given"))
      .def("optimizeGradientSearch", [](gtsam::GaussianBayesNet *self) { return self->optimizeGradientSearch(); })
      .def("sample", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &given) { return self->sample(given); }, nb::arg("given"))
      .def("sample", [](gtsam::GaussianBayesNet *self) { return self->sample(); })
      .def("backSubstitute", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &gx) { return self->backSubstitute(gx); }, nb::arg("gx"))
      .def("backSubstituteTranspose", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &gx) { return self->backSubstituteTranspose(gx); }, nb::arg("gx"))
      .def("at", [](gtsam::GaussianBayesNet *self, size_t idx) { return self->at(idx); }, nb::arg("idx"))
      .def("keys", [](gtsam::GaussianBayesNet *self) { return self->keys(); })
      .def("keyVector", [](gtsam::GaussianBayesNet *self) { return self->keyVector(); })
      .def("exists", [](gtsam::GaussianBayesNet *self, size_t idx) { return self->exists(idx); }, nb::arg("idx"))
      .def("saveGraph", [](gtsam::GaussianBayesNet *self, const string &s) { self->saveGraph(s); }, nb::arg("s"))
      .def("matrix", [](gtsam::GaussianBayesNet *self) { return self->matrix(); })
      .def("gradient", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &x0) { return self->gradient(x0); }, nb::arg("x0"))
      .def("gradientAtZero", [](gtsam::GaussianBayesNet *self) { return self->gradientAtZero(); })
      .def("error", [](gtsam::GaussianBayesNet *self, const gtsam::VectorValues &x) { return self->error(x); }, nb::arg("x"))
      .def("determinant", [](gtsam::GaussianBayesNet *self) { return self->determinant(); })
      .def("logDeterminant", [](gtsam::GaussianBayesNet *self) { return self->logDeterminant(); })
      .def("dot", [](gtsam::GaussianBayesNet *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::GaussianBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter());

  nb::class_<gtsam::GaussianBayesTree>(m_, "GaussianBayesTree")
      .def(nb::init<>())
      .def(nb::init<const gtsam::GaussianBayesTree &>(), nb::arg("other"))
      .def("equals", [](gtsam::GaussianBayesTree *self, const gtsam::GaussianBayesTree &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("print", [](gtsam::GaussianBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianBayesTree &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("size", [](gtsam::GaussianBayesTree *self) { return self->size(); })
      .def("empty", [](gtsam::GaussianBayesTree *self) { return self->empty(); })
      .def("numCachedSeparatorMarginals", [](gtsam::GaussianBayesTree *self) { return self->numCachedSeparatorMarginals(); })
      .def("dot", [](gtsam::GaussianBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->dot(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("saveGraph", [](gtsam::GaussianBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { self->saveGraph(s, keyFormatter); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("optimize", [](gtsam::GaussianBayesTree *self) { return self->optimize(); })
      .def("optimizeGradientSearch", [](gtsam::GaussianBayesTree *self) { return self->optimizeGradientSearch(); })
      .def("gradient", [](gtsam::GaussianBayesTree *self, const gtsam::VectorValues &x0) { return self->gradient(x0); }, nb::arg("x0"))
      .def("gradientAtZero", [](gtsam::GaussianBayesTree *self) { return self->gradientAtZero(); })
      .def("error", [](gtsam::GaussianBayesTree *self, const gtsam::VectorValues &x) { return self->error(x); }, nb::arg("x"))
      .def("determinant", [](gtsam::GaussianBayesTree *self) { return self->determinant(); })
      .def("logDeterminant", [](gtsam::GaussianBayesTree *self) { return self->logDeterminant(); })
      .def("marginalCovariance", [](gtsam::GaussianBayesTree *self, size_t key) { return self->marginalCovariance(key); }, nb::arg("key"))
      .def("marginalFactor", [](gtsam::GaussianBayesTree *self, size_t key) { return self->marginalFactor(key); }, nb::arg("key"))
      .def("joint", [](gtsam::GaussianBayesTree *self, size_t key1, size_t key2) { return self->joint(key1, key2); }, nb::arg("key1"), nb::arg("key2"))
      .def("jointBayesNet", [](gtsam::GaussianBayesTree *self, size_t key1, size_t key2) { return self->jointBayesNet(key1, key2); }, nb::arg("key1"), nb::arg("key2"));

  nb::class_<gtsam::GaussianISAM>(m_, "GaussianISAM")
      .def(nb::init<>())
      .def("update", [](gtsam::GaussianISAM *self, const gtsam::GaussianFactorGraph &newFactors) { self->update(newFactors); }, nb::arg("newFactors"))
      .def("saveGraph", [](gtsam::GaussianISAM *self, string s) { self->saveGraph(s); }, nb::arg("s"))
      .def("clear", [](gtsam::GaussianISAM *self) { self->clear(); });

  nb::class_<gtsam::IterativeOptimizationParameters>(m_, "IterativeOptimizationParameters")
      .def("getVerbosity", [](gtsam::IterativeOptimizationParameters *self) { return self->getVerbosity(); })
      .def("setVerbosity", [](gtsam::IterativeOptimizationParameters *self, string s) { self->setVerbosity(s); }, nb::arg("s"));

  nb::class_<gtsam::ConjugateGradientParameters, gtsam::IterativeOptimizationParameters>(m_, "ConjugateGradientParameters")
      .def(nb::init<>())
      .def("getMinIterations", [](gtsam::ConjugateGradientParameters *self) { return self->getMinIterations(); })
      .def("getMaxIterations", [](gtsam::ConjugateGradientParameters *self) { return self->getMaxIterations(); })
      .def("getReset", [](gtsam::ConjugateGradientParameters *self) { return self->getReset(); })
      .def("getEpsilon_rel", [](gtsam::ConjugateGradientParameters *self) { return self->getEpsilon_rel(); })
      .def("getEpsilon_abs", [](gtsam::ConjugateGradientParameters *self) { return self->getEpsilon_abs(); })
      .def("setMinIterations", [](gtsam::ConjugateGradientParameters *self, int value) { self->setMinIterations(value); }, nb::arg("value"))
      .def("setMaxIterations", [](gtsam::ConjugateGradientParameters *self, int value) { self->setMaxIterations(value); }, nb::arg("value"))
      .def("setReset", [](gtsam::ConjugateGradientParameters *self, int value) { self->setReset(value); }, nb::arg("value"))
      .def("setEpsilon_rel", [](gtsam::ConjugateGradientParameters *self, double value) { self->setEpsilon_rel(value); }, nb::arg("value"))
      .def("setEpsilon_abs", [](gtsam::ConjugateGradientParameters *self, double value) { self->setEpsilon_abs(value); }, nb::arg("value"));

  nb::class_<gtsam::PreconditionerParameters>(m_, "PreconditionerParameters")
      .def(nb::init<>());

  nb::class_<gtsam::DummyPreconditionerParameters, gtsam::PreconditionerParameters>(m_, "DummyPreconditionerParameters")
      .def(nb::init<>());

  nb::class_<gtsam::BlockJacobiPreconditionerParameters, gtsam::PreconditionerParameters>(m_, "BlockJacobiPreconditionerParameters")
      .def(nb::init<>());

  nb::class_<gtsam::PCGSolverParameters, gtsam::ConjugateGradientParameters>(m_, "PCGSolverParameters")
      .def(nb::init<>())
      .def("print", [](gtsam::PCGSolverParameters *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::PCGSolverParameters &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("setPreconditionerParams", [](gtsam::PCGSolverParameters *self, boost::shared_ptr<gtsam::PreconditionerParameters> preconditioner) { self->setPreconditionerParams(preconditioner); }, nb::arg("preconditioner"));

  nb::class_<gtsam::SubgraphSolverParameters, gtsam::ConjugateGradientParameters>(m_, "SubgraphSolverParameters")
      .def(nb::init<>());

  nb::class_<gtsam::SubgraphSolver>(m_, "SubgraphSolver")
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::SubgraphSolverParameters &, const gtsam::Ordering &>(), nb::arg("A"), nb::arg("parameters"), nb::arg("ordering"))
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::GaussianFactorGraph &, const gtsam::SubgraphSolverParameters &, const gtsam::Ordering &>(), nb::arg("Ab1"), nb::arg("Ab2"), nb::arg("parameters"), nb::arg("ordering"))
      .def("optimize", [](gtsam::SubgraphSolver *self) { return self->optimize(); });

  nb::class_<gtsam::KalmanFilter>(m_, "KalmanFilter")
      .def(nb::init<size_t>(), nb::arg("n"))
      .def("init", [](gtsam::KalmanFilter *self, const gtsam::Vector &x0, const gtsam::Matrix &P0) { return self->init(x0, P0); }, nb::arg("x0"), nb::arg("P0"))
      .def("print", [](gtsam::KalmanFilter *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::KalmanFilter &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("predict", [](gtsam::KalmanFilter *self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix &F, const gtsam::Matrix &B, const gtsam::Vector &u, const boost::shared_ptr<gtsam::noiseModel::Diagonal> modelQ) { return self->predict(p, F, B, u, modelQ); }, nb::arg("p"), nb::arg("F"), nb::arg("B"), nb::arg("u"), nb::arg("modelQ"))
      .def("predictQ", [](gtsam::KalmanFilter *self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix &F, const gtsam::Matrix &B, const gtsam::Vector &u, const gtsam::Matrix &Q) { return self->predictQ(p, F, B, u, Q); }, nb::arg("p"), nb::arg("F"), nb::arg("B"), nb::arg("u"), nb::arg("Q"))
      .def("predict2", [](gtsam::KalmanFilter *self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix &A0, const gtsam::Matrix &A1, const gtsam::Vector &b, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model) { return self->predict2(p, A0, A1, b, model); }, nb::arg("p"), nb::arg("A0"), nb::arg("A1"), nb::arg("b"), nb::arg("model"))
      .def("update", [](gtsam::KalmanFilter *self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix &H, const gtsam::Vector &z, const boost::shared_ptr<gtsam::noiseModel::Diagonal> model) { return self->update(p, H, z, model); }, nb::arg("p"), nb::arg("H"), nb::arg("z"), nb::arg("model"))
      .def("updateQ", [](gtsam::KalmanFilter *self, boost::shared_ptr<gtsam::GaussianDensity> p, const gtsam::Matrix &H, const gtsam::Vector &z, const gtsam::Matrix &Q) { return self->updateQ(p, H, z, Q); }, nb::arg("p"), nb::arg("H"), nb::arg("z"), nb::arg("Q"))
      .def_static("step", [](boost::shared_ptr<gtsam::GaussianDensity> p) { return gtsam::KalmanFilter::step(p); }, nb::arg("p"));
}
