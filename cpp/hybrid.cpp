/**
 * @file    hybrid.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal
 * @date    Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

#define PYBIND11_DETAILED_ERROR_MESSAGES

// Include relevant boost libraries required by GTSAM
#include <boost/shared_ptr.hpp>

#include "gtsam/base/serialization.h"
#include "gtsam/base/utilities.h" // for RedirectCout.
#include "gtsam/config.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/iostream.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

// These are the included headers listed in `gtsam.i`
#include "gtsam/hybrid/GaussianMixture.h"
#include "gtsam/hybrid/GaussianMixtureFactor.h"
#include "gtsam/hybrid/HybridBayesNet.h"
#include "gtsam/hybrid/HybridBayesTree.h"
#include "gtsam/hybrid/HybridConditional.h"
#include "gtsam/hybrid/HybridFactor.h"
#include "gtsam/hybrid/HybridGaussianFactorGraph.h"
#include "gtsam/hybrid/HybridNonlinearFactorGraph.h"
#include "gtsam/hybrid/HybridValues.h"
#include "gtsam/hybrid/MixtureFactor.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization

// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/hybrid.h"

using namespace std;

namespace py = pybind11;

void hybrid(py::module_ &m_) {
  m_.doc() = "pybind11 wrapper of hybrid";

  py::class_<gtsam::HybridValues, boost::shared_ptr<gtsam::HybridValues>>(m_, "HybridValues")
      .def(py::init<>())
      .def(py::init<const gtsam::VectorValues &, const gtsam::DiscreteValues &>(), py::arg("cv"), py::arg("dv"))
      .def("continuous", [](gtsam::HybridValues *self) { return self->continuous(); })
      .def("discrete", [](gtsam::HybridValues *self) { return self->discrete(); })
      .def("print", [](gtsam::HybridValues *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "HybridValues", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridValues &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "HybridValues", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridValues *self, const gtsam::HybridValues &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol"))
      .def("insert", [](gtsam::HybridValues *self, gtsam::Key j, int value) { self->insert(j, value); }, py::arg("j"), py::arg("value"))
      .def("insert", [](gtsam::HybridValues *self, gtsam::Key j, const gtsam::Vector &value) { self->insert(j, value); }, py::arg("j"), py::arg("value"))
      .def("insert_or_assign", [](gtsam::HybridValues *self, gtsam::Key j, const gtsam::Vector &value) { self->insert_or_assign(j, value); }, py::arg("j"), py::arg("value"))
      .def("insert_or_assign", [](gtsam::HybridValues *self, gtsam::Key j, size_t value) { self->insert_or_assign(j, value); }, py::arg("j"), py::arg("value"))
      .def("insert", [](gtsam::HybridValues *self, const gtsam::VectorValues &values) { self->insert(values); }, py::arg("values"))
      .def("insert", [](gtsam::HybridValues *self, const gtsam::DiscreteValues &values) { self->insert(values); }, py::arg("values"))
      .def("insert", [](gtsam::HybridValues *self, const gtsam::HybridValues &values) { self->insert(values); }, py::arg("values"))
      .def("update", [](gtsam::HybridValues *self, const gtsam::VectorValues &values) { self->update(values); }, py::arg("values"))
      .def("update", [](gtsam::HybridValues *self, const gtsam::DiscreteValues &values) { self->update(values); }, py::arg("values"))
      .def("update", [](gtsam::HybridValues *self, const gtsam::HybridValues &values) { self->update(values); }, py::arg("values"))
      .def("atDiscrete", [](gtsam::HybridValues *self, gtsam::Key j) { return self->atDiscrete(j); }, py::arg("j"))
      .def("at", [](gtsam::HybridValues *self, gtsam::Key j) { return self->at(j); }, py::arg("j"));

  py::class_<gtsam::HybridFactor, gtsam::Factor, boost::shared_ptr<gtsam::HybridFactor>>(m_, "HybridFactor")
      .def("print", [](gtsam::HybridFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "HybridFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "HybridFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridFactor *self, const gtsam::HybridFactor &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol") = 1e-9)
      .def("error", [](gtsam::HybridFactor *self, const gtsam::HybridValues &values) { return self->error(values); }, py::arg("values"))
      .def("isDiscrete", [](gtsam::HybridFactor *self) { return self->isDiscrete(); })
      .def("isContinuous", [](gtsam::HybridFactor *self) { return self->isContinuous(); })
      .def("isHybrid", [](gtsam::HybridFactor *self) { return self->isHybrid(); })
      .def("nrContinuous", [](gtsam::HybridFactor *self) { return self->nrContinuous(); })
      .def("discreteKeys", [](gtsam::HybridFactor *self) { return self->discreteKeys(); })
      .def("continuousKeys", [](gtsam::HybridFactor *self) { return self->continuousKeys(); });

  py::class_<gtsam::HybridConditional, boost::shared_ptr<gtsam::HybridConditional>>(m_, "HybridConditional")
      .def("print", [](gtsam::HybridConditional *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "Hybrid Conditional\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridConditional &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "Hybrid Conditional\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridConditional *self, const gtsam::HybridConditional &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol") = 1e-9)
      .def("nrFrontals", [](gtsam::HybridConditional *self) { return self->nrFrontals(); })
      .def("nrParents", [](gtsam::HybridConditional *self) { return self->nrParents(); })
      .def("logNormalizationConstant", [](gtsam::HybridConditional *self) { return self->logNormalizationConstant(); })
      .def("logProbability", [](gtsam::HybridConditional *self, const gtsam::HybridValues &values) { return self->logProbability(values); }, py::arg("values"))
      .def("evaluate", [](gtsam::HybridConditional *self, const gtsam::HybridValues &values) { return self->evaluate(values); }, py::arg("values"))
      .def("asMixture", [](gtsam::HybridConditional *self) { return self->asMixture(); })
      .def("asGaussian", [](gtsam::HybridConditional *self) { return self->asGaussian(); })
      .def("asDiscrete", [](gtsam::HybridConditional *self) { return self->asDiscrete(); })
      .def("inner", [](gtsam::HybridConditional *self) { return self->inner(); })
      .def("error", [](gtsam::HybridConditional *self, const gtsam::HybridValues &values) { return self->error(values); }, py::arg("values"))
      .def("__call__", &gtsam::HybridConditional::operator());

  py::class_<gtsam::GaussianMixtureFactor, gtsam::HybridFactor, boost::shared_ptr<gtsam::GaussianMixtureFactor>>(m_, "GaussianMixtureFactor")
      .def(py::init<const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const std::vector<gtsam::GaussianFactor::shared_ptr> &>(), py::arg("continuousKeys"), py::arg("discreteKeys"), py::arg("factorsList"))
      .def("print", [](gtsam::GaussianMixtureFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "GaussianMixtureFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianMixtureFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "GaussianMixtureFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  py::class_<gtsam::GaussianMixture, gtsam::HybridFactor, boost::shared_ptr<gtsam::GaussianMixture>>(m_, "GaussianMixture")
      .def(py::init<const gtsam::KeyVector &, const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const std::vector<gtsam::GaussianConditional::shared_ptr> &>(), py::arg("continuousFrontals"), py::arg("continuousParents"), py::arg("discreteParents"), py::arg("conditionalsList"))
      .def("likelihood", [](gtsam::GaussianMixture *self, const gtsam::VectorValues &frontals) { return self->likelihood(frontals); }, py::arg("frontals"))
      .def("print", [](gtsam::GaussianMixture *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "GaussianMixture\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianMixture &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "GaussianMixture\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  py::class_<gtsam::HybridBayesTreeClique, boost::shared_ptr<gtsam::HybridBayesTreeClique>>(m_, "HybridBayesTreeClique")
      .def(py::init<>())
      .def(py::init<const boost::shared_ptr<gtsam::HybridConditional>>(), py::arg("conditional"))
      .def("conditional", [](gtsam::HybridBayesTreeClique *self) { return self->conditional(); })
      .def("isRoot", [](gtsam::HybridBayesTreeClique *self) { return self->isRoot(); });

  py::class_<gtsam::HybridBayesTree, boost::shared_ptr<gtsam::HybridBayesTree>>(m_, "HybridBayesTree")
      .def(py::init<>())
      .def("print", [](gtsam::HybridBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "HybridBayesTree\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridBayesTree &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "HybridBayesTree\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridBayesTree *self, const gtsam::HybridBayesTree &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol") = 1e-9)
      .def("size", [](gtsam::HybridBayesTree *self) { return self->size(); })
      .def("empty", [](gtsam::HybridBayesTree *self) { return self->empty(); })
      .def("optimize", [](gtsam::HybridBayesTree *self) { return self->optimize(); })
      .def("dot", [](gtsam::HybridBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->dot(keyFormatter); }, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__getitem__", &gtsam::HybridBayesTree::operator[]);

  py::class_<gtsam::HybridBayesNet, boost::shared_ptr<gtsam::HybridBayesNet>>(m_, "HybridBayesNet")
      .def(py::init<>())
      .def("push_back", [](gtsam::HybridBayesNet *self, const boost::shared_ptr<gtsam::GaussianMixture> s) { self->push_back(s); }, py::arg("s"))
      .def("push_back", [](gtsam::HybridBayesNet *self, const boost::shared_ptr<gtsam::GaussianConditional> s) { self->push_back(s); }, py::arg("s"))
      .def("push_back", [](gtsam::HybridBayesNet *self, const boost::shared_ptr<gtsam::DiscreteConditional> s) { self->push_back(s); }, py::arg("s"))
      .def("empty", [](gtsam::HybridBayesNet *self) { return self->empty(); })
      .def("size", [](gtsam::HybridBayesNet *self) { return self->size(); })
      .def("keys", [](gtsam::HybridBayesNet *self) { return self->keys(); })
      .def("at", [](gtsam::HybridBayesNet *self, size_t i) { return self->at(i); }, py::arg("i"))
      .def("logProbability", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &values) { return self->logProbability(values); }, py::arg("values"))
      .def("evaluate", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &values) { return self->evaluate(values); }, py::arg("values"))
      .def("error", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &values) { return self->error(values); }, py::arg("values"))
      .def("toFactorGraph", [](gtsam::HybridBayesNet *self, const gtsam::VectorValues &measurements) { return self->toFactorGraph(measurements); }, py::arg("measurements"))
      .def("optimize", [](gtsam::HybridBayesNet *self) { return self->optimize(); })
      .def("sample", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &given) { return self->sample(given); }, py::arg("given"))
      .def("sample", [](gtsam::HybridBayesNet *self) { return self->sample(); })
      .def("print", [](gtsam::HybridBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "HybridBayesNet\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridBayesNet &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "HybridBayesNet\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridBayesNet *self, const gtsam::HybridBayesNet &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol") = 1e-9)
      .def("dot", [](gtsam::HybridBayesNet *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::HybridBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter());

  py::class_<gtsam::HybridGaussianFactorGraph, boost::shared_ptr<gtsam::HybridGaussianFactorGraph>>(m_, "HybridGaussianFactorGraph")
      .def(py::init<>())
      .def(py::init<const gtsam::HybridBayesNet &>(), py::arg("bayesNet"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const boost::shared_ptr<gtsam::HybridFactor> factor) { self->push_back(factor); }, py::arg("factor"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const boost::shared_ptr<gtsam::HybridConditional> conditional) { self->push_back(conditional); }, py::arg("conditional"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridGaussianFactorGraph &graph) { self->push_back(graph); }, py::arg("graph"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridBayesNet &bayesNet) { self->push_back(bayesNet); }, py::arg("bayesNet"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridBayesTree &bayesTree) { self->push_back(bayesTree); }, py::arg("bayesTree"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const boost::shared_ptr<gtsam::GaussianMixtureFactor> gmm) { self->push_back(gmm); }, py::arg("gmm"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, boost::shared_ptr<gtsam::DecisionTreeFactor> factor) { self->push_back(factor); }, py::arg("factor"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, boost::shared_ptr<gtsam::JacobianFactor> factor) { self->push_back(factor); }, py::arg("factor"))
      .def("empty", [](gtsam::HybridGaussianFactorGraph *self) { return self->empty(); })
      .def("remove", [](gtsam::HybridGaussianFactorGraph *self, size_t i) { self->remove(i); }, py::arg("i"))
      .def("size", [](gtsam::HybridGaussianFactorGraph *self) { return self->size(); })
      .def("keys", [](gtsam::HybridGaussianFactorGraph *self) { return self->keys(); })
      .def("at", [](gtsam::HybridGaussianFactorGraph *self, size_t i) { return self->at(i); }, py::arg("i"))
      .def("print", [](gtsam::HybridGaussianFactorGraph *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::HybridGaussianFactorGraph &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridGaussianFactorGraph &fg, double tol) { return self->equals(fg, tol); }, py::arg("fg"), py::arg("tol") = 1e-9)
      .def("error", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridValues &values) { return self->error(values); }, py::arg("values"))
      .def("probPrime", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridValues &values) { return self->probPrime(values); }, py::arg("values"))
      .def("eliminateSequential", [](gtsam::HybridGaussianFactorGraph *self) { return self->eliminateSequential(); })
      .def("eliminateSequential", [](gtsam::HybridGaussianFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateSequential(type); }, py::arg("type"))
      .def("eliminateSequential", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateSequential(ordering); }, py::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialSequential(ordering); }, py::arg("ordering"))
      .def("eliminateMultifrontal", [](gtsam::HybridGaussianFactorGraph *self) { return self->eliminateMultifrontal(); })
      .def("eliminateMultifrontal", [](gtsam::HybridGaussianFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateMultifrontal(type); }, py::arg("type"))
      .def("eliminateMultifrontal", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateMultifrontal(ordering); }, py::arg("ordering"))
      .def("eliminatePartialMultifrontal", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialMultifrontal(ordering); }, py::arg("ordering"))
      .def("dot", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter());

  py::class_<gtsam::HybridNonlinearFactorGraph, boost::shared_ptr<gtsam::HybridNonlinearFactorGraph>>(m_, "HybridNonlinearFactorGraph")
      .def(py::init<>())
      .def(py::init<const gtsam::HybridNonlinearFactorGraph &>(), py::arg("graph"))
      .def("push_back", [](gtsam::HybridNonlinearFactorGraph *self, boost::shared_ptr<gtsam::HybridFactor> factor) { self->push_back(factor); }, py::arg("factor"))
      .def("push_back", [](gtsam::HybridNonlinearFactorGraph *self, boost::shared_ptr<gtsam::NonlinearFactor> factor) { self->push_back(factor); }, py::arg("factor"))
      .def("push_back", [](gtsam::HybridNonlinearFactorGraph *self, boost::shared_ptr<gtsam::DiscreteFactor> factor) { self->push_back(factor); }, py::arg("factor"))
      .def("linearize", [](gtsam::HybridNonlinearFactorGraph *self, const gtsam::Values &continuousValues) { return self->linearize(continuousValues); }, py::arg("continuousValues"))
      .def("empty", [](gtsam::HybridNonlinearFactorGraph *self) { return self->empty(); })
      .def("remove", [](gtsam::HybridNonlinearFactorGraph *self, size_t i) { self->remove(i); }, py::arg("i"))
      .def("size", [](gtsam::HybridNonlinearFactorGraph *self) { return self->size(); })
      .def("keys", [](gtsam::HybridNonlinearFactorGraph *self) { return self->keys(); })
      .def("at", [](gtsam::HybridNonlinearFactorGraph *self, size_t i) { return self->at(i); }, py::arg("i"))
      .def("print", [](gtsam::HybridNonlinearFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "HybridNonlinearFactorGraph\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridNonlinearFactorGraph &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "HybridNonlinearFactorGraph\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  py::class_<gtsam::MixtureFactor, gtsam::HybridFactor, boost::shared_ptr<gtsam::MixtureFactor>>(m_, "MixtureFactor")
      .def(py::init<const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const gtsam::DecisionTree<gtsam::Key, boost::shared_ptr<gtsam::NonlinearFactor>> &, bool>(), py::arg("keys"), py::arg("discreteKeys"), py::arg("factors"), py::arg("normalized") = false)
      .def(py::init<const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const std::vector<boost::shared_ptr<gtsam::NonlinearFactor>> &, bool>(), py::arg("keys"), py::arg("discreteKeys"), py::arg("factors"), py::arg("normalized") = false)
      .def("error", [](gtsam::MixtureFactor *self, const gtsam::Values &continuousValues, const gtsam::DiscreteValues &discreteValues) { return self->error(continuousValues, discreteValues); }, py::arg("continuousValues"), py::arg("discreteValues"))
      .def("nonlinearFactorLogNormalizingConstant", [](gtsam::MixtureFactor *self, const boost::shared_ptr<gtsam::NonlinearFactor> factor, const gtsam::Values &values) { return self->nonlinearFactorLogNormalizingConstant(factor, values); }, py::arg("factor"), py::arg("values"))
      .def("linearize", [](gtsam::MixtureFactor *self, const gtsam::Values &continuousValues) { return self->linearize(continuousValues); }, py::arg("continuousValues"))
      .def("print", [](gtsam::MixtureFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "MixtureFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::MixtureFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "MixtureFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/hybrid.h"
}
