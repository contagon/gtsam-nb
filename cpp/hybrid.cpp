/**
 * @file    hybrid.cpp
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
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "utils/boost_shared_ptr.h"

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

using namespace std;

namespace nb = nanobind;

void hybrid(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of hybrid";

  nb::class_<gtsam::HybridValues>(m_, "HybridValues")
      .def(nb::init<>())
      .def(nb::init<const gtsam::VectorValues &, const gtsam::DiscreteValues &>(), nb::arg("cv"), nb::arg("dv"))
      .def("continuous", [](gtsam::HybridValues *self) { return self->continuous(); })
      .def("discrete", [](gtsam::HybridValues *self) { return self->discrete(); })
      .def("print", [](gtsam::HybridValues *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "HybridValues", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridValues &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "HybridValues", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridValues *self, const gtsam::HybridValues &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("insert", [](gtsam::HybridValues *self, gtsam::Key j, int value) { self->insert(j, value); }, nb::arg("j"), nb::arg("value"))
      .def("insert", [](gtsam::HybridValues *self, gtsam::Key j, const gtsam::Vector &value) { self->insert(j, value); }, nb::arg("j"), nb::arg("value"))
      .def("insert_or_assign", [](gtsam::HybridValues *self, gtsam::Key j, const gtsam::Vector &value) { self->insert_or_assign(j, value); }, nb::arg("j"), nb::arg("value"))
      .def("insert_or_assign", [](gtsam::HybridValues *self, gtsam::Key j, size_t value) { self->insert_or_assign(j, value); }, nb::arg("j"), nb::arg("value"))
      .def("insert", [](gtsam::HybridValues *self, const gtsam::VectorValues &values) { self->insert(values); }, nb::arg("values"))
      .def("insert", [](gtsam::HybridValues *self, const gtsam::DiscreteValues &values) { self->insert(values); }, nb::arg("values"))
      .def("insert", [](gtsam::HybridValues *self, const gtsam::HybridValues &values) { self->insert(values); }, nb::arg("values"))
      .def("update", [](gtsam::HybridValues *self, const gtsam::VectorValues &values) { self->update(values); }, nb::arg("values"))
      .def("update", [](gtsam::HybridValues *self, const gtsam::DiscreteValues &values) { self->update(values); }, nb::arg("values"))
      .def("update", [](gtsam::HybridValues *self, const gtsam::HybridValues &values) { self->update(values); }, nb::arg("values"))
      .def("atDiscrete", [](gtsam::HybridValues *self, gtsam::Key j) { return self->atDiscrete(j); }, nb::arg("j"))
      .def("at", [](gtsam::HybridValues *self, gtsam::Key j) { return self->at(j); }, nb::arg("j"));

  nb::class_<gtsam::HybridFactor, gtsam::Factor>(m_, "HybridFactor")
      .def("print", [](gtsam::HybridFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "HybridFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "HybridFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridFactor *self, const gtsam::HybridFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("error", [](gtsam::HybridFactor *self, const gtsam::HybridValues &values) { return self->error(values); }, nb::arg("values"))
      .def("isDiscrete", [](gtsam::HybridFactor *self) { return self->isDiscrete(); })
      .def("isContinuous", [](gtsam::HybridFactor *self) { return self->isContinuous(); })
      .def("isHybrid", [](gtsam::HybridFactor *self) { return self->isHybrid(); })
      .def("nrContinuous", [](gtsam::HybridFactor *self) { return self->nrContinuous(); })
      .def("discreteKeys", [](gtsam::HybridFactor *self) { return self->discreteKeys(); })
      .def("continuousKeys", [](gtsam::HybridFactor *self) { return self->continuousKeys(); });

  nb::class_<gtsam::HybridConditional>(m_, "HybridConditional")
      .def("print", [](gtsam::HybridConditional *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "Hybrid Conditional\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridConditional &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "Hybrid Conditional\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridConditional *self, const gtsam::HybridConditional &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("nrFrontals", [](gtsam::HybridConditional *self) { return self->nrFrontals(); })
      .def("nrParents", [](gtsam::HybridConditional *self) { return self->nrParents(); })
      .def("logNormalizationConstant", [](gtsam::HybridConditional *self) { return self->logNormalizationConstant(); })
      .def("logProbability", [](gtsam::HybridConditional *self, const gtsam::HybridValues &values) { return self->logProbability(values); }, nb::arg("values"))
      .def("evaluate", [](gtsam::HybridConditional *self, const gtsam::HybridValues &values) { return self->evaluate(values); }, nb::arg("values"))
      .def("asMixture", [](gtsam::HybridConditional *self) { return self->asMixture(); })
      .def("asGaussian", [](gtsam::HybridConditional *self) { return self->asGaussian(); })
      .def("asDiscrete", [](gtsam::HybridConditional *self) { return self->asDiscrete(); })
      .def("inner", [](gtsam::HybridConditional *self) { return self->inner(); })
      .def("error", [](gtsam::HybridConditional *self, const gtsam::HybridValues &values) { return self->error(values); }, nb::arg("values"))
      .def("__call__", &gtsam::HybridConditional::operator());

  nb::class_<gtsam::GaussianMixtureFactor, gtsam::HybridFactor>(m_, "GaussianMixtureFactor")
      .def(nb::init<const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const std::vector<gtsam::GaussianFactor::shared_ptr> &>(), nb::arg("continuousKeys"), nb::arg("discreteKeys"), nb::arg("factorsList"))
      .def("print", [](gtsam::GaussianMixtureFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "GaussianMixtureFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianMixtureFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "GaussianMixtureFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  nb::class_<gtsam::GaussianMixture, gtsam::HybridFactor>(m_, "GaussianMixture")
      .def(nb::init<const gtsam::KeyVector &, const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const std::vector<gtsam::GaussianConditional::shared_ptr> &>(), nb::arg("continuousFrontals"), nb::arg("continuousParents"), nb::arg("discreteParents"), nb::arg("conditionalsList"))
      .def("likelihood", [](gtsam::GaussianMixture *self, const gtsam::VectorValues &frontals) { return self->likelihood(frontals); }, nb::arg("frontals"))
      .def("print", [](gtsam::GaussianMixture *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "GaussianMixture\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::GaussianMixture &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "GaussianMixture\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  nb::class_<gtsam::HybridBayesTreeClique>(m_, "HybridBayesTreeClique")
      .def(nb::init<>())
      .def(nb::init<const boost::shared_ptr<gtsam::HybridConditional>>(), nb::arg("conditional"))
      .def("conditional", [](gtsam::HybridBayesTreeClique *self) { return self->conditional(); })
      .def("isRoot", [](gtsam::HybridBayesTreeClique *self) { return self->isRoot(); });

  nb::class_<gtsam::HybridBayesTree>(m_, "HybridBayesTree")
      .def(nb::init<>())
      .def("print", [](gtsam::HybridBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "HybridBayesTree\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridBayesTree &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "HybridBayesTree\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridBayesTree *self, const gtsam::HybridBayesTree &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("size", [](gtsam::HybridBayesTree *self) { return self->size(); })
      .def("empty", [](gtsam::HybridBayesTree *self) { return self->empty(); })
      .def("optimize", [](gtsam::HybridBayesTree *self) { return self->optimize(); })
      .def("dot", [](gtsam::HybridBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->dot(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__getitem__", &gtsam::HybridBayesTree::operator[]);

  nb::class_<gtsam::HybridBayesNet>(m_, "HybridBayesNet")
      .def(nb::init<>())
      .def("push_back", [](gtsam::HybridBayesNet *self, const boost::shared_ptr<gtsam::GaussianMixture> s) { self->push_back(s); }, nb::arg("s"))
      .def("push_back", [](gtsam::HybridBayesNet *self, const boost::shared_ptr<gtsam::GaussianConditional> s) { self->push_back(s); }, nb::arg("s"))
      .def("push_back", [](gtsam::HybridBayesNet *self, const boost::shared_ptr<gtsam::DiscreteConditional> s) { self->push_back(s); }, nb::arg("s"))
      .def("empty", [](gtsam::HybridBayesNet *self) { return self->empty(); })
      .def("size", [](gtsam::HybridBayesNet *self) { return self->size(); })
      .def("keys", [](gtsam::HybridBayesNet *self) { return self->keys(); })
      .def("at", [](gtsam::HybridBayesNet *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("logProbability", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &values) { return self->logProbability(values); }, nb::arg("values"))
      .def("evaluate", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &values) { return self->evaluate(values); }, nb::arg("values"))
      .def("error", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &values) { return self->error(values); }, nb::arg("values"))
      .def("toFactorGraph", [](gtsam::HybridBayesNet *self, const gtsam::VectorValues &measurements) { return self->toFactorGraph(measurements); }, nb::arg("measurements"))
      .def("optimize", [](gtsam::HybridBayesNet *self) { return self->optimize(); })
      .def("sample", [](gtsam::HybridBayesNet *self, const gtsam::HybridValues &given) { return self->sample(given); }, nb::arg("given"))
      .def("sample", [](gtsam::HybridBayesNet *self) { return self->sample(); })
      .def("print", [](gtsam::HybridBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "HybridBayesNet\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridBayesNet &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "HybridBayesNet\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::HybridBayesNet *self, const gtsam::HybridBayesNet &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("dot", [](gtsam::HybridBayesNet *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::HybridBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter());

  nb::class_<gtsam::HybridGaussianFactorGraph>(m_, "HybridGaussianFactorGraph")
      .def(nb::init<>())
      .def(nb::init<const gtsam::HybridBayesNet &>(), nb::arg("bayesNet"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const boost::shared_ptr<gtsam::HybridFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const boost::shared_ptr<gtsam::HybridConditional> conditional) { self->push_back(conditional); }, nb::arg("conditional"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridGaussianFactorGraph &graph) { self->push_back(graph); }, nb::arg("graph"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridBayesNet &bayesNet) { self->push_back(bayesNet); }, nb::arg("bayesNet"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridBayesTree &bayesTree) { self->push_back(bayesTree); }, nb::arg("bayesTree"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, const boost::shared_ptr<gtsam::GaussianMixtureFactor> gmm) { self->push_back(gmm); }, nb::arg("gmm"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, boost::shared_ptr<gtsam::DecisionTreeFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("push_back", [](gtsam::HybridGaussianFactorGraph *self, boost::shared_ptr<gtsam::JacobianFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("empty", [](gtsam::HybridGaussianFactorGraph *self) { return self->empty(); })
      .def("remove", [](gtsam::HybridGaussianFactorGraph *self, size_t i) { self->remove(i); }, nb::arg("i"))
      .def("size", [](gtsam::HybridGaussianFactorGraph *self) { return self->size(); })
      .def("keys", [](gtsam::HybridGaussianFactorGraph *self) { return self->keys(); })
      .def("at", [](gtsam::HybridGaussianFactorGraph *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("print", [](gtsam::HybridGaussianFactorGraph *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::HybridGaussianFactorGraph &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridGaussianFactorGraph &fg, double tol) { return self->equals(fg, tol); }, nb::arg("fg"), nb::arg("tol") = 1e-9)
      .def("error", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridValues &values) { return self->error(values); }, nb::arg("values"))
      .def("probPrime", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::HybridValues &values) { return self->probPrime(values); }, nb::arg("values"))
      .def("eliminateSequential", [](gtsam::HybridGaussianFactorGraph *self) { return self->eliminateSequential(); })
      .def("eliminateSequential", [](gtsam::HybridGaussianFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateSequential(type); }, nb::arg("type"))
      .def("eliminateSequential", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateSequential(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialSequential(ordering); }, nb::arg("ordering"))
      .def("eliminateMultifrontal", [](gtsam::HybridGaussianFactorGraph *self) { return self->eliminateMultifrontal(); })
      .def("eliminateMultifrontal", [](gtsam::HybridGaussianFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateMultifrontal(type); }, nb::arg("type"))
      .def("eliminateMultifrontal", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialMultifrontal", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialMultifrontal(ordering); }, nb::arg("ordering"))
      .def("dot", [](gtsam::HybridGaussianFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter());

  nb::class_<gtsam::HybridNonlinearFactorGraph>(m_, "HybridNonlinearFactorGraph")
      .def(nb::init<>())
      .def(nb::init<const gtsam::HybridNonlinearFactorGraph &>(), nb::arg("graph"))
      .def("push_back", [](gtsam::HybridNonlinearFactorGraph *self, boost::shared_ptr<gtsam::HybridFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("push_back", [](gtsam::HybridNonlinearFactorGraph *self, boost::shared_ptr<gtsam::NonlinearFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("push_back", [](gtsam::HybridNonlinearFactorGraph *self, boost::shared_ptr<gtsam::DiscreteFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("linearize", [](gtsam::HybridNonlinearFactorGraph *self, const gtsam::Values &continuousValues) { return self->linearize(continuousValues); }, nb::arg("continuousValues"))
      .def("empty", [](gtsam::HybridNonlinearFactorGraph *self) { return self->empty(); })
      .def("remove", [](gtsam::HybridNonlinearFactorGraph *self, size_t i) { self->remove(i); }, nb::arg("i"))
      .def("size", [](gtsam::HybridNonlinearFactorGraph *self) { return self->size(); })
      .def("keys", [](gtsam::HybridNonlinearFactorGraph *self) { return self->keys(); })
      .def("at", [](gtsam::HybridNonlinearFactorGraph *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("print", [](gtsam::HybridNonlinearFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "HybridNonlinearFactorGraph\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::HybridNonlinearFactorGraph &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "HybridNonlinearFactorGraph\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  nb::class_<gtsam::MixtureFactor, gtsam::HybridFactor>(m_, "MixtureFactor")
      // TODO: gtsam::DecisionTree still needs bindings
      // .def(nb::init<const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const gtsam::DecisionTree<gtsam::Key, boost::shared_ptr<gtsam::NonlinearFactor>> &, bool>(), nb::arg("keys"), nb::arg("discreteKeys"), nb::arg("factors"), nb::arg("normalized") = false)
      .def(nb::init<const gtsam::KeyVector &, const gtsam::DiscreteKeys &, const std::vector<boost::shared_ptr<gtsam::NonlinearFactor>> &, bool>(), nb::arg("keys"), nb::arg("discreteKeys"), nb::arg("factors"), nb::arg("normalized") = false)
      .def("error", [](gtsam::MixtureFactor *self, const gtsam::Values &continuousValues, const gtsam::DiscreteValues &discreteValues) { return self->error(continuousValues, discreteValues); }, nb::arg("continuousValues"), nb::arg("discreteValues"))
      .def("nonlinearFactorLogNormalizingConstant", [](gtsam::MixtureFactor *self, const boost::shared_ptr<gtsam::NonlinearFactor> factor, const gtsam::Values &values) { return self->nonlinearFactorLogNormalizingConstant(factor, values); }, nb::arg("factor"), nb::arg("values"))
      .def("linearize", [](gtsam::MixtureFactor *self, const gtsam::Values &continuousValues) { return self->linearize(continuousValues); }, nb::arg("continuousValues"))
      .def("print", [](gtsam::MixtureFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "MixtureFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::MixtureFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "MixtureFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
}
