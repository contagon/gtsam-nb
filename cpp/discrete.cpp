/**
 * @file    discrete.cpp
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
#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "utils/boost_shared_ptr.h"

// These are the included headers listed in `gtsam.i`
#include "gtsam/discrete/DecisionTreeFactor.h"
#include "gtsam/discrete/DiscreteBayesNet.h"
#include "gtsam/discrete/DiscreteBayesTree.h"
#include "gtsam/discrete/DiscreteConditional.h"
#include "gtsam/discrete/DiscreteDistribution.h"
#include "gtsam/discrete/DiscreteEliminationTree.h"
#include "gtsam/discrete/DiscreteFactor.h"
#include "gtsam/discrete/DiscreteFactorGraph.h"
#include "gtsam/discrete/DiscreteJunctionTree.h"
#include "gtsam/discrete/DiscreteKey.h"
#include "gtsam/discrete/DiscreteLookupDAG.h"
#include "gtsam/hybrid/HybridValues.h"

using namespace std;

namespace nb = nanobind;

void discrete(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of discrete";

  nb::class_<gtsam::DiscreteKeys>(m_, "DiscreteKeys")
      .def(nb::init<>())
      .def("size", [](gtsam::DiscreteKeys *self) { return self->size(); })
      .def("empty", [](gtsam::DiscreteKeys *self) { return self->empty(); })
      .def("at", [](gtsam::DiscreteKeys *self, size_t n) { return self->at(n); }, nb::arg("n"))
      .def("push_back", [](gtsam::DiscreteKeys *self, const gtsam::DiscreteKey &point_pair) { self->push_back(point_pair); }, nb::arg("point_pair"));

  nb::class_<gtsam::DiscreteFactor, gtsam::Factor>(m_, "DiscreteFactor")
      .def("print", [](gtsam::DiscreteFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "DiscreteFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "DiscreteFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::DiscreteFactor *self, const gtsam::DiscreteFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("__call__", &gtsam::DiscreteFactor::operator());

  nb::class_<gtsam::DecisionTreeFactor, gtsam::DiscreteFactor>(m_, "DecisionTreeFactor")
      .def(nb::init<>())
      .def(nb::init<const gtsam::DiscreteKey &, const std::vector<double> &>(), nb::arg("key"), nb::arg("spec"))
      .def(nb::init<const gtsam::DiscreteKey &, string>(), nb::arg("key"), nb::arg("table"))
      .def(nb::init<const gtsam::DiscreteKeys &, const std::vector<double> &>(), nb::arg("keys"), nb::arg("table"))
      .def(nb::init<const gtsam::DiscreteKeys &, string>(), nb::arg("keys"), nb::arg("table"))
      .def(nb::init<const std::vector<gtsam::DiscreteKey> &, const std::vector<double> &>(), nb::arg("keys"), nb::arg("table"))
      .def(nb::init<const std::vector<gtsam::DiscreteKey> &, string>(), nb::arg("keys"), nb::arg("table"))
      .def(nb::init<const gtsam::DiscreteConditional &>(), nb::arg("c"))
      .def("print", [](gtsam::DecisionTreeFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "DecisionTreeFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DecisionTreeFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "DecisionTreeFactor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::DecisionTreeFactor *self, const gtsam::DecisionTreeFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("cardinality", [](gtsam::DecisionTreeFactor *self, gtsam::Key j) { return self->cardinality(j); }, nb::arg("j"))
      .def("sum", [](gtsam::DecisionTreeFactor *self, size_t nrFrontals) { return self->sum(nrFrontals); }, nb::arg("nrFrontals"))
      .def("sum", [](gtsam::DecisionTreeFactor *self, const gtsam::Ordering &keys) { return self->sum(keys); }, nb::arg("keys"))
      .def("max", [](gtsam::DecisionTreeFactor *self, size_t nrFrontals) { return self->max(nrFrontals); }, nb::arg("nrFrontals"))
      .def("max", [](gtsam::DecisionTreeFactor *self, const gtsam::Ordering &keys) { return self->max(keys); }, nb::arg("keys"))
      .def("dot", [](gtsam::DecisionTreeFactor *self, const gtsam::KeyFormatter &keyFormatter, bool showZero) { return self->dot(keyFormatter, showZero); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("showZero") = true)
      .def("enumerate", [](gtsam::DecisionTreeFactor *self) { return self->enumerate(); })
      .def("_repr_markdown_", [](gtsam::DecisionTreeFactor *self, const gtsam::KeyFormatter &keyFormatter) { return self->markdown(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_markdown_", [](gtsam::DecisionTreeFactor *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->markdown(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("_repr_html_", [](gtsam::DecisionTreeFactor *self, const gtsam::KeyFormatter &keyFormatter) { return self->html(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_html_", [](gtsam::DecisionTreeFactor *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->html(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("__call__", &gtsam::DecisionTreeFactor::operator())
      .def(nb::self * nb::self)
      .def(nb::self / nb::self);

  nb::class_<gtsam::DiscreteConditional, gtsam::DecisionTreeFactor>(m_, "DiscreteConditional")
      .def(nb::init<>())
      .def(nb::init<size_t, const gtsam::DecisionTreeFactor &>(), nb::arg("nFrontals"), nb::arg("f"))
      .def(nb::init<const gtsam::DiscreteKey &, string>(), nb::arg("key"), nb::arg("spec"))
      .def(nb::init<const gtsam::DiscreteKey &, const gtsam::DiscreteKeys &, string>(), nb::arg("key"), nb::arg("parents"), nb::arg("spec"))
      .def(nb::init<const gtsam::DiscreteKey &, const std::vector<gtsam::DiscreteKey> &, string>(), nb::arg("key"), nb::arg("parents"), nb::arg("spec"))
      .def(nb::init<const gtsam::DecisionTreeFactor &, const gtsam::DecisionTreeFactor &>(), nb::arg("joint"), nb::arg("marginal"))
      .def(nb::init<const gtsam::DecisionTreeFactor &, const gtsam::DecisionTreeFactor &, const gtsam::Ordering &>(), nb::arg("joint"), nb::arg("marginal"), nb::arg("orderedKeys"))
      .def("logNormalizationConstant", [](gtsam::DiscreteConditional *self) { return self->logNormalizationConstant(); })
      .def("logProbability", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteValues &values) { return self->logProbability(values); }, nb::arg("values"))
      .def("evaluate", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteValues &values) { return self->evaluate(values); }, nb::arg("values"))
      .def("error", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteValues &values) { return self->error(values); }, nb::arg("values"))
      .def("marginal", [](gtsam::DiscreteConditional *self, gtsam::Key key) { return self->marginal(key); }, nb::arg("key"))
      .def("print", [](gtsam::DiscreteConditional *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "Discrete Conditional\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteConditional &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "Discrete Conditional\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteConditional &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("firstFrontalKey", [](gtsam::DiscreteConditional *self) { return self->firstFrontalKey(); })
      .def("nrFrontals", [](gtsam::DiscreteConditional *self) { return self->nrFrontals(); })
      .def("nrParents", [](gtsam::DiscreteConditional *self) { return self->nrParents(); })
      .def("printSignature", [](gtsam::DiscreteConditional *self, string s, const gtsam::KeyFormatter &formatter) { self->printSignature(s, formatter); }, nb::arg("s") = "Discrete Conditional: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("choose", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteValues &given) { return self->choose(given); }, nb::arg("given"))
      .def("likelihood", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteValues &frontalValues) { return self->likelihood(frontalValues); }, nb::arg("frontalValues"))
      .def("likelihood", [](gtsam::DiscreteConditional *self, size_t value) { return self->likelihood(value); }, nb::arg("value"))
      .def("sample", [](gtsam::DiscreteConditional *self, const gtsam::DiscreteValues &parentsValues) { return self->sample(parentsValues); }, nb::arg("parentsValues"))
      .def("sample", [](gtsam::DiscreteConditional *self, size_t value) { return self->sample(value); }, nb::arg("value"))
      .def("sample", [](gtsam::DiscreteConditional *self) { return self->sample(); })
      .def("sampleInPlace", [](gtsam::DiscreteConditional *self, gtsam::DiscreteValues *parentsValues) { self->sampleInPlace(parentsValues); }, nb::arg("parentsValues"))
      .def("_repr_markdown_", [](gtsam::DiscreteConditional *self, const gtsam::KeyFormatter &keyFormatter) { return self->markdown(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_markdown_", [](gtsam::DiscreteConditional *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->markdown(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("_repr_html_", [](gtsam::DiscreteConditional *self, const gtsam::KeyFormatter &keyFormatter) { return self->html(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_html_", [](gtsam::DiscreteConditional *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->html(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("logProbability", [](gtsam::DiscreteConditional *self, const gtsam::HybridValues &x) { return self->logProbability(x); }, nb::arg("x"))
      .def("evaluate", [](gtsam::DiscreteConditional *self, const gtsam::HybridValues &x) { return self->evaluate(x); }, nb::arg("x"))
      .def("error", [](gtsam::DiscreteConditional *self, const gtsam::HybridValues &x) { return self->error(x); }, nb::arg("x"))
      .def(nb::self * nb::self);

  nb::class_<gtsam::DiscreteDistribution, gtsam::DiscreteConditional>(m_, "DiscreteDistribution")
      .def(nb::init<>())
      .def(nb::init<const gtsam::DecisionTreeFactor &>(), nb::arg("f"))
      .def(nb::init<const gtsam::DiscreteKey &, string>(), nb::arg("key"), nb::arg("spec"))
      .def(nb::init<const gtsam::DiscreteKey &, std::vector<double>>(), nb::arg("key"), nb::arg("spec"))
      .def("print", [](gtsam::DiscreteDistribution *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "Discrete Prior\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteDistribution &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "Discrete Prior\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("pmf", [](gtsam::DiscreteDistribution *self) { return self->pmf(); })
      .def("argmax", [](gtsam::DiscreteDistribution *self) { return self->argmax(); })
      .def("__call__", &gtsam::DiscreteDistribution::operator());

  nb::class_<gtsam::DiscreteBayesNet>(m_, "DiscreteBayesNet")
      .def(nb::init<>())
      .def("add", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteConditional &s) { self->add(s); }, nb::arg("s"))
      .def("add", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteKey &key, string spec) { self->add(key, spec); }, nb::arg("key"), nb::arg("spec"))
      .def("add", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteKey &key, const gtsam::DiscreteKeys &parents, string spec) { self->add(key, parents, spec); }, nb::arg("key"), nb::arg("parents"), nb::arg("spec"))
      .def("add", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteKey &key, const std::vector<gtsam::DiscreteKey> &parents, string spec) { self->add(key, parents, spec); }, nb::arg("key"), nb::arg("parents"), nb::arg("spec"))
      .def("empty", [](gtsam::DiscreteBayesNet *self) { return self->empty(); })
      .def("size", [](gtsam::DiscreteBayesNet *self) { return self->size(); })
      .def("keys", [](gtsam::DiscreteBayesNet *self) { return self->keys(); })
      .def("at", [](gtsam::DiscreteBayesNet *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("print", [](gtsam::DiscreteBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "DiscreteBayesNet\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteBayesNet &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "DiscreteBayesNet\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteBayesNet &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("logProbability", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteValues &values) { return self->logProbability(values); }, nb::arg("values"))
      .def("evaluate", [](gtsam::DiscreteBayesNet *self, const gtsam::DiscreteValues &values) { return self->evaluate(values); }, nb::arg("values"))
      .def("sample", [](gtsam::DiscreteBayesNet *self) { return self->sample(); })
      .def("sample", [](gtsam::DiscreteBayesNet *self, gtsam::DiscreteValues given) { return self->sample(given); }, nb::arg("given"))
      .def("dot", [](gtsam::DiscreteBayesNet *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::DiscreteBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("_repr_markdown_", [](gtsam::DiscreteBayesNet *self, const gtsam::KeyFormatter &keyFormatter) { return self->markdown(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_markdown_", [](gtsam::DiscreteBayesNet *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->markdown(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("_repr_html_", [](gtsam::DiscreteBayesNet *self, const gtsam::KeyFormatter &keyFormatter) { return self->html(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_html_", [](gtsam::DiscreteBayesNet *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->html(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("__call__", &gtsam::DiscreteBayesNet::operator());

  nb::class_<gtsam::DiscreteBayesTreeClique>(m_, "DiscreteBayesTreeClique")
      .def(nb::init<>())
      .def(nb::init<const boost::shared_ptr<gtsam::DiscreteConditional>>(), nb::arg("conditional"))
      .def("conditional", [](gtsam::DiscreteBayesTreeClique *self) { return self->conditional(); })
      .def("isRoot", [](gtsam::DiscreteBayesTreeClique *self) { return self->isRoot(); })
      .def("nrChildren", [](gtsam::DiscreteBayesTreeClique *self) { return self->nrChildren(); })
      .def("print", [](gtsam::DiscreteBayesTreeClique *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "DiscreteBayesTreeClique", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteBayesTreeClique &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "DiscreteBayesTreeClique", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("printSignature", [](gtsam::DiscreteBayesTreeClique *self, const string &s, const gtsam::KeyFormatter &formatter) { self->printSignature(s, formatter); }, nb::arg("s") = "Clique: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("evaluate", [](gtsam::DiscreteBayesTreeClique *self, const gtsam::DiscreteValues &values) { return self->evaluate(values); }, nb::arg("values"))
      .def("__getitem__", &gtsam::DiscreteBayesTreeClique::operator[])
      .def("__call__", &gtsam::DiscreteBayesTreeClique::operator());

  nb::class_<gtsam::DiscreteBayesTree>(m_, "DiscreteBayesTree")
      .def(nb::init<>())
      .def("print", [](gtsam::DiscreteBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "DiscreteBayesTree\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteBayesTree &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "DiscreteBayesTree\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::DiscreteBayesTree *self, const gtsam::DiscreteBayesTree &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("size", [](gtsam::DiscreteBayesTree *self) { return self->size(); })
      .def("empty", [](gtsam::DiscreteBayesTree *self) { return self->empty(); })
      .def("evaluate", [](gtsam::DiscreteBayesTree *self, const gtsam::DiscreteValues &values) { return self->evaluate(values); }, nb::arg("values"))
      .def("dot", [](gtsam::DiscreteBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->dot(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("saveGraph", [](gtsam::DiscreteBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { self->saveGraph(s, keyFormatter); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_markdown_", [](gtsam::DiscreteBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->markdown(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_markdown_", [](gtsam::DiscreteBayesTree *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->markdown(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("_repr_html_", [](gtsam::DiscreteBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->html(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_html_", [](gtsam::DiscreteBayesTree *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->html(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("__getitem__", &gtsam::DiscreteBayesTree::operator[])
      .def("__call__", &gtsam::DiscreteBayesTree::operator());

  nb::class_<gtsam::DiscreteLookupTable, gtsam::DiscreteConditional>(m_, "DiscreteLookupTable")
      .def(nb::init<size_t, const gtsam::DiscreteKeys &, const gtsam::DecisionTreeFactor::ADT &>(), nb::arg("nFrontals"), nb::arg("keys"), nb::arg("potentials"))
      .def("print", [](gtsam::DiscreteLookupTable *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "Discrete Lookup Table: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteLookupTable &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "Discrete Lookup Table: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("argmax", [](gtsam::DiscreteLookupTable *self, const gtsam::DiscreteValues &parentsValues) { return self->argmax(parentsValues); }, nb::arg("parentsValues"));

  nb::class_<gtsam::DiscreteLookupDAG>(m_, "DiscreteLookupDAG")
      .def(nb::init<>())
      .def("push_back", [](gtsam::DiscreteLookupDAG *self, const boost::shared_ptr<gtsam::DiscreteLookupTable> table) { self->push_back(table); }, nb::arg("table"))
      .def("empty", [](gtsam::DiscreteLookupDAG *self) { return self->empty(); })
      .def("size", [](gtsam::DiscreteLookupDAG *self) { return self->size(); })
      .def("keys", [](gtsam::DiscreteLookupDAG *self) { return self->keys(); })
      .def("at", [](gtsam::DiscreteLookupDAG *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("print", [](gtsam::DiscreteLookupDAG *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "DiscreteLookupDAG\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteLookupDAG &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "DiscreteLookupDAG\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("argmax", [](gtsam::DiscreteLookupDAG *self) { return self->argmax(); })
      .def("argmax", [](gtsam::DiscreteLookupDAG *self, gtsam::DiscreteValues given) { return self->argmax(given); }, nb::arg("given"));

  nb::class_<gtsam::DiscreteFactorGraph>(m_, "DiscreteFactorGraph")
      .def(nb::init<>())
      .def(nb::init<const gtsam::DiscreteBayesNet &>(), nb::arg("bayesNet"))
      .def("push_back", [](gtsam::DiscreteFactorGraph *self, const boost::shared_ptr<gtsam::DiscreteFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("push_back", [](gtsam::DiscreteFactorGraph *self, const boost::shared_ptr<gtsam::DiscreteConditional> conditional) { self->push_back(conditional); }, nb::arg("conditional"))
      .def("push_back", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteFactorGraph &graph) { self->push_back(graph); }, nb::arg("graph"))
      .def("push_back", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteBayesNet &bayesNet) { self->push_back(bayesNet); }, nb::arg("bayesNet"))
      .def("push_back", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteBayesTree &bayesTree) { self->push_back(bayesTree); }, nb::arg("bayesTree"))
      .def("add", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteKey &j, string spec) { self->add(j, spec); }, nb::arg("j"), nb::arg("spec"))
      .def("add", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteKey &j, const std::vector<double> &spec) { self->add(j, spec); }, nb::arg("j"), nb::arg("spec"))
      .def("add", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteKeys &keys, string spec) { self->add(keys, spec); }, nb::arg("keys"), nb::arg("spec"))
      .def("add", [](gtsam::DiscreteFactorGraph *self, const std::vector<gtsam::DiscreteKey> &keys, string spec) { self->add(keys, spec); }, nb::arg("keys"), nb::arg("spec"))
      .def("add", [](gtsam::DiscreteFactorGraph *self, const std::vector<gtsam::DiscreteKey> &keys, const std::vector<double> &spec) { self->add(keys, spec); }, nb::arg("keys"), nb::arg("spec"))
      .def("empty", [](gtsam::DiscreteFactorGraph *self) { return self->empty(); })
      .def("size", [](gtsam::DiscreteFactorGraph *self) { return self->size(); })
      .def("keys", [](gtsam::DiscreteFactorGraph *self) { return self->keys(); })
      .def("at", [](gtsam::DiscreteFactorGraph *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("print", [](gtsam::DiscreteFactorGraph *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::DiscreteFactorGraph &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::DiscreteFactorGraph *self, const gtsam::DiscreteFactorGraph &fg, double tol) { return self->equals(fg, tol); }, nb::arg("fg"), nb::arg("tol") = 1e-9)
      .def("product", [](gtsam::DiscreteFactorGraph *self) { return self->product(); })
      .def("optimize", [](gtsam::DiscreteFactorGraph *self) { return self->optimize(); })
      .def("sumProduct", [](gtsam::DiscreteFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->sumProduct(type); }, nb::arg("type") = gtsam::Ordering::COLAMD)
      .def("sumProduct", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering) { return self->sumProduct(ordering); }, nb::arg("ordering"))
      .def("maxProduct", [](gtsam::DiscreteFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->maxProduct(type); }, nb::arg("type") = gtsam::Ordering::COLAMD)
      .def("maxProduct", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering) { return self->maxProduct(ordering); }, nb::arg("ordering"))
      .def("eliminateSequential", [](gtsam::DiscreteFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateSequential(type); }, nb::arg("type") = gtsam::Ordering::COLAMD)
      .def("eliminateSequential", [](gtsam::DiscreteFactorGraph *self, gtsam::Ordering::OrderingType type, const gtsam::DiscreteFactorGraph::Eliminate &function) { return self->eliminateSequential(type, function); }, nb::arg("type"), nb::arg("function"))
      .def("eliminateSequential", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateSequential(ordering); }, nb::arg("ordering"))
      .def("eliminateSequential", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering, const gtsam::DiscreteFactorGraph::Eliminate &function) { return self->eliminateSequential(ordering, function); }, nb::arg("ordering"), nb::arg("function"))
      .def("eliminatePartialSequential", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialSequential(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering, const gtsam::DiscreteFactorGraph::Eliminate &function) { return self->eliminatePartialSequential(ordering, function); }, nb::arg("ordering"), nb::arg("function"))
      .def("eliminateMultifrontal", [](gtsam::DiscreteFactorGraph *self, gtsam::Ordering::OrderingType type) { return self->eliminateMultifrontal(type); }, nb::arg("type") = gtsam::Ordering::COLAMD)
      .def("eliminateMultifrontal", [](gtsam::DiscreteFactorGraph *self, gtsam::Ordering::OrderingType type, const gtsam::DiscreteFactorGraph::Eliminate &function) { return self->eliminateMultifrontal(type, function); }, nb::arg("type"), nb::arg("function"))
      .def("eliminateMultifrontal", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminateMultifrontal", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering, const gtsam::DiscreteFactorGraph::Eliminate &function) { return self->eliminateMultifrontal(ordering, function); }, nb::arg("ordering"), nb::arg("function"))
      .def("eliminatePartialMultifrontal", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialMultifrontal", [](gtsam::DiscreteFactorGraph *self, const gtsam::Ordering &ordering, const gtsam::DiscreteFactorGraph::Eliminate &function) { return self->eliminatePartialMultifrontal(ordering, function); }, nb::arg("ordering"), nb::arg("function"))
      .def("dot", [](gtsam::DiscreteFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::DiscreteFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("_repr_markdown_", [](gtsam::DiscreteFactorGraph *self, const gtsam::KeyFormatter &keyFormatter) { return self->markdown(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_markdown_", [](gtsam::DiscreteFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->markdown(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("_repr_html_", [](gtsam::DiscreteFactorGraph *self, const gtsam::KeyFormatter &keyFormatter) { return self->html(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("_repr_html_", [](gtsam::DiscreteFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return self->html(keyFormatter, names); }, nb::arg("keyFormatter"), nb::arg("names"))
      .def("__call__", &gtsam::DiscreteFactorGraph::operator());

  nb::class_<gtsam::DiscreteEliminationTree>(m_, "DiscreteEliminationTree")
      .def(nb::init<const gtsam::DiscreteFactorGraph &, const gtsam::VariableIndex &, const gtsam::Ordering &>(), nb::arg("factorGraph"), nb::arg("structure"), nb::arg("order"))
      .def(nb::init<const gtsam::DiscreteFactorGraph &, const gtsam::Ordering &>(), nb::arg("factorGraph"), nb::arg("order"))
      .def("print", [](gtsam::DiscreteEliminationTree *self, string name, const gtsam::KeyFormatter &formatter) { /* nb::scoped_ostream_redirect output; */ self->print(name, formatter); }, nb::arg("name") = "EliminationTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteEliminationTree &self, string name, const gtsam::KeyFormatter &formatter) {
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str(); }, nb::arg("name") = "EliminationTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::DiscreteEliminationTree *self, const gtsam::DiscreteEliminationTree &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9);

  nb::class_<gtsam::DiscreteCluster>(m_, "DiscreteCluster")
      .def("nrChildren", [](gtsam::DiscreteCluster *self) { return self->nrChildren(); })
      .def("print", [](gtsam::DiscreteCluster *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteCluster &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def_rw("orderedFrontalKeys", &gtsam::DiscreteCluster::orderedFrontalKeys)
      .def_rw("factors", &gtsam::DiscreteCluster::factors)
      .def("__getitem__", &gtsam::DiscreteCluster::operator[]);

  nb::class_<gtsam::DiscreteJunctionTree>(m_, "DiscreteJunctionTree")
      .def(nb::init<const gtsam::DiscreteEliminationTree &>(), nb::arg("eliminationTree"))
      .def("print", [](gtsam::DiscreteJunctionTree *self, string name, const gtsam::KeyFormatter &formatter) { /* nb::scoped_ostream_redirect output; */ self->print(name, formatter); }, nb::arg("name") = "JunctionTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::DiscreteJunctionTree &self, string name, const gtsam::KeyFormatter &formatter) {
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str(); }, nb::arg("name") = "JunctionTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("nrRoots", [](gtsam::DiscreteJunctionTree *self) { return self->nrRoots(); })
      .def("__getitem__", &gtsam::DiscreteJunctionTree::operator[]);

  m_.def("cartesianProduct", [](const gtsam::DiscreteKeys &keys) { return gtsam::cartesianProduct(keys); }, nb::arg("keys"));
  m_.def("markdown", [](const gtsam::DiscreteValues &values, const gtsam::KeyFormatter &keyFormatter) { return gtsam::markdown(values, keyFormatter); }, nb::arg("values"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("markdown", [](const gtsam::DiscreteValues &values, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return gtsam::markdown(values, keyFormatter, names); }, nb::arg("values"), nb::arg("keyFormatter"), nb::arg("names"));
  m_.def("html", [](const gtsam::DiscreteValues &values, const gtsam::KeyFormatter &keyFormatter) { return gtsam::html(values, keyFormatter); }, nb::arg("values"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("html", [](const gtsam::DiscreteValues &values, const gtsam::KeyFormatter &keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names) { return gtsam::html(values, keyFormatter, names); }, nb::arg("values"), nb::arg("keyFormatter"), nb::arg("names"));
  m_.def("EliminateDiscrete", [](const gtsam::DiscreteFactorGraph &factors, const gtsam::Ordering &frontalKeys) { return gtsam::EliminateDiscrete(factors, frontalKeys); }, nb::arg("factors"), nb::arg("frontalKeys"));
  m_.def("EliminateForMPE", [](const gtsam::DiscreteFactorGraph &factors, const gtsam::Ordering &frontalKeys) { return gtsam::EliminateForMPE(factors, frontalKeys); }, nb::arg("factors"), nb::arg("frontalKeys"));
}
