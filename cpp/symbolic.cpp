/**
 * @file    symbolic.cpp
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

#include "utils/boost_shared_ptr.h"

// These are the included headers listed in `gtsam.i`
#include "gtsam/symbolic/SymbolicBayesNet.h"
#include "gtsam/symbolic/SymbolicBayesTree.h"
#include "gtsam/symbolic/SymbolicConditional.h"
#include "gtsam/symbolic/SymbolicEliminationTree.h"
#include "gtsam/symbolic/SymbolicFactor.h"
#include "gtsam/symbolic/SymbolicFactorGraph.h"
#include "gtsam/symbolic/SymbolicJunctionTree.h"

using namespace std;

namespace nb = nanobind;

void symbolic(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of symbolic";

  nb::class_<gtsam::SymbolicFactor, gtsam::Factor>(m_, "SymbolicFactor")
      .def(nb::init<const gtsam::SymbolicFactor &>(), nb::arg("f"))
      .def(nb::init<>())
      .def(nb::init<size_t>(), nb::arg("j"))
      .def(nb::init<size_t, size_t>(), nb::arg("j1"), nb::arg("j2"))
      .def(nb::init<size_t, size_t, size_t>(), nb::arg("j1"), nb::arg("j2"), nb::arg("j3"))
      .def(nb::init<size_t, size_t, size_t, size_t>(), nb::arg("j1"), nb::arg("j2"), nb::arg("j3"), nb::arg("j4"))
      .def(nb::init<size_t, size_t, size_t, size_t, size_t>(), nb::arg("j1"), nb::arg("j2"), nb::arg("j3"), nb::arg("j4"), nb::arg("j5"))
      .def(nb::init<size_t, size_t, size_t, size_t, size_t, size_t>(), nb::arg("j1"), nb::arg("j2"), nb::arg("j3"), nb::arg("j4"), nb::arg("j5"), nb::arg("j6"))
      .def("print", [](gtsam::SymbolicFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "SymbolicFactor", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "SymbolicFactor", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::SymbolicFactor *self, const gtsam::SymbolicFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def_static("FromKeys", [](const gtsam::KeyVector &js) { return gtsam::SymbolicFactor::FromKeys(js); }, nb::arg("js"));

  nb::class_<gtsam::SymbolicFactorGraph>(m_, "SymbolicFactorGraph")
      .def(nb::init<>())
      .def(nb::init<const gtsam::SymbolicBayesNet &>(), nb::arg("bayesNet"))
      .def(nb::init<const gtsam::SymbolicBayesTree &>(), nb::arg("bayesTree"))
      .def("push_back", [](gtsam::SymbolicFactorGraph *self, boost::shared_ptr<gtsam::SymbolicFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("print", [](gtsam::SymbolicFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "SymbolicFactorGraph", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicFactorGraph &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "SymbolicFactorGraph", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::SymbolicFactorGraph *self, const gtsam::SymbolicFactorGraph &rhs, double tol) { return self->equals(rhs, tol); }, nb::arg("rhs"), nb::arg("tol"))
      .def("size", [](gtsam::SymbolicFactorGraph *self) { return self->size(); })
      .def("exists", [](gtsam::SymbolicFactorGraph *self, size_t idx) { return self->exists(idx); }, nb::arg("idx"))
      .def("keys", [](gtsam::SymbolicFactorGraph *self) { return self->keys(); })
      .def("push_back", [](gtsam::SymbolicFactorGraph *self, const gtsam::SymbolicFactorGraph &graph) { self->push_back(graph); }, nb::arg("graph"))
      .def("push_back", [](gtsam::SymbolicFactorGraph *self, const gtsam::SymbolicBayesNet &bayesNet) { self->push_back(bayesNet); }, nb::arg("bayesNet"))
      .def("push_back", [](gtsam::SymbolicFactorGraph *self, const gtsam::SymbolicBayesTree &bayesTree) { self->push_back(bayesTree); }, nb::arg("bayesTree"))
      .def("push_factor", [](gtsam::SymbolicFactorGraph *self, size_t key) { self->push_factor(key); }, nb::arg("key"))
      .def("push_factor", [](gtsam::SymbolicFactorGraph *self, size_t key1, size_t key2) { self->push_factor(key1, key2); }, nb::arg("key1"), nb::arg("key2"))
      .def("push_factor", [](gtsam::SymbolicFactorGraph *self, size_t key1, size_t key2, size_t key3) { self->push_factor(key1, key2, key3); }, nb::arg("key1"), nb::arg("key2"), nb::arg("key3"))
      .def("push_factor", [](gtsam::SymbolicFactorGraph *self, size_t key1, size_t key2, size_t key3, size_t key4) { self->push_factor(key1, key2, key3, key4); }, nb::arg("key1"), nb::arg("key2"), nb::arg("key3"), nb::arg("key4"))
      .def("eliminateSequential", [](gtsam::SymbolicFactorGraph *self) { return self->eliminateSequential(); })
      .def("eliminateSequential", [](gtsam::SymbolicFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateSequential(ordering); }, nb::arg("ordering"))
      .def("eliminateMultifrontal", [](gtsam::SymbolicFactorGraph *self) { return self->eliminateMultifrontal(); })
      .def("eliminateMultifrontal", [](gtsam::SymbolicFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminateMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::SymbolicFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialSequential(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialSequential", [](gtsam::SymbolicFactorGraph *self, const gtsam::KeyVector &keys) { return self->eliminatePartialSequential(keys); }, nb::arg("keys"))
      .def("eliminatePartialMultifrontal", [](gtsam::SymbolicFactorGraph *self, const gtsam::Ordering &ordering) { return self->eliminatePartialMultifrontal(ordering); }, nb::arg("ordering"))
      .def("eliminatePartialMultifrontal", [](gtsam::SymbolicFactorGraph *self, const gtsam::KeyVector &keys) { return self->eliminatePartialMultifrontal(keys); }, nb::arg("keys"))
      .def("marginalMultifrontalBayesNet", [](gtsam::SymbolicFactorGraph *self, const gtsam::Ordering &ordering) { return self->marginalMultifrontalBayesNet(ordering); }, nb::arg("ordering"))
      .def("marginalMultifrontalBayesNet", [](gtsam::SymbolicFactorGraph *self, const gtsam::KeyVector &key_vector) { return self->marginalMultifrontalBayesNet(key_vector); }, nb::arg("key_vector"))
      .def("marginalMultifrontalBayesNet", [](gtsam::SymbolicFactorGraph *self, const gtsam::Ordering &ordering, const gtsam::Ordering &marginalizedVariableOrdering) { return self->marginalMultifrontalBayesNet(ordering, marginalizedVariableOrdering); }, nb::arg("ordering"), nb::arg("marginalizedVariableOrdering"))
      .def("marginalMultifrontalBayesNet", [](gtsam::SymbolicFactorGraph *self, const gtsam::KeyVector &key_vector, const gtsam::Ordering &marginalizedVariableOrdering) { return self->marginalMultifrontalBayesNet(key_vector, marginalizedVariableOrdering); }, nb::arg("key_vector"), nb::arg("marginalizedVariableOrdering"))
      .def("marginal", [](gtsam::SymbolicFactorGraph *self, const gtsam::KeyVector &key_vector) { return self->marginal(key_vector); }, nb::arg("key_vector"))
      .def("dot", [](gtsam::SymbolicFactorGraph *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::SymbolicFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter());

  nb::class_<gtsam::SymbolicConditional, gtsam::SymbolicFactor>(m_, "SymbolicConditional")
      .def(nb::init<>())
      .def(nb::init<const gtsam::SymbolicConditional &>(), nb::arg("other"))
      .def(nb::init<size_t>(), nb::arg("key"))
      .def(nb::init<size_t, size_t>(), nb::arg("key"), nb::arg("parent"))
      .def(nb::init<size_t, size_t, size_t>(), nb::arg("key"), nb::arg("parent1"), nb::arg("parent2"))
      .def(nb::init<size_t, size_t, size_t, size_t>(), nb::arg("key"), nb::arg("parent1"), nb::arg("parent2"), nb::arg("parent3"))
      .def("print", [](gtsam::SymbolicConditional *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicConditional &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::SymbolicConditional *self, const gtsam::SymbolicConditional &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("firstFrontalKey", [](gtsam::SymbolicConditional *self) { return self->firstFrontalKey(); })
      .def("nrFrontals", [](gtsam::SymbolicConditional *self) { return self->nrFrontals(); })
      .def("nrParents", [](gtsam::SymbolicConditional *self) { return self->nrParents(); })
      .def_static("FromKeys", [](const gtsam::KeyVector &keys, size_t nrFrontals) { return gtsam::SymbolicConditional::FromKeys(keys, nrFrontals); }, nb::arg("keys"), nb::arg("nrFrontals"));

  nb::class_<gtsam::SymbolicBayesNet>(m_, "SymbolicBayesNet")
      .def(nb::init<>())
      .def(nb::init<const gtsam::SymbolicBayesNet &>(), nb::arg("other"))
      .def("print", [](gtsam::SymbolicBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "SymbolicBayesNet", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicBayesNet &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "SymbolicBayesNet", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::SymbolicBayesNet *self, const gtsam::SymbolicBayesNet &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("size", [](gtsam::SymbolicBayesNet *self) { return self->size(); })
      .def("saveGraph", [](gtsam::SymbolicBayesNet *self, string s) { self->saveGraph(s); }, nb::arg("s"))
      .def("at", [](gtsam::SymbolicBayesNet *self, size_t idx) { return self->at(idx); }, nb::arg("idx"))
      .def("front", [](gtsam::SymbolicBayesNet *self) { return self->front(); })
      .def("back", [](gtsam::SymbolicBayesNet *self) { return self->back(); })
      .def("push_back", [](gtsam::SymbolicBayesNet *self, boost::shared_ptr<gtsam::SymbolicConditional> conditional) { self->push_back(conditional); }, nb::arg("conditional"))
      .def("push_back", [](gtsam::SymbolicBayesNet *self, const gtsam::SymbolicBayesNet &bayesNet) { self->push_back(bayesNet); }, nb::arg("bayesNet"))
      .def("dot", [](gtsam::SymbolicBayesNet *self, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { return self->dot(keyFormatter, writer); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter())
      .def("saveGraph", [](gtsam::SymbolicBayesNet *self, string s, const gtsam::KeyFormatter &keyFormatter, const gtsam::DotWriter &writer) { self->saveGraph(s, keyFormatter, writer); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::DotWriter());

  nb::class_<gtsam::SymbolicEliminationTree>(m_, "SymbolicEliminationTree")
      .def(nb::init<const gtsam::SymbolicFactorGraph &, const gtsam::VariableIndex &, const gtsam::Ordering &>(), nb::arg("factorGraph"), nb::arg("structure"), nb::arg("order"))
      .def(nb::init<const gtsam::SymbolicFactorGraph &, const gtsam::Ordering &>(), nb::arg("factorGraph"), nb::arg("order"))
      .def("print", [](gtsam::SymbolicEliminationTree *self, string name, const gtsam::KeyFormatter &formatter) { /* nb::scoped_ostream_redirect output; */ self->print(name, formatter); }, nb::arg("name") = "EliminationTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicEliminationTree &self, string name, const gtsam::KeyFormatter &formatter) {
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str(); }, nb::arg("name") = "EliminationTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::SymbolicEliminationTree *self, const gtsam::SymbolicEliminationTree &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9);

  nb::class_<gtsam::SymbolicCluster>(m_, "SymbolicCluster")
      .def("nrChildren", [](gtsam::SymbolicCluster *self) { return self->nrChildren(); })
      .def("print", [](gtsam::SymbolicCluster *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicCluster &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def_rw("orderedFrontalKeys", &gtsam::SymbolicCluster::orderedFrontalKeys)
      .def_rw("factors", &gtsam::SymbolicCluster::factors)
      .def("__getitem__", &gtsam::SymbolicCluster::operator[]);

  nb::class_<gtsam::SymbolicJunctionTree>(m_, "SymbolicJunctionTree")
      .def(nb::init<const gtsam::SymbolicEliminationTree &>(), nb::arg("eliminationTree"))
      .def("print", [](gtsam::SymbolicJunctionTree *self, string name, const gtsam::KeyFormatter &formatter) { /* nb::scoped_ostream_redirect output; */ self->print(name, formatter); }, nb::arg("name") = "JunctionTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicJunctionTree &self, string name, const gtsam::KeyFormatter &formatter) {
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str(); }, nb::arg("name") = "JunctionTree: ", nb::arg("formatter") = gtsam::DefaultKeyFormatter)
      .def("nrRoots", [](gtsam::SymbolicJunctionTree *self) { return self->nrRoots(); })
      .def("__getitem__", &gtsam::SymbolicJunctionTree::operator[]);

  nb::class_<gtsam::SymbolicBayesTreeClique>(m_, "SymbolicBayesTreeClique")
      .def(nb::init<>())
      .def(nb::init<const boost::shared_ptr<gtsam::SymbolicConditional>>(), nb::arg("conditional"))
      .def("equals", [](gtsam::SymbolicBayesTreeClique *self, const gtsam::SymbolicBayesTreeClique &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("print", [](gtsam::SymbolicBayesTreeClique *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicBayesTreeClique &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("conditional", [](gtsam::SymbolicBayesTreeClique *self) { return self->conditional(); })
      .def("isRoot", [](gtsam::SymbolicBayesTreeClique *self) { return self->isRoot(); })
      .def("parent", [](gtsam::SymbolicBayesTreeClique *self) { return self->parent(); })
      .def("treeSize", [](gtsam::SymbolicBayesTreeClique *self) { return self->treeSize(); })
      .def("numCachedSeparatorMarginals", [](gtsam::SymbolicBayesTreeClique *self) { return self->numCachedSeparatorMarginals(); })
      .def("deleteCachedShortcuts", [](gtsam::SymbolicBayesTreeClique *self) { self->deleteCachedShortcuts(); });

  nb::class_<gtsam::SymbolicBayesTree>(m_, "SymbolicBayesTree")
      .def(nb::init<>())
      .def(nb::init<const gtsam::SymbolicBayesTree &>(), nb::arg("other"))
      .def("print", [](gtsam::SymbolicBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::SymbolicBayesTree &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::SymbolicBayesTree *self, const gtsam::SymbolicBayesTree &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("empty", [](gtsam::SymbolicBayesTree *self) { return self->empty(); })
      .def("size", [](gtsam::SymbolicBayesTree *self) { return self->size(); })
      .def("saveGraph", [](gtsam::SymbolicBayesTree *self, string s, const gtsam::KeyFormatter &keyFormatter) { self->saveGraph(s, keyFormatter); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("clear", [](gtsam::SymbolicBayesTree *self) { self->clear(); })
      .def("deleteCachedShortcuts", [](gtsam::SymbolicBayesTree *self) { self->deleteCachedShortcuts(); })
      .def("numCachedSeparatorMarginals", [](gtsam::SymbolicBayesTree *self) { return self->numCachedSeparatorMarginals(); })
      .def("marginalFactor", [](gtsam::SymbolicBayesTree *self, size_t key) { return self->marginalFactor(key); }, nb::arg("key"))
      .def("joint", [](gtsam::SymbolicBayesTree *self, size_t key1, size_t key2) { return self->joint(key1, key2); }, nb::arg("key1"), nb::arg("key2"))
      .def("jointBayesNet", [](gtsam::SymbolicBayesTree *self, size_t key1, size_t key2) { return self->jointBayesNet(key1, key2); }, nb::arg("key1"), nb::arg("key2"))
      .def("dot", [](gtsam::SymbolicBayesTree *self, const gtsam::KeyFormatter &keyFormatter) { return self->dot(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__getitem__", &gtsam::SymbolicBayesTree::operator[]);
}
