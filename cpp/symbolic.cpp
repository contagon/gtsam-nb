/**
 * @file    symbolic.cpp
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
#include "gtsam/symbolic/SymbolicFactor.h"
#include "gtsam/symbolic/SymbolicFactorGraph.h"
#include "gtsam/symbolic/SymbolicConditional.h"
#include "gtsam/symbolic/SymbolicBayesNet.h"
#include "gtsam/symbolic/SymbolicEliminationTree.h"
#include "gtsam/symbolic/SymbolicJunctionTree.h"
#include "gtsam/symbolic/SymbolicBayesTree.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/symbolic.h"

using namespace std;

namespace py = pybind11;



void symbolic(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of symbolic";




    py::class_<gtsam::SymbolicFactor, gtsam::Factor, boost::shared_ptr<gtsam::SymbolicFactor>>(m_, "SymbolicFactor")
        .def(py::init<const gtsam::SymbolicFactor&>(), py::arg("f"))
        .def(py::init<>())
        .def(py::init<size_t>(), py::arg("j"))
        .def(py::init<size_t, size_t>(), py::arg("j1"), py::arg("j2"))
        .def(py::init<size_t, size_t, size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"))
        .def(py::init<size_t, size_t, size_t, size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"))
        .def(py::init<size_t, size_t, size_t, size_t, size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"), py::arg("j5"))
        .def(py::init<size_t, size_t, size_t, size_t, size_t, size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"), py::arg("j5"), py::arg("j6"))
        .def("print",[](gtsam::SymbolicFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "SymbolicFactor", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "SymbolicFactor", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::SymbolicFactor* self, const gtsam::SymbolicFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def_static("FromKeys",[](const gtsam::KeyVector& js){return gtsam::SymbolicFactor::FromKeys(js);}, py::arg("js"));

    py::class_<gtsam::SymbolicFactorGraph, boost::shared_ptr<gtsam::SymbolicFactorGraph>>(m_, "SymbolicFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicBayesNet&>(), py::arg("bayesNet"))
        .def(py::init<const gtsam::SymbolicBayesTree&>(), py::arg("bayesTree"))
        .def("push_back",[](gtsam::SymbolicFactorGraph* self, boost::shared_ptr<gtsam::SymbolicFactor> factor){ self->push_back(factor);}, py::arg("factor"))
        .def("print",[](gtsam::SymbolicFactorGraph* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "SymbolicFactorGraph", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicFactorGraph& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "SymbolicFactorGraph", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::SymbolicFactorGraph* self, const gtsam::SymbolicFactorGraph& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("size",[](gtsam::SymbolicFactorGraph* self){return self->size();})
        .def("exists",[](gtsam::SymbolicFactorGraph* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::SymbolicFactorGraph* self){return self->keys();})
        .def("push_back",[](gtsam::SymbolicFactorGraph* self, const gtsam::SymbolicFactorGraph& graph){ self->push_back(graph);}, py::arg("graph"))
        .def("push_back",[](gtsam::SymbolicFactorGraph* self, const gtsam::SymbolicBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("push_back",[](gtsam::SymbolicFactorGraph* self, const gtsam::SymbolicBayesTree& bayesTree){ self->push_back(bayesTree);}, py::arg("bayesTree"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key){ self->push_factor(key);}, py::arg("key"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key1, size_t key2){ self->push_factor(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key1, size_t key2, size_t key3){ self->push_factor(key1, key2, key3);}, py::arg("key1"), py::arg("key2"), py::arg("key3"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key1, size_t key2, size_t key3, size_t key4){ self->push_factor(key1, key2, key3, key4);}, py::arg("key1"), py::arg("key2"), py::arg("key3"), py::arg("key4"))
        .def("eliminateSequential",[](gtsam::SymbolicFactorGraph* self){return self->eliminateSequential();})
        .def("eliminateSequential",[](gtsam::SymbolicFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminateSequential(ordering);}, py::arg("ordering"))
        .def("eliminateMultifrontal",[](gtsam::SymbolicFactorGraph* self){return self->eliminateMultifrontal();})
        .def("eliminateMultifrontal",[](gtsam::SymbolicFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminateMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::SymbolicFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminatePartialSequential(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::SymbolicFactorGraph* self, const gtsam::KeyVector& keys){return self->eliminatePartialSequential(keys);}, py::arg("keys"))
        .def("eliminatePartialMultifrontal",[](gtsam::SymbolicFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminatePartialMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialMultifrontal",[](gtsam::SymbolicFactorGraph* self, const gtsam::KeyVector& keys){return self->eliminatePartialMultifrontal(keys);}, py::arg("keys"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self, const gtsam::Ordering& ordering){return self->marginalMultifrontalBayesNet(ordering);}, py::arg("ordering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self, const gtsam::KeyVector& key_vector){return self->marginalMultifrontalBayesNet(key_vector);}, py::arg("key_vector"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self, const gtsam::Ordering& ordering, const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(ordering, marginalizedVariableOrdering);}, py::arg("ordering"), py::arg("marginalizedVariableOrdering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self, const gtsam::KeyVector& key_vector, const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(key_vector, marginalizedVariableOrdering);}, py::arg("key_vector"), py::arg("marginalizedVariableOrdering"))
        .def("marginal",[](gtsam::SymbolicFactorGraph* self, const gtsam::KeyVector& key_vector){return self->marginal(key_vector);}, py::arg("key_vector"))
        .def("dot",[](gtsam::SymbolicFactorGraph* self, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){return self->dot(keyFormatter, writer);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("saveGraph",[](gtsam::SymbolicFactorGraph* self, string s, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){ self->saveGraph(s, keyFormatter, writer);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter());

    py::class_<gtsam::SymbolicConditional, gtsam::SymbolicFactor, boost::shared_ptr<gtsam::SymbolicConditional>>(m_, "SymbolicConditional")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicConditional&>(), py::arg("other"))
        .def(py::init<size_t>(), py::arg("key"))
        .def(py::init<size_t, size_t>(), py::arg("key"), py::arg("parent"))
        .def(py::init<size_t, size_t, size_t>(), py::arg("key"), py::arg("parent1"), py::arg("parent2"))
        .def(py::init<size_t, size_t, size_t, size_t>(), py::arg("key"), py::arg("parent1"), py::arg("parent2"), py::arg("parent3"))
        .def("print",[](gtsam::SymbolicConditional* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicConditional& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::SymbolicConditional* self, const gtsam::SymbolicConditional& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("firstFrontalKey",[](gtsam::SymbolicConditional* self){return self->firstFrontalKey();})
        .def("nrFrontals",[](gtsam::SymbolicConditional* self){return self->nrFrontals();})
        .def("nrParents",[](gtsam::SymbolicConditional* self){return self->nrParents();})
        .def_static("FromKeys",[](const gtsam::KeyVector& keys, size_t nrFrontals){return gtsam::SymbolicConditional::FromKeys(keys, nrFrontals);}, py::arg("keys"), py::arg("nrFrontals"));

    py::class_<gtsam::SymbolicBayesNet, boost::shared_ptr<gtsam::SymbolicBayesNet>>(m_, "SymbolicBayesNet")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicBayesNet&>(), py::arg("other"))
        .def("print",[](gtsam::SymbolicBayesNet* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "SymbolicBayesNet", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicBayesNet& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "SymbolicBayesNet", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::SymbolicBayesNet* self, const gtsam::SymbolicBayesNet& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("size",[](gtsam::SymbolicBayesNet* self){return self->size();})
        .def("saveGraph",[](gtsam::SymbolicBayesNet* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("at",[](gtsam::SymbolicBayesNet* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("front",[](gtsam::SymbolicBayesNet* self){return self->front();})
        .def("back",[](gtsam::SymbolicBayesNet* self){return self->back();})
        .def("push_back",[](gtsam::SymbolicBayesNet* self, boost::shared_ptr<gtsam::SymbolicConditional> conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::SymbolicBayesNet* self, const gtsam::SymbolicBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("dot",[](gtsam::SymbolicBayesNet* self, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){return self->dot(keyFormatter, writer);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("saveGraph",[](gtsam::SymbolicBayesNet* self, string s, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){ self->saveGraph(s, keyFormatter, writer);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter());

    py::class_<gtsam::SymbolicEliminationTree, boost::shared_ptr<gtsam::SymbolicEliminationTree>>(m_, "SymbolicEliminationTree")
        .def(py::init<const gtsam::SymbolicFactorGraph&, const gtsam::VariableIndex&, const gtsam::Ordering&>(), py::arg("factorGraph"), py::arg("structure"), py::arg("order"))
        .def(py::init<const gtsam::SymbolicFactorGraph&, const gtsam::Ordering&>(), py::arg("factorGraph"), py::arg("order"))
        .def("print",[](gtsam::SymbolicEliminationTree* self, string name, const gtsam::KeyFormatter& formatter){ py::scoped_ostream_redirect output; self->print(name, formatter);}, py::arg("name") = "EliminationTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicEliminationTree& self, string name, const gtsam::KeyFormatter& formatter){
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str();
                    }, py::arg("name") = "EliminationTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::SymbolicEliminationTree* self, const gtsam::SymbolicEliminationTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9);

    py::class_<gtsam::SymbolicCluster, boost::shared_ptr<gtsam::SymbolicCluster>>(m_, "SymbolicCluster")
        .def("nrChildren",[](gtsam::SymbolicCluster* self){return self->nrChildren();})
        .def("print",[](gtsam::SymbolicCluster* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicCluster& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def_readwrite("orderedFrontalKeys", &gtsam::SymbolicCluster::orderedFrontalKeys)
        .def_readwrite("factors", &gtsam::SymbolicCluster::factors)
        .def("__getitem__", &gtsam::SymbolicCluster::operator[]);

    py::class_<gtsam::SymbolicJunctionTree, boost::shared_ptr<gtsam::SymbolicJunctionTree>>(m_, "SymbolicJunctionTree")
        .def(py::init<const gtsam::SymbolicEliminationTree&>(), py::arg("eliminationTree"))
        .def("print",[](gtsam::SymbolicJunctionTree* self, string name, const gtsam::KeyFormatter& formatter){ py::scoped_ostream_redirect output; self->print(name, formatter);}, py::arg("name") = "JunctionTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicJunctionTree& self, string name, const gtsam::KeyFormatter& formatter){
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str();
                    }, py::arg("name") = "JunctionTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("nrRoots",[](gtsam::SymbolicJunctionTree* self){return self->nrRoots();})
        .def("__getitem__", &gtsam::SymbolicJunctionTree::operator[]);

    py::class_<gtsam::SymbolicBayesTreeClique, boost::shared_ptr<gtsam::SymbolicBayesTreeClique>>(m_, "SymbolicBayesTreeClique")
        .def(py::init<>())
        .def(py::init<const boost::shared_ptr<gtsam::SymbolicConditional>>(), py::arg("conditional"))
        .def("equals",[](gtsam::SymbolicBayesTreeClique* self, const gtsam::SymbolicBayesTreeClique& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print",[](gtsam::SymbolicBayesTreeClique* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicBayesTreeClique& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("conditional",[](gtsam::SymbolicBayesTreeClique* self){return self->conditional();})
        .def("isRoot",[](gtsam::SymbolicBayesTreeClique* self){return self->isRoot();})
        .def("parent",[](gtsam::SymbolicBayesTreeClique* self){return self->parent();})
        .def("treeSize",[](gtsam::SymbolicBayesTreeClique* self){return self->treeSize();})
        .def("numCachedSeparatorMarginals",[](gtsam::SymbolicBayesTreeClique* self){return self->numCachedSeparatorMarginals();})
        .def("deleteCachedShortcuts",[](gtsam::SymbolicBayesTreeClique* self){ self->deleteCachedShortcuts();});

    py::class_<gtsam::SymbolicBayesTree, boost::shared_ptr<gtsam::SymbolicBayesTree>>(m_, "SymbolicBayesTree")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicBayesTree&>(), py::arg("other"))
        .def("print",[](gtsam::SymbolicBayesTree* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::SymbolicBayesTree& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::SymbolicBayesTree* self, const gtsam::SymbolicBayesTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("empty",[](gtsam::SymbolicBayesTree* self){return self->empty();})
        .def("size",[](gtsam::SymbolicBayesTree* self){return self->size();})
        .def("saveGraph",[](gtsam::SymbolicBayesTree* self, string s, const gtsam::KeyFormatter& keyFormatter){ self->saveGraph(s, keyFormatter);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("clear",[](gtsam::SymbolicBayesTree* self){ self->clear();})
        .def("deleteCachedShortcuts",[](gtsam::SymbolicBayesTree* self){ self->deleteCachedShortcuts();})
        .def("numCachedSeparatorMarginals",[](gtsam::SymbolicBayesTree* self){return self->numCachedSeparatorMarginals();})
        .def("marginalFactor",[](gtsam::SymbolicBayesTree* self, size_t key){return self->marginalFactor(key);}, py::arg("key"))
        .def("joint",[](gtsam::SymbolicBayesTree* self, size_t key1, size_t key2){return self->joint(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("jointBayesNet",[](gtsam::SymbolicBayesTree* self, size_t key1, size_t key2){return self->jointBayesNet(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("dot",[](gtsam::SymbolicBayesTree* self, const gtsam::KeyFormatter& keyFormatter){return self->dot(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__getitem__", &gtsam::SymbolicBayesTree::operator[]);


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/symbolic.h"

}

