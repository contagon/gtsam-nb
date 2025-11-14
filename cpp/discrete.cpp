/**
 * @file    discrete.cpp
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
#include "gtsam/discrete/DiscreteKey.h"
#include "gtsam/discrete/DiscreteFactor.h"
#include "gtsam/discrete/DecisionTreeFactor.h"
#include "gtsam/discrete/DiscreteConditional.h"
#include "gtsam/hybrid/HybridValues.h"
#include "gtsam/discrete/DiscreteDistribution.h"
#include "gtsam/discrete/DiscreteBayesNet.h"
#include "gtsam/discrete/DiscreteBayesTree.h"
#include "gtsam/discrete/DiscreteLookupDAG.h"
#include "gtsam/discrete/DiscreteFactorGraph.h"
#include "gtsam/discrete/DiscreteEliminationTree.h"
#include "gtsam/discrete/DiscreteJunctionTree.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/discrete.h"

using namespace std;

namespace py = pybind11;



void discrete(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of discrete";




    py::class_<gtsam::DiscreteKeys, boost::shared_ptr<gtsam::DiscreteKeys>>(m_, "DiscreteKeys")
        .def(py::init<>())
        .def("size",[](gtsam::DiscreteKeys* self){return self->size();})
        .def("empty",[](gtsam::DiscreteKeys* self){return self->empty();})
        .def("at",[](gtsam::DiscreteKeys* self, size_t n){return self->at(n);}, py::arg("n"))
        .def("push_back",[](gtsam::DiscreteKeys* self, const gtsam::DiscreteKey& point_pair){ self->push_back(point_pair);}, py::arg("point_pair"));

    py::class_<gtsam::DiscreteFactor, gtsam::Factor, boost::shared_ptr<gtsam::DiscreteFactor>>(m_, "DiscreteFactor")
        .def("print",[](gtsam::DiscreteFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "DiscreteFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "DiscreteFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::DiscreteFactor* self, const gtsam::DiscreteFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9)
        .def("__call__", &gtsam::DiscreteFactor::operator());

    py::class_<gtsam::DecisionTreeFactor, gtsam::DiscreteFactor, boost::shared_ptr<gtsam::DecisionTreeFactor>>(m_, "DecisionTreeFactor")
        .def(py::init<>())
        .def(py::init<const gtsam::DiscreteKey&, const std::vector<double>&>(), py::arg("key"), py::arg("spec"))
        .def(py::init<const gtsam::DiscreteKey&, string>(), py::arg("key"), py::arg("table"))
        .def(py::init<const gtsam::DiscreteKeys&, const std::vector<double>&>(), py::arg("keys"), py::arg("table"))
        .def(py::init<const gtsam::DiscreteKeys&, string>(), py::arg("keys"), py::arg("table"))
        .def(py::init<const std::vector<gtsam::DiscreteKey>&, const std::vector<double>&>(), py::arg("keys"), py::arg("table"))
        .def(py::init<const std::vector<gtsam::DiscreteKey>&, string>(), py::arg("keys"), py::arg("table"))
        .def(py::init<const gtsam::DiscreteConditional&>(), py::arg("c"))
        .def("print",[](gtsam::DecisionTreeFactor* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "DecisionTreeFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DecisionTreeFactor& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "DecisionTreeFactor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::DecisionTreeFactor* self, const gtsam::DecisionTreeFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9)
        .def("cardinality",[](gtsam::DecisionTreeFactor* self, gtsam::Key j){return self->cardinality(j);}, py::arg("j"))
        .def("cardinality",[](gtsam::DecisionTreeFactor* self, gtsam::Key j){return self->cardinality(j);}, py::arg("j"))
        .def("sum",[](gtsam::DecisionTreeFactor* self, size_t nrFrontals){return self->sum(nrFrontals);}, py::arg("nrFrontals"))
        .def("sum",[](gtsam::DecisionTreeFactor* self, const gtsam::Ordering& keys){return self->sum(keys);}, py::arg("keys"))
        .def("max",[](gtsam::DecisionTreeFactor* self, size_t nrFrontals){return self->max(nrFrontals);}, py::arg("nrFrontals"))
        .def("max",[](gtsam::DecisionTreeFactor* self, const gtsam::Ordering& keys){return self->max(keys);}, py::arg("keys"))
        .def("dot",[](gtsam::DecisionTreeFactor* self, const gtsam::KeyFormatter& keyFormatter, bool showZero){return self->dot(keyFormatter, showZero);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("showZero") = true)
        .def("enumerate",[](gtsam::DecisionTreeFactor* self){return self->enumerate();})
        .def("_repr_markdown_",[](gtsam::DecisionTreeFactor* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_markdown_",[](gtsam::DecisionTreeFactor* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->markdown(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("_repr_html_",[](gtsam::DecisionTreeFactor* self, const gtsam::KeyFormatter& keyFormatter){return self->html(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_html_",[](gtsam::DecisionTreeFactor* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->html(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("__call__", &gtsam::DecisionTreeFactor::operator())
        .def(py::self * py::self)
        .def(py::self / py::self);

    py::class_<gtsam::DiscreteConditional, gtsam::DecisionTreeFactor, boost::shared_ptr<gtsam::DiscreteConditional>>(m_, "DiscreteConditional")
        .def(py::init<>())
        .def(py::init<size_t, const gtsam::DecisionTreeFactor&>(), py::arg("nFrontals"), py::arg("f"))
        .def(py::init<const gtsam::DiscreteKey&, string>(), py::arg("key"), py::arg("spec"))
        .def(py::init<const gtsam::DiscreteKey&, const gtsam::DiscreteKeys&, string>(), py::arg("key"), py::arg("parents"), py::arg("spec"))
        .def(py::init<const gtsam::DiscreteKey&, const std::vector<gtsam::DiscreteKey>&, string>(), py::arg("key"), py::arg("parents"), py::arg("spec"))
        .def(py::init<const gtsam::DecisionTreeFactor&, const gtsam::DecisionTreeFactor&>(), py::arg("joint"), py::arg("marginal"))
        .def(py::init<const gtsam::DecisionTreeFactor&, const gtsam::DecisionTreeFactor&, const gtsam::Ordering&>(), py::arg("joint"), py::arg("marginal"), py::arg("orderedKeys"))
        .def("logNormalizationConstant",[](gtsam::DiscreteConditional* self){return self->logNormalizationConstant();})
        .def("logProbability",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteValues& values){return self->logProbability(values);}, py::arg("values"))
        .def("evaluate",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteValues& values){return self->evaluate(values);}, py::arg("values"))
        .def("error",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteValues& values){return self->error(values);}, py::arg("values"))
        .def("marginal",[](gtsam::DiscreteConditional* self, gtsam::Key key){return self->marginal(key);}, py::arg("key"))
        .def("print",[](gtsam::DiscreteConditional* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "Discrete Conditional\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteConditional& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "Discrete Conditional\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteConditional& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9)
        .def("firstFrontalKey",[](gtsam::DiscreteConditional* self){return self->firstFrontalKey();})
        .def("nrFrontals",[](gtsam::DiscreteConditional* self){return self->nrFrontals();})
        .def("nrParents",[](gtsam::DiscreteConditional* self){return self->nrParents();})
        .def("printSignature",[](gtsam::DiscreteConditional* self, string s, const gtsam::KeyFormatter& formatter){ self->printSignature(s, formatter);}, py::arg("s") = "Discrete Conditional: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("choose",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteValues& given){return self->choose(given);}, py::arg("given"))
        .def("likelihood",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteValues& frontalValues){return self->likelihood(frontalValues);}, py::arg("frontalValues"))
        .def("likelihood",[](gtsam::DiscreteConditional* self, size_t value){return self->likelihood(value);}, py::arg("value"))
        .def("sample",[](gtsam::DiscreteConditional* self, const gtsam::DiscreteValues& parentsValues){return self->sample(parentsValues);}, py::arg("parentsValues"))
        .def("sample",[](gtsam::DiscreteConditional* self, size_t value){return self->sample(value);}, py::arg("value"))
        .def("sample",[](gtsam::DiscreteConditional* self){return self->sample();})
        .def("sampleInPlace",[](gtsam::DiscreteConditional* self, gtsam::DiscreteValues* parentsValues){ self->sampleInPlace(parentsValues);}, py::arg("parentsValues"))
        .def("_repr_markdown_",[](gtsam::DiscreteConditional* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_markdown_",[](gtsam::DiscreteConditional* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->markdown(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("_repr_html_",[](gtsam::DiscreteConditional* self, const gtsam::KeyFormatter& keyFormatter){return self->html(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_html_",[](gtsam::DiscreteConditional* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->html(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("logProbability",[](gtsam::DiscreteConditional* self, const gtsam::HybridValues& x){return self->logProbability(x);}, py::arg("x"))
        .def("evaluate",[](gtsam::DiscreteConditional* self, const gtsam::HybridValues& x){return self->evaluate(x);}, py::arg("x"))
        .def("error",[](gtsam::DiscreteConditional* self, const gtsam::HybridValues& x){return self->error(x);}, py::arg("x"))
        .def(py::self * py::self);

    py::class_<gtsam::DiscreteDistribution, gtsam::DiscreteConditional, boost::shared_ptr<gtsam::DiscreteDistribution>>(m_, "DiscreteDistribution")
        .def(py::init<>())
        .def(py::init<const gtsam::DecisionTreeFactor&>(), py::arg("f"))
        .def(py::init<const gtsam::DiscreteKey&, string>(), py::arg("key"), py::arg("spec"))
        .def(py::init<const gtsam::DiscreteKey&, std::vector<double>>(), py::arg("key"), py::arg("spec"))
        .def("print",[](gtsam::DiscreteDistribution* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "Discrete Prior\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteDistribution& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "Discrete Prior\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("pmf",[](gtsam::DiscreteDistribution* self){return self->pmf();})
        .def("argmax",[](gtsam::DiscreteDistribution* self){return self->argmax();})
        .def("__call__", &gtsam::DiscreteDistribution::operator());

    py::class_<gtsam::DiscreteBayesNet, boost::shared_ptr<gtsam::DiscreteBayesNet>>(m_, "DiscreteBayesNet")
        .def(py::init<>())
        .def("add",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteConditional& s){ self->add(s);}, py::arg("s"))
        .def("add",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteKey& key, string spec){ self->add(key, spec);}, py::arg("key"), py::arg("spec"))
        .def("add",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteKey& key, const gtsam::DiscreteKeys& parents, string spec){ self->add(key, parents, spec);}, py::arg("key"), py::arg("parents"), py::arg("spec"))
        .def("add",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteKey& key, const std::vector<gtsam::DiscreteKey>& parents, string spec){ self->add(key, parents, spec);}, py::arg("key"), py::arg("parents"), py::arg("spec"))
        .def("empty",[](gtsam::DiscreteBayesNet* self){return self->empty();})
        .def("size",[](gtsam::DiscreteBayesNet* self){return self->size();})
        .def("keys",[](gtsam::DiscreteBayesNet* self){return self->keys();})
        .def("at",[](gtsam::DiscreteBayesNet* self, size_t i){return self->at(i);}, py::arg("i"))
        .def("print",[](gtsam::DiscreteBayesNet* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "DiscreteBayesNet\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteBayesNet& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "DiscreteBayesNet\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteBayesNet& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9)
        .def("logProbability",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteValues& values){return self->logProbability(values);}, py::arg("values"))
        .def("evaluate",[](gtsam::DiscreteBayesNet* self, const gtsam::DiscreteValues& values){return self->evaluate(values);}, py::arg("values"))
        .def("sample",[](gtsam::DiscreteBayesNet* self){return self->sample();})
        .def("sample",[](gtsam::DiscreteBayesNet* self, gtsam::DiscreteValues given){return self->sample(given);}, py::arg("given"))
        .def("dot",[](gtsam::DiscreteBayesNet* self, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){return self->dot(keyFormatter, writer);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("saveGraph",[](gtsam::DiscreteBayesNet* self, string s, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){ self->saveGraph(s, keyFormatter, writer);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("_repr_markdown_",[](gtsam::DiscreteBayesNet* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_markdown_",[](gtsam::DiscreteBayesNet* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->markdown(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("_repr_html_",[](gtsam::DiscreteBayesNet* self, const gtsam::KeyFormatter& keyFormatter){return self->html(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_html_",[](gtsam::DiscreteBayesNet* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->html(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("__call__", &gtsam::DiscreteBayesNet::operator());

    py::class_<gtsam::DiscreteBayesTreeClique, boost::shared_ptr<gtsam::DiscreteBayesTreeClique>>(m_, "DiscreteBayesTreeClique")
        .def(py::init<>())
        .def(py::init<const boost::shared_ptr<gtsam::DiscreteConditional>>(), py::arg("conditional"))
        .def("conditional",[](gtsam::DiscreteBayesTreeClique* self){return self->conditional();})
        .def("isRoot",[](gtsam::DiscreteBayesTreeClique* self){return self->isRoot();})
        .def("nrChildren",[](gtsam::DiscreteBayesTreeClique* self){return self->nrChildren();})
        .def("print",[](gtsam::DiscreteBayesTreeClique* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "DiscreteBayesTreeClique", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteBayesTreeClique& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "DiscreteBayesTreeClique", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("printSignature",[](gtsam::DiscreteBayesTreeClique* self, const string& s, const gtsam::KeyFormatter& formatter){ self->printSignature(s, formatter);}, py::arg("s") = "Clique: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("evaluate",[](gtsam::DiscreteBayesTreeClique* self, const gtsam::DiscreteValues& values){return self->evaluate(values);}, py::arg("values"))
        .def("__getitem__", &gtsam::DiscreteBayesTreeClique::operator[])
        .def("__call__", &gtsam::DiscreteBayesTreeClique::operator());

    py::class_<gtsam::DiscreteBayesTree, boost::shared_ptr<gtsam::DiscreteBayesTree>>(m_, "DiscreteBayesTree")
        .def(py::init<>())
        .def("print",[](gtsam::DiscreteBayesTree* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "DiscreteBayesTree\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteBayesTree& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "DiscreteBayesTree\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::DiscreteBayesTree* self, const gtsam::DiscreteBayesTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9)
        .def("size",[](gtsam::DiscreteBayesTree* self){return self->size();})
        .def("empty",[](gtsam::DiscreteBayesTree* self){return self->empty();})
        .def("evaluate",[](gtsam::DiscreteBayesTree* self, const gtsam::DiscreteValues& values){return self->evaluate(values);}, py::arg("values"))
        .def("dot",[](gtsam::DiscreteBayesTree* self, const gtsam::KeyFormatter& keyFormatter){return self->dot(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("saveGraph",[](gtsam::DiscreteBayesTree* self, string s, const gtsam::KeyFormatter& keyFormatter){ self->saveGraph(s, keyFormatter);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_markdown_",[](gtsam::DiscreteBayesTree* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_markdown_",[](gtsam::DiscreteBayesTree* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->markdown(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("_repr_html_",[](gtsam::DiscreteBayesTree* self, const gtsam::KeyFormatter& keyFormatter){return self->html(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_html_",[](gtsam::DiscreteBayesTree* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->html(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("__getitem__", &gtsam::DiscreteBayesTree::operator[])
        .def("__call__", &gtsam::DiscreteBayesTree::operator())
        .def("__call__", &gtsam::DiscreteBayesTree::operator());

    py::class_<gtsam::DiscreteLookupTable, gtsam::DiscreteConditional, boost::shared_ptr<gtsam::DiscreteLookupTable>>(m_, "DiscreteLookupTable")
        .def(py::init<size_t, const gtsam::DiscreteKeys&, const gtsam::DecisionTreeFactor::ADT&>(), py::arg("nFrontals"), py::arg("keys"), py::arg("potentials"))
        .def("print",[](gtsam::DiscreteLookupTable* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "Discrete Lookup Table: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteLookupTable& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "Discrete Lookup Table: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("argmax",[](gtsam::DiscreteLookupTable* self, const gtsam::DiscreteValues& parentsValues){return self->argmax(parentsValues);}, py::arg("parentsValues"));

    py::class_<gtsam::DiscreteLookupDAG, boost::shared_ptr<gtsam::DiscreteLookupDAG>>(m_, "DiscreteLookupDAG")
        .def(py::init<>())
        .def("push_back",[](gtsam::DiscreteLookupDAG* self, const boost::shared_ptr<gtsam::DiscreteLookupTable> table){ self->push_back(table);}, py::arg("table"))
        .def("empty",[](gtsam::DiscreteLookupDAG* self){return self->empty();})
        .def("size",[](gtsam::DiscreteLookupDAG* self){return self->size();})
        .def("keys",[](gtsam::DiscreteLookupDAG* self){return self->keys();})
        .def("at",[](gtsam::DiscreteLookupDAG* self, size_t i){return self->at(i);}, py::arg("i"))
        .def("print",[](gtsam::DiscreteLookupDAG* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "DiscreteLookupDAG\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteLookupDAG& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "DiscreteLookupDAG\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("argmax",[](gtsam::DiscreteLookupDAG* self){return self->argmax();})
        .def("argmax",[](gtsam::DiscreteLookupDAG* self, gtsam::DiscreteValues given){return self->argmax(given);}, py::arg("given"));

    py::class_<gtsam::DiscreteFactorGraph, boost::shared_ptr<gtsam::DiscreteFactorGraph>>(m_, "DiscreteFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::DiscreteBayesNet&>(), py::arg("bayesNet"))
        .def("push_back",[](gtsam::DiscreteFactorGraph* self, const boost::shared_ptr<gtsam::DiscreteFactor> factor){ self->push_back(factor);}, py::arg("factor"))
        .def("push_back",[](gtsam::DiscreteFactorGraph* self, const boost::shared_ptr<gtsam::DiscreteConditional> conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteFactorGraph& graph){ self->push_back(graph);}, py::arg("graph"))
        .def("push_back",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("push_back",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteBayesTree& bayesTree){ self->push_back(bayesTree);}, py::arg("bayesTree"))
        .def("add",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteKey& j, string spec){ self->add(j, spec);}, py::arg("j"), py::arg("spec"))
        .def("add",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteKey& j, const std::vector<double>& spec){ self->add(j, spec);}, py::arg("j"), py::arg("spec"))
        .def("add",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteKeys& keys, string spec){ self->add(keys, spec);}, py::arg("keys"), py::arg("spec"))
        .def("add",[](gtsam::DiscreteFactorGraph* self, const std::vector<gtsam::DiscreteKey>& keys, string spec){ self->add(keys, spec);}, py::arg("keys"), py::arg("spec"))
        .def("add",[](gtsam::DiscreteFactorGraph* self, const std::vector<gtsam::DiscreteKey>& keys, const std::vector<double>& spec){ self->add(keys, spec);}, py::arg("keys"), py::arg("spec"))
        .def("empty",[](gtsam::DiscreteFactorGraph* self){return self->empty();})
        .def("size",[](gtsam::DiscreteFactorGraph* self){return self->size();})
        .def("keys",[](gtsam::DiscreteFactorGraph* self){return self->keys();})
        .def("at",[](gtsam::DiscreteFactorGraph* self, size_t i){return self->at(i);}, py::arg("i"))
        .def("print",[](gtsam::DiscreteFactorGraph* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::DiscreteFactorGraph& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::DiscreteFactorGraph* self, const gtsam::DiscreteFactorGraph& fg, double tol){return self->equals(fg, tol);}, py::arg("fg"), py::arg("tol") = 1e-9)
        .def("product",[](gtsam::DiscreteFactorGraph* self){return self->product();})
        .def("optimize",[](gtsam::DiscreteFactorGraph* self){return self->optimize();})
        .def("sumProduct",[](gtsam::DiscreteFactorGraph* self, gtsam::Ordering::OrderingType type){return self->sumProduct(type);}, py::arg("type") = gtsam::Ordering::COLAMD)
        .def("sumProduct",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering){return self->sumProduct(ordering);}, py::arg("ordering"))
        .def("maxProduct",[](gtsam::DiscreteFactorGraph* self, gtsam::Ordering::OrderingType type){return self->maxProduct(type);}, py::arg("type") = gtsam::Ordering::COLAMD)
        .def("maxProduct",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering){return self->maxProduct(ordering);}, py::arg("ordering"))
        .def("eliminateSequential",[](gtsam::DiscreteFactorGraph* self, gtsam::Ordering::OrderingType type){return self->eliminateSequential(type);}, py::arg("type") = gtsam::Ordering::COLAMD)
        .def("eliminateSequential",[](gtsam::DiscreteFactorGraph* self, gtsam::Ordering::OrderingType type, const gtsam::DiscreteFactorGraph::Eliminate& function){return self->eliminateSequential(type, function);}, py::arg("type"), py::arg("function"))
        .def("eliminateSequential",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminateSequential(ordering);}, py::arg("ordering"))
        .def("eliminateSequential",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering, const gtsam::DiscreteFactorGraph::Eliminate& function){return self->eliminateSequential(ordering, function);}, py::arg("ordering"), py::arg("function"))
        .def("eliminatePartialSequential",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminatePartialSequential(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering, const gtsam::DiscreteFactorGraph::Eliminate& function){return self->eliminatePartialSequential(ordering, function);}, py::arg("ordering"), py::arg("function"))
        .def("eliminateMultifrontal",[](gtsam::DiscreteFactorGraph* self, gtsam::Ordering::OrderingType type){return self->eliminateMultifrontal(type);}, py::arg("type") = gtsam::Ordering::COLAMD)
        .def("eliminateMultifrontal",[](gtsam::DiscreteFactorGraph* self, gtsam::Ordering::OrderingType type, const gtsam::DiscreteFactorGraph::Eliminate& function){return self->eliminateMultifrontal(type, function);}, py::arg("type"), py::arg("function"))
        .def("eliminateMultifrontal",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminateMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminateMultifrontal",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering, const gtsam::DiscreteFactorGraph::Eliminate& function){return self->eliminateMultifrontal(ordering, function);}, py::arg("ordering"), py::arg("function"))
        .def("eliminatePartialMultifrontal",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering){return self->eliminatePartialMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialMultifrontal",[](gtsam::DiscreteFactorGraph* self, const gtsam::Ordering& ordering, const gtsam::DiscreteFactorGraph::Eliminate& function){return self->eliminatePartialMultifrontal(ordering, function);}, py::arg("ordering"), py::arg("function"))
        .def("dot",[](gtsam::DiscreteFactorGraph* self, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){return self->dot(keyFormatter, writer);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("saveGraph",[](gtsam::DiscreteFactorGraph* self, string s, const gtsam::KeyFormatter& keyFormatter, const gtsam::DotWriter& writer){ self->saveGraph(s, keyFormatter, writer);}, py::arg("s"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter, py::arg("writer") = gtsam::DotWriter())
        .def("_repr_markdown_",[](gtsam::DiscreteFactorGraph* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_markdown_",[](gtsam::DiscreteFactorGraph* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->markdown(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("_repr_html_",[](gtsam::DiscreteFactorGraph* self, const gtsam::KeyFormatter& keyFormatter){return self->html(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("_repr_html_",[](gtsam::DiscreteFactorGraph* self, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return self->html(keyFormatter, names);}, py::arg("keyFormatter"), py::arg("names"))
        .def("__call__", &gtsam::DiscreteFactorGraph::operator());

    py::class_<gtsam::DiscreteEliminationTree, boost::shared_ptr<gtsam::DiscreteEliminationTree>>(m_, "DiscreteEliminationTree")
        .def(py::init<const gtsam::DiscreteFactorGraph&, const gtsam::VariableIndex&, const gtsam::Ordering&>(), py::arg("factorGraph"), py::arg("structure"), py::arg("order"))
        .def(py::init<const gtsam::DiscreteFactorGraph&, const gtsam::Ordering&>(), py::arg("factorGraph"), py::arg("order"))
        .def("print",[](gtsam::DiscreteEliminationTree* self, string name, const gtsam::KeyFormatter& formatter){ py::scoped_ostream_redirect output; self->print(name, formatter);}, py::arg("name") = "EliminationTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteEliminationTree& self, string name, const gtsam::KeyFormatter& formatter){
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str();
                    }, py::arg("name") = "EliminationTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("equals",[](gtsam::DiscreteEliminationTree* self, const gtsam::DiscreteEliminationTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol") = 1e-9);

    py::class_<gtsam::DiscreteCluster, boost::shared_ptr<gtsam::DiscreteCluster>>(m_, "DiscreteCluster")
        .def("nrChildren",[](gtsam::DiscreteCluster* self){return self->nrChildren();})
        .def("print",[](gtsam::DiscreteCluster* self, string s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteCluster& self, string s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def_readwrite("orderedFrontalKeys", &gtsam::DiscreteCluster::orderedFrontalKeys)
        .def_readwrite("factors", &gtsam::DiscreteCluster::factors)
        .def("__getitem__", &gtsam::DiscreteCluster::operator[]);

    py::class_<gtsam::DiscreteJunctionTree, boost::shared_ptr<gtsam::DiscreteJunctionTree>>(m_, "DiscreteJunctionTree")
        .def(py::init<const gtsam::DiscreteEliminationTree&>(), py::arg("eliminationTree"))
        .def("print",[](gtsam::DiscreteJunctionTree* self, string name, const gtsam::KeyFormatter& formatter){ py::scoped_ostream_redirect output; self->print(name, formatter);}, py::arg("name") = "JunctionTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const gtsam::DiscreteJunctionTree& self, string name, const gtsam::KeyFormatter& formatter){
                        gtsam::RedirectCout redirect;
                        self.print(name, formatter);
                        return redirect.str();
                    }, py::arg("name") = "JunctionTree: ", py::arg("formatter") = gtsam::DefaultKeyFormatter)
        .def("nrRoots",[](gtsam::DiscreteJunctionTree* self){return self->nrRoots();})
        .def("__getitem__", &gtsam::DiscreteJunctionTree::operator[]);

    m_.def("cartesianProduct",[](const gtsam::DiscreteKeys& keys){return gtsam::cartesianProduct(keys);}, py::arg("keys"));
    m_.def("markdown",[](const gtsam::DiscreteValues& values, const gtsam::KeyFormatter& keyFormatter){return gtsam::markdown(values, keyFormatter);}, py::arg("values"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
    m_.def("markdown",[](const gtsam::DiscreteValues& values, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return gtsam::markdown(values, keyFormatter, names);}, py::arg("values"), py::arg("keyFormatter"), py::arg("names"));
    m_.def("html",[](const gtsam::DiscreteValues& values, const gtsam::KeyFormatter& keyFormatter){return gtsam::html(values, keyFormatter);}, py::arg("values"), py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
    m_.def("html",[](const gtsam::DiscreteValues& values, const gtsam::KeyFormatter& keyFormatter, std::map<gtsam::Key, std::vector<std::string>> names){return gtsam::html(values, keyFormatter, names);}, py::arg("values"), py::arg("keyFormatter"), py::arg("names"));
    m_.def("EliminateDiscrete",[](const gtsam::DiscreteFactorGraph& factors, const gtsam::Ordering& frontalKeys){return gtsam::EliminateDiscrete(factors, frontalKeys);}, py::arg("factors"), py::arg("frontalKeys"));
    m_.def("EliminateForMPE",[](const gtsam::DiscreteFactorGraph& factors, const gtsam::Ordering& frontalKeys){return gtsam::EliminateForMPE(factors, frontalKeys);}, py::arg("factors"), py::arg("frontalKeys"));

// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/discrete.h"

}

