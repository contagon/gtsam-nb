/**
 * @file    inference.cpp
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
#include "gtsam/discrete/DiscreteFactorGraph.h"
#include "gtsam/hybrid/HybridGaussianFactorGraph.h"
#include "gtsam/inference/DotWriter.h"
#include "gtsam/inference/Factor.h"
#include "gtsam/inference/Key.h"
#include "gtsam/inference/LabeledSymbol.h"
#include "gtsam/inference/Ordering.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/inference/VariableIndex.h"
#include "gtsam/linear/GaussianFactorGraph.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/symbolic/SymbolicFactorGraph.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::Ordering)

// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/inference.h"

using namespace std;

namespace py = pybind11;

void inference(py::module_ &m_) {
  m_.doc() = "pybind11 wrapper of inference";

  py::class_<gtsam::Symbol, boost::shared_ptr<gtsam::Symbol>>(m_, "Symbol")
      .def(py::init<>())
      .def(py::init<char, uint64_t>(), py::arg("c"), py::arg("j"))
      .def(py::init<size_t>(), py::arg("key"))
      .def("key", [](gtsam::Symbol *self) { return self->key(); })
      .def("print", [](gtsam::Symbol *self, const string &s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Symbol &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Symbol *self, const gtsam::Symbol &expected, double tol) { return self->equals(expected, tol); }, py::arg("expected"), py::arg("tol"))
      .def("chr", [](gtsam::Symbol *self) { return self->chr(); })
      .def("index", [](gtsam::Symbol *self) { return self->index(); })
      .def("string", [](gtsam::Symbol *self) { return self->string(); });
  pybind11::module m_symbol_shorthand = m_.def_submodule("symbol_shorthand", "symbol_shorthand submodule");

  m_symbol_shorthand.def("A", [](size_t j) { return gtsam::symbol_shorthand::A(j); }, py::arg("j"));
  m_symbol_shorthand.def("B", [](size_t j) { return gtsam::symbol_shorthand::B(j); }, py::arg("j"));
  m_symbol_shorthand.def("C", [](size_t j) { return gtsam::symbol_shorthand::C(j); }, py::arg("j"));
  m_symbol_shorthand.def("D", [](size_t j) { return gtsam::symbol_shorthand::D(j); }, py::arg("j"));
  m_symbol_shorthand.def("E", [](size_t j) { return gtsam::symbol_shorthand::E(j); }, py::arg("j"));
  m_symbol_shorthand.def("F", [](size_t j) { return gtsam::symbol_shorthand::F(j); }, py::arg("j"));
  m_symbol_shorthand.def("G", [](size_t j) { return gtsam::symbol_shorthand::G(j); }, py::arg("j"));
  m_symbol_shorthand.def("H", [](size_t j) { return gtsam::symbol_shorthand::H(j); }, py::arg("j"));
  m_symbol_shorthand.def("I", [](size_t j) { return gtsam::symbol_shorthand::I(j); }, py::arg("j"));
  m_symbol_shorthand.def("J", [](size_t j) { return gtsam::symbol_shorthand::J(j); }, py::arg("j"));
  m_symbol_shorthand.def("K", [](size_t j) { return gtsam::symbol_shorthand::K(j); }, py::arg("j"));
  m_symbol_shorthand.def("L", [](size_t j) { return gtsam::symbol_shorthand::L(j); }, py::arg("j"));
  m_symbol_shorthand.def("M", [](size_t j) { return gtsam::symbol_shorthand::M(j); }, py::arg("j"));
  m_symbol_shorthand.def("N", [](size_t j) { return gtsam::symbol_shorthand::N(j); }, py::arg("j"));
  m_symbol_shorthand.def("O", [](size_t j) { return gtsam::symbol_shorthand::O(j); }, py::arg("j"));
  m_symbol_shorthand.def("P", [](size_t j) { return gtsam::symbol_shorthand::P(j); }, py::arg("j"));
  m_symbol_shorthand.def("Q", [](size_t j) { return gtsam::symbol_shorthand::Q(j); }, py::arg("j"));
  m_symbol_shorthand.def("R", [](size_t j) { return gtsam::symbol_shorthand::R(j); }, py::arg("j"));
  m_symbol_shorthand.def("S", [](size_t j) { return gtsam::symbol_shorthand::S(j); }, py::arg("j"));
  m_symbol_shorthand.def("T", [](size_t j) { return gtsam::symbol_shorthand::T(j); }, py::arg("j"));
  m_symbol_shorthand.def("U", [](size_t j) { return gtsam::symbol_shorthand::U(j); }, py::arg("j"));
  m_symbol_shorthand.def("V", [](size_t j) { return gtsam::symbol_shorthand::V(j); }, py::arg("j"));
  m_symbol_shorthand.def("W", [](size_t j) { return gtsam::symbol_shorthand::W(j); }, py::arg("j"));
  m_symbol_shorthand.def("X", [](size_t j) { return gtsam::symbol_shorthand::X(j); }, py::arg("j"));
  m_symbol_shorthand.def("Y", [](size_t j) { return gtsam::symbol_shorthand::Y(j); }, py::arg("j"));
  m_symbol_shorthand.def("Z", [](size_t j) { return gtsam::symbol_shorthand::Z(j); }, py::arg("j"));
  py::class_<gtsam::LabeledSymbol, boost::shared_ptr<gtsam::LabeledSymbol>>(m_, "LabeledSymbol")
      .def(py::init<size_t>(), py::arg("full_key"))
      .def(py::init<const gtsam::LabeledSymbol &>(), py::arg("key"))
      .def(py::init<unsigned char, unsigned char, size_t>(), py::arg("valType"), py::arg("label"), py::arg("j"))
      .def("key", [](gtsam::LabeledSymbol *self) { return self->key(); })
      .def("label", [](gtsam::LabeledSymbol *self) { return self->label(); })
      .def("chr", [](gtsam::LabeledSymbol *self) { return self->chr(); })
      .def("index", [](gtsam::LabeledSymbol *self) { return self->index(); })
      .def("upper", [](gtsam::LabeledSymbol *self) { return self->upper(); })
      .def("lower", [](gtsam::LabeledSymbol *self) { return self->lower(); })
      .def("newChr", [](gtsam::LabeledSymbol *self, unsigned char c) { return self->newChr(c); }, py::arg("c"))
      .def("newLabel", [](gtsam::LabeledSymbol *self, unsigned char label) { return self->newLabel(label); }, py::arg("label"))
      .def("print", [](gtsam::LabeledSymbol *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::LabeledSymbol &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "");

  py::class_<gtsam::Ordering, boost::shared_ptr<gtsam::Ordering>> ordering(m_, "Ordering");
  ordering
      .def(py::init<>())
      .def(py::init<const gtsam::Ordering &>(), py::arg("other"))
      .def(py::init<const std::vector<size_t> &>(), py::arg("keys"))
      .def("print", [](gtsam::Ordering *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Ordering &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::Ordering *self, const gtsam::Ordering &ord, double tol) { return self->equals(ord, tol); }, py::arg("ord"), py::arg("tol"))
      .def("size", [](gtsam::Ordering *self) { return self->size(); })
      .def("at", [](gtsam::Ordering *self, size_t i) { return self->at(i); }, py::arg("i"))
      .def("push_back", [](gtsam::Ordering *self, size_t key) { self->push_back(key); }, py::arg("key"))
      .def("serialize", [](gtsam::Ordering *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Ordering *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Ordering &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Ordering obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("ColamdNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::NonlinearFactorGraph>(graph); }, py::arg("graph"))
      .def_static("ColamdDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::DiscreteFactorGraph>(graph); }, py::arg("graph"))
      .def_static("ColamdSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::SymbolicFactorGraph>(graph); }, py::arg("graph"))
      .def_static("ColamdGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::GaussianFactorGraph>(graph); }, py::arg("graph"))
      .def_static("ColamdHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::HybridGaussianFactorGraph>(graph); }, py::arg("graph"))
      .def_static("ColamdConstrainedLastNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::NonlinearFactorGraph>(graph, constrainLast, forceOrder); }, py::arg("graph"), py::arg("constrainLast"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::DiscreteFactorGraph>(graph, constrainLast, forceOrder); }, py::arg("graph"), py::arg("constrainLast"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::SymbolicFactorGraph>(graph, constrainLast, forceOrder); }, py::arg("graph"), py::arg("constrainLast"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::GaussianFactorGraph>(graph, constrainLast, forceOrder); }, py::arg("graph"), py::arg("constrainLast"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::HybridGaussianFactorGraph>(graph, constrainLast, forceOrder); }, py::arg("graph"), py::arg("constrainLast"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::NonlinearFactorGraph>(graph, constrainFirst, forceOrder); }, py::arg("graph"), py::arg("constrainFirst"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::DiscreteFactorGraph>(graph, constrainFirst, forceOrder); }, py::arg("graph"), py::arg("constrainFirst"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::SymbolicFactorGraph>(graph, constrainFirst, forceOrder); }, py::arg("graph"), py::arg("constrainFirst"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::GaussianFactorGraph>(graph, constrainFirst, forceOrder); }, py::arg("graph"), py::arg("constrainFirst"), py::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::HybridGaussianFactorGraph>(graph, constrainFirst, forceOrder); }, py::arg("graph"), py::arg("constrainFirst"), py::arg("forceOrder") = false)
      .def_static("NaturalNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::NonlinearFactorGraph>(graph); }, py::arg("graph"))
      .def_static("NaturalDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::DiscreteFactorGraph>(graph); }, py::arg("graph"))
      .def_static("NaturalSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::SymbolicFactorGraph>(graph); }, py::arg("graph"))
      .def_static("NaturalGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::GaussianFactorGraph>(graph); }, py::arg("graph"))
      .def_static("NaturalHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::HybridGaussianFactorGraph>(graph); }, py::arg("graph"))
      .def_static("MetisNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::NonlinearFactorGraph>(graph); }, py::arg("graph"))
      .def_static("MetisDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::DiscreteFactorGraph>(graph); }, py::arg("graph"))
      .def_static("MetisSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::SymbolicFactorGraph>(graph); }, py::arg("graph"))
      .def_static("MetisGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::GaussianFactorGraph>(graph); }, py::arg("graph"))
      .def_static("MetisHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::HybridGaussianFactorGraph>(graph); }, py::arg("graph"))
      .def_static("CreateNonlinearFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::NonlinearFactorGraph>(orderingType, graph); }, py::arg("orderingType"), py::arg("graph"))
      .def_static("CreateDiscreteFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::DiscreteFactorGraph>(orderingType, graph); }, py::arg("orderingType"), py::arg("graph"))
      .def_static("CreateSymbolicFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::SymbolicFactorGraph>(orderingType, graph); }, py::arg("orderingType"), py::arg("graph"))
      .def_static("CreateGaussianFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::GaussianFactorGraph>(orderingType, graph); }, py::arg("orderingType"), py::arg("graph"))
      .def_static("CreateHybridGaussianFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::HybridGaussianFactorGraph>(orderingType, graph); }, py::arg("orderingType"), py::arg("graph"));

  py::enum_<gtsam::Ordering::OrderingType>(ordering, "OrderingType", py::arithmetic())
      .value("COLAMD", gtsam::Ordering::OrderingType::COLAMD)
      .value("METIS", gtsam::Ordering::OrderingType::METIS)
      .value("NATURAL", gtsam::Ordering::OrderingType::NATURAL)
      .value("CUSTOM", gtsam::Ordering::OrderingType::CUSTOM);

  py::class_<gtsam::DotWriter, boost::shared_ptr<gtsam::DotWriter>>(m_, "DotWriter")
      .def(py::init<double, double, bool, bool, bool>(), py::arg("figureWidthInches") = 5, py::arg("figureHeightInches") = 5, py::arg("plotFactorPoints") = true, py::arg("connectKeysToFactor") = true, py::arg("binaryEdges") = true)
      .def_readwrite("figureWidthInches", &gtsam::DotWriter::figureWidthInches)
      .def_readwrite("figureHeightInches", &gtsam::DotWriter::figureHeightInches)
      .def_readwrite("plotFactorPoints", &gtsam::DotWriter::plotFactorPoints)
      .def_readwrite("connectKeysToFactor", &gtsam::DotWriter::connectKeysToFactor)
      .def_readwrite("binaryEdges", &gtsam::DotWriter::binaryEdges)
      .def_readwrite("variablePositions", &gtsam::DotWriter::variablePositions)
      .def_readwrite("positionHints", &gtsam::DotWriter::positionHints)
      .def_readwrite("boxes", &gtsam::DotWriter::boxes)
      .def_readwrite("factorPositions", &gtsam::DotWriter::factorPositions);

  py::class_<gtsam::VariableIndex, boost::shared_ptr<gtsam::VariableIndex>>(m_, "VariableIndex")
      .def(py::init<>())
      .def(py::init<const gtsam::SymbolicFactorGraph &>(), py::arg("sfg"))
      .def(py::init<const gtsam::GaussianFactorGraph &>(), py::arg("gfg"))
      .def(py::init<const gtsam::NonlinearFactorGraph &>(), py::arg("fg"))
      .def(py::init<const gtsam::VariableIndex &>(), py::arg("other"))
      .def("equals", [](gtsam::VariableIndex *self, const gtsam::VariableIndex &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol"))
      .def("print", [](gtsam::VariableIndex *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "VariableIndex: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::VariableIndex &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "VariableIndex: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("size", [](gtsam::VariableIndex *self) { return self->size(); })
      .def("nFactors", [](gtsam::VariableIndex *self) { return self->nFactors(); })
      .def("nEntries", [](gtsam::VariableIndex *self) { return self->nEntries(); });

  py::class_<gtsam::Factor, boost::shared_ptr<gtsam::Factor>>(m_, "Factor")
      .def("print", [](gtsam::Factor *self, string s, const gtsam::KeyFormatter &keyFormatter) { py::scoped_ostream_redirect output; self->print(s, keyFormatter); }, py::arg("s") = "Factor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Factor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, py::arg("s") = "Factor\n", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("printKeys", [](gtsam::Factor *self, string s) { self->printKeys(s); }, py::arg("s") = "")
      .def("equals", [](gtsam::Factor *self, const gtsam::Factor &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol") = 1e-9)
      .def("empty", [](gtsam::Factor *self) { return self->empty(); })
      .def("size", [](gtsam::Factor *self) { return self->size(); })
      .def("keys", [](gtsam::Factor *self) { return self->keys(); });

  m_.def("PrintKeyList", [](const gtsam::KeyList &keys, const string &s, const gtsam::KeyFormatter &keyFormatter) { gtsam::PrintKeyList(keys, s, keyFormatter); }, py::arg("keys"), py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("PrintKeyVector", [](const gtsam::KeyVector &keys, const string &s, const gtsam::KeyFormatter &keyFormatter) { gtsam::PrintKeyVector(keys, s, keyFormatter); }, py::arg("keys"), py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("PrintKeySet", [](const gtsam::KeySet &keys, const string &s, const gtsam::KeyFormatter &keyFormatter) { gtsam::PrintKeySet(keys, s, keyFormatter); }, py::arg("keys"), py::arg("s") = "", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("symbol", [](char chr, size_t index) { return gtsam::symbol(chr, index); }, py::arg("chr"), py::arg("index"));
  m_.def("symbolChr", [](size_t key) { return gtsam::symbolChr(key); }, py::arg("key"));
  m_.def("symbolIndex", [](size_t key) { return gtsam::symbolIndex(key); }, py::arg("key"));
  m_.def("mrsymbol", [](unsigned char c, unsigned char label, size_t j) { return gtsam::mrsymbol(c, label, j); }, py::arg("c"), py::arg("label"), py::arg("j"));
  m_.def("mrsymbolChr", [](size_t key) { return gtsam::mrsymbolChr(key); }, py::arg("key"));
  m_.def("mrsymbolLabel", [](size_t key) { return gtsam::mrsymbolLabel(key); }, py::arg("key"));
  m_.def("mrsymbolIndex", [](size_t key) { return gtsam::mrsymbolIndex(key); }, py::arg("key"));

// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/inference.h"
}
