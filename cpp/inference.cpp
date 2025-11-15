/**
 * @file    inference.cpp
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
#include <nanobind/stl/map.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

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

using namespace std;

namespace nb = nanobind;

void inference(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of inference";

  nb::class_<gtsam::Symbol>(m_, "Symbol")
      .def(nb::init<>())
      .def(nb::init<unsigned char, uint64_t>(), nb::arg("c"), nb::arg("j"))
      .def(nb::init<size_t>(), nb::arg("key"))
      .def("key", [](gtsam::Symbol *self) { return self->key(); })
      .def("print", [](gtsam::Symbol *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::Symbol &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::Symbol *self, const gtsam::Symbol &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def("chr", [](gtsam::Symbol *self) { return self->chr(); })
      .def("index", [](gtsam::Symbol *self) { return self->index(); })
      .def("string", [](gtsam::Symbol *self) { return self->string(); });
  auto m_symbol_shorthand = m_.def_submodule("symbol_shorthand", "symbol_shorthand submodule");

  m_symbol_shorthand.def("A", [](size_t j) { return gtsam::symbol_shorthand::A(j); }, nb::arg("j"));
  m_symbol_shorthand.def("B", [](size_t j) { return gtsam::symbol_shorthand::B(j); }, nb::arg("j"));
  m_symbol_shorthand.def("C", [](size_t j) { return gtsam::symbol_shorthand::C(j); }, nb::arg("j"));
  m_symbol_shorthand.def("D", [](size_t j) { return gtsam::symbol_shorthand::D(j); }, nb::arg("j"));
  m_symbol_shorthand.def("E", [](size_t j) { return gtsam::symbol_shorthand::E(j); }, nb::arg("j"));
  m_symbol_shorthand.def("F", [](size_t j) { return gtsam::symbol_shorthand::F(j); }, nb::arg("j"));
  m_symbol_shorthand.def("G", [](size_t j) { return gtsam::symbol_shorthand::G(j); }, nb::arg("j"));
  m_symbol_shorthand.def("H", [](size_t j) { return gtsam::symbol_shorthand::H(j); }, nb::arg("j"));
  m_symbol_shorthand.def("I", [](size_t j) { return gtsam::symbol_shorthand::I(j); }, nb::arg("j"));
  m_symbol_shorthand.def("J", [](size_t j) { return gtsam::symbol_shorthand::J(j); }, nb::arg("j"));
  m_symbol_shorthand.def("K", [](size_t j) { return gtsam::symbol_shorthand::K(j); }, nb::arg("j"));
  m_symbol_shorthand.def("L", [](size_t j) { return gtsam::symbol_shorthand::L(j); }, nb::arg("j"));
  m_symbol_shorthand.def("M", [](size_t j) { return gtsam::symbol_shorthand::M(j); }, nb::arg("j"));
  m_symbol_shorthand.def("N", [](size_t j) { return gtsam::symbol_shorthand::N(j); }, nb::arg("j"));
  m_symbol_shorthand.def("O", [](size_t j) { return gtsam::symbol_shorthand::O(j); }, nb::arg("j"));
  m_symbol_shorthand.def("P", [](size_t j) { return gtsam::symbol_shorthand::P(j); }, nb::arg("j"));
  m_symbol_shorthand.def("Q", [](size_t j) { return gtsam::symbol_shorthand::Q(j); }, nb::arg("j"));
  m_symbol_shorthand.def("R", [](size_t j) { return gtsam::symbol_shorthand::R(j); }, nb::arg("j"));
  m_symbol_shorthand.def("S", [](size_t j) { return gtsam::symbol_shorthand::S(j); }, nb::arg("j"));
  m_symbol_shorthand.def("T", [](size_t j) { return gtsam::symbol_shorthand::T(j); }, nb::arg("j"));
  m_symbol_shorthand.def("U", [](size_t j) { return gtsam::symbol_shorthand::U(j); }, nb::arg("j"));
  m_symbol_shorthand.def("V", [](size_t j) { return gtsam::symbol_shorthand::V(j); }, nb::arg("j"));
  m_symbol_shorthand.def("W", [](size_t j) { return gtsam::symbol_shorthand::W(j); }, nb::arg("j"));
  m_symbol_shorthand.def("X", [](size_t j) { return gtsam::symbol_shorthand::X(j); }, nb::arg("j"));
  m_symbol_shorthand.def("Y", [](size_t j) { return gtsam::symbol_shorthand::Y(j); }, nb::arg("j"));
  m_symbol_shorthand.def("Z", [](size_t j) { return gtsam::symbol_shorthand::Z(j); }, nb::arg("j"));
  nb::class_<gtsam::LabeledSymbol>(m_, "LabeledSymbol")
      .def(nb::init<size_t>(), nb::arg("full_key"))
      .def(nb::init<const gtsam::LabeledSymbol &>(), nb::arg("key"))
      .def(nb::init<unsigned char, unsigned char, size_t>(), nb::arg("valType"), nb::arg("label"), nb::arg("j"))
      .def("key", [](gtsam::LabeledSymbol *self) { return self->key(); })
      .def("label", [](gtsam::LabeledSymbol *self) { return self->label(); })
      .def("chr", [](gtsam::LabeledSymbol *self) { return self->chr(); })
      .def("index", [](gtsam::LabeledSymbol *self) { return self->index(); })
      .def("upper", [](gtsam::LabeledSymbol *self) { return self->upper(); })
      .def("lower", [](gtsam::LabeledSymbol *self) { return self->lower(); })
      .def("newChr", [](gtsam::LabeledSymbol *self, unsigned char c) { return self->newChr(c); }, nb::arg("c"))
      .def("newLabel", [](gtsam::LabeledSymbol *self, unsigned char label) { return self->newLabel(label); }, nb::arg("label"))
      .def("print", [](gtsam::LabeledSymbol *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::LabeledSymbol &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::Ordering> ordering(m_, "Ordering");
  ordering
      .def(nb::init<>())
      .def(nb::init<const gtsam::Ordering &>(), nb::arg("other"))
      .def(nb::init<const std::vector<size_t> &>(), nb::arg("keys"))
      .def("print", [](gtsam::Ordering *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Ordering &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::Ordering *self, const gtsam::Ordering &ord, double tol) { return self->equals(ord, tol); }, nb::arg("ord"), nb::arg("tol"))
      .def("size", [](gtsam::Ordering *self) { return self->size(); })
      .def("at", [](gtsam::Ordering *self, size_t i) { return self->at(i); }, nb::arg("i"))
      .def("push_back", [](gtsam::Ordering *self, size_t key) { self->push_back(key); }, nb::arg("key"))
      // .def("serialize", [](gtsam::Ordering *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::Ordering *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::Ordering &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::Ordering obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("ColamdNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::NonlinearFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("ColamdDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::DiscreteFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("ColamdSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::SymbolicFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("ColamdGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::GaussianFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("ColamdHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Colamd<gtsam::HybridGaussianFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("ColamdConstrainedLastNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::NonlinearFactorGraph>(graph, constrainLast, forceOrder); }, nb::arg("graph"), nb::arg("constrainLast"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::DiscreteFactorGraph>(graph, constrainLast, forceOrder); }, nb::arg("graph"), nb::arg("constrainLast"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::SymbolicFactorGraph>(graph, constrainLast, forceOrder); }, nb::arg("graph"), nb::arg("constrainLast"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::GaussianFactorGraph>(graph, constrainLast, forceOrder); }, nb::arg("graph"), nb::arg("constrainLast"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedLastHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph, const gtsam::KeyVector &constrainLast, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedLast<gtsam::HybridGaussianFactorGraph>(graph, constrainLast, forceOrder); }, nb::arg("graph"), nb::arg("constrainLast"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::NonlinearFactorGraph>(graph, constrainFirst, forceOrder); }, nb::arg("graph"), nb::arg("constrainFirst"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::DiscreteFactorGraph>(graph, constrainFirst, forceOrder); }, nb::arg("graph"), nb::arg("constrainFirst"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::SymbolicFactorGraph>(graph, constrainFirst, forceOrder); }, nb::arg("graph"), nb::arg("constrainFirst"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::GaussianFactorGraph>(graph, constrainFirst, forceOrder); }, nb::arg("graph"), nb::arg("constrainFirst"), nb::arg("forceOrder") = false)
      .def_static("ColamdConstrainedFirstHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph, const gtsam::KeyVector &constrainFirst, bool forceOrder) { return gtsam::Ordering::ColamdConstrainedFirst<gtsam::HybridGaussianFactorGraph>(graph, constrainFirst, forceOrder); }, nb::arg("graph"), nb::arg("constrainFirst"), nb::arg("forceOrder") = false)
      .def_static("NaturalNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::NonlinearFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("NaturalDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::DiscreteFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("NaturalSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::SymbolicFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("NaturalGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::GaussianFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("NaturalHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Natural<gtsam::HybridGaussianFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("MetisNonlinearFactorGraph", [](const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::NonlinearFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("MetisDiscreteFactorGraph", [](const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::DiscreteFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("MetisSymbolicFactorGraph", [](const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::SymbolicFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("MetisGaussianFactorGraph", [](const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::GaussianFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("MetisHybridGaussianFactorGraph", [](const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Metis<gtsam::HybridGaussianFactorGraph>(graph); }, nb::arg("graph"))
      .def_static("CreateNonlinearFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::NonlinearFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::NonlinearFactorGraph>(orderingType, graph); }, nb::arg("orderingType"), nb::arg("graph"))
      .def_static("CreateDiscreteFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::DiscreteFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::DiscreteFactorGraph>(orderingType, graph); }, nb::arg("orderingType"), nb::arg("graph"))
      .def_static("CreateSymbolicFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::SymbolicFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::SymbolicFactorGraph>(orderingType, graph); }, nb::arg("orderingType"), nb::arg("graph"))
      .def_static("CreateGaussianFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::GaussianFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::GaussianFactorGraph>(orderingType, graph); }, nb::arg("orderingType"), nb::arg("graph"))
      .def_static("CreateHybridGaussianFactorGraph", [](gtsam::Ordering::OrderingType orderingType, const gtsam::HybridGaussianFactorGraph &graph) { return gtsam::Ordering::Create<gtsam::HybridGaussianFactorGraph>(orderingType, graph); }, nb::arg("orderingType"), nb::arg("graph"));

  nb::enum_<gtsam::Ordering::OrderingType>(ordering, "OrderingType", nb::is_arithmetic())
      .value("COLAMD", gtsam::Ordering::OrderingType::COLAMD)
      .value("METIS", gtsam::Ordering::OrderingType::METIS)
      .value("NATURAL", gtsam::Ordering::OrderingType::NATURAL)
      .value("CUSTOM", gtsam::Ordering::OrderingType::CUSTOM);

  nb::class_<gtsam::DotWriter>(m_, "DotWriter")
      .def(nb::init<double, double, bool, bool, bool>(), nb::arg("figureWidthInches") = 5, nb::arg("figureHeightInches") = 5, nb::arg("plotFactorPoints") = true, nb::arg("connectKeysToFactor") = true, nb::arg("binaryEdges") = true)
      .def_rw("figureWidthInches", &gtsam::DotWriter::figureWidthInches)
      .def_rw("figureHeightInches", &gtsam::DotWriter::figureHeightInches)
      .def_rw("plotFactorPoints", &gtsam::DotWriter::plotFactorPoints)
      .def_rw("connectKeysToFactor", &gtsam::DotWriter::connectKeysToFactor)
      .def_rw("binaryEdges", &gtsam::DotWriter::binaryEdges)
      .def_rw("variablePositions", &gtsam::DotWriter::variablePositions)
      .def_rw("positionHints", &gtsam::DotWriter::positionHints)
      .def_rw("boxes", &gtsam::DotWriter::boxes)
      .def_rw("factorPositions", &gtsam::DotWriter::factorPositions);

  nb::class_<gtsam::VariableIndex>(m_, "VariableIndex")
      .def(nb::init<>())
      .def(nb::init<const gtsam::SymbolicFactorGraph &>(), nb::arg("sfg"))
      .def(nb::init<const gtsam::GaussianFactorGraph &>(), nb::arg("gfg"))
      .def(nb::init<const gtsam::NonlinearFactorGraph &>(), nb::arg("fg"))
      .def(nb::init<const gtsam::VariableIndex &>(), nb::arg("other"))
      .def("equals", [](gtsam::VariableIndex *self, const gtsam::VariableIndex &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("print", [](gtsam::VariableIndex *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "VariableIndex: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::VariableIndex &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "VariableIndex: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("size", [](gtsam::VariableIndex *self) { return self->size(); })
      .def("nFactors", [](gtsam::VariableIndex *self) { return self->nFactors(); })
      .def("nEntries", [](gtsam::VariableIndex *self) { return self->nEntries(); });

  nb::class_<gtsam::Factor>(m_, "Factor")
      .def("print", [](gtsam::Factor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "Factor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Factor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "Factor\n", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("printKeys", [](gtsam::Factor *self, string s) { self->printKeys(s); }, nb::arg("s") = "")
      .def("equals", [](gtsam::Factor *self, const gtsam::Factor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol") = 1e-9)
      .def("empty", [](gtsam::Factor *self) { return self->empty(); })
      .def("size", [](gtsam::Factor *self) { return self->size(); })
      .def("keys", [](gtsam::Factor *self) { return self->keys(); });

  m_.def("PrintKeyList", [](const gtsam::KeyList &keys, const string &s, const gtsam::KeyFormatter &keyFormatter) { gtsam::PrintKeyList(keys, s, keyFormatter); }, nb::arg("keys"), nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("PrintKeyVector", [](const gtsam::KeyVector &keys, const string &s, const gtsam::KeyFormatter &keyFormatter) { gtsam::PrintKeyVector(keys, s, keyFormatter); }, nb::arg("keys"), nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("PrintKeySet", [](const gtsam::KeySet &keys, const string &s, const gtsam::KeyFormatter &keyFormatter) { gtsam::PrintKeySet(keys, s, keyFormatter); }, nb::arg("keys"), nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
  m_.def("symbol", [](char chr, size_t index) { return gtsam::symbol(chr, index); }, nb::arg("chr"), nb::arg("index"));
  m_.def("symbolChr", [](size_t key) { return gtsam::symbolChr(key); }, nb::arg("key"));
  m_.def("symbolIndex", [](size_t key) { return gtsam::symbolIndex(key); }, nb::arg("key"));
  m_.def("mrsymbol", [](unsigned char c, unsigned char label, size_t j) { return gtsam::mrsymbol(c, label, j); }, nb::arg("c"), nb::arg("label"), nb::arg("j"));
  m_.def("mrsymbolChr", [](size_t key) { return gtsam::mrsymbolChr(key); }, nb::arg("key"));
  m_.def("mrsymbolLabel", [](size_t key) { return gtsam::mrsymbolLabel(key); }, nb::arg("key"));
  m_.def("mrsymbolIndex", [](size_t key) { return gtsam::mrsymbolIndex(key); }, nb::arg("key"));
}
