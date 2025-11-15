/**
 * @file    basis.cpp
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
#include <nanobind/stl/string.h>

#include "utils/boost_shared_ptr.h"

// These are the included headers listed in `gtsam.i`
#include "gtsam/basis/BasisFactors.h"
#include "gtsam/basis/Chebyshev.h"
#include "gtsam/basis/Chebyshev2.h"
#include "gtsam/basis/FitBasis.h"
#include "gtsam/basis/Fourier.h"
#include "gtsam/basis/ParameterMatrix.h"

using namespace std;

namespace nb = nanobind;

void basis(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of basis";

  nb::class_<gtsam::FourierBasis>(m_, "FourierBasis")
      .def_static("CalculateWeights", [](size_t N, double x) { return gtsam::FourierBasis::CalculateWeights(N, x); }, nb::arg("N"), nb::arg("x"))
      .def_static("WeightMatrix", [](size_t N, const gtsam::Vector &x) { return gtsam::FourierBasis::WeightMatrix(N, x); }, nb::arg("N"), nb::arg("x"))
      .def_static("DifferentiationMatrix", [](size_t N) { return gtsam::FourierBasis::DifferentiationMatrix(N); }, nb::arg("N"))
      .def_static("DerivativeWeights", [](size_t N, double x) { return gtsam::FourierBasis::DerivativeWeights(N, x); }, nb::arg("N"), nb::arg("x"));

  nb::class_<gtsam::Chebyshev1Basis>(m_, "Chebyshev1Basis")
      .def_static("CalculateWeights", [](size_t N, double x) { return gtsam::Chebyshev1Basis::CalculateWeights(N, x); }, nb::arg("N"), nb::arg("x"))
      .def_static("WeightMatrix", [](size_t N, const gtsam::Vector &X) { return gtsam::Chebyshev1Basis::WeightMatrix(N, X); }, nb::arg("N"), nb::arg("X"));

  nb::class_<gtsam::Chebyshev2Basis>(m_, "Chebyshev2Basis")
      .def_static("CalculateWeights", [](size_t N, double x) { return gtsam::Chebyshev2Basis::CalculateWeights(N, x); }, nb::arg("N"), nb::arg("x"))
      .def_static("WeightMatrix", [](size_t N, const gtsam::Vector &x) { return gtsam::Chebyshev2Basis::WeightMatrix(N, x); }, nb::arg("N"), nb::arg("x"));

  nb::class_<gtsam::Chebyshev2>(m_, "Chebyshev2")
      .def_static("Point", [](size_t N, int j) { return gtsam::Chebyshev2::Point(N, j); }, nb::arg("N"), nb::arg("j"))
      .def_static("Point", [](size_t N, int j, double a, double b) { return gtsam::Chebyshev2::Point(N, j, a, b); }, nb::arg("N"), nb::arg("j"), nb::arg("a"), nb::arg("b"))
      .def_static("Points", [](size_t N) { return gtsam::Chebyshev2::Points(N); }, nb::arg("N"))
      .def_static("Points", [](size_t N, double a, double b) { return gtsam::Chebyshev2::Points(N, a, b); }, nb::arg("N"), nb::arg("a"), nb::arg("b"))
      .def_static("WeightMatrix", [](size_t N, const gtsam::Vector &X) { return gtsam::Chebyshev2::WeightMatrix(N, X); }, nb::arg("N"), nb::arg("X"))
      .def_static("WeightMatrix", [](size_t N, const gtsam::Vector &X, double a, double b) { return gtsam::Chebyshev2::WeightMatrix(N, X, a, b); }, nb::arg("N"), nb::arg("X"), nb::arg("a"), nb::arg("b"))
      .def_static("CalculateWeights", [](size_t N, double x, double a, double b) { return gtsam::Chebyshev2::CalculateWeights(N, x, a, b); }, nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"))
      .def_static("DerivativeWeights", [](size_t N, double x, double a, double b) { return gtsam::Chebyshev2::DerivativeWeights(N, x, a, b); }, nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"))
      .def_static("IntegrationWeights", [](size_t N, double a, double b) { return gtsam::Chebyshev2::IntegrationWeights(N, a, b); }, nb::arg("N"), nb::arg("a"), nb::arg("b"))
      .def_static("DifferentiationMatrix", [](size_t N, double a, double b) { return gtsam::Chebyshev2::DifferentiationMatrix(N, a, b); }, nb::arg("N"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::ParameterMatrix<1>>(m_, "ParameterMatrix1")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<1> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<1> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<1> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<2>>(m_, "ParameterMatrix2")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<2> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<2> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<2> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<3>>(m_, "ParameterMatrix3")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<3> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<3> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<3> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<4>>(m_, "ParameterMatrix4")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<4> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<4> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<4> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<5>>(m_, "ParameterMatrix5")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<5> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<5> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<5> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<6>>(m_, "ParameterMatrix6")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<6> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<6> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<6> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<7>>(m_, "ParameterMatrix7")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<7> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<7> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<7> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<8>>(m_, "ParameterMatrix8")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<8> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<8> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<8> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<9>>(m_, "ParameterMatrix9")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<9> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<9> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<9> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<10>>(m_, "ParameterMatrix10")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<10> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<10> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<10> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<11>>(m_, "ParameterMatrix11")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<11> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<11> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<11> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<12>>(m_, "ParameterMatrix12")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<12> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<12> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<12> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<13>>(m_, "ParameterMatrix13")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<13> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<13> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<13> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<14>>(m_, "ParameterMatrix14")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<14> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<14> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<14> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ParameterMatrix<15>>(m_, "ParameterMatrix15")
      .def(nb::init<const size_t>(), nb::arg("N"))
      .def(nb::init<const gtsam::Matrix &>(), nb::arg("matrix"))
      .def("matrix", [](gtsam::ParameterMatrix<15> *self) { return self->matrix(); })
      .def("print", [](gtsam::ParameterMatrix<15> *self, const string &s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ParameterMatrix<15> &self, const string &s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::EvaluationFactor<gtsam::Chebyshev2>, gtsam::NoiseModelFactor>(m_, "EvaluationFactorChebyshev2")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::EvaluationFactor<gtsam::Chebyshev1Basis>, gtsam::NoiseModelFactor>(m_, "EvaluationFactorChebyshev1Basis")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::EvaluationFactor<gtsam::Chebyshev2Basis>, gtsam::NoiseModelFactor>(m_, "EvaluationFactorChebyshev2Basis")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::EvaluationFactor<gtsam::FourierBasis>, gtsam::NoiseModelFactor>(m_, "EvaluationFactorFourierBasis")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::FitBasis<gtsam::FourierBasis>>(m_, "FitBasisFourierBasis")
      .def(nb::init<const std::map<double, double> &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def("parameters", [](gtsam::FitBasis<gtsam::FourierBasis> *self) { return self->parameters(); })
      .def_static("NonlinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::FourierBasis>::NonlinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def_static("LinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::FourierBasis>::LinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"));

  nb::class_<gtsam::FitBasis<gtsam::Chebyshev1Basis>>(m_, "FitBasisChebyshev1Basis")
      .def(nb::init<const std::map<double, double> &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def("parameters", [](gtsam::FitBasis<gtsam::Chebyshev1Basis> *self) { return self->parameters(); })
      .def_static("NonlinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::Chebyshev1Basis>::NonlinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def_static("LinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::Chebyshev1Basis>::LinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"));

  nb::class_<gtsam::FitBasis<gtsam::Chebyshev2Basis>>(m_, "FitBasisChebyshev2Basis")
      .def(nb::init<const std::map<double, double> &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def("parameters", [](gtsam::FitBasis<gtsam::Chebyshev2Basis> *self) { return self->parameters(); })
      .def_static("NonlinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::Chebyshev2Basis>::NonlinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def_static("LinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::Chebyshev2Basis>::LinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"));

  nb::class_<gtsam::FitBasis<gtsam::Chebyshev2>>(m_, "FitBasisChebyshev2")
      .def(nb::init<const std::map<double, double> &, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def("parameters", [](gtsam::FitBasis<gtsam::Chebyshev2> *self) { return self->parameters(); })
      .def_static("NonlinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::Chebyshev2>::NonlinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"))
      .def_static("LinearGraph", [](const std::map<double, double> &sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N) { return gtsam::FitBasis<gtsam::Chebyshev2>::LinearGraph(sequence, model, N); }, nb::arg("sequence"), nb::arg("model"), nb::arg("N"));

  nb::class_<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 3>, gtsam::NoiseModelFactor>(m_, "VectorEvaluationFactorChebyshev2D3")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 4>, gtsam::NoiseModelFactor>(m_, "VectorEvaluationFactorChebyshev2D4")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 12>, gtsam::NoiseModelFactor>(m_, "VectorEvaluationFactorChebyshev2D12")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 3>, gtsam::NoiseModelFactor>(m_, "VectorComponentFactorChebyshev2D3")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("i"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("i"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 4>, gtsam::NoiseModelFactor>(m_, "VectorComponentFactorChebyshev2D4")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("i"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("i"), nb::arg("x"), nb::arg("a"), nb::arg("b"));

  nb::class_<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 12>, gtsam::NoiseModelFactor>(m_, "VectorComponentFactorChebyshev2D12")
      .def(nb::init<>())
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("i"), nb::arg("x"))
      .def(nb::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double, double, double>(), nb::arg("key"), nb::arg("z"), nb::arg("model"), nb::arg("N"), nb::arg("i"), nb::arg("x"), nb::arg("a"), nb::arg("b"));
}
