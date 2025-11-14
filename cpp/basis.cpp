/**
 * @file    basis.cpp
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
#include "gtsam/basis/Fourier.h"
#include "gtsam/basis/Chebyshev.h"
#include "gtsam/basis/Chebyshev2.h"
#include "gtsam/basis/ParameterMatrix.h"
#include "gtsam/basis/BasisFactors.h"
#include "gtsam/basis/FitBasis.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/basis.h"

using namespace std;

namespace py = pybind11;



void basis(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of basis";




    py::class_<gtsam::FourierBasis, boost::shared_ptr<gtsam::FourierBasis>>(m_, "FourierBasis")
        .def_static("CalculateWeights",[](size_t N, double x){return gtsam::FourierBasis::CalculateWeights(N, x);}, py::arg("N"), py::arg("x"))
        .def_static("WeightMatrix",[](size_t N, const gtsam::Vector& x){return gtsam::FourierBasis::WeightMatrix(N, x);}, py::arg("N"), py::arg("x"))
        .def_static("DifferentiationMatrix",[](size_t N){return gtsam::FourierBasis::DifferentiationMatrix(N);}, py::arg("N"))
        .def_static("DerivativeWeights",[](size_t N, double x){return gtsam::FourierBasis::DerivativeWeights(N, x);}, py::arg("N"), py::arg("x"));

    py::class_<gtsam::Chebyshev1Basis, boost::shared_ptr<gtsam::Chebyshev1Basis>>(m_, "Chebyshev1Basis")
        .def_static("CalculateWeights",[](size_t N, double x){return gtsam::Chebyshev1Basis::CalculateWeights(N, x);}, py::arg("N"), py::arg("x"))
        .def_static("WeightMatrix",[](size_t N, const gtsam::Vector& X){return gtsam::Chebyshev1Basis::WeightMatrix(N, X);}, py::arg("N"), py::arg("X"));

    py::class_<gtsam::Chebyshev2Basis, boost::shared_ptr<gtsam::Chebyshev2Basis>>(m_, "Chebyshev2Basis")
        .def_static("CalculateWeights",[](size_t N, double x){return gtsam::Chebyshev2Basis::CalculateWeights(N, x);}, py::arg("N"), py::arg("x"))
        .def_static("WeightMatrix",[](size_t N, const gtsam::Vector& x){return gtsam::Chebyshev2Basis::WeightMatrix(N, x);}, py::arg("N"), py::arg("x"));

    py::class_<gtsam::Chebyshev2, boost::shared_ptr<gtsam::Chebyshev2>>(m_, "Chebyshev2")
        .def_static("Point",[](size_t N, int j){return gtsam::Chebyshev2::Point(N, j);}, py::arg("N"), py::arg("j"))
        .def_static("Point",[](size_t N, int j, double a, double b){return gtsam::Chebyshev2::Point(N, j, a, b);}, py::arg("N"), py::arg("j"), py::arg("a"), py::arg("b"))
        .def_static("Points",[](size_t N){return gtsam::Chebyshev2::Points(N);}, py::arg("N"))
        .def_static("Points",[](size_t N, double a, double b){return gtsam::Chebyshev2::Points(N, a, b);}, py::arg("N"), py::arg("a"), py::arg("b"))
        .def_static("WeightMatrix",[](size_t N, const gtsam::Vector& X){return gtsam::Chebyshev2::WeightMatrix(N, X);}, py::arg("N"), py::arg("X"))
        .def_static("WeightMatrix",[](size_t N, const gtsam::Vector& X, double a, double b){return gtsam::Chebyshev2::WeightMatrix(N, X, a, b);}, py::arg("N"), py::arg("X"), py::arg("a"), py::arg("b"))
        .def_static("CalculateWeights",[](size_t N, double x, double a, double b){return gtsam::Chebyshev2::CalculateWeights(N, x, a, b);}, py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"))
        .def_static("DerivativeWeights",[](size_t N, double x, double a, double b){return gtsam::Chebyshev2::DerivativeWeights(N, x, a, b);}, py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"))
        .def_static("IntegrationWeights",[](size_t N, double a, double b){return gtsam::Chebyshev2::IntegrationWeights(N, a, b);}, py::arg("N"), py::arg("a"), py::arg("b"))
        .def_static("DifferentiationMatrix",[](size_t N, double a, double b){return gtsam::Chebyshev2::DifferentiationMatrix(N, a, b);}, py::arg("N"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::ParameterMatrix<1>, boost::shared_ptr<gtsam::ParameterMatrix<1>>>(m_, "ParameterMatrix1")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<1>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<1>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<1>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<2>, boost::shared_ptr<gtsam::ParameterMatrix<2>>>(m_, "ParameterMatrix2")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<2>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<2>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<2>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<3>, boost::shared_ptr<gtsam::ParameterMatrix<3>>>(m_, "ParameterMatrix3")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<3>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<3>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<3>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<4>, boost::shared_ptr<gtsam::ParameterMatrix<4>>>(m_, "ParameterMatrix4")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<4>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<4>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<4>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<5>, boost::shared_ptr<gtsam::ParameterMatrix<5>>>(m_, "ParameterMatrix5")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<5>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<5>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<5>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<6>, boost::shared_ptr<gtsam::ParameterMatrix<6>>>(m_, "ParameterMatrix6")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<6>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<6>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<6>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<7>, boost::shared_ptr<gtsam::ParameterMatrix<7>>>(m_, "ParameterMatrix7")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<7>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<7>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<7>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<8>, boost::shared_ptr<gtsam::ParameterMatrix<8>>>(m_, "ParameterMatrix8")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<8>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<8>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<8>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<9>, boost::shared_ptr<gtsam::ParameterMatrix<9>>>(m_, "ParameterMatrix9")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<9>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<9>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<9>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<10>, boost::shared_ptr<gtsam::ParameterMatrix<10>>>(m_, "ParameterMatrix10")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<10>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<10>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<10>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<11>, boost::shared_ptr<gtsam::ParameterMatrix<11>>>(m_, "ParameterMatrix11")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<11>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<11>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<11>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<12>, boost::shared_ptr<gtsam::ParameterMatrix<12>>>(m_, "ParameterMatrix12")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<12>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<12>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<12>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<13>, boost::shared_ptr<gtsam::ParameterMatrix<13>>>(m_, "ParameterMatrix13")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<13>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<13>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<13>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<14>, boost::shared_ptr<gtsam::ParameterMatrix<14>>>(m_, "ParameterMatrix14")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<14>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<14>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<14>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::ParameterMatrix<15>, boost::shared_ptr<gtsam::ParameterMatrix<15>>>(m_, "ParameterMatrix15")
        .def(py::init<const size_t>(), py::arg("N"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("matrix"))
        .def("matrix",[](gtsam::ParameterMatrix<15>* self){return self->matrix();})
        .def("print",[](gtsam::ParameterMatrix<15>* self, const string& s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::ParameterMatrix<15>& self, const string& s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "");

    py::class_<gtsam::EvaluationFactor<gtsam::Chebyshev2>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::EvaluationFactor<gtsam::Chebyshev2>>>(m_, "EvaluationFactorChebyshev2")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::EvaluationFactor<gtsam::Chebyshev1Basis>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::EvaluationFactor<gtsam::Chebyshev1Basis>>>(m_, "EvaluationFactorChebyshev1Basis")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::EvaluationFactor<gtsam::Chebyshev2Basis>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::EvaluationFactor<gtsam::Chebyshev2Basis>>>(m_, "EvaluationFactorChebyshev2Basis")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::EvaluationFactor<gtsam::FourierBasis>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::EvaluationFactor<gtsam::FourierBasis>>>(m_, "EvaluationFactorFourierBasis")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::FitBasis<gtsam::FourierBasis>, boost::shared_ptr<gtsam::FitBasis<gtsam::FourierBasis>>>(m_, "FitBasisFourierBasis")
        .def(py::init<const std::map<double, double>&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def("parameters",[](gtsam::FitBasis<gtsam::FourierBasis>* self){return self->parameters();})
        .def_static("NonlinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::FourierBasis>::NonlinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def_static("LinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::FourierBasis>::LinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"));

    py::class_<gtsam::FitBasis<gtsam::Chebyshev1Basis>, boost::shared_ptr<gtsam::FitBasis<gtsam::Chebyshev1Basis>>>(m_, "FitBasisChebyshev1Basis")
        .def(py::init<const std::map<double, double>&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def("parameters",[](gtsam::FitBasis<gtsam::Chebyshev1Basis>* self){return self->parameters();})
        .def_static("NonlinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::Chebyshev1Basis>::NonlinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def_static("LinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::Chebyshev1Basis>::LinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"));

    py::class_<gtsam::FitBasis<gtsam::Chebyshev2Basis>, boost::shared_ptr<gtsam::FitBasis<gtsam::Chebyshev2Basis>>>(m_, "FitBasisChebyshev2Basis")
        .def(py::init<const std::map<double, double>&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def("parameters",[](gtsam::FitBasis<gtsam::Chebyshev2Basis>* self){return self->parameters();})
        .def_static("NonlinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::Chebyshev2Basis>::NonlinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def_static("LinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::Chebyshev2Basis>::LinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"));

    py::class_<gtsam::FitBasis<gtsam::Chebyshev2>, boost::shared_ptr<gtsam::FitBasis<gtsam::Chebyshev2>>>(m_, "FitBasisChebyshev2")
        .def(py::init<const std::map<double, double>&, const boost::shared_ptr<gtsam::noiseModel::Base>, size_t>(), py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def("parameters",[](gtsam::FitBasis<gtsam::Chebyshev2>* self){return self->parameters();})
        .def_static("NonlinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::Chebyshev2>::NonlinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"))
        .def_static("LinearGraph",[](const std::map<double, double>& sequence, const boost::shared_ptr<gtsam::noiseModel::Base> model, size_t N){return gtsam::FitBasis<gtsam::Chebyshev2>::LinearGraph(sequence, model, N);}, py::arg("sequence"), py::arg("model"), py::arg("N"));

    py::class_<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 3>>>(m_, "VectorEvaluationFactorChebyshev2D3")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 4>>>(m_, "VectorEvaluationFactorChebyshev2D4")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 12>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 12>>>(m_, "VectorEvaluationFactorChebyshev2D12")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"))
        .def(py::init<gtsam::Key, const gtsam::Vector&, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 3>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 3>>>(m_, "VectorComponentFactorChebyshev2D3")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("i"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("i"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 4>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 4>>>(m_, "VectorComponentFactorChebyshev2D4")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("i"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("i"), py::arg("x"), py::arg("a"), py::arg("b"));

    py::class_<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 12>, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::VectorComponentFactor<gtsam::Chebyshev2, 12>>>(m_, "VectorComponentFactorChebyshev2D12")
        .def(py::init<>())
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("i"), py::arg("x"))
        .def(py::init<gtsam::Key, const double, const boost::shared_ptr<gtsam::noiseModel::Base>, const size_t, size_t, double, double, double>(), py::arg("key"), py::arg("z"), py::arg("model"), py::arg("N"), py::arg("i"), py::arg("x"), py::arg("a"), py::arg("b"));


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/basis.h"

}

