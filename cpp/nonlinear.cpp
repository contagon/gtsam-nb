/**
 * @file    nonlinear.cpp
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
#include "gtsam/basis/ParameterMatrix.h"
#include "gtsam/geometry/Cal3Bundler.h"
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam/geometry/Cal3Fisheye.h"
#include "gtsam/geometry/Cal3Unified.h"
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/EssentialMatrix.h"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/SO3.h"
#include "gtsam/geometry/SO4.h"
#include "gtsam/geometry/SOn.h"
#include "gtsam/geometry/StereoPoint2.h"
#include "gtsam/geometry/Unit3.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/NavState.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/GncOptimizer.h"
#include "gtsam/nonlinear/GncParams.h"
#include "gtsam/nonlinear/GraphvizFormatting.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LinearContainerFactor.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearEquality.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/NonlinearISAM.h"
#include "gtsam/nonlinear/NonlinearOptimizer.h"
#include "gtsam/nonlinear/NonlinearOptimizerParams.h"
#include "gtsam/nonlinear/PriorFactor.h"
#include "gtsam/nonlinear/Values.h"

using namespace std;

namespace nb = nanobind;

void nonlinear(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of nonlinear";

  nb::class_<gtsam::GraphvizFormatting, gtsam::DotWriter> graphvizformatting(m_, "GraphvizFormatting");
  graphvizformatting
      .def(nb::init<>())
      .def_rw("paperHorizontalAxis", &gtsam::GraphvizFormatting::paperHorizontalAxis)
      .def_rw("paperVerticalAxis", &gtsam::GraphvizFormatting::paperVerticalAxis)
      .def_rw("scale", &gtsam::GraphvizFormatting::scale)
      .def_rw("mergeSimilarFactors", &gtsam::GraphvizFormatting::mergeSimilarFactors);

  nb::enum_<gtsam::GraphvizFormatting::Axis>(graphvizformatting, "Axis", nb::is_arithmetic())
      .value("X", gtsam::GraphvizFormatting::Axis::X)
      .value("Y", gtsam::GraphvizFormatting::Axis::Y)
      .value("Z", gtsam::GraphvizFormatting::Axis::Z)
      .value("NEGX", gtsam::GraphvizFormatting::Axis::NEGX)
      .value("NEGY", gtsam::GraphvizFormatting::Axis::NEGY)
      .value("NEGZ", gtsam::GraphvizFormatting::Axis::NEGZ);

  nb::class_<gtsam::NonlinearFactorGraph>(m_, "NonlinearFactorGraph")
      .def(nb::init<>())
      .def(nb::init<const gtsam::NonlinearFactorGraph &>(), nb::arg("graph"))
      .def("print", [](gtsam::NonlinearFactorGraph *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "NonlinearFactorGraph: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::NonlinearFactorGraph &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "NonlinearFactorGraph: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::NonlinearFactorGraph *self, const gtsam::NonlinearFactorGraph &fg, double tol) { return self->equals(fg, tol); }, nb::arg("fg"), nb::arg("tol"))
      .def("size", [](gtsam::NonlinearFactorGraph *self) { return self->size(); })
      .def("empty", [](gtsam::NonlinearFactorGraph *self) { return self->empty(); })
      .def("remove", [](gtsam::NonlinearFactorGraph *self, size_t i) { self->remove(i); }, nb::arg("i"))
      .def("replace", [](gtsam::NonlinearFactorGraph *self, size_t i, boost::shared_ptr<gtsam::NonlinearFactor> factors) { self->replace(i, factors); }, nb::arg("i"), nb::arg("factors"))
      .def("resize", [](gtsam::NonlinearFactorGraph *self, size_t size) { self->resize(size); }, nb::arg("size"))
      .def("nrFactors", [](gtsam::NonlinearFactorGraph *self) { return self->nrFactors(); })
      .def("at", [](gtsam::NonlinearFactorGraph *self, size_t idx) { return self->at(idx); }, nb::arg("idx"))
      .def("push_back", [](gtsam::NonlinearFactorGraph *self, const gtsam::NonlinearFactorGraph &factors) { self->push_back(factors); }, nb::arg("factors"))
      .def("push_back", [](gtsam::NonlinearFactorGraph *self, boost::shared_ptr<gtsam::NonlinearFactor> factor) { self->push_back(factor); }, nb::arg("factor"))
      .def("add", [](gtsam::NonlinearFactorGraph *self, boost::shared_ptr<gtsam::NonlinearFactor> factor) { self->add(factor); }, nb::arg("factor"))
      .def("exists", [](gtsam::NonlinearFactorGraph *self, size_t idx) { return self->exists(idx); }, nb::arg("idx"))
      .def("keys", [](gtsam::NonlinearFactorGraph *self) { return self->keys(); })
      .def("keyVector", [](gtsam::NonlinearFactorGraph *self) { return self->keyVector(); })
      .def("addPriorDouble", [](gtsam::NonlinearFactorGraph *self, size_t key, const double &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<double>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorVector", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Vector &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Vector>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPoint2", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Point2 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Point2>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorStereoPoint2", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::StereoPoint2 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::StereoPoint2>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPoint3", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Point3 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Point3>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorRot2", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Rot2 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Rot2>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorSO3", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::SO3 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::SO3>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorSO4", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::SO4 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::SO4>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorRot3", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Rot3 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Rot3>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPose2", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Pose2 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Pose2>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPose3", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Pose3 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Pose3>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorCal3_S2", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Cal3_S2 &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Cal3_S2>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorCal3Fisheye", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Cal3Fisheye &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Cal3Fisheye>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorCal3Unified", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::Cal3Unified &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::Cal3Unified>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorCalibratedCamera", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::CalibratedCamera &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::CalibratedCamera>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPinholeCameraCal3_S2", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3_S2> &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::PinholeCamera<gtsam::Cal3_S2>>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPinholeCameraCal3Bundler", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPinholeCameraCal3Fisheye", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorPinholeCameraCal3Unified", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Unified> &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Unified>>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("addPriorConstantBias", [](gtsam::NonlinearFactorGraph *self, size_t key, const gtsam::imuBias::ConstantBias &prior, const boost::shared_ptr<gtsam::noiseModel::Base> noiseModel) { self->addPrior<gtsam::imuBias::ConstantBias>(key, prior, noiseModel); }, nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("printErrors", [](gtsam::NonlinearFactorGraph *self, const gtsam::Values &values) { self->printErrors(values); }, nb::arg("values"))
      .def("error", [](gtsam::NonlinearFactorGraph *self, const gtsam::Values &values) { return self->error(values); }, nb::arg("values"))
      .def("probPrime", [](gtsam::NonlinearFactorGraph *self, const gtsam::Values &values) { return self->probPrime(values); }, nb::arg("values"))
      .def("orderingCOLAMD", [](gtsam::NonlinearFactorGraph *self) { return self->orderingCOLAMD(); })
      .def("linearize", [](gtsam::NonlinearFactorGraph *self, const gtsam::Values &values) { return self->linearize(values); }, nb::arg("values"))
      .def("clone", [](gtsam::NonlinearFactorGraph *self) { return self->clone(); })
      .def("dot", [](gtsam::NonlinearFactorGraph *self, const gtsam::Values &values, const gtsam::KeyFormatter &keyFormatter, const gtsam::GraphvizFormatting &writer) { return self->dot(values, keyFormatter, writer); }, nb::arg("values"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::GraphvizFormatting())
      .def("saveGraph", [](gtsam::NonlinearFactorGraph *self, const string &s, const gtsam::Values &values, const gtsam::KeyFormatter &keyFormatter, const gtsam::GraphvizFormatting &writer) { self->saveGraph(s, values, keyFormatter, writer); }, nb::arg("s"), nb::arg("values"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter, nb::arg("writer") = gtsam::GraphvizFormatting());
  // .def("serialize", [](gtsam::NonlinearFactorGraph *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearFactorGraph *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearFactorGraph &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearFactorGraph obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearFactor, gtsam::Factor>(m_, "NonlinearFactor")
      .def("print", [](gtsam::NonlinearFactor *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::NonlinearFactor &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::NonlinearFactor *self, const gtsam::NonlinearFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("error", [](gtsam::NonlinearFactor *self, const gtsam::Values &c) { return self->error(c); }, nb::arg("c"))
      .def("dim", [](gtsam::NonlinearFactor *self) { return self->dim(); })
      .def("active", [](gtsam::NonlinearFactor *self, const gtsam::Values &c) { return self->active(c); }, nb::arg("c"))
      .def("linearize", [](gtsam::NonlinearFactor *self, const gtsam::Values &c) { return self->linearize(c); }, nb::arg("c"))
      .def("clone", [](gtsam::NonlinearFactor *self) { return self->clone(); })
      .def("rekey", [](gtsam::NonlinearFactor *self, const gtsam::KeyVector &newKeys) { return self->rekey(newKeys); }, nb::arg("newKeys"));

  nb::class_<gtsam::NoiseModelFactor, gtsam::NonlinearFactor>(m_, "NoiseModelFactor")
      .def("equals", [](gtsam::NoiseModelFactor *self, const gtsam::NoiseModelFactor &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("noiseModel", [](gtsam::NoiseModelFactor *self) { return self->noiseModel(); })
      .def("unwhitenedError", [](gtsam::NoiseModelFactor *self, const gtsam::Values &x) { return self->unwhitenedError(x); }, nb::arg("x"))
      .def("whitenedError", [](gtsam::NoiseModelFactor *self, const gtsam::Values &x) { return self->whitenedError(x); }, nb::arg("x"));

  nb::class_<gtsam::Values>(m_, "Values")
      .def(nb::init<>())
      .def(nb::init<const gtsam::Values &>(), nb::arg("other"))
      .def("size", [](gtsam::Values *self) { return self->size(); })
      .def("empty", [](gtsam::Values *self) { return self->empty(); })
      .def("clear", [](gtsam::Values *self) { self->clear(); })
      .def("dim", [](gtsam::Values *self) { return self->dim(); })
      .def("print", [](gtsam::Values *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Values &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("equals", [](gtsam::Values *self, const gtsam::Values &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("insert", [](gtsam::Values *self, const gtsam::Values &values) { self->insert(values); }, nb::arg("values"))
      .def("update", [](gtsam::Values *self, const gtsam::Values &values) { self->update(values); }, nb::arg("values"))
      .def("insert_or_assign", [](gtsam::Values *self, const gtsam::Values &values) { self->insert_or_assign(values); }, nb::arg("values"))
      .def("erase", [](gtsam::Values *self, size_t j) { self->erase(j); }, nb::arg("j"))
      .def("swap", [](gtsam::Values *self, gtsam::Values &values) { self->swap(values); }, nb::arg("values"))
      .def("exists", [](gtsam::Values *self, size_t j) { return self->exists(j); }, nb::arg("j"))
      .def("keys", [](gtsam::Values *self) { return self->keys(); })
      .def("zeroVectors", [](gtsam::Values *self) { return self->zeroVectors(); })
      .def("retract", [](gtsam::Values *self, const gtsam::VectorValues &delta) { return self->retract(delta); }, nb::arg("delta"))
      .def("localCoordinates", [](gtsam::Values *self, const gtsam::Values &cp) { return self->localCoordinates(cp); }, nb::arg("cp"))
      // .def("serialize", [](gtsam::Values *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::Values *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::Values &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::Values obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("insert_vector", [](gtsam::Values *self, size_t j, const gtsam::Vector &vector) { self->insert(j, vector); }, nb::arg("j"), nb::arg("vector"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Vector &vector) { self->insert(j, vector); }, nb::arg("j"), nb::arg("vector"))
      .def("insert_matrix", [](gtsam::Values *self, size_t j, const gtsam::Matrix &matrix) { self->insert(j, matrix); }, nb::arg("j"), nb::arg("matrix"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Matrix &matrix) { self->insert(j, matrix); }, nb::arg("j"), nb::arg("matrix"))
      .def("insert_point2", [](gtsam::Values *self, size_t j, const gtsam::Point2 &point2) { self->insert(j, point2); }, nb::arg("j"), nb::arg("point2"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Point2 &point2) { self->insert(j, point2); }, nb::arg("j"), nb::arg("point2"))
      .def("insert_point3", [](gtsam::Values *self, size_t j, const gtsam::Point3 &point3) { self->insert(j, point3); }, nb::arg("j"), nb::arg("point3"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Point3 &point3) { self->insert(j, point3); }, nb::arg("j"), nb::arg("point3"))
      .def("insert_rot2", [](gtsam::Values *self, size_t j, const gtsam::Rot2 &rot2) { self->insert(j, rot2); }, nb::arg("j"), nb::arg("rot2"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Rot2 &rot2) { self->insert(j, rot2); }, nb::arg("j"), nb::arg("rot2"))
      .def("insert_pose2", [](gtsam::Values *self, size_t j, const gtsam::Pose2 &pose2) { self->insert(j, pose2); }, nb::arg("j"), nb::arg("pose2"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Pose2 &pose2) { self->insert(j, pose2); }, nb::arg("j"), nb::arg("pose2"))
      .def("insert_R", [](gtsam::Values *self, size_t j, const gtsam::SO3 &R) { self->insert(j, R); }, nb::arg("j"), nb::arg("R"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::SO3 &R) { self->insert(j, R); }, nb::arg("j"), nb::arg("R"))
      .def("insert_Q", [](gtsam::Values *self, size_t j, const gtsam::SO4 &Q) { self->insert(j, Q); }, nb::arg("j"), nb::arg("Q"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::SO4 &Q) { self->insert(j, Q); }, nb::arg("j"), nb::arg("Q"))
      .def("insert_P", [](gtsam::Values *self, size_t j, const gtsam::SOn &P) { self->insert(j, P); }, nb::arg("j"), nb::arg("P"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::SOn &P) { self->insert(j, P); }, nb::arg("j"), nb::arg("P"))
      .def("insert_rot3", [](gtsam::Values *self, size_t j, const gtsam::Rot3 &rot3) { self->insert(j, rot3); }, nb::arg("j"), nb::arg("rot3"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Rot3 &rot3) { self->insert(j, rot3); }, nb::arg("j"), nb::arg("rot3"))
      .def("insert_pose3", [](gtsam::Values *self, size_t j, const gtsam::Pose3 &pose3) { self->insert(j, pose3); }, nb::arg("j"), nb::arg("pose3"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Pose3 &pose3) { self->insert(j, pose3); }, nb::arg("j"), nb::arg("pose3"))
      .def("insert_unit3", [](gtsam::Values *self, size_t j, const gtsam::Unit3 &unit3) { self->insert(j, unit3); }, nb::arg("j"), nb::arg("unit3"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Unit3 &unit3) { self->insert(j, unit3); }, nb::arg("j"), nb::arg("unit3"))
      .def("insert_cal3_s2", [](gtsam::Values *self, size_t j, const gtsam::Cal3_S2 &cal3_s2) { self->insert(j, cal3_s2); }, nb::arg("j"), nb::arg("cal3_s2"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Cal3_S2 &cal3_s2) { self->insert(j, cal3_s2); }, nb::arg("j"), nb::arg("cal3_s2"))
      .def("insert_cal3ds2", [](gtsam::Values *self, size_t j, const gtsam::Cal3DS2 &cal3ds2) { self->insert(j, cal3ds2); }, nb::arg("j"), nb::arg("cal3ds2"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Cal3DS2 &cal3ds2) { self->insert(j, cal3ds2); }, nb::arg("j"), nb::arg("cal3ds2"))
      .def("insert_cal3bundler", [](gtsam::Values *self, size_t j, const gtsam::Cal3Bundler &cal3bundler) { self->insert(j, cal3bundler); }, nb::arg("j"), nb::arg("cal3bundler"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Cal3Bundler &cal3bundler) { self->insert(j, cal3bundler); }, nb::arg("j"), nb::arg("cal3bundler"))
      .def("insert_cal3fisheye", [](gtsam::Values *self, size_t j, const gtsam::Cal3Fisheye &cal3fisheye) { self->insert(j, cal3fisheye); }, nb::arg("j"), nb::arg("cal3fisheye"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Cal3Fisheye &cal3fisheye) { self->insert(j, cal3fisheye); }, nb::arg("j"), nb::arg("cal3fisheye"))
      .def("insert_cal3unified", [](gtsam::Values *self, size_t j, const gtsam::Cal3Unified &cal3unified) { self->insert(j, cal3unified); }, nb::arg("j"), nb::arg("cal3unified"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::Cal3Unified &cal3unified) { self->insert(j, cal3unified); }, nb::arg("j"), nb::arg("cal3unified"))
      .def("insert_essential_matrix", [](gtsam::Values *self, size_t j, const gtsam::EssentialMatrix &essential_matrix) { self->insert(j, essential_matrix); }, nb::arg("j"), nb::arg("essential_matrix"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::EssentialMatrix &essential_matrix) { self->insert(j, essential_matrix); }, nb::arg("j"), nb::arg("essential_matrix"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_camera", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified> &camera) { self->insert(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_constant_bias", [](gtsam::Values *self, size_t j, const gtsam::imuBias::ConstantBias &constant_bias) { self->insert(j, constant_bias); }, nb::arg("j"), nb::arg("constant_bias"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::imuBias::ConstantBias &constant_bias) { self->insert(j, constant_bias); }, nb::arg("j"), nb::arg("constant_bias"))
      .def("insert_nav_state", [](gtsam::Values *self, size_t j, const gtsam::NavState &nav_state) { self->insert(j, nav_state); }, nb::arg("j"), nb::arg("nav_state"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::NavState &nav_state) { self->insert(j, nav_state); }, nb::arg("j"), nb::arg("nav_state"))
      .def("insert_c", [](gtsam::Values *self, size_t j, double c) { self->insert(j, c); }, nb::arg("j"), nb::arg("c"))
      .def("insert", [](gtsam::Values *self, size_t j, double c) { self->insert(j, c); }, nb::arg("j"), nb::arg("c"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<1> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<1> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<2> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<2> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<3> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<3> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<4> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<4> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<5> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<5> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<6> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<6> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<7> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<7> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<8> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<8> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<9> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<9> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<10> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<10> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<11> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<11> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<12> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<12> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<13> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<13> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<14> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<14> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_X", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<15> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<15> &X) { self->insert(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insertPoint2", [](gtsam::Values *self, size_t j, const gtsam::Point2 &val) { self->insert<gtsam::Point2>(j, val); }, nb::arg("j"), nb::arg("val"))
      .def("insertPoint3", [](gtsam::Values *self, size_t j, const gtsam::Point3 &val) { self->insert<gtsam::Point3>(j, val); }, nb::arg("j"), nb::arg("val"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Point2 &point2) { self->update(j, point2); }, nb::arg("j"), nb::arg("point2"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Point3 &point3) { self->update(j, point3); }, nb::arg("j"), nb::arg("point3"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Rot2 &rot2) { self->update(j, rot2); }, nb::arg("j"), nb::arg("rot2"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Pose2 &pose2) { self->update(j, pose2); }, nb::arg("j"), nb::arg("pose2"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::SO3 &R) { self->update(j, R); }, nb::arg("j"), nb::arg("R"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::SO4 &Q) { self->update(j, Q); }, nb::arg("j"), nb::arg("Q"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::SOn &P) { self->update(j, P); }, nb::arg("j"), nb::arg("P"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Rot3 &rot3) { self->update(j, rot3); }, nb::arg("j"), nb::arg("rot3"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Pose3 &pose3) { self->update(j, pose3); }, nb::arg("j"), nb::arg("pose3"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Unit3 &unit3) { self->update(j, unit3); }, nb::arg("j"), nb::arg("unit3"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Cal3_S2 &cal3_s2) { self->update(j, cal3_s2); }, nb::arg("j"), nb::arg("cal3_s2"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Cal3DS2 &cal3ds2) { self->update(j, cal3ds2); }, nb::arg("j"), nb::arg("cal3ds2"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Cal3Bundler &cal3bundler) { self->update(j, cal3bundler); }, nb::arg("j"), nb::arg("cal3bundler"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Cal3Fisheye &cal3fisheye) { self->update(j, cal3fisheye); }, nb::arg("j"), nb::arg("cal3fisheye"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Cal3Unified &cal3unified) { self->update(j, cal3unified); }, nb::arg("j"), nb::arg("cal3unified"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::EssentialMatrix &essential_matrix) { self->update(j, essential_matrix); }, nb::arg("j"), nb::arg("essential_matrix"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified> &camera) { self->update(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::imuBias::ConstantBias &constant_bias) { self->update(j, constant_bias); }, nb::arg("j"), nb::arg("constant_bias"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::NavState &nav_state) { self->update(j, nav_state); }, nb::arg("j"), nb::arg("nav_state"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Vector &vector) { self->update(j, vector); }, nb::arg("j"), nb::arg("vector"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::Matrix &matrix) { self->update(j, matrix); }, nb::arg("j"), nb::arg("matrix"))
      .def("update", [](gtsam::Values *self, size_t j, double c) { self->update(j, c); }, nb::arg("j"), nb::arg("c"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<1> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<2> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<3> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<4> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<5> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<6> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<7> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<8> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<9> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<10> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<11> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<12> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<13> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<14> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("update", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<15> &X) { self->update(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Point2 &point2) { self->insert_or_assign(j, point2); }, nb::arg("j"), nb::arg("point2"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Point3 &point3) { self->insert_or_assign(j, point3); }, nb::arg("j"), nb::arg("point3"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Rot2 &rot2) { self->insert_or_assign(j, rot2); }, nb::arg("j"), nb::arg("rot2"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Pose2 &pose2) { self->insert_or_assign(j, pose2); }, nb::arg("j"), nb::arg("pose2"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::SO3 &R) { self->insert_or_assign(j, R); }, nb::arg("j"), nb::arg("R"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::SO4 &Q) { self->insert_or_assign(j, Q); }, nb::arg("j"), nb::arg("Q"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::SOn &P) { self->insert_or_assign(j, P); }, nb::arg("j"), nb::arg("P"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Rot3 &rot3) { self->insert_or_assign(j, rot3); }, nb::arg("j"), nb::arg("rot3"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Pose3 &pose3) { self->insert_or_assign(j, pose3); }, nb::arg("j"), nb::arg("pose3"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Unit3 &unit3) { self->insert_or_assign(j, unit3); }, nb::arg("j"), nb::arg("unit3"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Cal3_S2 &cal3_s2) { self->insert_or_assign(j, cal3_s2); }, nb::arg("j"), nb::arg("cal3_s2"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Cal3DS2 &cal3ds2) { self->insert_or_assign(j, cal3ds2); }, nb::arg("j"), nb::arg("cal3ds2"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Cal3Bundler &cal3bundler) { self->insert_or_assign(j, cal3bundler); }, nb::arg("j"), nb::arg("cal3bundler"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Cal3Fisheye &cal3fisheye) { self->insert_or_assign(j, cal3fisheye); }, nb::arg("j"), nb::arg("cal3fisheye"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Cal3Unified &cal3unified) { self->insert_or_assign(j, cal3unified); }, nb::arg("j"), nb::arg("cal3unified"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::EssentialMatrix &essential_matrix) { self->insert_or_assign(j, essential_matrix); }, nb::arg("j"), nb::arg("essential_matrix"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3_S2> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::PinholePose<gtsam::Cal3Unified> &camera) { self->insert_or_assign(j, camera); }, nb::arg("j"), nb::arg("camera"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::imuBias::ConstantBias &constant_bias) { self->insert_or_assign(j, constant_bias); }, nb::arg("j"), nb::arg("constant_bias"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::NavState &nav_state) { self->insert_or_assign(j, nav_state); }, nb::arg("j"), nb::arg("nav_state"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Vector &vector) { self->insert_or_assign(j, vector); }, nb::arg("j"), nb::arg("vector"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::Matrix &matrix) { self->insert_or_assign(j, matrix); }, nb::arg("j"), nb::arg("matrix"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, double c) { self->insert_or_assign(j, c); }, nb::arg("j"), nb::arg("c"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<1> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<2> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<3> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<4> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<5> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<6> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<7> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<8> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<9> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<10> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<11> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<12> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<13> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<14> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("insert_or_assign", [](gtsam::Values *self, size_t j, const gtsam::ParameterMatrix<15> &X) { self->insert_or_assign(j, X); }, nb::arg("j"), nb::arg("X"))
      .def("atPoint2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Point2>(j); }, nb::arg("j"))
      .def("atPoint3", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Point3>(j); }, nb::arg("j"))
      .def("atRot2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Rot2>(j); }, nb::arg("j"))
      .def("atPose2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Pose2>(j); }, nb::arg("j"))
      .def("atSO3", [](gtsam::Values *self, size_t j) { return self->at<gtsam::SO3>(j); }, nb::arg("j"))
      .def("atSO4", [](gtsam::Values *self, size_t j) { return self->at<gtsam::SO4>(j); }, nb::arg("j"))
      .def("atSOn", [](gtsam::Values *self, size_t j) { return self->at<gtsam::SOn>(j); }, nb::arg("j"))
      .def("atRot3", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Rot3>(j); }, nb::arg("j"))
      .def("atPose3", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Pose3>(j); }, nb::arg("j"))
      .def("atUnit3", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Unit3>(j); }, nb::arg("j"))
      .def("atCal3_S2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Cal3_S2>(j); }, nb::arg("j"))
      .def("atCal3DS2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Cal3DS2>(j); }, nb::arg("j"))
      .def("atCal3Bundler", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Cal3Bundler>(j); }, nb::arg("j"))
      .def("atCal3Fisheye", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Cal3Fisheye>(j); }, nb::arg("j"))
      .def("atCal3Unified", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Cal3Unified>(j); }, nb::arg("j"))
      .def("atEssentialMatrix", [](gtsam::Values *self, size_t j) { return self->at<gtsam::EssentialMatrix>(j); }, nb::arg("j"))
      .def("atPinholeCameraCal3_S2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholeCamera<gtsam::Cal3_S2>>(j); }, nb::arg("j"))
      .def("atPinholeCameraCal3Bundler", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(j); }, nb::arg("j"))
      .def("atPinholeCameraCal3Fisheye", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(j); }, nb::arg("j"))
      .def("atPinholeCameraCal3Unified", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholeCamera<gtsam::Cal3Unified>>(j); }, nb::arg("j"))
      .def("atPinholePoseCal3_S2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholePose<gtsam::Cal3_S2>>(j); }, nb::arg("j"))
      .def("atPinholePoseCal3Bundler", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholePose<gtsam::Cal3Bundler>>(j); }, nb::arg("j"))
      .def("atPinholePoseCal3Fisheye", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholePose<gtsam::Cal3Fisheye>>(j); }, nb::arg("j"))
      .def("atPinholePoseCal3Unified", [](gtsam::Values *self, size_t j) { return self->at<gtsam::PinholePose<gtsam::Cal3Unified>>(j); }, nb::arg("j"))
      .def("atConstantBias", [](gtsam::Values *self, size_t j) { return self->at<gtsam::imuBias::ConstantBias>(j); }, nb::arg("j"))
      .def("atNavState", [](gtsam::Values *self, size_t j) { return self->at<gtsam::NavState>(j); }, nb::arg("j"))
      .def("atVector", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Vector>(j); }, nb::arg("j"))
      .def("atMatrix", [](gtsam::Values *self, size_t j) { return self->at<gtsam::Matrix>(j); }, nb::arg("j"))
      .def("atDouble", [](gtsam::Values *self, size_t j) { return self->at<double>(j); }, nb::arg("j"))
      .def("atParameterMatrix1", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<1>>(j); }, nb::arg("j"))
      .def("atParameterMatrix2", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<2>>(j); }, nb::arg("j"))
      .def("atParameterMatrix3", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<3>>(j); }, nb::arg("j"))
      .def("atParameterMatrix4", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<4>>(j); }, nb::arg("j"))
      .def("atParameterMatrix5", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<5>>(j); }, nb::arg("j"))
      .def("atParameterMatrix6", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<6>>(j); }, nb::arg("j"))
      .def("atParameterMatrix7", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<7>>(j); }, nb::arg("j"))
      .def("atParameterMatrix8", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<8>>(j); }, nb::arg("j"))
      .def("atParameterMatrix9", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<9>>(j); }, nb::arg("j"))
      .def("atParameterMatrix10", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<10>>(j); }, nb::arg("j"))
      .def("atParameterMatrix11", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<11>>(j); }, nb::arg("j"))
      .def("atParameterMatrix12", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<12>>(j); }, nb::arg("j"))
      .def("atParameterMatrix13", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<13>>(j); }, nb::arg("j"))
      .def("atParameterMatrix14", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<14>>(j); }, nb::arg("j"))
      .def("atParameterMatrix15", [](gtsam::Values *self, size_t j) { return self->at<gtsam::ParameterMatrix<15>>(j); }, nb::arg("j"));

  nb::class_<gtsam::Marginals>(m_, "Marginals")
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &>(), nb::arg("graph"), nb::arg("solution"))
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::Values &>(), nb::arg("gfgraph"), nb::arg("solution"))
      .def(nb::init<const gtsam::GaussianFactorGraph &, const gtsam::VectorValues &>(), nb::arg("gfgraph"), nb::arg("solutionvec"))
      .def("print", [](gtsam::Marginals *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "Marginals: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::Marginals &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "Marginals: ", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("marginalCovariance", [](gtsam::Marginals *self, size_t variable) { return self->marginalCovariance(variable); }, nb::arg("variable"))
      .def("marginalInformation", [](gtsam::Marginals *self, size_t variable) { return self->marginalInformation(variable); }, nb::arg("variable"))
      .def("jointMarginalCovariance", [](gtsam::Marginals *self, const gtsam::KeyVector &variables) { return self->jointMarginalCovariance(variables); }, nb::arg("variables"))
      .def("jointMarginalInformation", [](gtsam::Marginals *self, const gtsam::KeyVector &variables) { return self->jointMarginalInformation(variables); }, nb::arg("variables"));

  nb::class_<gtsam::JointMarginal>(m_, "JointMarginal")
      .def("at", [](gtsam::JointMarginal *self, size_t iVariable, size_t jVariable) { return self->at(iVariable, jVariable); }, nb::arg("iVariable"), nb::arg("jVariable"))
      .def("fullMatrix", [](gtsam::JointMarginal *self) { return self->fullMatrix(); })
      .def("print", [](gtsam::JointMarginal *self, string s, gtsam::KeyFormatter keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::JointMarginal &self, string s, gtsam::KeyFormatter keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  nb::class_<gtsam::LinearContainerFactor, gtsam::NonlinearFactor>(m_, "LinearContainerFactor")
      .def(nb::init<boost::shared_ptr<gtsam::GaussianFactor>, const gtsam::Values &>(), nb::arg("factor"), nb::arg("linearizationPoint"))
      .def(nb::init<boost::shared_ptr<gtsam::GaussianFactor>>(), nb::arg("factor"))
      .def("factor", [](gtsam::LinearContainerFactor *self) { return self->factor(); })
      .def("isJacobian", [](gtsam::LinearContainerFactor *self) { return self->isJacobian(); })
      .def("toJacobian", [](gtsam::LinearContainerFactor *self) { return self->toJacobian(); })
      .def("toHessian", [](gtsam::LinearContainerFactor *self) { return self->toHessian(); })
      // .def("serialize", [](gtsam::LinearContainerFactor *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::LinearContainerFactor *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::LinearContainerFactor &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::LinearContainerFactor obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("ConvertLinearGraph", [](const gtsam::GaussianFactorGraph &linear_graph, const gtsam::Values &linearizationPoint) { return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph, linearizationPoint); }, nb::arg("linear_graph"), nb::arg("linearizationPoint"))
      .def_static("ConvertLinearGraph", [](const gtsam::GaussianFactorGraph &linear_graph) { return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph); }, nb::arg("linear_graph"));

  nb::class_<gtsam::NonlinearOptimizerParams>(m_, "NonlinearOptimizerParams")
      .def(nb::init<>())
      .def("print", [](gtsam::NonlinearOptimizerParams *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::NonlinearOptimizerParams &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("getMaxIterations", [](gtsam::NonlinearOptimizerParams *self) { return self->getMaxIterations(); })
      .def("getRelativeErrorTol", [](gtsam::NonlinearOptimizerParams *self) { return self->getRelativeErrorTol(); })
      .def("getAbsoluteErrorTol", [](gtsam::NonlinearOptimizerParams *self) { return self->getAbsoluteErrorTol(); })
      .def("getErrorTol", [](gtsam::NonlinearOptimizerParams *self) { return self->getErrorTol(); })
      .def("getVerbosity", [](gtsam::NonlinearOptimizerParams *self) { return self->getVerbosity(); })
      .def("setMaxIterations", [](gtsam::NonlinearOptimizerParams *self, int value) { self->setMaxIterations(value); }, nb::arg("value"))
      .def("setRelativeErrorTol", [](gtsam::NonlinearOptimizerParams *self, double value) { self->setRelativeErrorTol(value); }, nb::arg("value"))
      .def("setAbsoluteErrorTol", [](gtsam::NonlinearOptimizerParams *self, double value) { self->setAbsoluteErrorTol(value); }, nb::arg("value"))
      .def("setErrorTol", [](gtsam::NonlinearOptimizerParams *self, double value) { self->setErrorTol(value); }, nb::arg("value"))
      .def("setVerbosity", [](gtsam::NonlinearOptimizerParams *self, string s) { self->setVerbosity(s); }, nb::arg("s"))
      .def("getLinearSolverType", [](gtsam::NonlinearOptimizerParams *self) { return self->getLinearSolverType(); })
      .def("setLinearSolverType", [](gtsam::NonlinearOptimizerParams *self, string solver) { self->setLinearSolverType(solver); }, nb::arg("solver"))
      .def("setIterativeParams", [](gtsam::NonlinearOptimizerParams *self, boost::shared_ptr<gtsam::IterativeOptimizationParameters> params) { self->setIterativeParams(params); }, nb::arg("params"))
      .def("setOrdering", [](gtsam::NonlinearOptimizerParams *self, const gtsam::Ordering &ordering) { self->setOrdering(ordering); }, nb::arg("ordering"))
      .def("getOrderingType", [](gtsam::NonlinearOptimizerParams *self) { return self->getOrderingType(); })
      .def("setOrderingType", [](gtsam::NonlinearOptimizerParams *self, string ordering) { self->setOrderingType(ordering); }, nb::arg("ordering"))
      .def("isMultifrontal", [](gtsam::NonlinearOptimizerParams *self) { return self->isMultifrontal(); })
      .def("isSequential", [](gtsam::NonlinearOptimizerParams *self) { return self->isSequential(); })
      .def("isCholmod", [](gtsam::NonlinearOptimizerParams *self) { return self->isCholmod(); })
      .def("isIterative", [](gtsam::NonlinearOptimizerParams *self) { return self->isIterative(); })
      .def_rw("iterationHook", &gtsam::NonlinearOptimizerParams::iterationHook);

  nb::class_<gtsam::GaussNewtonParams, gtsam::NonlinearOptimizerParams>(m_, "GaussNewtonParams")
      .def(nb::init<>());

  nb::class_<gtsam::LevenbergMarquardtParams, gtsam::NonlinearOptimizerParams>(m_, "LevenbergMarquardtParams")
      .def(nb::init<>())
      .def("getDiagonalDamping", [](gtsam::LevenbergMarquardtParams *self) { return self->getDiagonalDamping(); })
      .def("getlambdaFactor", [](gtsam::LevenbergMarquardtParams *self) { return self->getlambdaFactor(); })
      .def("getlambdaInitial", [](gtsam::LevenbergMarquardtParams *self) { return self->getlambdaInitial(); })
      .def("getlambdaLowerBound", [](gtsam::LevenbergMarquardtParams *self) { return self->getlambdaLowerBound(); })
      .def("getlambdaUpperBound", [](gtsam::LevenbergMarquardtParams *self) { return self->getlambdaUpperBound(); })
      .def("getUseFixedLambdaFactor", [](gtsam::LevenbergMarquardtParams *self) { return self->getUseFixedLambdaFactor(); })
      .def("getLogFile", [](gtsam::LevenbergMarquardtParams *self) { return self->getLogFile(); })
      .def("getVerbosityLM", [](gtsam::LevenbergMarquardtParams *self) { return self->getVerbosityLM(); })
      .def("setDiagonalDamping", [](gtsam::LevenbergMarquardtParams *self, bool flag) { self->setDiagonalDamping(flag); }, nb::arg("flag"))
      .def("setlambdaFactor", [](gtsam::LevenbergMarquardtParams *self, double value) { self->setlambdaFactor(value); }, nb::arg("value"))
      .def("setlambdaInitial", [](gtsam::LevenbergMarquardtParams *self, double value) { self->setlambdaInitial(value); }, nb::arg("value"))
      .def("setlambdaLowerBound", [](gtsam::LevenbergMarquardtParams *self, double value) { self->setlambdaLowerBound(value); }, nb::arg("value"))
      .def("setlambdaUpperBound", [](gtsam::LevenbergMarquardtParams *self, double value) { self->setlambdaUpperBound(value); }, nb::arg("value"))
      .def("setUseFixedLambdaFactor", [](gtsam::LevenbergMarquardtParams *self, bool flag) { self->setUseFixedLambdaFactor(flag); }, nb::arg("flag"))
      .def("setLogFile", [](gtsam::LevenbergMarquardtParams *self, string s) { self->setLogFile(s); }, nb::arg("s"))
      .def("setVerbosityLM", [](gtsam::LevenbergMarquardtParams *self, string s) { self->setVerbosityLM(s); }, nb::arg("s"))
      .def_static("LegacyDefaults", []() { return gtsam::LevenbergMarquardtParams::LegacyDefaults(); })
      .def_static("CeresDefaults", []() { return gtsam::LevenbergMarquardtParams::CeresDefaults(); })
      .def_static("EnsureHasOrdering", [](gtsam::LevenbergMarquardtParams params, const gtsam::NonlinearFactorGraph &graph) { return gtsam::LevenbergMarquardtParams::EnsureHasOrdering(params, graph); }, nb::arg("params"), nb::arg("graph"))
      .def_static("ReplaceOrdering", [](gtsam::LevenbergMarquardtParams params, const gtsam::Ordering &ordering) { return gtsam::LevenbergMarquardtParams::ReplaceOrdering(params, ordering); }, nb::arg("params"), nb::arg("ordering"));

  nb::class_<gtsam::DoglegParams, gtsam::NonlinearOptimizerParams>(m_, "DoglegParams")
      .def(nb::init<>())
      .def("getDeltaInitial", [](gtsam::DoglegParams *self) { return self->getDeltaInitial(); })
      .def("getVerbosityDL", [](gtsam::DoglegParams *self) { return self->getVerbosityDL(); })
      .def("setDeltaInitial", [](gtsam::DoglegParams *self, double deltaInitial) { self->setDeltaInitial(deltaInitial); }, nb::arg("deltaInitial"))
      .def("setVerbosityDL", [](gtsam::DoglegParams *self, string verbosityDL) { self->setVerbosityDL(verbosityDL); }, nb::arg("verbosityDL"));
  nb::enum_<gtsam::GncLossType>(m_, "GncLossType", nb::is_arithmetic())
      .value("GM", gtsam::GncLossType::GM)
      .value("TLS", gtsam::GncLossType::TLS);

  nb::class_<gtsam::NonlinearOptimizer>(m_, "NonlinearOptimizer")
      .def("optimize", [](gtsam::NonlinearOptimizer *self) { return self->optimize(); })
      .def("optimizeSafely", [](gtsam::NonlinearOptimizer *self) { return self->optimizeSafely(); })
      .def("error", [](gtsam::NonlinearOptimizer *self) { return self->error(); })
      .def("iterations", [](gtsam::NonlinearOptimizer *self) { return self->iterations(); })
      .def("values", [](gtsam::NonlinearOptimizer *self) { return self->values(); })
      .def("graph", [](gtsam::NonlinearOptimizer *self) { return self->graph(); })
      .def("iterate", [](gtsam::NonlinearOptimizer *self) { return self->iterate(); });

  nb::class_<gtsam::GaussNewtonOptimizer, gtsam::NonlinearOptimizer>(m_, "GaussNewtonOptimizer")
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &>(), nb::arg("graph"), nb::arg("initialValues"))
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &, const gtsam::GaussNewtonParams &>(), nb::arg("graph"), nb::arg("initialValues"), nb::arg("params"));

  nb::class_<gtsam::DoglegOptimizer, gtsam::NonlinearOptimizer>(m_, "DoglegOptimizer")
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &>(), nb::arg("graph"), nb::arg("initialValues"))
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &, const gtsam::DoglegParams &>(), nb::arg("graph"), nb::arg("initialValues"), nb::arg("params"))
      .def("getDelta", [](gtsam::DoglegOptimizer *self) { return self->getDelta(); });

  nb::class_<gtsam::LevenbergMarquardtOptimizer, gtsam::NonlinearOptimizer>(m_, "LevenbergMarquardtOptimizer")
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &>(), nb::arg("graph"), nb::arg("initialValues"))
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &, const gtsam::LevenbergMarquardtParams &>(), nb::arg("graph"), nb::arg("initialValues"), nb::arg("params"))
      .def("lambda_", [](gtsam::LevenbergMarquardtOptimizer *self) { return self->lambda(); })
      .def("print", [](gtsam::LevenbergMarquardtOptimizer *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::LevenbergMarquardtOptimizer &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "");

  nb::class_<gtsam::ISAM2GaussNewtonParams>(m_, "ISAM2GaussNewtonParams")
      .def(nb::init<>())
      .def("print", [](gtsam::ISAM2GaussNewtonParams *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ISAM2GaussNewtonParams &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("getWildfireThreshold", [](gtsam::ISAM2GaussNewtonParams *self) { return self->getWildfireThreshold(); })
      .def("setWildfireThreshold", [](gtsam::ISAM2GaussNewtonParams *self, double wildfireThreshold) { self->setWildfireThreshold(wildfireThreshold); }, nb::arg("wildfireThreshold"));

  nb::class_<gtsam::ISAM2DoglegParams>(m_, "ISAM2DoglegParams")
      .def(nb::init<>())
      .def("print", [](gtsam::ISAM2DoglegParams *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ISAM2DoglegParams &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("getWildfireThreshold", [](gtsam::ISAM2DoglegParams *self) { return self->getWildfireThreshold(); })
      .def("setWildfireThreshold", [](gtsam::ISAM2DoglegParams *self, double wildfireThreshold) { self->setWildfireThreshold(wildfireThreshold); }, nb::arg("wildfireThreshold"))
      .def("getInitialDelta", [](gtsam::ISAM2DoglegParams *self) { return self->getInitialDelta(); })
      .def("setInitialDelta", [](gtsam::ISAM2DoglegParams *self, double initialDelta) { self->setInitialDelta(initialDelta); }, nb::arg("initialDelta"))
      .def("getAdaptationMode", [](gtsam::ISAM2DoglegParams *self) { return self->getAdaptationMode(); })
      .def("setAdaptationMode", [](gtsam::ISAM2DoglegParams *self, string adaptationMode) { self->setAdaptationMode(adaptationMode); }, nb::arg("adaptationMode"))
      .def("isVerbose", [](gtsam::ISAM2DoglegParams *self) { return self->isVerbose(); })
      .def("setVerbose", [](gtsam::ISAM2DoglegParams *self, bool verbose) { self->setVerbose(verbose); }, nb::arg("verbose"));

  nb::class_<gtsam::ISAM2ThresholdMap>(m_, "ISAM2ThresholdMap")
      .def(nb::init<>())
      .def(nb::init<const gtsam::ISAM2ThresholdMap &>(), nb::arg("other"))
      .def("size", [](gtsam::ISAM2ThresholdMap *self) { return self->size(); })
      .def("empty", [](gtsam::ISAM2ThresholdMap *self) { return self->empty(); })
      .def("clear", [](gtsam::ISAM2ThresholdMap *self) { self->clear(); })
      .def("insert", [](gtsam::ISAM2ThresholdMap *self, const gtsam::ISAM2ThresholdMapValue &value) { self->insert(value); }, nb::arg("value"));

  nb::class_<gtsam::ISAM2Params> isam2params(m_, "ISAM2Params");
  isam2params
      .def(nb::init<>())
      .def("print", [](gtsam::ISAM2Params *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ISAM2Params &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("setOptimizationParams", [](gtsam::ISAM2Params *self, const gtsam::ISAM2GaussNewtonParams &gauss_newton__params) { self->setOptimizationParams(gauss_newton__params); }, nb::arg("gauss_newton__params"))
      .def("setOptimizationParams", [](gtsam::ISAM2Params *self, const gtsam::ISAM2DoglegParams &dogleg_params) { self->setOptimizationParams(dogleg_params); }, nb::arg("dogleg_params"))
      .def("setRelinearizeThreshold", [](gtsam::ISAM2Params *self, double threshold) { self->setRelinearizeThreshold(threshold); }, nb::arg("threshold"))
      .def("setRelinearizeThreshold", [](gtsam::ISAM2Params *self, const gtsam::ISAM2ThresholdMap &threshold_map) { self->setRelinearizeThreshold(threshold_map); }, nb::arg("threshold_map"))
      .def("getFactorization", [](gtsam::ISAM2Params *self) { return self->getFactorization(); })
      .def("setFactorization", [](gtsam::ISAM2Params *self, string factorization) { self->setFactorization(factorization); }, nb::arg("factorization"))
      .def_rw("relinearizeSkip", &gtsam::ISAM2Params::relinearizeSkip)
      .def_rw("enableRelinearization", &gtsam::ISAM2Params::enableRelinearization)
      .def_rw("evaluateNonlinearError", &gtsam::ISAM2Params::evaluateNonlinearError)
      .def_rw("cacheLinearizedFactors", &gtsam::ISAM2Params::cacheLinearizedFactors)
      .def_rw("enableDetailedResults", &gtsam::ISAM2Params::enableDetailedResults)
      .def_rw("enablePartialRelinearizationCheck", &gtsam::ISAM2Params::enablePartialRelinearizationCheck)
      .def_rw("findUnusedFactorSlots", &gtsam::ISAM2Params::findUnusedFactorSlots)
      .def_rw("factorization", &gtsam::ISAM2Params::factorization);

  nb::enum_<gtsam::ISAM2Params::Factorization>(isam2params, "Factorization", nb::is_arithmetic())
      .value("CHOLESKY", gtsam::ISAM2Params::Factorization::CHOLESKY)
      .value("QR", gtsam::ISAM2Params::Factorization::QR);

  nb::class_<gtsam::ISAM2Clique>(m_, "ISAM2Clique")
      .def(nb::init<>())
      .def("gradientContribution", [](gtsam::ISAM2Clique *self) { return self->gradientContribution(); })
      .def("print", [](gtsam::ISAM2Clique *self, string s, gtsam::KeyFormatter keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::ISAM2Clique &self, string s, gtsam::KeyFormatter keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  nb::class_<gtsam::ISAM2Result>(m_, "ISAM2Result")
      .def(nb::init<>())
      .def("print", [](gtsam::ISAM2Result *self, string s) { /* nb::scoped_ostream_redirect output; */ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::ISAM2Result &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("getVariablesRelinearized", [](gtsam::ISAM2Result *self) { return self->getVariablesRelinearized(); })
      .def("getVariablesReeliminated", [](gtsam::ISAM2Result *self) { return self->getVariablesReeliminated(); })
      .def("getNewFactorsIndices", [](gtsam::ISAM2Result *self) { return self->getNewFactorsIndices(); })
      .def("getCliques", [](gtsam::ISAM2Result *self) { return self->getCliques(); })
      .def("getErrorBefore", [](gtsam::ISAM2Result *self) { return self->getErrorBefore(); })
      .def("getErrorAfter", [](gtsam::ISAM2Result *self) { return self->getErrorAfter(); });

  nb::class_<gtsam::ISAM2>(m_, "ISAM2")
      .def(nb::init<>())
      .def(nb::init<const gtsam::ISAM2Params &>(), nb::arg("params"))
      .def(nb::init<const gtsam::ISAM2 &>(), nb::arg("other"))
      .def("equals", [](gtsam::ISAM2 *self, const gtsam::ISAM2 &other, double tol) { return self->equals(other, tol); }, nb::arg("other"), nb::arg("tol"))
      .def("print", [](gtsam::ISAM2 *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::ISAM2 &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("saveGraph", [](gtsam::ISAM2 *self, string s) { self->saveGraph(s); }, nb::arg("s"))
      .def("update", [](gtsam::ISAM2 *self) { return self->update(); })
      .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta) { return self->update(newFactors, newTheta); }, nb::arg("newFactors"), nb::arg("newTheta"))
      .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta, const gtsam::FactorIndices &removeFactorIndices) { return self->update(newFactors, newTheta, removeFactorIndices); }, nb::arg("newFactors"), nb::arg("newTheta"), nb::arg("removeFactorIndices"))
      .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta, const gtsam::FactorIndices &removeFactorIndices, const gtsam::KeyGroupMap &constrainedKeys) { return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys); }, nb::arg("newFactors"), nb::arg("newTheta"), nb::arg("removeFactorIndices"), nb::arg("constrainedKeys"))
      .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta, const gtsam::FactorIndices &removeFactorIndices, gtsam::KeyGroupMap &constrainedKeys, const gtsam::KeyList &noRelinKeys) { return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys, noRelinKeys); }, nb::arg("newFactors"), nb::arg("newTheta"), nb::arg("removeFactorIndices"), nb::arg("constrainedKeys"), nb::arg("noRelinKeys"))
      .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta, const gtsam::FactorIndices &removeFactorIndices, gtsam::KeyGroupMap &constrainedKeys, const gtsam::KeyList &noRelinKeys, const gtsam::KeyList &extraReelimKeys) { return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys, noRelinKeys, extraReelimKeys); }, nb::arg("newFactors"), nb::arg("newTheta"), nb::arg("removeFactorIndices"), nb::arg("constrainedKeys"), nb::arg("noRelinKeys"), nb::arg("extraReelimKeys"))
      .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta, const gtsam::FactorIndices &removeFactorIndices, gtsam::KeyGroupMap &constrainedKeys, const gtsam::KeyList &noRelinKeys, const gtsam::KeyList &extraReelimKeys, bool force_relinearize) { return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys, noRelinKeys, extraReelimKeys, force_relinearize); }, nb::arg("newFactors"), nb::arg("newTheta"), nb::arg("removeFactorIndices"), nb::arg("constrainedKeys"), nb::arg("noRelinKeys"), nb::arg("extraReelimKeys"), nb::arg("force_relinearize"))
      // TODO: ISAM2UpdateParams isn't bound, need to add in
      // .def("update", [](gtsam::ISAM2 *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &newTheta, const gtsam::ISAM2UpdateParams &updateParams) { return self->update(newFactors, newTheta, updateParams); }, nb::arg("newFactors"), nb::arg("newTheta"), nb::arg("updateParams"))
      .def("getLinearizationPoint", [](gtsam::ISAM2 *self) { return self->getLinearizationPoint(); })
      .def("valueExists", [](gtsam::ISAM2 *self, gtsam::Key key) { return self->valueExists(key); }, nb::arg("key"))
      .def("calculateEstimate", [](gtsam::ISAM2 *self) { return self->calculateEstimate(); })
      .def("calculateEstimatePoint2", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Point2>(key); }, nb::arg("key"))
      .def("calculateEstimateRot2", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Rot2>(key); }, nb::arg("key"))
      .def("calculateEstimatePose2", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Pose2>(key); }, nb::arg("key"))
      .def("calculateEstimatePoint3", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Point3>(key); }, nb::arg("key"))
      .def("calculateEstimateRot3", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Rot3>(key); }, nb::arg("key"))
      .def("calculateEstimatePose3", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Pose3>(key); }, nb::arg("key"))
      .def("calculateEstimateCal3_S2", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Cal3_S2>(key); }, nb::arg("key"))
      .def("calculateEstimateCal3DS2", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Cal3DS2>(key); }, nb::arg("key"))
      .def("calculateEstimateCal3Bundler", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Cal3Bundler>(key); }, nb::arg("key"))
      .def("calculateEstimateEssentialMatrix", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::EssentialMatrix>(key); }, nb::arg("key"))
      .def("calculateEstimatePinholeCameraCal3_S2", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3_S2>>(key); }, nb::arg("key"))
      .def("calculateEstimatePinholeCameraCal3Bundler", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key); }, nb::arg("key"))
      .def("calculateEstimatePinholeCameraCal3Fisheye", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(key); }, nb::arg("key"))
      .def("calculateEstimatePinholeCameraCal3Unified", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::PinholeCamera<gtsam::Cal3Unified>>(key); }, nb::arg("key"))
      .def("calculateEstimateVector", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Vector>(key); }, nb::arg("key"))
      .def("calculateEstimateMatrix", [](gtsam::ISAM2 *self, size_t key) { return self->calculateEstimate<gtsam::Matrix>(key); }, nb::arg("key"))
      .def("marginalCovariance", [](gtsam::ISAM2 *self, size_t key) { return self->marginalCovariance(key); }, nb::arg("key"))
      .def("calculateBestEstimate", [](gtsam::ISAM2 *self) { return self->calculateBestEstimate(); })
      .def("getDelta", [](gtsam::ISAM2 *self) { return self->getDelta(); })
      .def("error", [](gtsam::ISAM2 *self, const gtsam::VectorValues &x) { return self->error(x); }, nb::arg("x"))
      .def("getFactorsUnsafe", [](gtsam::ISAM2 *self) { return self->getFactorsUnsafe(); })
      .def("getVariableIndex", [](gtsam::ISAM2 *self) { return self->getVariableIndex(); })
      .def("getFixedVariables", [](gtsam::ISAM2 *self) { return self->getFixedVariables(); })
      .def("params", [](gtsam::ISAM2 *self) { return self->params(); })
      .def("printStats", [](gtsam::ISAM2 *self) { self->printStats(); })
      .def("gradientAtZero", [](gtsam::ISAM2 *self) { return self->gradientAtZero(); })
      .def("dot", [](gtsam::ISAM2 *self, const gtsam::KeyFormatter &keyFormatter) { return self->dot(keyFormatter); }, nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("saveGraph", [](gtsam::ISAM2 *self, string s, const gtsam::KeyFormatter &keyFormatter) { self->saveGraph(s, keyFormatter); }, nb::arg("s"), nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

  nb::class_<gtsam::NonlinearISAM>(m_, "NonlinearISAM")
      .def(nb::init<>())
      .def(nb::init<int>(), nb::arg("reorderInterval"))
      .def("print", [](gtsam::NonlinearISAM *self, string s, const gtsam::KeyFormatter &keyFormatter) { /* nb::scoped_ostream_redirect output; */ self->print(s, keyFormatter); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("__repr__", [](const gtsam::NonlinearISAM &self, string s, const gtsam::KeyFormatter &keyFormatter) {
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str(); }, nb::arg("s") = "", nb::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
      .def("printStats", [](gtsam::NonlinearISAM *self) { self->printStats(); })
      .def("saveGraph", [](gtsam::NonlinearISAM *self, string s) { self->saveGraph(s); }, nb::arg("s"))
      .def("estimate", [](gtsam::NonlinearISAM *self) { return self->estimate(); })
      .def("marginalCovariance", [](gtsam::NonlinearISAM *self, size_t key) { return self->marginalCovariance(key); }, nb::arg("key"))
      .def("reorderInterval", [](gtsam::NonlinearISAM *self) { return self->reorderInterval(); })
      .def("reorderCounter", [](gtsam::NonlinearISAM *self) { return self->reorderCounter(); })
      .def("update", [](gtsam::NonlinearISAM *self, const gtsam::NonlinearFactorGraph &newFactors, const gtsam::Values &initialValues) { self->update(newFactors, initialValues); }, nb::arg("newFactors"), nb::arg("initialValues"))
      .def("reorder_relinearize", [](gtsam::NonlinearISAM *self) { self->reorder_relinearize(); })
      .def("bayesTree", [](gtsam::NonlinearISAM *self) { return self->bayesTree(); })
      .def("getLinearizationPoint", [](gtsam::NonlinearISAM *self) { return self->getLinearizationPoint(); })
      .def("getFactorsUnsafe", [](gtsam::NonlinearISAM *self) { return self->getFactorsUnsafe(); });

  nb::class_<gtsam::PriorFactor<double>, gtsam::NoiseModelFactor>(m_, "PriorFactorDouble")
      .def(nb::init<size_t, const double &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<double> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<double> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<double> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<double> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<double> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Vector>, gtsam::NoiseModelFactor>(m_, "PriorFactorVector")
      .def(nb::init<size_t, const gtsam::Vector &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Vector> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Vector> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Vector> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Vector> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Vector> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Point2>, gtsam::NoiseModelFactor>(m_, "PriorFactorPoint2")
      .def(nb::init<size_t, const gtsam::Point2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Point2> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Point2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::StereoPoint2>, gtsam::NoiseModelFactor>(m_, "PriorFactorStereoPoint2")
      .def(nb::init<size_t, const gtsam::StereoPoint2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::StereoPoint2> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::StereoPoint2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::StereoPoint2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::StereoPoint2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::StereoPoint2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "PriorFactorPoint3")
      .def(nb::init<size_t, const gtsam::Point3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Point3> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Point3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Rot2>, gtsam::NoiseModelFactor>(m_, "PriorFactorRot2")
      .def(nb::init<size_t, const gtsam::Rot2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Rot2> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Rot2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Rot2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Rot2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::SO3>, gtsam::NoiseModelFactor>(m_, "PriorFactorSO3")
      .def(nb::init<size_t, const gtsam::SO3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::SO3> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::SO3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::SO3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::SO3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::SO3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::SO4>, gtsam::NoiseModelFactor>(m_, "PriorFactorSO4")
      .def(nb::init<size_t, const gtsam::SO4 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::SO4> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::SO4> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::SO4> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::SO4> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::SO4> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::SOn>, gtsam::NoiseModelFactor>(m_, "PriorFactorSOn")
      .def(nb::init<size_t, const gtsam::SOn &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::SOn> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::SOn> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::SOn> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::SOn> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::SOn> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Rot3>, gtsam::NoiseModelFactor>(m_, "PriorFactorRot3")
      .def(nb::init<size_t, const gtsam::Rot3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Rot3> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Rot3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Rot3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Rot3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Rot3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Pose2>, gtsam::NoiseModelFactor>(m_, "PriorFactorPose2")
      .def(nb::init<size_t, const gtsam::Pose2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Pose2> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Pose2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Pose3>, gtsam::NoiseModelFactor>(m_, "PriorFactorPose3")
      .def(nb::init<size_t, const gtsam::Pose3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Pose3> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Pose3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Unit3>, gtsam::NoiseModelFactor>(m_, "PriorFactorUnit3")
      .def(nb::init<size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Unit3> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Unit3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Unit3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Unit3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Unit3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Cal3_S2>, gtsam::NoiseModelFactor>(m_, "PriorFactorCal3_S2")
      .def(nb::init<size_t, const gtsam::Cal3_S2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Cal3_S2> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Cal3DS2>, gtsam::NoiseModelFactor>(m_, "PriorFactorCal3DS2")
      .def(nb::init<size_t, const gtsam::Cal3DS2 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Cal3DS2> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3DS2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3DS2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Cal3DS2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Cal3Bundler>, gtsam::NoiseModelFactor>(m_, "PriorFactorCal3Bundler")
      .def(nb::init<size_t, const gtsam::Cal3Bundler &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Cal3Bundler> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3Bundler> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3Bundler> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Cal3Bundler> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3Bundler> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Cal3Fisheye>, gtsam::NoiseModelFactor>(m_, "PriorFactorCal3Fisheye")
      .def(nb::init<size_t, const gtsam::Cal3Fisheye &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Cal3Fisheye> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3Fisheye> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3Fisheye> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Cal3Fisheye> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::Cal3Unified>, gtsam::NoiseModelFactor>(m_, "PriorFactorCal3Unified")
      .def(nb::init<size_t, const gtsam::Cal3Unified &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::Cal3Unified> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::Cal3Unified> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::Cal3Unified> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::Cal3Unified> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor>(m_, "PriorFactorCalibratedCamera")
      .def(nb::init<size_t, const gtsam::CalibratedCamera &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::CalibratedCamera> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::CalibratedCamera> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::CalibratedCamera> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::CalibratedCamera> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor>(m_, "PriorFactorPinholeCameraCal3_S2")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3_S2> &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>>, gtsam::NoiseModelFactor>(m_, "PriorFactorPinholeCameraCal3Bundler")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>, gtsam::NoiseModelFactor>(m_, "PriorFactorPinholeCameraCal3Fisheye")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>>, gtsam::NoiseModelFactor>(m_, "PriorFactorPinholeCameraCal3Unified")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Unified> &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::PinholeCamera<gtsam::Cal3Unified>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor>(m_, "PriorFactorConstantBias")
      .def(nb::init<size_t, const gtsam::imuBias::ConstantBias &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key"), nb::arg("prior"), nb::arg("noiseModel"))
      .def("prior", [](gtsam::PriorFactor<gtsam::imuBias::ConstantBias> *self) { return self->prior(); });
  // .def("serialize", [](gtsam::PriorFactor<gtsam::imuBias::ConstantBias> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::PriorFactor<gtsam::imuBias::ConstantBias> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::PriorFactor<gtsam::imuBias::ConstantBias> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::PriorFactor<gtsam::imuBias::ConstantBias> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Point2>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPoint2")
      .def(nb::init<size_t, const gtsam::Point2 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Point2 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Point2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Point2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Point2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Point2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::StereoPoint2>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityStereoPoint2")
      .def(nb::init<size_t, const gtsam::StereoPoint2 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::StereoPoint2 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::StereoPoint2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::StereoPoint2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::StereoPoint2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::StereoPoint2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPoint3")
      .def(nb::init<size_t, const gtsam::Point3 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Point3 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Point3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Point3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Point3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Point3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Rot2>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityRot2")
      .def(nb::init<size_t, const gtsam::Rot2 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Rot2 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Rot2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Rot2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Rot2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Rot2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::SO3>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualitySO3")
      .def(nb::init<size_t, const gtsam::SO3 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::SO3 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::SO3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::SO3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::SO3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::SO3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::SO4>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualitySO4")
      .def(nb::init<size_t, const gtsam::SO4 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::SO4 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::SO4> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::SO4> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::SO4> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::SO4> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::SOn>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualitySOn")
      .def(nb::init<size_t, const gtsam::SOn &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::SOn &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::SOn> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::SOn> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::SOn> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::SOn> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Rot3>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityRot3")
      .def(nb::init<size_t, const gtsam::Rot3 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Rot3 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Rot3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Rot3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Rot3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Rot3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Pose2>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPose2")
      .def(nb::init<size_t, const gtsam::Pose2 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Pose2 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Pose2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Pose2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Pose2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Pose2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Pose3>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPose3")
      .def(nb::init<size_t, const gtsam::Pose3 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Pose3 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Pose3> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Pose3> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Pose3> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Pose3> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::Cal3_S2>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityCal3_S2")
      .def(nb::init<size_t, const gtsam::Cal3_S2 &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::Cal3_S2 &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityCalibratedCamera")
      .def(nb::init<size_t, const gtsam::CalibratedCamera &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::CalibratedCamera &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::CalibratedCamera> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::CalibratedCamera> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::CalibratedCamera> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::CalibratedCamera> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPinholeCameraCal3_S2")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3_S2> &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3_S2> &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3_S2>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPinholeCameraCal3Bundler")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Bundler>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPinholeCameraCal3Fisheye")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityPinholeCameraCal3Unified")
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Unified> &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::PinholeCamera<gtsam::Cal3Unified> &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::PinholeCamera<gtsam::Cal3Unified>> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor>(m_, "NonlinearEqualityConstantBias")
      .def(nb::init<size_t, const gtsam::imuBias::ConstantBias &>(), nb::arg("j"), nb::arg("feasible"))
      .def(nb::init<size_t, const gtsam::imuBias::ConstantBias &, double>(), nb::arg("j"), nb::arg("feasible"), nb::arg("error_gain"));
  // .def("serialize", [](gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias> *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias> *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Point2>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Point2")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Point2> *self, const gtsam::Point2 &x1, const gtsam::Point2 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::StereoPoint2>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2StereoPoint2")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::StereoPoint2> *self, const gtsam::StereoPoint2 &x1, const gtsam::StereoPoint2 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Point3>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Point3")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Point3> *self, const gtsam::Point3 &x1, const gtsam::Point3 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Rot2>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Rot2")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Rot2> *self, const gtsam::Rot2 &x1, const gtsam::Rot2 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::SO3>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2SO3")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::SO3> *self, const gtsam::SO3 &x1, const gtsam::SO3 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::SO4>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2SO4")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::SO4> *self, const gtsam::SO4 &x1, const gtsam::SO4 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::SOn>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2SOn")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::SOn> *self, const gtsam::SOn &x1, const gtsam::SOn &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Rot3>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Rot3")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Rot3> *self, const gtsam::Rot3 &x1, const gtsam::Rot3 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Pose2>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Pose2")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Pose2> *self, const gtsam::Pose2 &x1, const gtsam::Pose2 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Pose3>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Pose3")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Pose3> *self, const gtsam::Pose3 &x1, const gtsam::Pose3 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::Cal3_S2>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2Cal3_S2")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::Cal3_S2> *self, const gtsam::Cal3_S2 &x1, const gtsam::Cal3_S2 &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2CalibratedCamera")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::CalibratedCamera> *self, const gtsam::CalibratedCamera &x1, const gtsam::CalibratedCamera &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2PinholeCameraCal3_S2")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3_S2>> *self, const gtsam::PinholeCamera<gtsam::Cal3_S2> &x1, const gtsam::PinholeCamera<gtsam::Cal3_S2> &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Bundler>>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2PinholeCameraCal3Bundler")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Bundler>> *self, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &x1, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2PinholeCameraCal3Fisheye")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Fisheye>> *self, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &x1, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Unified>>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2PinholeCameraCal3Unified")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::PinholeCamera<gtsam::Cal3Unified>> *self, const gtsam::PinholeCamera<gtsam::Cal3Unified> &x1, const gtsam::PinholeCamera<gtsam::Cal3Unified> &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::NonlinearEquality2<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor>(m_, "NonlinearEquality2ConstantBias")
      .def(nb::init<gtsam::Key, gtsam::Key, double>(), nb::arg("key1"), nb::arg("key2"), nb::arg("mu") = 1e4)
      .def("evaluateError", [](gtsam::NonlinearEquality2<gtsam::imuBias::ConstantBias> *self, const gtsam::imuBias::ConstantBias &x1, const gtsam::imuBias::ConstantBias &x2) { return self->evaluateError(x1, x2); }, nb::arg("x1"), nb::arg("x2"));

  nb::class_<gtsam::GncParams<gtsam::GaussNewtonParams>> gncgaussnewtonparams(m_, "GncGaussNewtonParams");
  gncgaussnewtonparams
      .def(nb::init<const gtsam::GaussNewtonParams &>(), nb::arg("baseOptimizerParams"))
      .def(nb::init<>())
      .def("setLossType", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const gtsam::GncLossType type) { self->setLossType(type); }, nb::arg("type"))
      .def("setMaxIterations", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const size_t maxIter) { self->setMaxIterations(maxIter); }, nb::arg("maxIter"))
      .def("setMuStep", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const double step) { self->setMuStep(step); }, nb::arg("step"))
      .def("setRelativeCostTol", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, double value) { self->setRelativeCostTol(value); }, nb::arg("value"))
      .def("setWeightsTol", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, double value) { self->setWeightsTol(value); }, nb::arg("value"))
      .def("setVerbosityGNC", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity value) { self->setVerbosityGNC(value); }, nb::arg("value"))
      .def("setKnownInliers", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const gtsam::KeyVector &knownIn) { self->setKnownInliers(knownIn); }, nb::arg("knownIn"))
      .def("setKnownOutliers", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const gtsam::KeyVector &knownOut) { self->setKnownOutliers(knownOut); }, nb::arg("knownOut"))
      .def("print", [](gtsam::GncParams<gtsam::GaussNewtonParams> *self, const string &str) { /* nb::scoped_ostream_redirect output; */ self->print(str); }, nb::arg("str") = "GncParams: ")
      .def("__repr__", [](const gtsam::GncParams<gtsam::GaussNewtonParams> &self, const string &str) {
                        gtsam::RedirectCout redirect;
                        self.print(str);
                        return redirect.str(); }, nb::arg("str") = "GncParams: ")
      .def_rw("baseOptimizerParams", &gtsam::GncParams<gtsam::GaussNewtonParams>::baseOptimizerParams)
      .def_rw("lossType", &gtsam::GncParams<gtsam::GaussNewtonParams>::lossType)
      .def_rw("maxIterations", &gtsam::GncParams<gtsam::GaussNewtonParams>::maxIterations)
      .def_rw("muStep", &gtsam::GncParams<gtsam::GaussNewtonParams>::muStep)
      .def_rw("relativeCostTol", &gtsam::GncParams<gtsam::GaussNewtonParams>::relativeCostTol)
      .def_rw("weightsTol", &gtsam::GncParams<gtsam::GaussNewtonParams>::weightsTol)
      .def_rw("verbosity", &gtsam::GncParams<gtsam::GaussNewtonParams>::verbosity)
      .def_rw("knownInliers", &gtsam::GncParams<gtsam::GaussNewtonParams>::knownInliers)
      .def_rw("knownOutliers", &gtsam::GncParams<gtsam::GaussNewtonParams>::knownOutliers);

  nb::enum_<gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity>(gncgaussnewtonparams, "Verbosity", nb::is_arithmetic())
      .value("SILENT", gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity::SILENT)
      .value("SUMMARY", gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity::SUMMARY)
      .value("VALUES", gtsam::GncParams<gtsam::GaussNewtonParams>::Verbosity::VALUES);

  nb::class_<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> gnclmparams(m_, "GncLMParams");
  gnclmparams
      .def(nb::init<const gtsam::LevenbergMarquardtParams &>(), nb::arg("baseOptimizerParams"))
      .def(nb::init<>())
      .def("setLossType", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const gtsam::GncLossType type) { self->setLossType(type); }, nb::arg("type"))
      .def("setMaxIterations", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const size_t maxIter) { self->setMaxIterations(maxIter); }, nb::arg("maxIter"))
      .def("setMuStep", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const double step) { self->setMuStep(step); }, nb::arg("step"))
      .def("setRelativeCostTol", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, double value) { self->setRelativeCostTol(value); }, nb::arg("value"))
      .def("setWeightsTol", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, double value) { self->setWeightsTol(value); }, nb::arg("value"))
      .def("setVerbosityGNC", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity value) { self->setVerbosityGNC(value); }, nb::arg("value"))
      .def("setKnownInliers", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const gtsam::KeyVector &knownIn) { self->setKnownInliers(knownIn); }, nb::arg("knownIn"))
      .def("setKnownOutliers", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const gtsam::KeyVector &knownOut) { self->setKnownOutliers(knownOut); }, nb::arg("knownOut"))
      .def("print", [](gtsam::GncParams<gtsam::LevenbergMarquardtParams> *self, const string &str) { /* nb::scoped_ostream_redirect output; */ self->print(str); }, nb::arg("str") = "GncParams: ")
      .def("__repr__", [](const gtsam::GncParams<gtsam::LevenbergMarquardtParams> &self, const string &str) {
                        gtsam::RedirectCout redirect;
                        self.print(str);
                        return redirect.str(); }, nb::arg("str") = "GncParams: ")
      .def_rw("baseOptimizerParams", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::baseOptimizerParams)
      .def_rw("lossType", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::lossType)
      .def_rw("maxIterations", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::maxIterations)
      .def_rw("muStep", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::muStep)
      .def_rw("relativeCostTol", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::relativeCostTol)
      .def_rw("weightsTol", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::weightsTol)
      .def_rw("verbosity", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::verbosity)
      .def_rw("knownInliers", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::knownInliers)
      .def_rw("knownOutliers", &gtsam::GncParams<gtsam::LevenbergMarquardtParams>::knownOutliers);

  nb::enum_<gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity>(gnclmparams, "Verbosity", nb::is_arithmetic())
      .value("SILENT", gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SILENT)
      .value("SUMMARY", gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::SUMMARY)
      .value("VALUES", gtsam::GncParams<gtsam::LevenbergMarquardtParams>::Verbosity::VALUES);

  nb::class_<gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>>>(m_, "GncGaussNewtonOptimizer")
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &, const gtsam::GncParams<gtsam::GaussNewtonParams> &>(), nb::arg("graph"), nb::arg("initialValues"), nb::arg("params"))
      .def("setInlierCostThresholds", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> *self, const double inth) { self->setInlierCostThresholds(inth); }, nb::arg("inth"))
      .def("getInlierCostThresholds", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> *self) { return self->getInlierCostThresholds(); })
      .def("setInlierCostThresholdsAtProbability", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> *self, const double alpha) { self->setInlierCostThresholdsAtProbability(alpha); }, nb::arg("alpha"))
      .def("setWeights", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> *self, const gtsam::Vector &w) { self->setWeights(w); }, nb::arg("w"))
      .def("getWeights", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> *self) { return self->getWeights(); })
      .def("optimize", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> *self) { return self->optimize(); });

  nb::class_<gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>>>(m_, "GncLMOptimizer")
      .def(nb::init<const gtsam::NonlinearFactorGraph &, const gtsam::Values &, const gtsam::GncParams<gtsam::LevenbergMarquardtParams> &>(), nb::arg("graph"), nb::arg("initialValues"), nb::arg("params"))
      .def("setInlierCostThresholds", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> *self, const double inth) { self->setInlierCostThresholds(inth); }, nb::arg("inth"))
      .def("getInlierCostThresholds", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> *self) { return self->getInlierCostThresholds(); })
      .def("setInlierCostThresholdsAtProbability", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> *self, const double alpha) { self->setInlierCostThresholdsAtProbability(alpha); }, nb::arg("alpha"))
      .def("setWeights", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> *self, const gtsam::Vector &w) { self->setWeights(w); }, nb::arg("w"))
      .def("getWeights", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> *self) { return self->getWeights(); })
      .def("optimize", [](gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> *self) { return self->optimize(); });

  m_.def("checkConvergence", [](double relativeErrorTreshold, double absoluteErrorTreshold, double errorThreshold, double currentError, double newError) { return gtsam::checkConvergence(relativeErrorTreshold, absoluteErrorTreshold, errorThreshold, currentError, newError); }, nb::arg("relativeErrorTreshold"), nb::arg("absoluteErrorTreshold"), nb::arg("errorThreshold"), nb::arg("currentError"), nb::arg("newError"));
  m_.def("checkConvergence", [](const gtsam::NonlinearOptimizerParams &params, double currentError, double newError) { return gtsam::checkConvergence(params, currentError, newError); }, nb::arg("params"), nb::arg("currentError"), nb::arg("newError"));
}
