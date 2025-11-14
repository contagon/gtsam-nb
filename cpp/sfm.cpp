/**
 * @file    sfm.cpp
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
#include <nanobind/stl/string.h>

// These are the included headers listed in `gtsam.i`
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/sfm/BinaryMeasurement.h"
#include "gtsam/sfm/DsfTrackGenerator.h"
#include "gtsam/sfm/MFAS.h"
#include "gtsam/sfm/SfmData.h"
#include "gtsam/sfm/SfmTrack.h"
#include "gtsam/sfm/ShonanAveraging.h"
#include "gtsam/sfm/ShonanFactor.h"
#include "gtsam/sfm/TranslationRecovery.h"

using namespace std;

namespace nb = nanobind;

void sfm(nb::module_ &m_) {
  m_.doc() = "pybind11 wrapper of sfm";

  nb::class_<gtsam::SfmTrack2d>(m_, "SfmTrack2d")
      .def(nb::init<>())
      .def(nb::init<const std::vector<gtsam::SfmMeasurement> &>(), nb::arg("measurements"))
      .def("numberMeasurements", [](gtsam::SfmTrack2d *self) { return self->numberMeasurements(); })
      .def("measurement", [](gtsam::SfmTrack2d *self, size_t idx) { return self->measurement(idx); }, nb::arg("idx"))
      .def("siftIndex", [](gtsam::SfmTrack2d *self, size_t idx) { return self->siftIndex(idx); }, nb::arg("idx"))
      .def("addMeasurement", [](gtsam::SfmTrack2d *self, size_t idx, const gtsam::Point2 &m) { self->addMeasurement(idx, m); }, nb::arg("idx"), nb::arg("m"))
      .def("measurement", [](gtsam::SfmTrack2d *self, size_t idx) { return self->measurement(idx); }, nb::arg("idx"))
      .def("hasUniqueCameras", [](gtsam::SfmTrack2d *self) { return self->hasUniqueCameras(); })
      .def("measurementMatrix", [](gtsam::SfmTrack2d *self) { return self->measurementMatrix(); })
      .def("indexVector", [](gtsam::SfmTrack2d *self) { return self->indexVector(); })
      .def_rw("measurements", &gtsam::SfmTrack2d::measurements);

  nb::class_<gtsam::SfmTrack, gtsam::SfmTrack2d>(m_, "SfmTrack")
      .def(nb::init<>())
      .def(nb::init<const gtsam::Point3 &>(), nb::arg("pt"))
      .def("point3", [](gtsam::SfmTrack *self) { return self->point3(); })
      // .def("serialize", [](gtsam::SfmTrack *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::SfmTrack *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::SfmTrack &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::SfmTrack obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("equals", [](gtsam::SfmTrack *self, const gtsam::SfmTrack &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def_rw("p", &gtsam::SfmTrack::p)
      .def_rw("r", &gtsam::SfmTrack::r)
      .def_rw("g", &gtsam::SfmTrack::g)
      .def_rw("b", &gtsam::SfmTrack::b);

  nb::class_<gtsam::SfmData>(m_, "SfmData")
      .def(nb::init<>())
      .def("trackList", [](gtsam::SfmData *self) { return self->trackList(); })
      .def("cameraList", [](gtsam::SfmData *self) { return self->cameraList(); })
      .def("addTrack", [](gtsam::SfmData *self, const gtsam::SfmTrack &t) { self->addTrack(t); }, nb::arg("t"))
      .def("addCamera", [](gtsam::SfmData *self, const gtsam::SfmCamera &cam) { self->addCamera(cam); }, nb::arg("cam"))
      .def("numberTracks", [](gtsam::SfmData *self) { return self->numberTracks(); })
      .def("numberCameras", [](gtsam::SfmData *self) { return self->numberCameras(); })
      .def("track", [](gtsam::SfmData *self, size_t idx) { return self->track(idx); }, nb::arg("idx"))
      .def("camera", [](gtsam::SfmData *self, size_t idx) { return self->camera(idx); }, nb::arg("idx"))
      .def("generalSfmFactors", [](gtsam::SfmData *self, const gtsam::SharedNoiseModel &model) { return self->generalSfmFactors(model); }, nb::arg("model") = gtsam::noiseModel::Isotropic::Sigma(2, 1.0))
      .def("sfmFactorGraph", [](gtsam::SfmData *self, const gtsam::SharedNoiseModel &model, size_t fixedCamera, size_t fixedPoint) { return self->sfmFactorGraph(model, fixedCamera, fixedPoint); }, nb::arg("model") = gtsam::noiseModel::Isotropic::Sigma(2, 1.0), nb::arg("fixedCamera") = 0, nb::arg("fixedPoint") = 0)
      // .def("serialize", [](gtsam::SfmData *self) { return gtsam::serialize(*self); })
      // .def("deserialize", [](gtsam::SfmData *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
      // .def(nb::pickle([](const gtsam::SfmData &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::SfmData obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("equals", [](gtsam::SfmData *self, const gtsam::SfmData &expected, double tol) { return self->equals(expected, tol); }, nb::arg("expected"), nb::arg("tol"))
      .def_static("FromBundlerFile", [](string filename) { return gtsam::SfmData::FromBundlerFile(filename); }, nb::arg("filename"))
      .def_static("FromBalFile", [](string filename) { return gtsam::SfmData::FromBalFile(filename); }, nb::arg("filename"));

  nb::class_<gtsam::ShonanFactor3, gtsam::NoiseModelFactor>(m_, "ShonanFactor3")
      .def(nb::init<size_t, size_t, const gtsam::Rot3 &, size_t>(), nb::arg("key1"), nb::arg("key2"), nb::arg("R12"), nb::arg("p"))
      .def(nb::init<size_t, size_t, const gtsam::Rot3 &, size_t, boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("R12"), nb::arg("p"), nb::arg("model"))
      .def("evaluateError", [](gtsam::ShonanFactor3 *self, const gtsam::SOn &Q1, const gtsam::SOn &Q2) { return self->evaluateError(Q1, Q2); }, nb::arg("Q1"), nb::arg("Q2"));

  nb::class_<gtsam::ShonanAveragingParameters2>(m_, "ShonanAveragingParameters2")
      .def(nb::init<const gtsam::LevenbergMarquardtParams &>(), nb::arg("lm"))
      .def(nb::init<const gtsam::LevenbergMarquardtParams &, string>(), nb::arg("lm"), nb::arg("method"))
      .def("getLMParams", [](gtsam::ShonanAveragingParameters2 *self) { return self->getLMParams(); })
      .def("setOptimalityThreshold", [](gtsam::ShonanAveragingParameters2 *self, double value) { self->setOptimalityThreshold(value); }, nb::arg("value"))
      .def("getOptimalityThreshold", [](gtsam::ShonanAveragingParameters2 *self) { return self->getOptimalityThreshold(); })
      .def("setAnchor", [](gtsam::ShonanAveragingParameters2 *self, size_t index, const gtsam::Rot2 &value) { self->setAnchor(index, value); }, nb::arg("index"), nb::arg("value"))
      .def("getAnchor", [](gtsam::ShonanAveragingParameters2 *self) { return self->getAnchor(); })
      .def("setAnchorWeight", [](gtsam::ShonanAveragingParameters2 *self, double value) { self->setAnchorWeight(value); }, nb::arg("value"))
      .def("getAnchorWeight", [](gtsam::ShonanAveragingParameters2 *self) { return self->getAnchorWeight(); })
      .def("setKarcherWeight", [](gtsam::ShonanAveragingParameters2 *self, double value) { self->setKarcherWeight(value); }, nb::arg("value"))
      .def("getKarcherWeight", [](gtsam::ShonanAveragingParameters2 *self) { return self->getKarcherWeight(); })
      .def("setGaugesWeight", [](gtsam::ShonanAveragingParameters2 *self, double value) { self->setGaugesWeight(value); }, nb::arg("value"))
      .def("getGaugesWeight", [](gtsam::ShonanAveragingParameters2 *self) { return self->getGaugesWeight(); })
      .def("setUseHuber", [](gtsam::ShonanAveragingParameters2 *self, bool value) { self->setUseHuber(value); }, nb::arg("value"))
      .def("getUseHuber", [](gtsam::ShonanAveragingParameters2 *self) { return self->getUseHuber(); })
      .def("setCertifyOptimality", [](gtsam::ShonanAveragingParameters2 *self, bool value) { self->setCertifyOptimality(value); }, nb::arg("value"))
      .def("getCertifyOptimality", [](gtsam::ShonanAveragingParameters2 *self) { return self->getCertifyOptimality(); });

  nb::class_<gtsam::ShonanAveragingParameters3>(m_, "ShonanAveragingParameters3")
      .def(nb::init<const gtsam::LevenbergMarquardtParams &>(), nb::arg("lm"))
      .def(nb::init<const gtsam::LevenbergMarquardtParams &, string>(), nb::arg("lm"), nb::arg("method"))
      .def("getLMParams", [](gtsam::ShonanAveragingParameters3 *self) { return self->getLMParams(); })
      .def("setOptimalityThreshold", [](gtsam::ShonanAveragingParameters3 *self, double value) { self->setOptimalityThreshold(value); }, nb::arg("value"))
      .def("getOptimalityThreshold", [](gtsam::ShonanAveragingParameters3 *self) { return self->getOptimalityThreshold(); })
      .def("setAnchor", [](gtsam::ShonanAveragingParameters3 *self, size_t index, const gtsam::Rot3 &value) { self->setAnchor(index, value); }, nb::arg("index"), nb::arg("value"))
      .def("getAnchor", [](gtsam::ShonanAveragingParameters3 *self) { return self->getAnchor(); })
      .def("setAnchorWeight", [](gtsam::ShonanAveragingParameters3 *self, double value) { self->setAnchorWeight(value); }, nb::arg("value"))
      .def("getAnchorWeight", [](gtsam::ShonanAveragingParameters3 *self) { return self->getAnchorWeight(); })
      .def("setKarcherWeight", [](gtsam::ShonanAveragingParameters3 *self, double value) { self->setKarcherWeight(value); }, nb::arg("value"))
      .def("getKarcherWeight", [](gtsam::ShonanAveragingParameters3 *self) { return self->getKarcherWeight(); })
      .def("setGaugesWeight", [](gtsam::ShonanAveragingParameters3 *self, double value) { self->setGaugesWeight(value); }, nb::arg("value"))
      .def("getGaugesWeight", [](gtsam::ShonanAveragingParameters3 *self) { return self->getGaugesWeight(); })
      .def("setUseHuber", [](gtsam::ShonanAveragingParameters3 *self, bool value) { self->setUseHuber(value); }, nb::arg("value"))
      .def("getUseHuber", [](gtsam::ShonanAveragingParameters3 *self) { return self->getUseHuber(); })
      .def("setCertifyOptimality", [](gtsam::ShonanAveragingParameters3 *self, bool value) { self->setCertifyOptimality(value); }, nb::arg("value"))
      .def("getCertifyOptimality", [](gtsam::ShonanAveragingParameters3 *self) { return self->getCertifyOptimality(); });

  nb::class_<gtsam::ShonanAveraging2>(m_, "ShonanAveraging2")
      .def(nb::init<string>(), nb::arg("g2oFile"))
      .def(nb::init<string, const gtsam::ShonanAveragingParameters2 &>(), nb::arg("g2oFile"), nb::arg("parameters"))
      .def(nb::init<const gtsam::BetweenFactorPose2s &, const gtsam::ShonanAveragingParameters2 &>(), nb::arg("factors"), nb::arg("parameters"))
      .def("nrUnknowns", [](gtsam::ShonanAveraging2 *self) { return self->nrUnknowns(); })
      .def("numberMeasurements", [](gtsam::ShonanAveraging2 *self) { return self->numberMeasurements(); })
      .def("measured", [](gtsam::ShonanAveraging2 *self, size_t i) { return self->measured(i); }, nb::arg("i"))
      .def("keys", [](gtsam::ShonanAveraging2 *self, size_t i) { return self->keys(i); }, nb::arg("i"))
      .def("denseD", [](gtsam::ShonanAveraging2 *self) { return self->denseD(); })
      .def("denseQ", [](gtsam::ShonanAveraging2 *self) { return self->denseQ(); })
      .def("denseL", [](gtsam::ShonanAveraging2 *self) { return self->denseL(); })
      .def("computeLambda_", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->computeLambda_(values); }, nb::arg("values"))
      .def("computeA_", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->computeA_(values); }, nb::arg("values"))
      .def("computeMinEigenValue", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->computeMinEigenValue(values); }, nb::arg("values"))
      .def("initializeWithDescent", [](gtsam::ShonanAveraging2 *self, size_t p, const gtsam::Values &values, const gtsam::Vector &minEigenVector, double minEigenValue) { return self->initializeWithDescent(p, values, minEigenVector, minEigenValue); }, nb::arg("p"), nb::arg("values"), nb::arg("minEigenVector"), nb::arg("minEigenValue"))
      .def("buildGraphAt", [](gtsam::ShonanAveraging2 *self, size_t p) { return self->buildGraphAt(p); }, nb::arg("p"))
      .def("initializeRandomlyAt", [](gtsam::ShonanAveraging2 *self, size_t p) { return self->initializeRandomlyAt(p); }, nb::arg("p"))
      .def("costAt", [](gtsam::ShonanAveraging2 *self, size_t p, const gtsam::Values &values) { return self->costAt(p, values); }, nb::arg("p"), nb::arg("values"))
      .def("computeMinEigenVector", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->computeMinEigenVector(values); }, nb::arg("values"))
      .def("checkOptimality", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->checkOptimality(values); }, nb::arg("values"))
      .def("createOptimizerAt", [](gtsam::ShonanAveraging2 *self, size_t p, const gtsam::Values &initial) { return self->createOptimizerAt(p, initial); }, nb::arg("p"), nb::arg("initial"))
      .def("tryOptimizingAt", [](gtsam::ShonanAveraging2 *self, size_t p, const gtsam::Values &initial) { return self->tryOptimizingAt(p, initial); }, nb::arg("p"), nb::arg("initial"))
      .def("projectFrom", [](gtsam::ShonanAveraging2 *self, size_t p, const gtsam::Values &values) { return self->projectFrom(p, values); }, nb::arg("p"), nb::arg("values"))
      .def("roundSolution", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->roundSolution(values); }, nb::arg("values"))
      .def("cost", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &values) { return self->cost(values); }, nb::arg("values"))
      .def("initializeRandomly", [](gtsam::ShonanAveraging2 *self) { return self->initializeRandomly(); })
      .def("run", [](gtsam::ShonanAveraging2 *self, const gtsam::Values &initial, size_t min_p, size_t max_p) { return self->run(initial, min_p, max_p); }, nb::arg("initial"), nb::arg("min_p"), nb::arg("max_p"));

  nb::class_<gtsam::ShonanAveraging3>(m_, "ShonanAveraging3")
      .def(nb::init<const std::vector<gtsam::BinaryMeasurement<gtsam::Rot3>> &, const gtsam::ShonanAveragingParameters3 &>(), nb::arg("measurements"), nb::arg("parameters") = gtsam::ShonanAveragingParameters3())
      .def(nb::init<string>(), nb::arg("g2oFile"))
      .def(nb::init<string, const gtsam::ShonanAveragingParameters3 &>(), nb::arg("g2oFile"), nb::arg("parameters"))
      .def(nb::init<const gtsam::BetweenFactorPose3s &>(), nb::arg("factors"))
      .def(nb::init<const gtsam::BetweenFactorPose3s &, const gtsam::ShonanAveragingParameters3 &>(), nb::arg("factors"), nb::arg("parameters"))
      .def("nrUnknowns", [](gtsam::ShonanAveraging3 *self) { return self->nrUnknowns(); })
      .def("numberMeasurements", [](gtsam::ShonanAveraging3 *self) { return self->numberMeasurements(); })
      .def("measured", [](gtsam::ShonanAveraging3 *self, size_t i) { return self->measured(i); }, nb::arg("i"))
      .def("keys", [](gtsam::ShonanAveraging3 *self, size_t i) { return self->keys(i); }, nb::arg("i"))
      .def("denseD", [](gtsam::ShonanAveraging3 *self) { return self->denseD(); })
      .def("denseQ", [](gtsam::ShonanAveraging3 *self) { return self->denseQ(); })
      .def("denseL", [](gtsam::ShonanAveraging3 *self) { return self->denseL(); })
      .def("computeLambda_", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->computeLambda_(values); }, nb::arg("values"))
      .def("computeA_", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->computeA_(values); }, nb::arg("values"))
      .def("computeMinEigenValue", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->computeMinEigenValue(values); }, nb::arg("values"))
      .def("initializeWithDescent", [](gtsam::ShonanAveraging3 *self, size_t p, const gtsam::Values &values, const gtsam::Vector &minEigenVector, double minEigenValue) { return self->initializeWithDescent(p, values, minEigenVector, minEigenValue); }, nb::arg("p"), nb::arg("values"), nb::arg("minEigenVector"), nb::arg("minEigenValue"))
      .def("buildGraphAt", [](gtsam::ShonanAveraging3 *self, size_t p) { return self->buildGraphAt(p); }, nb::arg("p"))
      .def("initializeRandomlyAt", [](gtsam::ShonanAveraging3 *self, size_t p) { return self->initializeRandomlyAt(p); }, nb::arg("p"))
      .def("costAt", [](gtsam::ShonanAveraging3 *self, size_t p, const gtsam::Values &values) { return self->costAt(p, values); }, nb::arg("p"), nb::arg("values"))
      .def("computeMinEigenVector", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->computeMinEigenVector(values); }, nb::arg("values"))
      .def("checkOptimality", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->checkOptimality(values); }, nb::arg("values"))
      .def("createOptimizerAt", [](gtsam::ShonanAveraging3 *self, size_t p, const gtsam::Values &initial) { return self->createOptimizerAt(p, initial); }, nb::arg("p"), nb::arg("initial"))
      .def("tryOptimizingAt", [](gtsam::ShonanAveraging3 *self, size_t p, const gtsam::Values &initial) { return self->tryOptimizingAt(p, initial); }, nb::arg("p"), nb::arg("initial"))
      .def("projectFrom", [](gtsam::ShonanAveraging3 *self, size_t p, const gtsam::Values &values) { return self->projectFrom(p, values); }, nb::arg("p"), nb::arg("values"))
      .def("roundSolution", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->roundSolution(values); }, nb::arg("values"))
      .def("cost", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &values) { return self->cost(values); }, nb::arg("values"))
      .def("initializeRandomly", [](gtsam::ShonanAveraging3 *self) { return self->initializeRandomly(); })
      .def("run", [](gtsam::ShonanAveraging3 *self, const gtsam::Values &initial, size_t min_p, size_t max_p) { return self->run(initial, min_p, max_p); }, nb::arg("initial"), nb::arg("min_p"), nb::arg("max_p"));

  nb::class_<gtsam::MFAS>(m_, "MFAS")
      .def(nb::init<const gtsam::BinaryMeasurementsUnit3 &, const gtsam::Unit3 &>(), nb::arg("relativeTranslations"), nb::arg("projectionDirection"))
      .def("computeOutlierWeights", [](gtsam::MFAS *self) { return self->computeOutlierWeights(); })
      .def("computeOrdering", [](gtsam::MFAS *self) { return self->computeOrdering(); });

  nb::class_<gtsam::TranslationRecovery>(m_, "TranslationRecovery")
      .def(nb::init<const gtsam::LevenbergMarquardtParams &>(), nb::arg("lmParams"))
      .def(nb::init<>())
      .def("addPrior", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3 &betweenTranslations, gtsam::NonlinearFactorGraph *graph, const gtsam::SharedNoiseModel &priorNoiseModel) { self->addPrior(relativeTranslations, scale, betweenTranslations, graph, priorNoiseModel); }, nb::arg("relativeTranslations"), nb::arg("scale"), nb::arg("betweenTranslations"), nb::arg("graph"), nb::arg("priorNoiseModel"))
      .def("addPrior", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3 &betweenTranslations, gtsam::NonlinearFactorGraph *graph) { self->addPrior(relativeTranslations, scale, betweenTranslations, graph); }, nb::arg("relativeTranslations"), nb::arg("scale"), nb::arg("betweenTranslations"), nb::arg("graph"))
      .def("buildGraph", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations) { return self->buildGraph(relativeTranslations); }, nb::arg("relativeTranslations"))
      .def("run", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3 &betweenTranslations, const gtsam::Values &initialValues) { return self->run(relativeTranslations, scale, betweenTranslations, initialValues); }, nb::arg("relativeTranslations"), nb::arg("scale"), nb::arg("betweenTranslations"), nb::arg("initialValues"))
      .def("run", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3 &betweenTranslations) { return self->run(relativeTranslations, scale, betweenTranslations); }, nb::arg("relativeTranslations"), nb::arg("scale"), nb::arg("betweenTranslations"))
      .def("run", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations, const double scale) { return self->run(relativeTranslations, scale); }, nb::arg("relativeTranslations"), nb::arg("scale"))
      .def("run", [](gtsam::TranslationRecovery *self, const gtsam::BinaryMeasurementsUnit3 &relativeTranslations) { return self->run(relativeTranslations); }, nb::arg("relativeTranslations"));
  auto m_gtsfm = m_.def_submodule("gtsfm", "gtsfm submodule");

  nb::class_<gtsam::gtsfm::Keypoints>(m_gtsfm, "Keypoints")
      .def(nb::init<const Eigen::MatrixX2d &>(), nb::arg("coordinates"))
      .def_rw("coordinates", &gtsam::gtsfm::Keypoints::coordinates);

  m_gtsfm.def("tracksFromPairwiseMatches", [](const gtsam::gtsfm::MatchIndicesMap &matches_dict, const gtsam::gtsfm::KeypointsVector &keypoints_list, bool verbose) { return gtsam::gtsfm::tracksFromPairwiseMatches(matches_dict, keypoints_list, verbose); }, nb::arg("matches_dict"), nb::arg("keypoints_list"), nb::arg("verbose") = false);
  nb::class_<gtsam::BinaryMeasurement<gtsam::Unit3>>(m_, "BinaryMeasurementUnit3")
      .def(nb::init<size_t, size_t, const gtsam::Unit3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"))
      .def("key1", [](gtsam::BinaryMeasurement<gtsam::Unit3> *self) { return self->key1(); })
      .def("key2", [](gtsam::BinaryMeasurement<gtsam::Unit3> *self) { return self->key2(); })
      .def("measured", [](gtsam::BinaryMeasurement<gtsam::Unit3> *self) { return self->measured(); })
      .def("noiseModel", [](gtsam::BinaryMeasurement<gtsam::Unit3> *self) { return self->noiseModel(); });

  nb::class_<gtsam::BinaryMeasurement<gtsam::Rot3>>(m_, "BinaryMeasurementRot3")
      .def(nb::init<size_t, size_t, const gtsam::Rot3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"))
      .def("key1", [](gtsam::BinaryMeasurement<gtsam::Rot3> *self) { return self->key1(); })
      .def("key2", [](gtsam::BinaryMeasurement<gtsam::Rot3> *self) { return self->key2(); })
      .def("measured", [](gtsam::BinaryMeasurement<gtsam::Rot3> *self) { return self->measured(); })
      .def("noiseModel", [](gtsam::BinaryMeasurement<gtsam::Rot3> *self) { return self->noiseModel(); });

  nb::class_<gtsam::BinaryMeasurement<gtsam::Point3>>(m_, "BinaryMeasurementPoint3")
      .def(nb::init<size_t, size_t, const gtsam::Point3 &, const boost::shared_ptr<gtsam::noiseModel::Base>>(), nb::arg("key1"), nb::arg("key2"), nb::arg("measured"), nb::arg("model"))
      .def("key1", [](gtsam::BinaryMeasurement<gtsam::Point3> *self) { return self->key1(); })
      .def("key2", [](gtsam::BinaryMeasurement<gtsam::Point3> *self) { return self->key2(); })
      .def("measured", [](gtsam::BinaryMeasurement<gtsam::Point3> *self) { return self->measured(); })
      .def("noiseModel", [](gtsam::BinaryMeasurement<gtsam::Point3> *self) { return self->noiseModel(); });

  m_.def("readBal", [](string filename) { return gtsam::readBal(filename); }, nb::arg("filename"));
  m_.def("writeBAL", [](string filename, gtsam::SfmData &data) { return gtsam::writeBAL(filename, data); }, nb::arg("filename"), nb::arg("data"));
  m_.def("initialCamerasEstimate", [](const gtsam::SfmData &db) { return gtsam::initialCamerasEstimate(db); }, nb::arg("db"));
  m_.def("initialCamerasAndPointsEstimate", [](const gtsam::SfmData &db) { return gtsam::initialCamerasAndPointsEstimate(db); }, nb::arg("db"));
}
