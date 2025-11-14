/**
 * @file    sfm.cpp
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
#include "gtsam/sfm/SfmTrack.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/sfm/SfmData.h"
#include "gtsam/sfm/ShonanFactor.h"
#include "gtsam/sfm/BinaryMeasurement.h"
#include "gtsam/sfm/ShonanAveraging.h"
#include "gtsam/sfm/MFAS.h"
#include "gtsam/sfm/TranslationRecovery.h"
#include "gtsam/sfm/DsfTrackGenerator.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::SfmTrack)
BOOST_CLASS_EXPORT(gtsam::SfmData)


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/sfm.h"

using namespace std;

namespace py = pybind11;



void sfm(py::module_ &m_) {
    m_.doc() = "pybind11 wrapper of sfm";




    py::class_<gtsam::SfmTrack2d, boost::shared_ptr<gtsam::SfmTrack2d>>(m_, "SfmTrack2d")
        .def(py::init<>())
        .def(py::init<const std::vector<gtsam::SfmMeasurement>&>(), py::arg("measurements"))
        .def("numberMeasurements",[](gtsam::SfmTrack2d* self){return self->numberMeasurements();})
        .def("measurement",[](gtsam::SfmTrack2d* self, size_t idx){return self->measurement(idx);}, py::arg("idx"))
        .def("siftIndex",[](gtsam::SfmTrack2d* self, size_t idx){return self->siftIndex(idx);}, py::arg("idx"))
        .def("addMeasurement",[](gtsam::SfmTrack2d* self, size_t idx, const gtsam::Point2& m){ self->addMeasurement(idx, m);}, py::arg("idx"), py::arg("m"))
        .def("measurement",[](gtsam::SfmTrack2d* self, size_t idx){return self->measurement(idx);}, py::arg("idx"))
        .def("hasUniqueCameras",[](gtsam::SfmTrack2d* self){return self->hasUniqueCameras();})
        .def("measurementMatrix",[](gtsam::SfmTrack2d* self){return self->measurementMatrix();})
        .def("indexVector",[](gtsam::SfmTrack2d* self){return self->indexVector();})
        .def_readwrite("measurements", &gtsam::SfmTrack2d::measurements);

    py::class_<gtsam::SfmTrack, gtsam::SfmTrack2d, boost::shared_ptr<gtsam::SfmTrack>>(m_, "SfmTrack")
        .def(py::init<>())
        .def(py::init<const gtsam::Point3&>(), py::arg("pt"))
        .def("point3",[](gtsam::SfmTrack* self){return self->point3();})
        .def("serialize", [](gtsam::SfmTrack* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::SfmTrack* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::SfmTrack &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::SfmTrack obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("equals",[](gtsam::SfmTrack* self, const gtsam::SfmTrack& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def_readwrite("p", &gtsam::SfmTrack::p)
        .def_readwrite("r", &gtsam::SfmTrack::r)
        .def_readwrite("g", &gtsam::SfmTrack::g)
        .def_readwrite("b", &gtsam::SfmTrack::b);

    py::class_<gtsam::SfmData, boost::shared_ptr<gtsam::SfmData>>(m_, "SfmData")
        .def(py::init<>())
        .def("trackList",[](gtsam::SfmData* self){return self->trackList();})
        .def("cameraList",[](gtsam::SfmData* self){return self->cameraList();})
        .def("addTrack",[](gtsam::SfmData* self, const gtsam::SfmTrack& t){ self->addTrack(t);}, py::arg("t"))
        .def("addCamera",[](gtsam::SfmData* self, const gtsam::SfmCamera& cam){ self->addCamera(cam);}, py::arg("cam"))
        .def("numberTracks",[](gtsam::SfmData* self){return self->numberTracks();})
        .def("numberCameras",[](gtsam::SfmData* self){return self->numberCameras();})
        .def("track",[](gtsam::SfmData* self, size_t idx){return self->track(idx);}, py::arg("idx"))
        .def("camera",[](gtsam::SfmData* self, size_t idx){return self->camera(idx);}, py::arg("idx"))
        .def("generalSfmFactors",[](gtsam::SfmData* self, const gtsam::SharedNoiseModel& model){return self->generalSfmFactors(model);}, py::arg("model") = gtsam::noiseModel::Isotropic::Sigma(2, 1.0))
        .def("sfmFactorGraph",[](gtsam::SfmData* self, const gtsam::SharedNoiseModel& model, size_t fixedCamera, size_t fixedPoint){return self->sfmFactorGraph(model, fixedCamera, fixedPoint);}, py::arg("model") = gtsam::noiseModel::Isotropic::Sigma(2, 1.0), py::arg("fixedCamera") = 0, py::arg("fixedPoint") = 0)
        .def("serialize", [](gtsam::SfmData* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::SfmData* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::SfmData &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::SfmData obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def("equals",[](gtsam::SfmData* self, const gtsam::SfmData& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def_static("FromBundlerFile",[](string filename){return gtsam::SfmData::FromBundlerFile(filename);}, py::arg("filename"))
        .def_static("FromBalFile",[](string filename){return gtsam::SfmData::FromBalFile(filename);}, py::arg("filename"));

    py::class_<gtsam::ShonanFactor3, gtsam::NoiseModelFactor, boost::shared_ptr<gtsam::ShonanFactor3>>(m_, "ShonanFactor3")
        .def(py::init<size_t, size_t, const gtsam::Rot3&, size_t>(), py::arg("key1"), py::arg("key2"), py::arg("R12"), py::arg("p"))
        .def(py::init<size_t, size_t, const gtsam::Rot3&, size_t, boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("R12"), py::arg("p"), py::arg("model"))
        .def("evaluateError",[](gtsam::ShonanFactor3* self, const gtsam::SOn& Q1, const gtsam::SOn& Q2){return self->evaluateError(Q1, Q2);}, py::arg("Q1"), py::arg("Q2"));

    py::class_<gtsam::ShonanAveragingParameters2, boost::shared_ptr<gtsam::ShonanAveragingParameters2>>(m_, "ShonanAveragingParameters2")
        .def(py::init<const gtsam::LevenbergMarquardtParams&>(), py::arg("lm"))
        .def(py::init<const gtsam::LevenbergMarquardtParams&, string>(), py::arg("lm"), py::arg("method"))
        .def("getLMParams",[](gtsam::ShonanAveragingParameters2* self){return self->getLMParams();})
        .def("setOptimalityThreshold",[](gtsam::ShonanAveragingParameters2* self, double value){ self->setOptimalityThreshold(value);}, py::arg("value"))
        .def("getOptimalityThreshold",[](gtsam::ShonanAveragingParameters2* self){return self->getOptimalityThreshold();})
        .def("setAnchor",[](gtsam::ShonanAveragingParameters2* self, size_t index, const gtsam::Rot2& value){ self->setAnchor(index, value);}, py::arg("index"), py::arg("value"))
        .def("getAnchor",[](gtsam::ShonanAveragingParameters2* self){return self->getAnchor();})
        .def("setAnchorWeight",[](gtsam::ShonanAveragingParameters2* self, double value){ self->setAnchorWeight(value);}, py::arg("value"))
        .def("getAnchorWeight",[](gtsam::ShonanAveragingParameters2* self){return self->getAnchorWeight();})
        .def("setKarcherWeight",[](gtsam::ShonanAveragingParameters2* self, double value){ self->setKarcherWeight(value);}, py::arg("value"))
        .def("getKarcherWeight",[](gtsam::ShonanAveragingParameters2* self){return self->getKarcherWeight();})
        .def("setGaugesWeight",[](gtsam::ShonanAveragingParameters2* self, double value){ self->setGaugesWeight(value);}, py::arg("value"))
        .def("getGaugesWeight",[](gtsam::ShonanAveragingParameters2* self){return self->getGaugesWeight();})
        .def("setUseHuber",[](gtsam::ShonanAveragingParameters2* self, bool value){ self->setUseHuber(value);}, py::arg("value"))
        .def("getUseHuber",[](gtsam::ShonanAveragingParameters2* self){return self->getUseHuber();})
        .def("setCertifyOptimality",[](gtsam::ShonanAveragingParameters2* self, bool value){ self->setCertifyOptimality(value);}, py::arg("value"))
        .def("getCertifyOptimality",[](gtsam::ShonanAveragingParameters2* self){return self->getCertifyOptimality();});

    py::class_<gtsam::ShonanAveragingParameters3, boost::shared_ptr<gtsam::ShonanAveragingParameters3>>(m_, "ShonanAveragingParameters3")
        .def(py::init<const gtsam::LevenbergMarquardtParams&>(), py::arg("lm"))
        .def(py::init<const gtsam::LevenbergMarquardtParams&, string>(), py::arg("lm"), py::arg("method"))
        .def("getLMParams",[](gtsam::ShonanAveragingParameters3* self){return self->getLMParams();})
        .def("setOptimalityThreshold",[](gtsam::ShonanAveragingParameters3* self, double value){ self->setOptimalityThreshold(value);}, py::arg("value"))
        .def("getOptimalityThreshold",[](gtsam::ShonanAveragingParameters3* self){return self->getOptimalityThreshold();})
        .def("setAnchor",[](gtsam::ShonanAveragingParameters3* self, size_t index, const gtsam::Rot3& value){ self->setAnchor(index, value);}, py::arg("index"), py::arg("value"))
        .def("getAnchor",[](gtsam::ShonanAveragingParameters3* self){return self->getAnchor();})
        .def("setAnchorWeight",[](gtsam::ShonanAveragingParameters3* self, double value){ self->setAnchorWeight(value);}, py::arg("value"))
        .def("getAnchorWeight",[](gtsam::ShonanAveragingParameters3* self){return self->getAnchorWeight();})
        .def("setKarcherWeight",[](gtsam::ShonanAveragingParameters3* self, double value){ self->setKarcherWeight(value);}, py::arg("value"))
        .def("getKarcherWeight",[](gtsam::ShonanAveragingParameters3* self){return self->getKarcherWeight();})
        .def("setGaugesWeight",[](gtsam::ShonanAveragingParameters3* self, double value){ self->setGaugesWeight(value);}, py::arg("value"))
        .def("getGaugesWeight",[](gtsam::ShonanAveragingParameters3* self){return self->getGaugesWeight();})
        .def("setUseHuber",[](gtsam::ShonanAveragingParameters3* self, bool value){ self->setUseHuber(value);}, py::arg("value"))
        .def("getUseHuber",[](gtsam::ShonanAveragingParameters3* self){return self->getUseHuber();})
        .def("setCertifyOptimality",[](gtsam::ShonanAveragingParameters3* self, bool value){ self->setCertifyOptimality(value);}, py::arg("value"))
        .def("getCertifyOptimality",[](gtsam::ShonanAveragingParameters3* self){return self->getCertifyOptimality();});

    py::class_<gtsam::ShonanAveraging2, boost::shared_ptr<gtsam::ShonanAveraging2>>(m_, "ShonanAveraging2")
        .def(py::init<string>(), py::arg("g2oFile"))
        .def(py::init<string, const gtsam::ShonanAveragingParameters2&>(), py::arg("g2oFile"), py::arg("parameters"))
        .def(py::init<const gtsam::BetweenFactorPose2s&, const gtsam::ShonanAveragingParameters2&>(), py::arg("factors"), py::arg("parameters"))
        .def("nrUnknowns",[](gtsam::ShonanAveraging2* self){return self->nrUnknowns();})
        .def("numberMeasurements",[](gtsam::ShonanAveraging2* self){return self->numberMeasurements();})
        .def("measured",[](gtsam::ShonanAveraging2* self, size_t i){return self->measured(i);}, py::arg("i"))
        .def("keys",[](gtsam::ShonanAveraging2* self, size_t i){return self->keys(i);}, py::arg("i"))
        .def("denseD",[](gtsam::ShonanAveraging2* self){return self->denseD();})
        .def("denseQ",[](gtsam::ShonanAveraging2* self){return self->denseQ();})
        .def("denseL",[](gtsam::ShonanAveraging2* self){return self->denseL();})
        .def("computeLambda_",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->computeLambda_(values);}, py::arg("values"))
        .def("computeA_",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->computeA_(values);}, py::arg("values"))
        .def("computeMinEigenValue",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->computeMinEigenValue(values);}, py::arg("values"))
        .def("initializeWithDescent",[](gtsam::ShonanAveraging2* self, size_t p, const gtsam::Values& values, const gtsam::Vector& minEigenVector, double minEigenValue){return self->initializeWithDescent(p, values, minEigenVector, minEigenValue);}, py::arg("p"), py::arg("values"), py::arg("minEigenVector"), py::arg("minEigenValue"))
        .def("buildGraphAt",[](gtsam::ShonanAveraging2* self, size_t p){return self->buildGraphAt(p);}, py::arg("p"))
        .def("initializeRandomlyAt",[](gtsam::ShonanAveraging2* self, size_t p){return self->initializeRandomlyAt(p);}, py::arg("p"))
        .def("costAt",[](gtsam::ShonanAveraging2* self, size_t p, const gtsam::Values& values){return self->costAt(p, values);}, py::arg("p"), py::arg("values"))
        .def("computeMinEigenVector",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->computeMinEigenVector(values);}, py::arg("values"))
        .def("checkOptimality",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->checkOptimality(values);}, py::arg("values"))
        .def("createOptimizerAt",[](gtsam::ShonanAveraging2* self, size_t p, const gtsam::Values& initial){return self->createOptimizerAt(p, initial);}, py::arg("p"), py::arg("initial"))
        .def("tryOptimizingAt",[](gtsam::ShonanAveraging2* self, size_t p, const gtsam::Values& initial){return self->tryOptimizingAt(p, initial);}, py::arg("p"), py::arg("initial"))
        .def("projectFrom",[](gtsam::ShonanAveraging2* self, size_t p, const gtsam::Values& values){return self->projectFrom(p, values);}, py::arg("p"), py::arg("values"))
        .def("roundSolution",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->roundSolution(values);}, py::arg("values"))
        .def("cost",[](gtsam::ShonanAveraging2* self, const gtsam::Values& values){return self->cost(values);}, py::arg("values"))
        .def("initializeRandomly",[](gtsam::ShonanAveraging2* self){return self->initializeRandomly();})
        .def("run",[](gtsam::ShonanAveraging2* self, const gtsam::Values& initial, size_t min_p, size_t max_p){return self->run(initial, min_p, max_p);}, py::arg("initial"), py::arg("min_p"), py::arg("max_p"));

    py::class_<gtsam::ShonanAveraging3, boost::shared_ptr<gtsam::ShonanAveraging3>>(m_, "ShonanAveraging3")
        .def(py::init<const std::vector<gtsam::BinaryMeasurement<gtsam::Rot3>>&, const gtsam::ShonanAveragingParameters3&>(), py::arg("measurements"), py::arg("parameters") = gtsam::ShonanAveragingParameters3())
        .def(py::init<string>(), py::arg("g2oFile"))
        .def(py::init<string, const gtsam::ShonanAveragingParameters3&>(), py::arg("g2oFile"), py::arg("parameters"))
        .def(py::init<const gtsam::BetweenFactorPose3s&>(), py::arg("factors"))
        .def(py::init<const gtsam::BetweenFactorPose3s&, const gtsam::ShonanAveragingParameters3&>(), py::arg("factors"), py::arg("parameters"))
        .def("nrUnknowns",[](gtsam::ShonanAveraging3* self){return self->nrUnknowns();})
        .def("numberMeasurements",[](gtsam::ShonanAveraging3* self){return self->numberMeasurements();})
        .def("measured",[](gtsam::ShonanAveraging3* self, size_t i){return self->measured(i);}, py::arg("i"))
        .def("keys",[](gtsam::ShonanAveraging3* self, size_t i){return self->keys(i);}, py::arg("i"))
        .def("denseD",[](gtsam::ShonanAveraging3* self){return self->denseD();})
        .def("denseQ",[](gtsam::ShonanAveraging3* self){return self->denseQ();})
        .def("denseL",[](gtsam::ShonanAveraging3* self){return self->denseL();})
        .def("computeLambda_",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->computeLambda_(values);}, py::arg("values"))
        .def("computeA_",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->computeA_(values);}, py::arg("values"))
        .def("computeMinEigenValue",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->computeMinEigenValue(values);}, py::arg("values"))
        .def("initializeWithDescent",[](gtsam::ShonanAveraging3* self, size_t p, const gtsam::Values& values, const gtsam::Vector& minEigenVector, double minEigenValue){return self->initializeWithDescent(p, values, minEigenVector, minEigenValue);}, py::arg("p"), py::arg("values"), py::arg("minEigenVector"), py::arg("minEigenValue"))
        .def("buildGraphAt",[](gtsam::ShonanAveraging3* self, size_t p){return self->buildGraphAt(p);}, py::arg("p"))
        .def("initializeRandomlyAt",[](gtsam::ShonanAveraging3* self, size_t p){return self->initializeRandomlyAt(p);}, py::arg("p"))
        .def("costAt",[](gtsam::ShonanAveraging3* self, size_t p, const gtsam::Values& values){return self->costAt(p, values);}, py::arg("p"), py::arg("values"))
        .def("computeMinEigenVector",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->computeMinEigenVector(values);}, py::arg("values"))
        .def("checkOptimality",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->checkOptimality(values);}, py::arg("values"))
        .def("createOptimizerAt",[](gtsam::ShonanAveraging3* self, size_t p, const gtsam::Values& initial){return self->createOptimizerAt(p, initial);}, py::arg("p"), py::arg("initial"))
        .def("tryOptimizingAt",[](gtsam::ShonanAveraging3* self, size_t p, const gtsam::Values& initial){return self->tryOptimizingAt(p, initial);}, py::arg("p"), py::arg("initial"))
        .def("projectFrom",[](gtsam::ShonanAveraging3* self, size_t p, const gtsam::Values& values){return self->projectFrom(p, values);}, py::arg("p"), py::arg("values"))
        .def("roundSolution",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->roundSolution(values);}, py::arg("values"))
        .def("cost",[](gtsam::ShonanAveraging3* self, const gtsam::Values& values){return self->cost(values);}, py::arg("values"))
        .def("initializeRandomly",[](gtsam::ShonanAveraging3* self){return self->initializeRandomly();})
        .def("run",[](gtsam::ShonanAveraging3* self, const gtsam::Values& initial, size_t min_p, size_t max_p){return self->run(initial, min_p, max_p);}, py::arg("initial"), py::arg("min_p"), py::arg("max_p"));

    py::class_<gtsam::MFAS, boost::shared_ptr<gtsam::MFAS>>(m_, "MFAS")
        .def(py::init<const gtsam::BinaryMeasurementsUnit3&, const gtsam::Unit3&>(), py::arg("relativeTranslations"), py::arg("projectionDirection"))
        .def("computeOutlierWeights",[](gtsam::MFAS* self){return self->computeOutlierWeights();})
        .def("computeOrdering",[](gtsam::MFAS* self){return self->computeOrdering();});

    py::class_<gtsam::TranslationRecovery, boost::shared_ptr<gtsam::TranslationRecovery>>(m_, "TranslationRecovery")
        .def(py::init<const gtsam::LevenbergMarquardtParams&>(), py::arg("lmParams"))
        .def(py::init<>())
        .def("addPrior",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3& betweenTranslations, gtsam::NonlinearFactorGraph* graph, const gtsam::SharedNoiseModel& priorNoiseModel){ self->addPrior(relativeTranslations, scale, betweenTranslations, graph, priorNoiseModel);}, py::arg("relativeTranslations"), py::arg("scale"), py::arg("betweenTranslations"), py::arg("graph"), py::arg("priorNoiseModel"))
        .def("addPrior",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3& betweenTranslations, gtsam::NonlinearFactorGraph* graph){ self->addPrior(relativeTranslations, scale, betweenTranslations, graph);}, py::arg("relativeTranslations"), py::arg("scale"), py::arg("betweenTranslations"), py::arg("graph"))
        .def("buildGraph",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations){return self->buildGraph(relativeTranslations);}, py::arg("relativeTranslations"))
        .def("run",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3& betweenTranslations, const gtsam::Values& initialValues){return self->run(relativeTranslations, scale, betweenTranslations, initialValues);}, py::arg("relativeTranslations"), py::arg("scale"), py::arg("betweenTranslations"), py::arg("initialValues"))
        .def("run",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations, const double scale, const gtsam::BinaryMeasurementsPoint3& betweenTranslations){return self->run(relativeTranslations, scale, betweenTranslations);}, py::arg("relativeTranslations"), py::arg("scale"), py::arg("betweenTranslations"))
        .def("run",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations, const double scale){return self->run(relativeTranslations, scale);}, py::arg("relativeTranslations"), py::arg("scale"))
        .def("run",[](gtsam::TranslationRecovery* self, const gtsam::BinaryMeasurementsUnit3& relativeTranslations){return self->run(relativeTranslations);}, py::arg("relativeTranslations"));
    pybind11::module m_gtsfm = m_.def_submodule("gtsfm", "gtsfm submodule");

    py::class_<gtsam::gtsfm::Keypoints, boost::shared_ptr<gtsam::gtsfm::Keypoints>>(m_gtsfm, "Keypoints")
        .def(py::init<const Eigen::MatrixX2d&>(), py::arg("coordinates"))
        .def_readwrite("coordinates", &gtsam::gtsfm::Keypoints::coordinates);

    m_gtsfm.def("tracksFromPairwiseMatches",[](const gtsam::gtsfm::MatchIndicesMap& matches_dict, const gtsam::gtsfm::KeypointsVector& keypoints_list, bool verbose){return gtsam::gtsfm::tracksFromPairwiseMatches(matches_dict, keypoints_list, verbose);}, py::arg("matches_dict"), py::arg("keypoints_list"), py::arg("verbose") = false);
    py::class_<gtsam::BinaryMeasurement<gtsam::Unit3>, boost::shared_ptr<gtsam::BinaryMeasurement<gtsam::Unit3>>>(m_, "BinaryMeasurementUnit3")
        .def(py::init<size_t, size_t, const gtsam::Unit3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("model"))
        .def("key1",[](gtsam::BinaryMeasurement<gtsam::Unit3>* self){return self->key1();})
        .def("key2",[](gtsam::BinaryMeasurement<gtsam::Unit3>* self){return self->key2();})
        .def("measured",[](gtsam::BinaryMeasurement<gtsam::Unit3>* self){return self->measured();})
        .def("noiseModel",[](gtsam::BinaryMeasurement<gtsam::Unit3>* self){return self->noiseModel();});

    py::class_<gtsam::BinaryMeasurement<gtsam::Rot3>, boost::shared_ptr<gtsam::BinaryMeasurement<gtsam::Rot3>>>(m_, "BinaryMeasurementRot3")
        .def(py::init<size_t, size_t, const gtsam::Rot3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("model"))
        .def("key1",[](gtsam::BinaryMeasurement<gtsam::Rot3>* self){return self->key1();})
        .def("key2",[](gtsam::BinaryMeasurement<gtsam::Rot3>* self){return self->key2();})
        .def("measured",[](gtsam::BinaryMeasurement<gtsam::Rot3>* self){return self->measured();})
        .def("noiseModel",[](gtsam::BinaryMeasurement<gtsam::Rot3>* self){return self->noiseModel();});

    py::class_<gtsam::BinaryMeasurement<gtsam::Point3>, boost::shared_ptr<gtsam::BinaryMeasurement<gtsam::Point3>>>(m_, "BinaryMeasurementPoint3")
        .def(py::init<size_t, size_t, const gtsam::Point3&, const boost::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("model"))
        .def("key1",[](gtsam::BinaryMeasurement<gtsam::Point3>* self){return self->key1();})
        .def("key2",[](gtsam::BinaryMeasurement<gtsam::Point3>* self){return self->key2();})
        .def("measured",[](gtsam::BinaryMeasurement<gtsam::Point3>* self){return self->measured();})
        .def("noiseModel",[](gtsam::BinaryMeasurement<gtsam::Point3>* self){return self->noiseModel();});

    m_.def("readBal",[](string filename){return gtsam::readBal(filename);}, py::arg("filename"));
    m_.def("writeBAL",[](string filename, gtsam::SfmData& data){return gtsam::writeBAL(filename, data);}, py::arg("filename"), py::arg("data"));
    m_.def("initialCamerasEstimate",[](const gtsam::SfmData& db){return gtsam::initialCamerasEstimate(db);}, py::arg("db"));
    m_.def("initialCamerasAndPointsEstimate",[](const gtsam::SfmData& db){return gtsam::initialCamerasAndPointsEstimate(db);}, py::arg("db"));

// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/sfm.h"

}

