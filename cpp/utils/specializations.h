#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/sfm/BinaryMeasurement.h>
#include <gtsam/sfm/DsfTrackGenerator.h>
#include <gtsam/sfm/MFAS.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/SfmTrack.h>

#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/base/DSFMap.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/bind_vector.h>
#include <vector>

namespace nb = nanobind;

inline void bind_specializations(nb::module_ &m_) {
  // base.h
  nb::bind_map<gtsam::IndexPairSetMap>(m_, "IndexPairSetMap");
  nb::bind_vector<gtsam::IndexPairVector>(m_, "IndexPairVector");
  nb::bind_vector<std::vector<gtsam::Matrix>>(m_, "JacobianVector");

  // basis.h

  // custom.h

  // discrete.h
  nb::bind_map<gtsam::DiscreteValues>(m_, "DiscreteValues");

  // geometry.h
  nb::bind_vector<std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2>>>(m_, "Point2Vector");
  nb::bind_vector<std::vector<gtsam::Point2Pair>>(m_, "Point2Pairs");
  nb::bind_vector<std::vector<gtsam::Point3Pair>>(m_, "Point3Pairs");
  nb::bind_vector<std::vector<gtsam::Pose2Pair>>(m_, "Pose2Pairs");
  nb::bind_vector<std::vector<gtsam::Pose3Pair>>(m_, "Pose3Pairs");
  nb::bind_vector<std::vector<gtsam::Pose3>>(m_, "Pose3Vector");
  nb::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>>>(m_, "CameraSetCal3_S2");
  nb::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3DS2>>>(m_, "CameraSetCal3DS2");
  nb::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>(m_, "CameraSetCal3Bundler");
  nb::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Unified>>>(m_, "CameraSetCal3Unified");
  nb::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>(m_, "CameraSetCal3Fisheye");

// gtsam.h
#ifdef GTSAM_ALLOCATOR_TBB
  nb::bind_vector<std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key>>>(m_, "KeyVector");
  nb::implicitly_convertible<nb::list, std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key>>>();
#else
  nb::bind_vector<std::vector<gtsam::Key>>(m_, "KeyVector");
  nb::implicitly_convertible<nb::list, std::vector<gtsam::Key>>();
#endif

  // hybrid.h

  // inference.h
  nb::bind_map<std::map<char, double>>(m_, "__MapCharDouble");

  // linear.h

  // navigation.h

  // nonlinear.h

  // sam.h

  // sfm.h
  nb::bind_vector<std::vector<gtsam::SfmMeasurement>>(m_, "SfmMeasurementVector");
  nb::bind_vector<std::vector<gtsam::SfmTrack>>(m_, "SfmTracks");
  nb::bind_vector<std::vector<gtsam::SfmCamera>>(m_, "SfmCameras");
  nb::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Unit3>>>(m_, "BinaryMeasurementsUnit3");
  nb::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Rot3>>>(m_, "BinaryMeasurementsRot3");
  nb::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Point3>>>(m_, "BinaryMeasurementsPoint3");
  nb::bind_vector<std::vector<gtsam::gtsfm::Keypoints>>(m_, "KeypointsVector");
  nb::bind_map<gtsam::gtsfm::MatchIndicesMap>(m_, "MatchIndicesMap");
  nb::bind_map<gtsam::KeyPairDoubleMap>(m_, "KeyPairDoubleMap");
  nb::bind_vector<std::vector<gtsam::SfmTrack2d>>(m_, "SfmTrack2dVector");

  // slam.h
  nb::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>>>(m_, "BetweenFactorPose3s");
  nb::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>>>(m_, "BetweenFactorPose2s");
  nb::bind_vector<gtsam::Rot3Vector>(m_, "Rot3Vector");

  // symbolic.h
}