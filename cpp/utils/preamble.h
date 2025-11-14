/*
These all come from the python/gtsam/preamble/ files.

They are to help make certain STL containers opaque to nanobind so that we can pass things by reference.
*/
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
#include <vector>

// base.h
NB_MAKE_OPAQUE(gtsam::IndexPairVector);
NB_MAKE_OPAQUE(gtsam::IndexPairSetMap);
// TODO: Not sure if this should be included...
NB_MAKE_OPAQUE(gtsam::IndexPairSet);
NB_MAKE_OPAQUE(std::vector<gtsam::Matrix>);

// basis.h

// custom.h

// discrete.h
NB_MAKE_OPAQUE(gtsam::DiscreteValues);

// geometry.h
NB_MAKE_OPAQUE(std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2>>);
NB_MAKE_OPAQUE(gtsam::Point2Pairs);
NB_MAKE_OPAQUE(gtsam::Point3Pairs);
NB_MAKE_OPAQUE(gtsam::Pose2Pairs);
NB_MAKE_OPAQUE(gtsam::Pose3Pairs);
NB_MAKE_OPAQUE(std::vector<gtsam::Pose3>);
NB_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>>);
NB_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3DS2>>);
NB_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler>>);
NB_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Unified>>);
NB_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>);

// gtsam.h
#ifdef GTSAM_ALLOCATOR_TBB
NB_MAKE_OPAQUE(std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key>>);
#else
NB_MAKE_OPAQUE(std::vector<gtsam::Key>);
#endif

// hybrid.h

// inference.h
NB_MAKE_OPAQUE(std::map<char, double>);

// linear.h

// navigation.h

// nonlinear.h

// sam.h

// sfm.h
NB_MAKE_OPAQUE(std::vector<gtsam::SfmMeasurement>);
NB_MAKE_OPAQUE(std::vector<gtsam::SfmTrack>);
NB_MAKE_OPAQUE(std::vector<gtsam::SfmCamera>);
NB_MAKE_OPAQUE(std::vector<gtsam::BinaryMeasurement<gtsam::Unit3>>);
NB_MAKE_OPAQUE(std::vector<gtsam::BinaryMeasurement<gtsam::Rot3>>);
NB_MAKE_OPAQUE(std::vector<gtsam::BinaryMeasurement<gtsam::Point3>>);
NB_MAKE_OPAQUE(std::vector<gtsam::gtsfm::Keypoints>);
NB_MAKE_OPAQUE(gtsam::gtsfm::MatchIndicesMap);
NB_MAKE_OPAQUE(gtsam::KeyPairDoubleMap);
NB_MAKE_OPAQUE(std::vector<gtsam::SfmTrack2d>);

// slam.h
NB_MAKE_OPAQUE(std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>>);
NB_MAKE_OPAQUE(std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>>);
NB_MAKE_OPAQUE(gtsam::Rot3Vector);

// symbolic.h
