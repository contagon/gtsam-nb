/**
 * @file    geometry.cpp
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
#include "gtsam/geometry/BearingRange.h"
#include "gtsam/geometry/Cal3Bundler.h"
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam/geometry/Cal3DS2_Base.h"
#include "gtsam/geometry/Cal3Fisheye.h"
#include "gtsam/geometry/Cal3Unified.h"
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/Cal3_S2Stereo.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/EssentialMatrix.h"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/PinholePose.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Quaternion.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/SO3.h"
#include "gtsam/geometry/SO4.h"
#include "gtsam/geometry/SOn.h"
#include "gtsam/geometry/Similarity2.h"
#include "gtsam/geometry/Similarity3.h"
#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/geometry/StereoCamera.h"
#include "gtsam/geometry/StereoPoint2.h"
#include "gtsam/geometry/Unit3.h"
#include "gtsam/geometry/triangulation.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::StereoPoint2)
BOOST_CLASS_EXPORT(gtsam::Rot2)
BOOST_CLASS_EXPORT(gtsam::SOn)
BOOST_CLASS_EXPORT(gtsam::Rot3)
BOOST_CLASS_EXPORT(gtsam::Pose2)
BOOST_CLASS_EXPORT(gtsam::Pose3)
BOOST_CLASS_EXPORT(gtsam::Unit3)
BOOST_CLASS_EXPORT(gtsam::Cal3_S2)
BOOST_CLASS_EXPORT(gtsam::Cal3DS2_Base)
BOOST_CLASS_EXPORT(gtsam::Cal3DS2)
BOOST_CLASS_EXPORT(gtsam::Cal3Unified)
BOOST_CLASS_EXPORT(gtsam::Cal3Fisheye)
BOOST_CLASS_EXPORT(gtsam::Cal3Bundler)
BOOST_CLASS_EXPORT(gtsam::CalibratedCamera)
BOOST_CLASS_EXPORT(gtsam::StereoCamera)
BOOST_CLASS_EXPORT(gtsam::PinholeCamera<gtsam::Cal3_S2>)
BOOST_CLASS_EXPORT(gtsam::PinholeCamera<gtsam::Cal3DS2>)
BOOST_CLASS_EXPORT(gtsam::PinholeCamera<gtsam::Cal3Unified>)
BOOST_CLASS_EXPORT(gtsam::PinholeCamera<gtsam::Cal3Bundler>)
BOOST_CLASS_EXPORT(gtsam::PinholeCamera<gtsam::Cal3Fisheye>)
BOOST_CLASS_EXPORT(gtsam::PinholePose<gtsam::Cal3_S2>)
BOOST_CLASS_EXPORT(gtsam::PinholePose<gtsam::Cal3DS2>)
BOOST_CLASS_EXPORT(gtsam::PinholePose<gtsam::Cal3Unified>)
BOOST_CLASS_EXPORT(gtsam::PinholePose<gtsam::Cal3Bundler>)
BOOST_CLASS_EXPORT(gtsam::PinholePose<gtsam::Cal3Fisheye>)

// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/geometry.h"

using namespace std;

namespace py = pybind11;

void geometry(py::module_ &m_) {
  m_.doc() = "pybind11 wrapper of geometry";

  py::class_<gtsam::StereoPoint2, boost::shared_ptr<gtsam::StereoPoint2>>(m_, "StereoPoint2")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("uL"), py::arg("uR"), py::arg("v"))
      .def("print", [](gtsam::StereoPoint2 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::StereoPoint2 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::StereoPoint2 *self, const gtsam::StereoPoint2 &point, double tol) { return self->equals(point, tol); }, py::arg("point"), py::arg("tol"))
      .def("inverse", [](gtsam::StereoPoint2 *self) { return self->inverse(); })
      .def("compose", [](gtsam::StereoPoint2 *self, const gtsam::StereoPoint2 &p2) { return self->compose(p2); }, py::arg("p2"))
      .def("between", [](gtsam::StereoPoint2 *self, const gtsam::StereoPoint2 &p2) { return self->between(p2); }, py::arg("p2"))
      .def("retract", [](gtsam::StereoPoint2 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::StereoPoint2 *self, const gtsam::StereoPoint2 &p) { return self->localCoordinates(p); }, py::arg("p"))
      .def("vector", [](gtsam::StereoPoint2 *self) { return self->vector(); })
      .def("uL", [](gtsam::StereoPoint2 *self) { return self->uL(); })
      .def("uR", [](gtsam::StereoPoint2 *self) { return self->uR(); })
      .def("v", [](gtsam::StereoPoint2 *self) { return self->v(); })
      .def("serialize", [](gtsam::StereoPoint2 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::StereoPoint2 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::StereoPoint2 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::StereoPoint2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Identity", []() { return gtsam::StereoPoint2::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::StereoPoint2::Expmap(v); }, py::arg("v"))
      .def_static("Logmap", [](const gtsam::StereoPoint2 &p) { return gtsam::StereoPoint2::Logmap(p); }, py::arg("p"))
      .def(-py::self)
      .def(py::self + py::self)
      .def(py::self - py::self);

  py::class_<gtsam::Rot2, boost::shared_ptr<gtsam::Rot2>>(m_, "Rot2")
      .def(py::init<>())
      .def(py::init<double>(), py::arg("theta"))
      .def("print", [](gtsam::Rot2 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "theta")
      .def("__repr__", [](const gtsam::Rot2 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "theta")
      .def("equals", [](gtsam::Rot2 *self, const gtsam::Rot2 &rot, double tol) { return self->equals(rot, tol); }, py::arg("rot"), py::arg("tol"))
      .def("inverse", [](gtsam::Rot2 *self) { return self->inverse(); })
      .def("compose", [](gtsam::Rot2 *self, const gtsam::Rot2 &p2) { return self->compose(p2); }, py::arg("p2"))
      .def("between", [](gtsam::Rot2 *self, const gtsam::Rot2 &p2) { return self->between(p2); }, py::arg("p2"))
      .def("retract", [](gtsam::Rot2 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Rot2 *self, const gtsam::Rot2 &p) { return self->localCoordinates(p); }, py::arg("p"))
      .def("logmap", [](gtsam::Rot2 *self, const gtsam::Rot2 &p) { return self->logmap(p); }, py::arg("p"))
      .def("rotate", [](gtsam::Rot2 *self, const gtsam::Point2 &point) { return self->rotate(point); }, py::arg("point"))
      .def("unrotate", [](gtsam::Rot2 *self, const gtsam::Point2 &point) { return self->unrotate(point); }, py::arg("point"))
      .def("theta", [](gtsam::Rot2 *self) { return self->theta(); })
      .def("degrees", [](gtsam::Rot2 *self) { return self->degrees(); })
      .def("c", [](gtsam::Rot2 *self) { return self->c(); })
      .def("s", [](gtsam::Rot2 *self) { return self->s(); })
      .def("matrix", [](gtsam::Rot2 *self) { return self->matrix(); })
      .def("serialize", [](gtsam::Rot2 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Rot2 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Rot2 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Rot2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("fromAngle", [](double theta) { return gtsam::Rot2::fromAngle(theta); }, py::arg("theta"))
      .def_static("fromDegrees", [](double theta) { return gtsam::Rot2::fromDegrees(theta); }, py::arg("theta"))
      .def_static("fromCosSin", [](double c, double s) { return gtsam::Rot2::fromCosSin(c, s); }, py::arg("c"), py::arg("s"))
      .def_static("Identity", []() { return gtsam::Rot2::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::Rot2::Expmap(v); }, py::arg("v"))
      .def_static("Logmap", [](const gtsam::Rot2 &p) { return gtsam::Rot2::Logmap(p); }, py::arg("p"))
      .def_static("relativeBearing", [](const gtsam::Point2 &d) { return gtsam::Rot2::relativeBearing(d); }, py::arg("d"))
      .def_static("atan2", [](double y, double x) { return gtsam::Rot2::atan2(y, x); }, py::arg("y"), py::arg("x"))
      .def(py::self * py::self);

  py::class_<gtsam::SO3, boost::shared_ptr<gtsam::SO3>>(m_, "SO3")
      .def(py::init<>())
      .def(py::init<const gtsam::Matrix &>(), py::arg("R"))
      .def("print", [](gtsam::SO3 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::SO3 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::SO3 *self, const gtsam::SO3 &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol"))
      .def("inverse", [](gtsam::SO3 *self) { return self->inverse(); })
      .def("between", [](gtsam::SO3 *self, const gtsam::SO3 &R) { return self->between(R); }, py::arg("R"))
      .def("compose", [](gtsam::SO3 *self, const gtsam::SO3 &R) { return self->compose(R); }, py::arg("R"))
      .def("retract", [](gtsam::SO3 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::SO3 *self, const gtsam::SO3 &R) { return self->localCoordinates(R); }, py::arg("R"))
      .def("vec", [](gtsam::SO3 *self) { return self->vec(); })
      .def("matrix", [](gtsam::SO3 *self) { return self->matrix(); })
      .def_static("FromMatrix", [](const gtsam::Matrix &R) { return gtsam::SO3::FromMatrix(R); }, py::arg("R"))
      .def_static("AxisAngle", [](const gtsam::Vector &axis, double theta) { return gtsam::SO3::AxisAngle(axis, theta); }, py::arg("axis"), py::arg("theta"))
      .def_static("ClosestTo", [](const gtsam::Matrix &M) { return gtsam::SO3::ClosestTo(M); }, py::arg("M"))
      .def_static("Identity", []() { return gtsam::SO3::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::SO3::Expmap(v); }, py::arg("v"))
      .def(py::self * py::self);

  py::class_<gtsam::SO4, boost::shared_ptr<gtsam::SO4>>(m_, "SO4")
      .def(py::init<>())
      .def(py::init<const gtsam::Matrix &>(), py::arg("R"))
      .def("print", [](gtsam::SO4 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::SO4 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::SO4 *self, const gtsam::SO4 &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol"))
      .def("inverse", [](gtsam::SO4 *self) { return self->inverse(); })
      .def("between", [](gtsam::SO4 *self, const gtsam::SO4 &Q) { return self->between(Q); }, py::arg("Q"))
      .def("compose", [](gtsam::SO4 *self, const gtsam::SO4 &Q) { return self->compose(Q); }, py::arg("Q"))
      .def("retract", [](gtsam::SO4 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::SO4 *self, const gtsam::SO4 &Q) { return self->localCoordinates(Q); }, py::arg("Q"))
      .def("vec", [](gtsam::SO4 *self) { return self->vec(); })
      .def("matrix", [](gtsam::SO4 *self) { return self->matrix(); })
      .def_static("FromMatrix", [](const gtsam::Matrix &R) { return gtsam::SO4::FromMatrix(R); }, py::arg("R"))
      .def_static("Identity", []() { return gtsam::SO4::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::SO4::Expmap(v); }, py::arg("v"))
      .def(py::self * py::self);

  py::class_<gtsam::SOn, boost::shared_ptr<gtsam::SOn>>(m_, "SOn")
      .def(py::init<size_t>(), py::arg("n"))
      .def("print", [](gtsam::SOn *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::SOn &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::SOn *self, const gtsam::SOn &other, double tol) { return self->equals(other, tol); }, py::arg("other"), py::arg("tol"))
      .def("inverse", [](gtsam::SOn *self) { return self->inverse(); })
      .def("between", [](gtsam::SOn *self, const gtsam::SOn &Q) { return self->between(Q); }, py::arg("Q"))
      .def("compose", [](gtsam::SOn *self, const gtsam::SOn &Q) { return self->compose(Q); }, py::arg("Q"))
      .def("retract", [](gtsam::SOn *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::SOn *self, const gtsam::SOn &Q) { return self->localCoordinates(Q); }, py::arg("Q"))
      .def("vec", [](gtsam::SOn *self) { return self->vec(); })
      .def("matrix", [](gtsam::SOn *self) { return self->matrix(); })
      .def("serialize", [](gtsam::SOn *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::SOn *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::SOn &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::SOn obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("FromMatrix", [](const gtsam::Matrix &R) { return gtsam::SOn::FromMatrix(R); }, py::arg("R"))
      .def_static("Lift", [](size_t n, const gtsam::Matrix &R) { return gtsam::SOn::Lift(n, R); }, py::arg("n"), py::arg("R"))
      .def_static("Identity", []() { return gtsam::SOn::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::SOn::Expmap(v); }, py::arg("v"))
      .def(py::self * py::self);

  py::class_<gtsam::Quaternion, boost::shared_ptr<gtsam::Quaternion>>(m_, "Quaternion")
      .def("w", [](gtsam::Quaternion *self) { return self->w(); })
      .def("x", [](gtsam::Quaternion *self) { return self->x(); })
      .def("y", [](gtsam::Quaternion *self) { return self->y(); })
      .def("z", [](gtsam::Quaternion *self) { return self->z(); })
      .def("coeffs", [](gtsam::Quaternion *self) { return self->coeffs(); });

  py::class_<gtsam::Rot3, boost::shared_ptr<gtsam::Rot3>>(m_, "Rot3")
      .def(py::init<>())
      .def(py::init<const gtsam::Matrix &>(), py::arg("R"))
      .def(py::init<const gtsam::Point3 &, const gtsam::Point3 &, const gtsam::Point3 &>(), py::arg("col1"), py::arg("col2"), py::arg("col3"))
      .def(py::init<double, double, double, double, double, double, double, double, double>(), py::arg("R11"), py::arg("R12"), py::arg("R13"), py::arg("R21"), py::arg("R22"), py::arg("R23"), py::arg("R31"), py::arg("R32"), py::arg("R33"))
      .def(py::init<double, double, double, double>(), py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
      .def("print", [](gtsam::Rot3 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Rot3 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Rot3 *self, const gtsam::Rot3 &rot, double tol) { return self->equals(rot, tol); }, py::arg("rot"), py::arg("tol"))
      .def("inverse", [](gtsam::Rot3 *self) { return self->inverse(); })
      .def("compose", [](gtsam::Rot3 *self, const gtsam::Rot3 &p2) { return self->compose(p2); }, py::arg("p2"))
      .def("between", [](gtsam::Rot3 *self, const gtsam::Rot3 &p2) { return self->between(p2); }, py::arg("p2"))
      .def("retract", [](gtsam::Rot3 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Rot3 *self, const gtsam::Rot3 &p) { return self->localCoordinates(p); }, py::arg("p"))
      .def("rotate", [](gtsam::Rot3 *self, const gtsam::Point3 &p) { return self->rotate(p); }, py::arg("p"))
      .def("unrotate", [](gtsam::Rot3 *self, const gtsam::Point3 &p) { return self->unrotate(p); }, py::arg("p"))
      .def("rotate", [](gtsam::Rot3 *self, const gtsam::Unit3 &p) { return self->rotate(p); }, py::arg("p"))
      .def("unrotate", [](gtsam::Rot3 *self, const gtsam::Unit3 &p) { return self->unrotate(p); }, py::arg("p"))
      .def("logmap", [](gtsam::Rot3 *self, const gtsam::Rot3 &p) { return self->logmap(p); }, py::arg("p"))
      .def("matrix", [](gtsam::Rot3 *self) { return self->matrix(); })
      .def("transpose", [](gtsam::Rot3 *self) { return self->transpose(); })
      .def("column", [](gtsam::Rot3 *self, size_t index) { return self->column(index); }, py::arg("index"))
      .def("xyz", [](gtsam::Rot3 *self) { return self->xyz(); })
      .def("ypr", [](gtsam::Rot3 *self) { return self->ypr(); })
      .def("rpy", [](gtsam::Rot3 *self) { return self->rpy(); })
      .def("roll", [](gtsam::Rot3 *self) { return self->roll(); })
      .def("pitch", [](gtsam::Rot3 *self) { return self->pitch(); })
      .def("yaw", [](gtsam::Rot3 *self) { return self->yaw(); })
      .def("axisAngle", [](gtsam::Rot3 *self) { return self->axisAngle(); })
      .def("toQuaternion", [](gtsam::Rot3 *self) { return self->toQuaternion(); })
      .def("slerp", [](gtsam::Rot3 *self, double t, const gtsam::Rot3 &other) { return self->slerp(t, other); }, py::arg("t"), py::arg("other"))
      .def("serialize", [](gtsam::Rot3 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Rot3 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Rot3 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Rot3 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Rx", [](double t) { return gtsam::Rot3::Rx(t); }, py::arg("t"))
      .def_static("Ry", [](double t) { return gtsam::Rot3::Ry(t); }, py::arg("t"))
      .def_static("Rz", [](double t) { return gtsam::Rot3::Rz(t); }, py::arg("t"))
      .def_static("RzRyRx", [](double x, double y, double z) { return gtsam::Rot3::RzRyRx(x, y, z); }, py::arg("x"), py::arg("y"), py::arg("z"))
      .def_static("RzRyRx", [](const gtsam::Vector &xyz) { return gtsam::Rot3::RzRyRx(xyz); }, py::arg("xyz"))
      .def_static("Yaw", [](double t) { return gtsam::Rot3::Yaw(t); }, py::arg("t"))
      .def_static("Pitch", [](double t) { return gtsam::Rot3::Pitch(t); }, py::arg("t"))
      .def_static("Roll", [](double t) { return gtsam::Rot3::Roll(t); }, py::arg("t"))
      .def_static("Ypr", [](double y, double p, double r) { return gtsam::Rot3::Ypr(y, p, r); }, py::arg("y"), py::arg("p"), py::arg("r"))
      .def_static("Quaternion", [](double w, double x, double y, double z) { return gtsam::Rot3::Quaternion(w, x, y, z); }, py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
      .def_static("AxisAngle", [](const gtsam::Point3 &axis, double angle) { return gtsam::Rot3::AxisAngle(axis, angle); }, py::arg("axis"), py::arg("angle"))
      .def_static("Rodrigues", [](const gtsam::Vector &v) { return gtsam::Rot3::Rodrigues(v); }, py::arg("v"))
      .def_static("Rodrigues", [](double wx, double wy, double wz) { return gtsam::Rot3::Rodrigues(wx, wy, wz); }, py::arg("wx"), py::arg("wy"), py::arg("wz"))
      .def_static("ClosestTo", [](const gtsam::Matrix &M) { return gtsam::Rot3::ClosestTo(M); }, py::arg("M"))
      .def_static("Identity", []() { return gtsam::Rot3::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::Rot3::Expmap(v); }, py::arg("v"))
      .def_static("Logmap", [](const gtsam::Rot3 &p) { return gtsam::Rot3::Logmap(p); }, py::arg("p"))
      .def(py::self * py::self);

  py::class_<gtsam::Pose2, boost::shared_ptr<gtsam::Pose2>>(m_, "Pose2")
      .def(py::init<>())
      .def(py::init<const gtsam::Pose2 &>(), py::arg("other"))
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("theta"))
      .def(py::init<double, const gtsam::Point2 &>(), py::arg("theta"), py::arg("t"))
      .def(py::init<const gtsam::Rot2 &, const gtsam::Point2 &>(), py::arg("r"), py::arg("t"))
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def("print", [](gtsam::Pose2 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Pose2 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Pose2 *self, const gtsam::Pose2 &pose, double tol) { return self->equals(pose, tol); }, py::arg("pose"), py::arg("tol"))
      .def("inverse", [](gtsam::Pose2 *self) { return self->inverse(); })
      .def("compose", [](gtsam::Pose2 *self, const gtsam::Pose2 &p2) { return self->compose(p2); }, py::arg("p2"))
      .def("between", [](gtsam::Pose2 *self, const gtsam::Pose2 &p2) { return self->between(p2); }, py::arg("p2"))
      .def("retract", [](gtsam::Pose2 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Pose2 *self, const gtsam::Pose2 &p) { return self->localCoordinates(p); }, py::arg("p"))
      .def("logmap", [](gtsam::Pose2 *self, const gtsam::Pose2 &p) { return self->logmap(p); }, py::arg("p"))
      .def("AdjointMap", [](gtsam::Pose2 *self) { return self->AdjointMap(); })
      .def("Adjoint", [](gtsam::Pose2 *self, const gtsam::Vector &xi) { return self->Adjoint(xi); }, py::arg("xi"))
      .def("transformFrom", [](gtsam::Pose2 *self, const gtsam::Point2 &p) { return self->transformFrom(p); }, py::arg("p"))
      .def("transformTo", [](gtsam::Pose2 *self, const gtsam::Point2 &p) { return self->transformTo(p); }, py::arg("p"))
      .def("transformFrom", [](gtsam::Pose2 *self, const gtsam::Matrix &points) { return self->transformFrom(points); }, py::arg("points"))
      .def("transformTo", [](gtsam::Pose2 *self, const gtsam::Matrix &points) { return self->transformTo(points); }, py::arg("points"))
      .def("x", [](gtsam::Pose2 *self) { return self->x(); })
      .def("y", [](gtsam::Pose2 *self) { return self->y(); })
      .def("theta", [](gtsam::Pose2 *self) { return self->theta(); })
      .def("bearing", [](gtsam::Pose2 *self, const gtsam::Point2 &point) { return self->bearing(point); }, py::arg("point"))
      .def("range", [](gtsam::Pose2 *self, const gtsam::Point2 &point) { return self->range(point); }, py::arg("point"))
      .def("translation", [](gtsam::Pose2 *self) { return self->translation(); })
      .def("rotation", [](gtsam::Pose2 *self) { return self->rotation(); })
      .def("matrix", [](gtsam::Pose2 *self) { return self->matrix(); })
      .def("serialize", [](gtsam::Pose2 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Pose2 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Pose2 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Pose2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Align", [](const gtsam::Point2Pairs &abPointPairs) { return gtsam::Pose2::Align(abPointPairs); }, py::arg("abPointPairs"))
      .def_static("Align", [](const gtsam::Matrix &a, const gtsam::Matrix &b) { return gtsam::Pose2::Align(a, b); }, py::arg("a"), py::arg("b"))
      .def_static("Identity", []() { return gtsam::Pose2::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::Pose2::Expmap(v); }, py::arg("v"))
      .def_static("Logmap", [](const gtsam::Pose2 &p) { return gtsam::Pose2::Logmap(p); }, py::arg("p"))
      .def_static("ExpmapDerivative", [](const gtsam::Vector &v) { return gtsam::Pose2::ExpmapDerivative(v); }, py::arg("v"))
      .def_static("LogmapDerivative", [](const gtsam::Pose2 &v) { return gtsam::Pose2::LogmapDerivative(v); }, py::arg("v"))
      .def_static("adjointMap_", [](const gtsam::Vector &v) { return gtsam::Pose2::adjointMap_(v); }, py::arg("v"))
      .def_static("adjoint_", [](const gtsam::Vector &xi, const gtsam::Vector &y) { return gtsam::Pose2::adjoint_(xi, y); }, py::arg("xi"), py::arg("y"))
      .def_static("adjointTranspose", [](const gtsam::Vector &xi, const gtsam::Vector &y) { return gtsam::Pose2::adjointTranspose(xi, y); }, py::arg("xi"), py::arg("y"))
      .def_static("wedge", [](double vx, double vy, double w) { return gtsam::Pose2::wedge(vx, vy, w); }, py::arg("vx"), py::arg("vy"), py::arg("w"))
      .def(py::self * py::self);

  py::class_<gtsam::Pose3, boost::shared_ptr<gtsam::Pose3>>(m_, "Pose3")
      .def(py::init<>())
      .def(py::init<const gtsam::Pose3 &>(), py::arg("other"))
      .def(py::init<const gtsam::Rot3 &, const gtsam::Point3 &>(), py::arg("r"), py::arg("t"))
      .def(py::init<const gtsam::Pose2 &>(), py::arg("pose2"))
      .def(py::init<const gtsam::Matrix &>(), py::arg("mat"))
      .def("print", [](gtsam::Pose3 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Pose3 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, double tol) { return self->equals(pose, tol); }, py::arg("pose"), py::arg("tol"))
      .def("inverse", [](gtsam::Pose3 *self) { return self->inverse(); })
      .def("inverse", [](gtsam::Pose3 *self, Eigen::Ref<Eigen::MatrixXd> H) { return self->inverse(H); }, py::arg("H"))
      .def("compose", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->compose(pose); }, py::arg("pose"))
      .def("compose", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) { return self->compose(pose, H1, H2); }, py::arg("pose"), py::arg("H1"), py::arg("H2"))
      .def("between", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->between(pose); }, py::arg("pose"))
      .def("between", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) { return self->between(pose, H1, H2); }, py::arg("pose"), py::arg("H1"), py::arg("H2"))
      .def("slerp", [](gtsam::Pose3 *self, double t, const gtsam::Pose3 &pose) { return self->slerp(t, pose); }, py::arg("t"), py::arg("pose"))
      .def("slerp", [](gtsam::Pose3 *self, double t, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Hx, Eigen::Ref<Eigen::MatrixXd> Hy) { return self->slerp(t, pose, Hx, Hy); }, py::arg("t"), py::arg("pose"), py::arg("Hx"), py::arg("Hy"))
      .def("retract", [](gtsam::Pose3 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("retract", [](gtsam::Pose3 *self, const gtsam::Vector &v, Eigen::Ref<Eigen::MatrixXd> Hxi) { return self->retract(v, Hxi); }, py::arg("v"), py::arg("Hxi"))
      .def("localCoordinates", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->localCoordinates(pose); }, py::arg("pose"))
      .def("localCoordinates", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Hxi) { return self->localCoordinates(pose, Hxi); }, py::arg("pose"), py::arg("Hxi"))
      .def("expmap", [](gtsam::Pose3 *self, const gtsam::Vector &v) { return self->expmap(v); }, py::arg("v"))
      .def("logmap", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->logmap(pose); }, py::arg("pose"))
      .def("AdjointMap", [](gtsam::Pose3 *self) { return self->AdjointMap(); })
      .def("Adjoint", [](gtsam::Pose3 *self, const gtsam::Vector &xi) { return self->Adjoint(xi); }, py::arg("xi"))
      .def("Adjoint", [](gtsam::Pose3 *self, const gtsam::Vector &xi, Eigen::Ref<Eigen::MatrixXd> H_this, Eigen::Ref<Eigen::MatrixXd> H_xib) { return self->Adjoint(xi, H_this, H_xib); }, py::arg("xi"), py::arg("H_this"), py::arg("H_xib"))
      .def("AdjointTranspose", [](gtsam::Pose3 *self, const gtsam::Vector &xi) { return self->AdjointTranspose(xi); }, py::arg("xi"))
      .def("AdjointTranspose", [](gtsam::Pose3 *self, const gtsam::Vector &xi, Eigen::Ref<Eigen::MatrixXd> H_this, Eigen::Ref<Eigen::MatrixXd> H_x) { return self->AdjointTranspose(xi, H_this, H_x); }, py::arg("xi"), py::arg("H_this"), py::arg("H_x"))
      .def("transformFrom", [](gtsam::Pose3 *self, const gtsam::Point3 &point) { return self->transformFrom(point); }, py::arg("point"))
      .def("transformFrom", [](gtsam::Pose3 *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Hself, Eigen::Ref<Eigen::MatrixXd> Hpoint) { return self->transformFrom(point, Hself, Hpoint); }, py::arg("point"), py::arg("Hself"), py::arg("Hpoint"))
      .def("transformTo", [](gtsam::Pose3 *self, const gtsam::Point3 &point) { return self->transformTo(point); }, py::arg("point"))
      .def("transformTo", [](gtsam::Pose3 *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Hself, Eigen::Ref<Eigen::MatrixXd> Hpoint) { return self->transformTo(point, Hself, Hpoint); }, py::arg("point"), py::arg("Hself"), py::arg("Hpoint"))
      .def("transformFrom", [](gtsam::Pose3 *self, const gtsam::Matrix &points) { return self->transformFrom(points); }, py::arg("points"))
      .def("transformTo", [](gtsam::Pose3 *self, const gtsam::Matrix &points) { return self->transformTo(points); }, py::arg("points"))
      .def("rotation", [](gtsam::Pose3 *self) { return self->rotation(); })
      .def("rotation", [](gtsam::Pose3 *self, Eigen::Ref<Eigen::MatrixXd> Hself) { return self->rotation(Hself); }, py::arg("Hself"))
      .def("translation", [](gtsam::Pose3 *self) { return self->translation(); })
      .def("translation", [](gtsam::Pose3 *self, Eigen::Ref<Eigen::MatrixXd> Hself) { return self->translation(Hself); }, py::arg("Hself"))
      .def("x", [](gtsam::Pose3 *self) { return self->x(); })
      .def("y", [](gtsam::Pose3 *self) { return self->y(); })
      .def("z", [](gtsam::Pose3 *self) { return self->z(); })
      .def("matrix", [](gtsam::Pose3 *self) { return self->matrix(); })
      .def("transformPoseFrom", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->transformPoseFrom(pose); }, py::arg("pose"))
      .def("transformPoseFrom", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Hself, Eigen::Ref<Eigen::MatrixXd> HaTb) { return self->transformPoseFrom(pose, Hself, HaTb); }, py::arg("pose"), py::arg("Hself"), py::arg("HaTb"))
      .def("transformPoseTo", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->transformPoseTo(pose); }, py::arg("pose"))
      .def("transformPoseTo", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Hself, Eigen::Ref<Eigen::MatrixXd> HwTb) { return self->transformPoseTo(pose, Hself, HwTb); }, py::arg("pose"), py::arg("Hself"), py::arg("HwTb"))
      .def("range", [](gtsam::Pose3 *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::Pose3 *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Hself, Eigen::Ref<Eigen::MatrixXd> Hpoint) { return self->range(point, Hself, Hpoint); }, py::arg("point"), py::arg("Hself"), py::arg("Hpoint"))
      .def("range", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::Pose3 *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Hself, Eigen::Ref<Eigen::MatrixXd> Hpose) { return self->range(pose, Hself, Hpose); }, py::arg("pose"), py::arg("Hself"), py::arg("Hpose"))
      .def("serialize", [](gtsam::Pose3 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Pose3 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Pose3 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Pose3 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Align", [](const gtsam::Point3Pairs &abPointPairs) { return gtsam::Pose3::Align(abPointPairs); }, py::arg("abPointPairs"))
      .def_static("Align", [](const gtsam::Matrix &a, const gtsam::Matrix &b) { return gtsam::Pose3::Align(a, b); }, py::arg("a"), py::arg("b"))
      .def_static("Identity", []() { return gtsam::Pose3::Identity(); })
      .def_static("Expmap", [](const gtsam::Vector &v) { return gtsam::Pose3::Expmap(v); }, py::arg("v"))
      .def_static("Expmap", [](const gtsam::Vector &v, Eigen::Ref<Eigen::MatrixXd> Hxi) { return gtsam::Pose3::Expmap(v, Hxi); }, py::arg("v"), py::arg("Hxi"))
      .def_static("Logmap", [](const gtsam::Pose3 &pose) { return gtsam::Pose3::Logmap(pose); }, py::arg("pose"))
      .def_static("Logmap", [](const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Hpose) { return gtsam::Pose3::Logmap(pose, Hpose); }, py::arg("pose"), py::arg("Hpose"))
      .def_static("adjointMap", [](const gtsam::Vector &xi) { return gtsam::Pose3::adjointMap(xi); }, py::arg("xi"))
      .def_static("adjoint", [](const gtsam::Vector &xi, const gtsam::Vector &y) { return gtsam::Pose3::adjoint(xi, y); }, py::arg("xi"), py::arg("y"))
      .def_static("adjointMap_", [](const gtsam::Vector &xi) { return gtsam::Pose3::adjointMap_(xi); }, py::arg("xi"))
      .def_static("adjoint_", [](const gtsam::Vector &xi, const gtsam::Vector &y) { return gtsam::Pose3::adjoint_(xi, y); }, py::arg("xi"), py::arg("y"))
      .def_static("adjointTranspose", [](const gtsam::Vector &xi, const gtsam::Vector &y) { return gtsam::Pose3::adjointTranspose(xi, y); }, py::arg("xi"), py::arg("y"))
      .def_static("ExpmapDerivative", [](const gtsam::Vector &xi) { return gtsam::Pose3::ExpmapDerivative(xi); }, py::arg("xi"))
      .def_static("LogmapDerivative", [](const gtsam::Pose3 &xi) { return gtsam::Pose3::LogmapDerivative(xi); }, py::arg("xi"))
      .def_static("wedge", [](double wx, double wy, double wz, double vx, double vy, double vz) { return gtsam::Pose3::wedge(wx, wy, wz, vx, vy, vz); }, py::arg("wx"), py::arg("wy"), py::arg("wz"), py::arg("vx"), py::arg("vy"), py::arg("vz"))
      .def(py::self * py::self);

  py::class_<gtsam::Unit3, boost::shared_ptr<gtsam::Unit3>>(m_, "Unit3")
      .def(py::init<>())
      .def(py::init<const gtsam::Point3 &>(), py::arg("pose"))
      .def("print", [](gtsam::Unit3 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Unit3 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Unit3 *self, const gtsam::Unit3 &pose, double tol) { return self->equals(pose, tol); }, py::arg("pose"), py::arg("tol"))
      .def("basis", [](gtsam::Unit3 *self) { return self->basis(); })
      .def("skew", [](gtsam::Unit3 *self) { return self->skew(); })
      .def("point3", [](gtsam::Unit3 *self) { return self->point3(); })
      .def("dim", [](gtsam::Unit3 *self) { return self->dim(); })
      .def("retract", [](gtsam::Unit3 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Unit3 *self, const gtsam::Unit3 &s) { return self->localCoordinates(s); }, py::arg("s"))
      .def("serialize", [](gtsam::Unit3 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Unit3 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Unit3 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Unit3 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def("equals", [](gtsam::Unit3 *self, const gtsam::Unit3 &expected, double tol) { return self->equals(expected, tol); }, py::arg("expected"), py::arg("tol"))
      .def_static("Dim", []() { return gtsam::Unit3::Dim(); });

  py::class_<gtsam::EssentialMatrix, boost::shared_ptr<gtsam::EssentialMatrix>>(m_, "EssentialMatrix")
      .def(py::init<const gtsam::Rot3 &, const gtsam::Unit3 &>(), py::arg("aRb"), py::arg("aTb"))
      .def("FromPose3", [](gtsam::EssentialMatrix *self, const gtsam::Pose3 &_1P2_) { return self->FromPose3(_1P2_); }, py::arg("_1P2_"))
      .def("FromPose3", [](gtsam::EssentialMatrix *self, const gtsam::Pose3 &_1P2_, Eigen::Ref<Eigen::MatrixXd> H) { return self->FromPose3(_1P2_, H); }, py::arg("_1P2_"), py::arg("H"))
      .def("print", [](gtsam::EssentialMatrix *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::EssentialMatrix &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::EssentialMatrix *self, const gtsam::EssentialMatrix &pose, double tol) { return self->equals(pose, tol); }, py::arg("pose"), py::arg("tol"))
      .def("dim", [](gtsam::EssentialMatrix *self) { return self->dim(); })
      .def("retract", [](gtsam::EssentialMatrix *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::EssentialMatrix *self, const gtsam::EssentialMatrix &s) { return self->localCoordinates(s); }, py::arg("s"))
      .def("rotation", [](gtsam::EssentialMatrix *self) { return self->rotation(); })
      .def("direction", [](gtsam::EssentialMatrix *self) { return self->direction(); })
      .def("matrix", [](gtsam::EssentialMatrix *self) { return self->matrix(); })
      .def("error", [](gtsam::EssentialMatrix *self, const gtsam::Vector &vA, const gtsam::Vector &vB) { return self->error(vA, vB); }, py::arg("vA"), py::arg("vB"))
      .def_static("Dim", []() { return gtsam::EssentialMatrix::Dim(); });

  py::class_<gtsam::Cal3_S2, boost::shared_ptr<gtsam::Cal3_S2>>(m_, "Cal3_S2")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"))
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def(py::init<double, int, int>(), py::arg("fov"), py::arg("w"), py::arg("h"))
      .def("print", [](gtsam::Cal3_S2 *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "Cal3_S2")
      .def("__repr__", [](const gtsam::Cal3_S2 &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "Cal3_S2")
      .def("equals", [](gtsam::Cal3_S2 *self, const gtsam::Cal3_S2 &rhs, double tol) { return self->equals(rhs, tol); }, py::arg("rhs"), py::arg("tol"))
      .def("dim", [](gtsam::Cal3_S2 *self) { return self->dim(); })
      .def("retract", [](gtsam::Cal3_S2 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Cal3_S2 *self, const gtsam::Cal3_S2 &c) { return self->localCoordinates(c); }, py::arg("c"))
      .def("calibrate", [](gtsam::Cal3_S2 *self, const gtsam::Point2 &p) { return self->calibrate(p); }, py::arg("p"))
      .def("calibrate", [](gtsam::Cal3_S2 *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->calibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("uncalibrate", [](gtsam::Cal3_S2 *self, const gtsam::Point2 &p) { return self->uncalibrate(p); }, py::arg("p"))
      .def("uncalibrate", [](gtsam::Cal3_S2 *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->uncalibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("fx", [](gtsam::Cal3_S2 *self) { return self->fx(); })
      .def("fy", [](gtsam::Cal3_S2 *self) { return self->fy(); })
      .def("skew", [](gtsam::Cal3_S2 *self) { return self->skew(); })
      .def("px", [](gtsam::Cal3_S2 *self) { return self->px(); })
      .def("py", [](gtsam::Cal3_S2 *self) { return self->py(); })
      .def("principalPoint", [](gtsam::Cal3_S2 *self) { return self->principalPoint(); })
      .def("vector", [](gtsam::Cal3_S2 *self) { return self->vector(); })
      .def("K", [](gtsam::Cal3_S2 *self) { return self->K(); })
      .def("inverse", [](gtsam::Cal3_S2 *self) { return self->inverse(); })
      .def("serialize", [](gtsam::Cal3_S2 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Cal3_S2 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Cal3_S2 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Cal3_S2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Dim", []() { return gtsam::Cal3_S2::Dim(); });

  py::class_<gtsam::Cal3DS2_Base, boost::shared_ptr<gtsam::Cal3DS2_Base>>(m_, "Cal3DS2_Base")
      .def(py::init<>())
      .def("print", [](gtsam::Cal3DS2_Base *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Cal3DS2_Base &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("fx", [](gtsam::Cal3DS2_Base *self) { return self->fx(); })
      .def("fy", [](gtsam::Cal3DS2_Base *self) { return self->fy(); })
      .def("skew", [](gtsam::Cal3DS2_Base *self) { return self->skew(); })
      .def("px", [](gtsam::Cal3DS2_Base *self) { return self->px(); })
      .def("py", [](gtsam::Cal3DS2_Base *self) { return self->py(); })
      .def("k1", [](gtsam::Cal3DS2_Base *self) { return self->k1(); })
      .def("k2", [](gtsam::Cal3DS2_Base *self) { return self->k2(); })
      .def("K", [](gtsam::Cal3DS2_Base *self) { return self->K(); })
      .def("k", [](gtsam::Cal3DS2_Base *self) { return self->k(); })
      .def("vector", [](gtsam::Cal3DS2_Base *self) { return self->vector(); })
      .def("uncalibrate", [](gtsam::Cal3DS2_Base *self, const gtsam::Point2 &p) { return self->uncalibrate(p); }, py::arg("p"))
      .def("uncalibrate", [](gtsam::Cal3DS2_Base *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->uncalibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("calibrate", [](gtsam::Cal3DS2_Base *self, const gtsam::Point2 &p) { return self->calibrate(p); }, py::arg("p"))
      .def("calibrate", [](gtsam::Cal3DS2_Base *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->calibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("serialize", [](gtsam::Cal3DS2_Base *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Cal3DS2_Base *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Cal3DS2_Base &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Cal3DS2_Base obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  py::class_<gtsam::Cal3DS2, gtsam::Cal3DS2_Base, boost::shared_ptr<gtsam::Cal3DS2>>(m_, "Cal3DS2")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"))
      .def(py::init<double, double, double, double, double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"))
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def("equals", [](gtsam::Cal3DS2 *self, const gtsam::Cal3DS2 &rhs, double tol) { return self->equals(rhs, tol); }, py::arg("rhs"), py::arg("tol"))
      .def("dim", [](gtsam::Cal3DS2 *self) { return self->dim(); })
      .def("retract", [](gtsam::Cal3DS2 *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Cal3DS2 *self, const gtsam::Cal3DS2 &c) { return self->localCoordinates(c); }, py::arg("c"))
      .def("serialize", [](gtsam::Cal3DS2 *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Cal3DS2 *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Cal3DS2 &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Cal3DS2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Dim", []() { return gtsam::Cal3DS2::Dim(); });

  py::class_<gtsam::Cal3Unified, gtsam::Cal3DS2_Base, boost::shared_ptr<gtsam::Cal3Unified>>(m_, "Cal3Unified")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"))
      .def(py::init<double, double, double, double, double, double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"), py::arg("xi"))
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def("equals", [](gtsam::Cal3Unified *self, const gtsam::Cal3Unified &rhs, double tol) { return self->equals(rhs, tol); }, py::arg("rhs"), py::arg("tol"))
      .def("xi", [](gtsam::Cal3Unified *self) { return self->xi(); })
      .def("spaceToNPlane", [](gtsam::Cal3Unified *self, const gtsam::Point2 &p) { return self->spaceToNPlane(p); }, py::arg("p"))
      .def("nPlaneToSpace", [](gtsam::Cal3Unified *self, const gtsam::Point2 &p) { return self->nPlaneToSpace(p); }, py::arg("p"))
      .def("dim", [](gtsam::Cal3Unified *self) { return self->dim(); })
      .def("retract", [](gtsam::Cal3Unified *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Cal3Unified *self, const gtsam::Cal3Unified &c) { return self->localCoordinates(c); }, py::arg("c"))
      .def("calibrate", [](gtsam::Cal3Unified *self, const gtsam::Point2 &p) { return self->calibrate(p); }, py::arg("p"))
      .def("calibrate", [](gtsam::Cal3Unified *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->calibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("uncalibrate", [](gtsam::Cal3Unified *self, const gtsam::Point2 &p) { return self->uncalibrate(p); }, py::arg("p"))
      .def("uncalibrate", [](gtsam::Cal3Unified *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->uncalibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("serialize", [](gtsam::Cal3Unified *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Cal3Unified *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Cal3Unified &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Cal3Unified obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Dim", []() { return gtsam::Cal3Unified::Dim(); });

  py::class_<gtsam::Cal3Fisheye, boost::shared_ptr<gtsam::Cal3Fisheye>>(m_, "Cal3Fisheye")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double, double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"), py::arg("k3"), py::arg("k4"), py::arg("tol") = 1e-5)
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def("print", [](gtsam::Cal3Fisheye *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "Cal3Fisheye")
      .def("__repr__", [](const gtsam::Cal3Fisheye &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "Cal3Fisheye")
      .def("equals", [](gtsam::Cal3Fisheye *self, const gtsam::Cal3Fisheye &rhs, double tol) { return self->equals(rhs, tol); }, py::arg("rhs"), py::arg("tol"))
      .def("dim", [](gtsam::Cal3Fisheye *self) { return self->dim(); })
      .def("retract", [](gtsam::Cal3Fisheye *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Cal3Fisheye *self, const gtsam::Cal3Fisheye &c) { return self->localCoordinates(c); }, py::arg("c"))
      .def("calibrate", [](gtsam::Cal3Fisheye *self, const gtsam::Point2 &p) { return self->calibrate(p); }, py::arg("p"))
      .def("calibrate", [](gtsam::Cal3Fisheye *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->calibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("uncalibrate", [](gtsam::Cal3Fisheye *self, const gtsam::Point2 &p) { return self->uncalibrate(p); }, py::arg("p"))
      .def("uncalibrate", [](gtsam::Cal3Fisheye *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->uncalibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("fx", [](gtsam::Cal3Fisheye *self) { return self->fx(); })
      .def("fy", [](gtsam::Cal3Fisheye *self) { return self->fy(); })
      .def("skew", [](gtsam::Cal3Fisheye *self) { return self->skew(); })
      .def("k1", [](gtsam::Cal3Fisheye *self) { return self->k1(); })
      .def("k2", [](gtsam::Cal3Fisheye *self) { return self->k2(); })
      .def("k3", [](gtsam::Cal3Fisheye *self) { return self->k3(); })
      .def("k4", [](gtsam::Cal3Fisheye *self) { return self->k4(); })
      .def("px", [](gtsam::Cal3Fisheye *self) { return self->px(); })
      .def("py", [](gtsam::Cal3Fisheye *self) { return self->py(); })
      .def("principalPoint", [](gtsam::Cal3Fisheye *self) { return self->principalPoint(); })
      .def("vector", [](gtsam::Cal3Fisheye *self) { return self->vector(); })
      .def("k", [](gtsam::Cal3Fisheye *self) { return self->k(); })
      .def("K", [](gtsam::Cal3Fisheye *self) { return self->K(); })
      .def("inverse", [](gtsam::Cal3Fisheye *self) { return self->inverse(); })
      .def("serialize", [](gtsam::Cal3Fisheye *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Cal3Fisheye *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Cal3Fisheye &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Cal3Fisheye obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Dim", []() { return gtsam::Cal3Fisheye::Dim(); });

  py::class_<gtsam::Cal3_S2Stereo, boost::shared_ptr<gtsam::Cal3_S2Stereo>>(m_, "Cal3_S2Stereo")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double, double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("b"))
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def("print", [](gtsam::Cal3_S2Stereo *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Cal3_S2Stereo &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Cal3_S2Stereo *self, const gtsam::Cal3_S2Stereo &K, double tol) { return self->equals(K, tol); }, py::arg("K"), py::arg("tol"))
      .def("fx", [](gtsam::Cal3_S2Stereo *self) { return self->fx(); })
      .def("fy", [](gtsam::Cal3_S2Stereo *self) { return self->fy(); })
      .def("skew", [](gtsam::Cal3_S2Stereo *self) { return self->skew(); })
      .def("px", [](gtsam::Cal3_S2Stereo *self) { return self->px(); })
      .def("py", [](gtsam::Cal3_S2Stereo *self) { return self->py(); })
      .def("principalPoint", [](gtsam::Cal3_S2Stereo *self) { return self->principalPoint(); })
      .def("baseline", [](gtsam::Cal3_S2Stereo *self) { return self->baseline(); });

  py::class_<gtsam::Cal3Bundler, boost::shared_ptr<gtsam::Cal3Bundler>>(m_, "Cal3Bundler")
      .def(py::init<>())
      .def(py::init<double, double, double, double, double>(), py::arg("fx"), py::arg("k1"), py::arg("k2"), py::arg("u0"), py::arg("v0"))
      .def(py::init<double, double, double, double, double, double>(), py::arg("fx"), py::arg("k1"), py::arg("k2"), py::arg("u0"), py::arg("v0"), py::arg("tol"))
      .def("print", [](gtsam::Cal3Bundler *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::Cal3Bundler &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::Cal3Bundler *self, const gtsam::Cal3Bundler &rhs, double tol) { return self->equals(rhs, tol); }, py::arg("rhs"), py::arg("tol"))
      .def("dim", [](gtsam::Cal3Bundler *self) { return self->dim(); })
      .def("retract", [](gtsam::Cal3Bundler *self, const gtsam::Vector &v) { return self->retract(v); }, py::arg("v"))
      .def("localCoordinates", [](gtsam::Cal3Bundler *self, const gtsam::Cal3Bundler &c) { return self->localCoordinates(c); }, py::arg("c"))
      .def("calibrate", [](gtsam::Cal3Bundler *self, const gtsam::Point2 &p) { return self->calibrate(p); }, py::arg("p"))
      .def("calibrate", [](gtsam::Cal3Bundler *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->calibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("uncalibrate", [](gtsam::Cal3Bundler *self, const gtsam::Point2 &p) { return self->uncalibrate(p); }, py::arg("p"))
      .def("uncalibrate", [](gtsam::Cal3Bundler *self, const gtsam::Point2 &p, Eigen::Ref<Eigen::MatrixXd> Dcal, Eigen::Ref<Eigen::MatrixXd> Dp) { return self->uncalibrate(p, Dcal, Dp); }, py::arg("p"), py::arg("Dcal"), py::arg("Dp"))
      .def("fx", [](gtsam::Cal3Bundler *self) { return self->fx(); })
      .def("fy", [](gtsam::Cal3Bundler *self) { return self->fy(); })
      .def("k1", [](gtsam::Cal3Bundler *self) { return self->k1(); })
      .def("k2", [](gtsam::Cal3Bundler *self) { return self->k2(); })
      .def("px", [](gtsam::Cal3Bundler *self) { return self->px(); })
      .def("py", [](gtsam::Cal3Bundler *self) { return self->py(); })
      .def("vector", [](gtsam::Cal3Bundler *self) { return self->vector(); })
      .def("k", [](gtsam::Cal3Bundler *self) { return self->k(); })
      .def("K", [](gtsam::Cal3Bundler *self) { return self->K(); })
      .def("serialize", [](gtsam::Cal3Bundler *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::Cal3Bundler *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::Cal3Bundler &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::Cal3Bundler obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Dim", []() { return gtsam::Cal3Bundler::Dim(); });

  py::class_<gtsam::CalibratedCamera, boost::shared_ptr<gtsam::CalibratedCamera>>(m_, "CalibratedCamera")
      .def(py::init<>())
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Vector &>(), py::arg("v"))
      .def("print", [](gtsam::CalibratedCamera *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "CalibratedCamera")
      .def("__repr__", [](const gtsam::CalibratedCamera &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "CalibratedCamera")
      .def("equals", [](gtsam::CalibratedCamera *self, const gtsam::CalibratedCamera &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("dim", [](gtsam::CalibratedCamera *self) { return self->dim(); })
      .def("retract", [](gtsam::CalibratedCamera *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::CalibratedCamera *self, const gtsam::CalibratedCamera &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("project", [](gtsam::CalibratedCamera *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::CalibratedCamera *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->project(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("backproject", [](gtsam::CalibratedCamera *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::CalibratedCamera *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"))
      .def("pose", [](gtsam::CalibratedCamera *self) { return self->pose(); })
      .def("range", [](gtsam::CalibratedCamera *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::CalibratedCamera *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::CalibratedCamera *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::CalibratedCamera *self, const gtsam::Pose3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(point, Dcamera, Dpose); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("range", [](gtsam::CalibratedCamera *self, const gtsam::CalibratedCamera &camera) { return self->range(camera); }, py::arg("camera"))
      .def("serialize", [](gtsam::CalibratedCamera *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::CalibratedCamera *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::CalibratedCamera &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::CalibratedCamera obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Pose2 &pose2, double height) { return gtsam::CalibratedCamera::Level(pose2, height); }, py::arg("pose2"), py::arg("height"))
      .def_static("Dim", []() { return gtsam::CalibratedCamera::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::CalibratedCamera::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::Similarity2, boost::shared_ptr<gtsam::Similarity2>>(m_, "Similarity2")
      .def(py::init<>())
      .def(py::init<double>(), py::arg("s"))
      .def(py::init<const gtsam::Rot2 &, const gtsam::Point2 &, double>(), py::arg("R"), py::arg("t"), py::arg("s"))
      .def(py::init<const gtsam::Matrix &, const gtsam::Vector &, double>(), py::arg("R"), py::arg("t"), py::arg("s"))
      .def(py::init<const gtsam::Matrix &>(), py::arg("T"))
      .def("transformFrom", [](gtsam::Similarity2 *self, const gtsam::Point2 &p) { return self->transformFrom(p); }, py::arg("p"))
      .def("transformFrom", [](gtsam::Similarity2 *self, const gtsam::Pose2 &T) { return self->transformFrom(T); }, py::arg("T"))
      .def("equals", [](gtsam::Similarity2 *self, const gtsam::Similarity2 &sim, double tol) { return self->equals(sim, tol); }, py::arg("sim"), py::arg("tol"))
      .def("matrix", [](gtsam::Similarity2 *self) { return self->matrix(); })
      .def("rotation", [](gtsam::Similarity2 *self) { return self->rotation(); })
      .def("translation", [](gtsam::Similarity2 *self) { return self->translation(); })
      .def("scale", [](gtsam::Similarity2 *self) { return self->scale(); })
      .def_static("Align", [](const gtsam::Point2Pairs &abPointPairs) { return gtsam::Similarity2::Align(abPointPairs); }, py::arg("abPointPairs"))
      .def_static("Align", [](const gtsam::Pose2Pairs &abPosePairs) { return gtsam::Similarity2::Align(abPosePairs); }, py::arg("abPosePairs"));

  py::class_<gtsam::Similarity3, boost::shared_ptr<gtsam::Similarity3>>(m_, "Similarity3")
      .def(py::init<>())
      .def(py::init<double>(), py::arg("s"))
      .def(py::init<const gtsam::Rot3 &, const gtsam::Point3 &, double>(), py::arg("R"), py::arg("t"), py::arg("s"))
      .def(py::init<const gtsam::Matrix &, const gtsam::Vector &, double>(), py::arg("R"), py::arg("t"), py::arg("s"))
      .def(py::init<const gtsam::Matrix &>(), py::arg("T"))
      .def("transformFrom", [](gtsam::Similarity3 *self, const gtsam::Point3 &p) { return self->transformFrom(p); }, py::arg("p"))
      .def("transformFrom", [](gtsam::Similarity3 *self, const gtsam::Pose3 &T) { return self->transformFrom(T); }, py::arg("T"))
      .def("equals", [](gtsam::Similarity3 *self, const gtsam::Similarity3 &sim, double tol) { return self->equals(sim, tol); }, py::arg("sim"), py::arg("tol"))
      .def("matrix", [](gtsam::Similarity3 *self) { return self->matrix(); })
      .def("rotation", [](gtsam::Similarity3 *self) { return self->rotation(); })
      .def("translation", [](gtsam::Similarity3 *self) { return self->translation(); })
      .def("scale", [](gtsam::Similarity3 *self) { return self->scale(); })
      .def_static("Align", [](const gtsam::Point3Pairs &abPointPairs) { return gtsam::Similarity3::Align(abPointPairs); }, py::arg("abPointPairs"))
      .def_static("Align", [](const gtsam::Pose3Pairs &abPosePairs) { return gtsam::Similarity3::Align(abPosePairs); }, py::arg("abPosePairs"));

  py::class_<gtsam::StereoCamera, boost::shared_ptr<gtsam::StereoCamera>>(m_, "StereoCamera")
      .def(py::init<>())
      .def(py::init<const gtsam::Pose3 &, const boost::shared_ptr<gtsam::Cal3_S2Stereo>>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::StereoCamera *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::StereoCamera &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def("equals", [](gtsam::StereoCamera *self, const gtsam::StereoCamera &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::StereoCamera *self) { return self->pose(); })
      .def("baseline", [](gtsam::StereoCamera *self) { return self->baseline(); })
      .def("calibration", [](gtsam::StereoCamera *self) { return self->calibration(); })
      .def("retract", [](gtsam::StereoCamera *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::StereoCamera *self, const gtsam::StereoCamera &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::StereoCamera *self) { return self->dim(); })
      .def("project", [](gtsam::StereoCamera *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("backproject", [](gtsam::StereoCamera *self, const gtsam::StereoPoint2 &p) { return self->backproject(p); }, py::arg("p"))
      .def("project2", [](gtsam::StereoCamera *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) { return self->project2(point, H1, H2); }, py::arg("point"), py::arg("H1"), py::arg("H2"))
      .def("backproject2", [](gtsam::StereoCamera *self, const gtsam::StereoPoint2 &p, Eigen::Ref<Eigen::MatrixXd> H1, Eigen::Ref<Eigen::MatrixXd> H2) { return self->backproject2(p, H1, H2); }, py::arg("p"), py::arg("H1"), py::arg("H2"))
      .def("serialize", [](gtsam::StereoCamera *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::StereoCamera *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::StereoCamera &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::StereoCamera obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Dim", []() { return gtsam::StereoCamera::Dim(); });

  py::class_<gtsam::TriangulationResult, boost::shared_ptr<gtsam::TriangulationResult>> triangulationresult(m_, "TriangulationResult");
  triangulationresult
      .def(py::init<const gtsam::Point3 &>(), py::arg("p"))
      .def("get", [](gtsam::TriangulationResult *self) { return self->get(); })
      .def("valid", [](gtsam::TriangulationResult *self) { return self->valid(); })
      .def("degenerate", [](gtsam::TriangulationResult *self) { return self->degenerate(); })
      .def("outlier", [](gtsam::TriangulationResult *self) { return self->outlier(); })
      .def("farPoint", [](gtsam::TriangulationResult *self) { return self->farPoint(); })
      .def("behindCamera", [](gtsam::TriangulationResult *self) { return self->behindCamera(); })
      .def_static("Degenerate", []() { return gtsam::TriangulationResult::Degenerate(); })
      .def_static("Outlier", []() { return gtsam::TriangulationResult::Outlier(); })
      .def_static("FarPoint", []() { return gtsam::TriangulationResult::FarPoint(); })
      .def_static("BehindCamera", []() { return gtsam::TriangulationResult::BehindCamera(); })
      .def_readwrite("status", &gtsam::TriangulationResult::status);

  py::enum_<gtsam::TriangulationResult::Status>(triangulationresult, "Status", py::arithmetic())
      .value("VALID", gtsam::TriangulationResult::Status::VALID)
      .value("DEGENERATE", gtsam::TriangulationResult::Status::DEGENERATE)
      .value("BEHIND_CAMERA", gtsam::TriangulationResult::Status::BEHIND_CAMERA)
      .value("OUTLIER", gtsam::TriangulationResult::Status::OUTLIER)
      .value("FAR_POINT", gtsam::TriangulationResult::Status::FAR_POINT);

  py::class_<gtsam::TriangulationParameters, boost::shared_ptr<gtsam::TriangulationParameters>>(m_, "TriangulationParameters")
      .def(py::init<const double, const bool, double, double, const gtsam::SharedNoiseModel &>(), py::arg("rankTolerance") = 1.0, py::arg("enableEPI") = false, py::arg("landmarkDistanceThreshold") = -1, py::arg("dynamicOutlierRejectionThreshold") = -1, py::arg("noiseModel") = nullptr)
      .def_readwrite("rankTolerance", &gtsam::TriangulationParameters::rankTolerance)
      .def_readwrite("enableEPI", &gtsam::TriangulationParameters::enableEPI)
      .def_readwrite("landmarkDistanceThreshold", &gtsam::TriangulationParameters::landmarkDistanceThreshold)
      .def_readwrite("dynamicOutlierRejectionThreshold", &gtsam::TriangulationParameters::dynamicOutlierRejectionThreshold)
      .def_readwrite("noiseModel", &gtsam::TriangulationParameters::noiseModel);

  py::class_<gtsam::PinholeCamera<gtsam::Cal3_S2>, boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3_S2>>>(m_, "PinholeCameraCal3_S2")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholeCamera<gtsam::Cal3_S2>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const gtsam::Cal3_S2 &>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholeCamera")
      .def("__repr__", [](const gtsam::PinholeCamera<gtsam::Cal3_S2> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholeCamera")
      .def("equals", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::PinholeCamera<gtsam::Cal3_S2> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::PinholeCamera<gtsam::Cal3_S2> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("reprojectionError", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point3 &pw, const gtsam::Point2 &measured, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->reprojectionError(pw, measured, Dpose, Dpoint, Dcal); }, py::arg("pw"), py::arg("measured"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholeCamera<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholeCamera<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholeCamera<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Cal3_S2 &K, const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3_S2>::Level(K, pose, height); }, py::arg("K"), py::arg("pose"), py::arg("height"))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3_S2>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const gtsam::Cal3_S2 &K) { return gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholeCamera<gtsam::Cal3_S2>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholeCamera<gtsam::Cal3_S2>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholeCamera<gtsam::Cal3DS2>, boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3DS2>>>(m_, "PinholeCameraCal3DS2")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholeCamera<gtsam::Cal3DS2>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const gtsam::Cal3DS2 &>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholeCamera")
      .def("__repr__", [](const gtsam::PinholeCamera<gtsam::Cal3DS2> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholeCamera")
      .def("equals", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::PinholeCamera<gtsam::Cal3DS2> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::PinholeCamera<gtsam::Cal3DS2> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("reprojectionError", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point3 &pw, const gtsam::Point2 &measured, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->reprojectionError(pw, measured, Dpose, Dpoint, Dcal); }, py::arg("pw"), py::arg("measured"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholeCamera<gtsam::Cal3DS2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholeCamera<gtsam::Cal3DS2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholeCamera<gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Cal3DS2 &K, const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3DS2>::Level(K, pose, height); }, py::arg("K"), py::arg("pose"), py::arg("height"))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3DS2>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const gtsam::Cal3DS2 &K) { return gtsam::PinholeCamera<gtsam::Cal3DS2>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholeCamera<gtsam::Cal3DS2>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholeCamera<gtsam::Cal3Unified>, boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Unified>>>(m_, "PinholeCameraCal3Unified")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholeCamera<gtsam::Cal3Unified>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const gtsam::Cal3Unified &>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholeCamera")
      .def("__repr__", [](const gtsam::PinholeCamera<gtsam::Cal3Unified> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholeCamera")
      .def("equals", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::PinholeCamera<gtsam::Cal3Unified> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::PinholeCamera<gtsam::Cal3Unified> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("reprojectionError", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point3 &pw, const gtsam::Point2 &measured, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->reprojectionError(pw, measured, Dpose, Dpoint, Dcal); }, py::arg("pw"), py::arg("measured"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholeCamera<gtsam::Cal3Unified> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholeCamera<gtsam::Cal3Unified> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholeCamera<gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Cal3Unified &K, const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3Unified>::Level(K, pose, height); }, py::arg("K"), py::arg("pose"), py::arg("height"))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3Unified>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const gtsam::Cal3Unified &K) { return gtsam::PinholeCamera<gtsam::Cal3Unified>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholeCamera<gtsam::Cal3Unified>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholeCamera<gtsam::Cal3Unified>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholeCamera<gtsam::Cal3Bundler>, boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>(m_, "PinholeCameraCal3Bundler")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholeCamera<gtsam::Cal3Bundler>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const gtsam::Cal3Bundler &>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholeCamera")
      .def("__repr__", [](const gtsam::PinholeCamera<gtsam::Cal3Bundler> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholeCamera")
      .def("equals", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::PinholeCamera<gtsam::Cal3Bundler> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("reprojectionError", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point3 &pw, const gtsam::Point2 &measured, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->reprojectionError(pw, measured, Dpose, Dpoint, Dcal); }, py::arg("pw"), py::arg("measured"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholeCamera<gtsam::Cal3Bundler> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholeCamera<gtsam::Cal3Bundler> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholeCamera<gtsam::Cal3Bundler> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Cal3Bundler &K, const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3Bundler>::Level(K, pose, height); }, py::arg("K"), py::arg("pose"), py::arg("height"))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3Bundler>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const gtsam::Cal3Bundler &K) { return gtsam::PinholeCamera<gtsam::Cal3Bundler>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholeCamera<gtsam::Cal3Bundler>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholeCamera<gtsam::Cal3Bundler>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholeCamera<gtsam::Cal3Fisheye>, boost::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>(m_, "PinholeCameraCal3Fisheye")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholeCamera<gtsam::Cal3Fisheye>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const gtsam::Cal3Fisheye &>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholeCamera")
      .def("__repr__", [](const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholeCamera")
      .def("equals", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("reprojectionError", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &pw, const gtsam::Point2 &measured, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->reprojectionError(pw, measured, Dpose, Dpoint, Dcal); }, py::arg("pw"), py::arg("measured"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholeCamera<gtsam::Cal3Fisheye> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholeCamera<gtsam::Cal3Fisheye> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholeCamera<gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Cal3Fisheye &K, const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3Fisheye>::Level(K, pose, height); }, py::arg("K"), py::arg("pose"), py::arg("height"))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholeCamera<gtsam::Cal3Fisheye>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const gtsam::Cal3Fisheye &K) { return gtsam::PinholeCamera<gtsam::Cal3Fisheye>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholeCamera<gtsam::Cal3Fisheye>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholeCamera<gtsam::Cal3Fisheye>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholePose<gtsam::Cal3_S2>, boost::shared_ptr<gtsam::PinholePose<gtsam::Cal3_S2>>>(m_, "PinholePoseCal3_S2")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholePose<gtsam::Cal3_S2>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const boost::shared_ptr<gtsam::Cal3_S2>>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholePose")
      .def("__repr__", [](const gtsam::PinholePose<gtsam::Cal3_S2> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholePose")
      .def("equals", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::PinholePose<gtsam::Cal3_S2> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholePose<gtsam::Cal3_S2> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholePose<gtsam::Cal3_S2> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::PinholePose<gtsam::Cal3_S2> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholePose<gtsam::Cal3_S2> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholePose<gtsam::Cal3_S2> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholePose<gtsam::Cal3_S2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholePose<gtsam::Cal3_S2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholePose<gtsam::Cal3_S2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholePose<gtsam::Cal3_S2>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const boost::shared_ptr<gtsam::Cal3_S2> K) { return gtsam::PinholePose<gtsam::Cal3_S2>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholePose<gtsam::Cal3_S2>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholePose<gtsam::Cal3_S2>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholePose<gtsam::Cal3DS2>, boost::shared_ptr<gtsam::PinholePose<gtsam::Cal3DS2>>>(m_, "PinholePoseCal3DS2")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholePose<gtsam::Cal3DS2>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const boost::shared_ptr<gtsam::Cal3DS2>>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholePose")
      .def("__repr__", [](const gtsam::PinholePose<gtsam::Cal3DS2> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholePose")
      .def("equals", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::PinholePose<gtsam::Cal3DS2> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholePose<gtsam::Cal3DS2> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholePose<gtsam::Cal3DS2> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::PinholePose<gtsam::Cal3DS2> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholePose<gtsam::Cal3DS2> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholePose<gtsam::Cal3DS2> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholePose<gtsam::Cal3DS2> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholePose<gtsam::Cal3DS2> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholePose<gtsam::Cal3DS2> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholePose<gtsam::Cal3DS2>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const boost::shared_ptr<gtsam::Cal3DS2> K) { return gtsam::PinholePose<gtsam::Cal3DS2>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholePose<gtsam::Cal3DS2>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholePose<gtsam::Cal3DS2>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholePose<gtsam::Cal3Unified>, boost::shared_ptr<gtsam::PinholePose<gtsam::Cal3Unified>>>(m_, "PinholePoseCal3Unified")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholePose<gtsam::Cal3Unified>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const boost::shared_ptr<gtsam::Cal3Unified>>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholePose")
      .def("__repr__", [](const gtsam::PinholePose<gtsam::Cal3Unified> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholePose")
      .def("equals", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::PinholePose<gtsam::Cal3Unified> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholePose<gtsam::Cal3Unified> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholePose<gtsam::Cal3Unified> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::PinholePose<gtsam::Cal3Unified> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholePose<gtsam::Cal3Unified> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholePose<gtsam::Cal3Unified> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholePose<gtsam::Cal3Unified> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholePose<gtsam::Cal3Unified> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholePose<gtsam::Cal3Unified> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholePose<gtsam::Cal3Unified>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const boost::shared_ptr<gtsam::Cal3Unified> K) { return gtsam::PinholePose<gtsam::Cal3Unified>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholePose<gtsam::Cal3Unified>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholePose<gtsam::Cal3Unified>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholePose<gtsam::Cal3Bundler>, boost::shared_ptr<gtsam::PinholePose<gtsam::Cal3Bundler>>>(m_, "PinholePoseCal3Bundler")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholePose<gtsam::Cal3Bundler>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const boost::shared_ptr<gtsam::Cal3Bundler>>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholePose")
      .def("__repr__", [](const gtsam::PinholePose<gtsam::Cal3Bundler> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholePose")
      .def("equals", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::PinholePose<gtsam::Cal3Bundler> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::PinholePose<gtsam::Cal3Bundler> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholePose<gtsam::Cal3Bundler> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholePose<gtsam::Cal3Bundler> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholePose<gtsam::Cal3Bundler> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholePose<gtsam::Cal3Bundler>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const boost::shared_ptr<gtsam::Cal3Bundler> K) { return gtsam::PinholePose<gtsam::Cal3Bundler>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholePose<gtsam::Cal3Bundler>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholePose<gtsam::Cal3Bundler>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::PinholePose<gtsam::Cal3Fisheye>, boost::shared_ptr<gtsam::PinholePose<gtsam::Cal3Fisheye>>>(m_, "PinholePoseCal3Fisheye")
      .def(py::init<>())
      .def(py::init<const gtsam::PinholePose<gtsam::Cal3Fisheye>>(), py::arg("other"))
      .def(py::init<const gtsam::Pose3 &>(), py::arg("pose"))
      .def(py::init<const gtsam::Pose3 &, const boost::shared_ptr<gtsam::Cal3Fisheye>>(), py::arg("pose"), py::arg("K"))
      .def("print", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "PinholePose")
      .def("__repr__", [](const gtsam::PinholePose<gtsam::Cal3Fisheye> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "PinholePose")
      .def("equals", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::PinholePose<gtsam::Cal3Fisheye> &camera, double tol) { return self->equals(camera, tol); }, py::arg("camera"), py::arg("tol"))
      .def("pose", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self) { return self->pose(); })
      .def("calibration", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self) { return self->calibration(); })
      .def("retract", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Vector &d) { return self->retract(d); }, py::arg("d"))
      .def("localCoordinates", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::PinholePose<gtsam::Cal3Fisheye> &T2) { return self->localCoordinates(T2); }, py::arg("T2"))
      .def("dim", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self) { return self->dim(); })
      .def("projectSafe", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &pw) { return self->projectSafe(pw); }, py::arg("pw"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point) { return self->project(point); }, py::arg("point"))
      .def("project", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dpose, Eigen::Ref<Eigen::MatrixXd> Dpoint, Eigen::Ref<Eigen::MatrixXd> Dcal) { return self->project(point, Dpose, Dpoint, Dcal); }, py::arg("point"), py::arg("Dpose"), py::arg("Dpoint"), py::arg("Dcal"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point2 &p, double depth) { return self->backproject(p, depth); }, py::arg("p"), py::arg("depth"))
      .def("backproject", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point2 &p, double depth, Eigen::Ref<Eigen::MatrixXd> Dresult_dpose, Eigen::Ref<Eigen::MatrixXd> Dresult_dp, Eigen::Ref<Eigen::MatrixXd> Dresult_ddepth, Eigen::Ref<Eigen::MatrixXd> Dresult_dcal) { return self->backproject(p, depth, Dresult_dpose, Dresult_dp, Dresult_ddepth, Dresult_dcal); }, py::arg("p"), py::arg("depth"), py::arg("Dresult_dpose"), py::arg("Dresult_dp"), py::arg("Dresult_ddepth"), py::arg("Dresult_dcal"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point) { return self->range(point); }, py::arg("point"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Point3 &point, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpoint) { return self->range(point, Dcamera, Dpoint); }, py::arg("point"), py::arg("Dcamera"), py::arg("Dpoint"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Pose3 &pose) { return self->range(pose); }, py::arg("pose"))
      .def("range", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, const gtsam::Pose3 &pose, Eigen::Ref<Eigen::MatrixXd> Dcamera, Eigen::Ref<Eigen::MatrixXd> Dpose) { return self->range(pose, Dcamera, Dpose); }, py::arg("pose"), py::arg("Dcamera"), py::arg("Dpose"))
      .def("serialize", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self) { return gtsam::serialize(*self); })
      .def("deserialize", [](gtsam::PinholePose<gtsam::Cal3Fisheye> *self, string serialized) { gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
      .def(py::pickle([](const gtsam::PinholePose<gtsam::Cal3Fisheye> &a) { /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); }, [](py::tuple t) { /* __setstate__ */ gtsam::PinholePose<gtsam::Cal3Fisheye> obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
      .def_static("Level", [](const gtsam::Pose2 &pose, double height) { return gtsam::PinholePose<gtsam::Cal3Fisheye>::Level(pose, height); }, py::arg("pose"), py::arg("height"))
      .def_static("Lookat", [](const gtsam::Point3 &eye, const gtsam::Point3 &target, const gtsam::Point3 &upVector, const boost::shared_ptr<gtsam::Cal3Fisheye> K) { return gtsam::PinholePose<gtsam::Cal3Fisheye>::Lookat(eye, target, upVector, K); }, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
      .def_static("Dim", []() { return gtsam::PinholePose<gtsam::Cal3Fisheye>::Dim(); })
      .def_static("Project", [](const gtsam::Point3 &cameraPoint) { return gtsam::PinholePose<gtsam::Cal3Fisheye>::Project(cameraPoint); }, py::arg("cameraPoint"));

  py::class_<gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>, boost::shared_ptr<gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>>>(m_, "BearingRange2D")
      .def(py::init<const gtsam::Rot2 &, const double &>(), py::arg("b"), py::arg("r"))
      .def("bearing", [](gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> *self) { return self->bearing(); })
      .def("range", [](gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> *self) { return self->range(); })
      .def("print", [](gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def_static("Measure", [](const gtsam::Pose2 &pose, const gtsam::Point2 &point) { return gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>::Measure(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureBearing", [](const gtsam::Pose2 &pose, const gtsam::Point2 &point) { return gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>::MeasureBearing(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureRange", [](const gtsam::Pose2 &pose, const gtsam::Point2 &point) { return gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>::MeasureRange(pose, point); }, py::arg("pose"), py::arg("point"));

  py::class_<gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>, boost::shared_ptr<gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>>>(m_, "BearingRangePose2")
      .def(py::init<const gtsam::Rot2 &, const double &>(), py::arg("b"), py::arg("r"))
      .def("bearing", [](gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> *self) { return self->bearing(); })
      .def("range", [](gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> *self) { return self->range(); })
      .def("print", [](gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def_static("Measure", [](const gtsam::Pose2 &pose, const gtsam::Pose2 &point) { return gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>::Measure(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureBearing", [](const gtsam::Pose2 &pose, const gtsam::Pose2 &point) { return gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>::MeasureBearing(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureRange", [](const gtsam::Pose2 &pose, const gtsam::Pose2 &point) { return gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>::MeasureRange(pose, point); }, py::arg("pose"), py::arg("point"));

  py::class_<gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>, boost::shared_ptr<gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>>>(m_, "BearingRange3D")
      .def(py::init<const gtsam::Unit3 &, const double &>(), py::arg("b"), py::arg("r"))
      .def("bearing", [](gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> *self) { return self->bearing(); })
      .def("range", [](gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> *self) { return self->range(); })
      .def("print", [](gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def_static("Measure", [](const gtsam::Pose3 &pose, const gtsam::Point3 &point) { return gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>::Measure(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureBearing", [](const gtsam::Pose3 &pose, const gtsam::Point3 &point) { return gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>::MeasureBearing(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureRange", [](const gtsam::Pose3 &pose, const gtsam::Point3 &point) { return gtsam::BearingRange<gtsam::Pose3, gtsam::Point3, gtsam::Unit3, double>::MeasureRange(pose, point); }, py::arg("pose"), py::arg("point"));

  py::class_<gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>, boost::shared_ptr<gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>>>(m_, "BearingRangePose3")
      .def(py::init<const gtsam::Unit3 &, const double &>(), py::arg("b"), py::arg("r"))
      .def("bearing", [](gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> *self) { return self->bearing(); })
      .def("range", [](gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> *self) { return self->range(); })
      .def("print", [](gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> *self, string s) { py::scoped_ostream_redirect output; self->print(s); }, py::arg("s") = "")
      .def("__repr__", [](const gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double> &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, py::arg("s") = "")
      .def_static("Measure", [](const gtsam::Pose3 &pose, const gtsam::Pose3 &point) { return gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>::Measure(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureBearing", [](const gtsam::Pose3 &pose, const gtsam::Pose3 &point) { return gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>::MeasureBearing(pose, point); }, py::arg("pose"), py::arg("point"))
      .def_static("MeasureRange", [](const gtsam::Pose3 &pose, const gtsam::Pose3 &point) { return gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3, gtsam::Unit3, double>::MeasureRange(pose, point); }, py::arg("pose"), py::arg("point"));

  m_.def("triangulatePoint3", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3_S2> sharedCal, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model) { return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize, model); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr);
  m_.def("triangulatePoint3", [](const gtsam::CameraSetCal3_S2 &cameras, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model, const bool useLOST) { return gtsam::triangulatePoint3(cameras, measurements, rank_tol, optimize, model, useLOST); }, py::arg("cameras"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr, py::arg("useLOST") = false);
  m_.def("triangulateNonlinear", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3_S2> sharedCal, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(poses, sharedCal, measurements, initialEstimate); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateNonlinear", [](const gtsam::CameraSetCal3_S2 &cameras, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(cameras, measurements, initialEstimate); }, py::arg("cameras"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateSafe", [](const gtsam::CameraSetCal3_S2 &cameras, const gtsam::Point2Vector &measurements, const gtsam::TriangulationParameters &params) { return gtsam::triangulateSafe(cameras, measurements, params); }, py::arg("cameras"), py::arg("measurements"), py::arg("params"));
  m_.def("triangulatePoint3", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3DS2> sharedCal, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model) { return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize, model); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr);
  m_.def("triangulatePoint3", [](const gtsam::CameraSetCal3DS2 &cameras, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model, const bool useLOST) { return gtsam::triangulatePoint3(cameras, measurements, rank_tol, optimize, model, useLOST); }, py::arg("cameras"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr, py::arg("useLOST") = false);
  m_.def("triangulateNonlinear", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3DS2> sharedCal, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(poses, sharedCal, measurements, initialEstimate); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateNonlinear", [](const gtsam::CameraSetCal3DS2 &cameras, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(cameras, measurements, initialEstimate); }, py::arg("cameras"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateSafe", [](const gtsam::CameraSetCal3DS2 &cameras, const gtsam::Point2Vector &measurements, const gtsam::TriangulationParameters &params) { return gtsam::triangulateSafe(cameras, measurements, params); }, py::arg("cameras"), py::arg("measurements"), py::arg("params"));
  m_.def("triangulatePoint3", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3Bundler> sharedCal, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model) { return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize, model); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr);
  m_.def("triangulatePoint3", [](const gtsam::CameraSetCal3Bundler &cameras, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model, const bool useLOST) { return gtsam::triangulatePoint3(cameras, measurements, rank_tol, optimize, model, useLOST); }, py::arg("cameras"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr, py::arg("useLOST") = false);
  m_.def("triangulateNonlinear", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3Bundler> sharedCal, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(poses, sharedCal, measurements, initialEstimate); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateNonlinear", [](const gtsam::CameraSetCal3Bundler &cameras, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(cameras, measurements, initialEstimate); }, py::arg("cameras"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateSafe", [](const gtsam::CameraSetCal3Bundler &cameras, const gtsam::Point2Vector &measurements, const gtsam::TriangulationParameters &params) { return gtsam::triangulateSafe(cameras, measurements, params); }, py::arg("cameras"), py::arg("measurements"), py::arg("params"));
  m_.def("triangulatePoint3", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3Fisheye> sharedCal, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model) { return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize, model); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr);
  m_.def("triangulatePoint3", [](const gtsam::CameraSetCal3Fisheye &cameras, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model, const bool useLOST) { return gtsam::triangulatePoint3(cameras, measurements, rank_tol, optimize, model, useLOST); }, py::arg("cameras"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr, py::arg("useLOST") = false);
  m_.def("triangulateNonlinear", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3Fisheye> sharedCal, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(poses, sharedCal, measurements, initialEstimate); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateNonlinear", [](const gtsam::CameraSetCal3Fisheye &cameras, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(cameras, measurements, initialEstimate); }, py::arg("cameras"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateSafe", [](const gtsam::CameraSetCal3Fisheye &cameras, const gtsam::Point2Vector &measurements, const gtsam::TriangulationParameters &params) { return gtsam::triangulateSafe(cameras, measurements, params); }, py::arg("cameras"), py::arg("measurements"), py::arg("params"));
  m_.def("triangulatePoint3", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3Unified> sharedCal, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model) { return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize, model); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr);
  m_.def("triangulatePoint3", [](const gtsam::CameraSetCal3Unified &cameras, const gtsam::Point2Vector &measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel &model, const bool useLOST) { return gtsam::triangulatePoint3(cameras, measurements, rank_tol, optimize, model, useLOST); }, py::arg("cameras"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"), py::arg("model") = nullptr, py::arg("useLOST") = false);
  m_.def("triangulateNonlinear", [](const gtsam::Pose3Vector &poses, boost::shared_ptr<gtsam::Cal3Unified> sharedCal, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(poses, sharedCal, measurements, initialEstimate); }, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateNonlinear", [](const gtsam::CameraSetCal3Unified &cameras, const gtsam::Point2Vector &measurements, const gtsam::Point3 &initialEstimate) { return gtsam::triangulateNonlinear(cameras, measurements, initialEstimate); }, py::arg("cameras"), py::arg("measurements"), py::arg("initialEstimate"));
  m_.def("triangulateSafe", [](const gtsam::CameraSetCal3Unified &cameras, const gtsam::Point2Vector &measurements, const gtsam::TriangulationParameters &params) { return gtsam::triangulateSafe(cameras, measurements, params); }, py::arg("cameras"), py::arg("measurements"), py::arg("params"));

// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/geometry.h"
}
