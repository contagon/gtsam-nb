/**
 * @file    gtsam.cpp
 * @brief   The auto-generated wrapper C++ source code.
 * @author  Duy-Nguyen Ta, Fan Jiang, Matthew Sklar, Varun Agrawal
 * @date    Aug. 18, 2020
 *
 * ** THIS FILE IS AUTO-GENERATED, DO NOT MODIFY! **
 */

#define PYBIND11_DETAILED_ERROR_MESSAGES

// Include relevant boost libraries required by GTSAM
#include <boost/shared_ptr.hpp>

#include "gtsam/base/utilities.h" // for RedirectCout.
#include "gtsam/config.h"
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/vector.h>

// These are the included headers listed in `gtsam.i`
#include "gtsam/inference/Key.h"
#include "gtsam/nonlinear/utilities.h"

// TODO: Need to add in boost optional support
#include "utils/boost_shared_ptr.h"
#include "utils/preamble.h"
#include "utils/specializations.h"

using namespace std;

namespace nb = nanobind;

// void base(nb::module_ &);
// void inference(nb::module_ &);
// void discrete(nb::module_ &);
// void geometry(nb::module_ &);
// void linear(nb::module_ &);
// void nonlinear(nb::module_ &);
// void custom(nb::module_ &);
// void symbolic(nb::module_ &);
// void sam(nb::module_ &);
// void slam(nb::module_ &);
// void sfm(nb::module_ &);
// void navigation(nb::module_ &);
// void basis(nb::module_ &);
// void hybrid(nb::module_ &);

NB_MODULE(_core, m_) {
  m_.doc() = "nanobind wrapper of gtsam";
  m_.def(
      "abi_tag",
      []() { return nb::detail::abi_tag(); },
      "Get the ABI tag of the current module. Useful for debugging when extending gtsam.");

  bind_specializations(m_);

  //   base(m_);
  //   inference(m_);
  //   discrete(m_);
  //   geometry(m_);
  //   linear(m_);
  //   nonlinear(m_);
  //   custom(m_);
  //   symbolic(m_);
  //   sam(m_);
  //   slam(m_);
  //   sfm(m_);
  //   navigation(m_);
  //   basis(m_);
  //   hybrid(m_);

  m_.attr("DefaultKeyFormatter") = gtsam::DefaultKeyFormatter;
  nb::class_<gtsam::KeyList>(m_, "KeyList")
      .def(nb::init<>())
      .def(nb::init<const gtsam::KeyList &>(), nb::arg("other"))
      .def("size", [](gtsam::KeyList *self) { return self->size(); })
      .def("empty", [](gtsam::KeyList *self) { return self->empty(); })
      .def("clear", [](gtsam::KeyList *self) { self->clear(); })
      .def("front", [](gtsam::KeyList *self) { return self->front(); })
      .def("back", [](gtsam::KeyList *self) { return self->back(); })
      .def("push_back", [](gtsam::KeyList *self, size_t key) { self->push_back(key); }, nb::arg("key"))
      .def("push_front", [](gtsam::KeyList *self, size_t key) { self->push_front(key); }, nb::arg("key"))
      .def("pop_back", [](gtsam::KeyList *self) { self->pop_back(); })
      .def("pop_front", [](gtsam::KeyList *self) { self->pop_front(); })
      .def("sort", [](gtsam::KeyList *self) { self->sort(); })
      .def("remove", [](gtsam::KeyList *self, size_t key) { self->remove(key); }, nb::arg("key"));
  // .def("serialize", [](gtsam::KeyList *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::KeyList *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::KeyList &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::KeyList obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::KeySet>(m_, "KeySet")
      .def(nb::init<>())
      .def(nb::init<const gtsam::KeySet &>(), nb::arg("set"))
      .def(nb::init<const gtsam::KeyVector &>(), nb::arg("vector"))
      .def(nb::init<const gtsam::KeyList &>(), nb::arg("list"))
      .def("print", [](gtsam::KeySet *self, string s) { /*nb::scoped_ostream_redirect output;*/ self->print(s); }, nb::arg("s") = "")
      .def("__repr__", [](const gtsam::KeySet &self, string s) {
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str(); }, nb::arg("s") = "")
      .def("equals", [](gtsam::KeySet *self, const gtsam::KeySet &other) { return self->equals(other); }, nb::arg("other"))
      .def("size", [](gtsam::KeySet *self) { return self->size(); })
      .def("empty", [](gtsam::KeySet *self) { return self->empty(); })
      .def("clear", [](gtsam::KeySet *self) { self->clear(); })
      .def("insert", [](gtsam::KeySet *self, size_t key) { self->insert(key); }, nb::arg("key"))
      .def("merge", [](gtsam::KeySet *self, const gtsam::KeySet &other) { self->merge(other); }, nb::arg("other"))
      .def("erase", [](gtsam::KeySet *self, size_t key) { return self->erase(key); }, nb::arg("key"))
      .def("count", [](gtsam::KeySet *self, size_t key) { return self->count(key); }, nb::arg("key"));
  // .def("serialize", [](gtsam::KeySet *self) { return gtsam::serialize(*self); })
  // .def("deserialize", [](gtsam::KeySet *self, string serialized) { gtsam::deserialize(serialized, *self); }, nb::arg("serialized"))
  // .def(nb::pickle([](const gtsam::KeySet &a) { /* __getstate__: Returns a string that encodes the state of the object */ return nb::make_tuple(gtsam::serialize(a)); }, [](nb::tuple t) { /* __setstate__ */ gtsam::KeySet obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

  nb::class_<gtsam::KeyGroupMap>(m_, "KeyGroupMap")
      .def(nb::init<>())
      .def("size", [](gtsam::KeyGroupMap *self) { return self->size(); })
      .def("empty", [](gtsam::KeyGroupMap *self) { return self->empty(); })
      .def("clear", [](gtsam::KeyGroupMap *self) { self->clear(); })
      .def("at", [](gtsam::KeyGroupMap *self, size_t key) { return self->at(key); }, nb::arg("key"))
      .def("erase", [](gtsam::KeyGroupMap *self, size_t key) { return self->erase(key); }, nb::arg("key"))
      .def("insert2", [](gtsam::KeyGroupMap *self, size_t key, int val) { return self->insert2(key, val); }, nb::arg("key"), nb::arg("val"));

  // TODO: These need more modules to work first
  // auto m_utilities = m_.def_submodule("utilities", "utilities submodule");
  // m_utilities.def("createKeyList", [](const gtsam::Vector &I) { return gtsam::utilities::createKeyList(I); }, nb::arg("I"));
  // m_utilities.def("createKeyList", [](string s, const gtsam::Vector &I) { return gtsam::utilities::createKeyList(s, I); }, nb::arg("s"), nb::arg("I"));
  // m_utilities.def("createKeyVector", [](const gtsam::Vector &I) { return gtsam::utilities::createKeyVector(I); }, nb::arg("I"));
  // m_utilities.def("createKeyVector", [](string s, const gtsam::Vector &I) { return gtsam::utilities::createKeyVector(s, I); }, nb::arg("s"), nb::arg("I"));
  // m_utilities.def("createKeySet", [](const gtsam::Vector &I) { return gtsam::utilities::createKeySet(I); }, nb::arg("I"));
  // m_utilities.def("createKeySet", [](string s, const gtsam::Vector &I) { return gtsam::utilities::createKeySet(s, I); }, nb::arg("s"), nb::arg("I"));
  // m_utilities.def("extractPoint2", [](const gtsam::Values &values) { return gtsam::utilities::extractPoint2(values); }, nb::arg("values"));
  // m_utilities.def("extractPoint3", [](const gtsam::Values &values) { return gtsam::utilities::extractPoint3(values); }, nb::arg("values"));
  // m_utilities.def("allPose2s", [](gtsam::Values &values) { return gtsam::utilities::allPose2s(values); }, nb::arg("values"));
  // m_utilities.def("extractPose2", [](const gtsam::Values &values) { return gtsam::utilities::extractPose2(values); }, nb::arg("values"));
  // m_utilities.def("allPose3s", [](gtsam::Values &values) { return gtsam::utilities::allPose3s(values); }, nb::arg("values"));
  // m_utilities.def("extractPose3", [](const gtsam::Values &values) { return gtsam::utilities::extractPose3(values); }, nb::arg("values"));
  // m_utilities.def("extractVectors", [](const gtsam::Values &values, char c) { return gtsam::utilities::extractVectors(values, c); }, nb::arg("values"), nb::arg("c"));
  // m_utilities.def("perturbPoint2", [](gtsam::Values &values, double sigma, int seed) { gtsam::utilities::perturbPoint2(values, sigma, seed); }, nb::arg("values"), nb::arg("sigma"), nb::arg("seed") = 42u);
  // m_utilities.def("perturbPose2", [](gtsam::Values &values, double sigmaT, double sigmaR, int seed) { gtsam::utilities::perturbPose2(values, sigmaT, sigmaR, seed); }, nb::arg("values"), nb::arg("sigmaT"), nb::arg("sigmaR"), nb::arg("seed") = 42u);
  // m_utilities.def("perturbPoint3", [](gtsam::Values &values, double sigma, int seed) { gtsam::utilities::perturbPoint3(values, sigma, seed); }, nb::arg("values"), nb::arg("sigma"), nb::arg("seed") = 42u);
  // m_utilities.def("insertBackprojections", [](gtsam::Values &values, const gtsam::PinholeCamera<gtsam::Cal3_S2> &c, const gtsam::Vector &J, const gtsam::Matrix &Z, double depth) { gtsam::utilities::insertBackprojections(values, c, J, Z, depth); }, nb::arg("values"), nb::arg("c"), nb::arg("J"), nb::arg("Z"), nb::arg("depth"));
  // m_utilities.def("insertProjectionFactors", [](gtsam::NonlinearFactorGraph &graph, size_t i, const gtsam::Vector &J, const gtsam::Matrix &Z, const boost::shared_ptr<gtsam::noiseModel::Base> model, const boost::shared_ptr<gtsam::Cal3_S2> K, const gtsam::Pose3 &body_P_sensor) { gtsam::utilities::insertProjectionFactors(graph, i, J, Z, model, K, body_P_sensor); }, nb::arg("graph"), nb::arg("i"), nb::arg("J"), nb::arg("Z"), nb::arg("model"), nb::arg("K"), nb::arg("body_P_sensor") = gtsam::Pose3());
  // m_utilities.def("reprojectionErrors", [](const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &values) { return gtsam::utilities::reprojectionErrors(graph, values); }, nb::arg("graph"), nb::arg("values"));
  // m_utilities.def("localToWorld", [](const gtsam::Values &local, const gtsam::Pose2 &base) { return gtsam::utilities::localToWorld(local, base); }, nb::arg("local"), nb::arg("base"));
  // m_utilities.def("localToWorld", [](const gtsam::Values &local, const gtsam::Pose2 &base, const gtsam::KeyVector &keys) { return gtsam::utilities::localToWorld(local, base, keys); }, nb::arg("local"), nb::arg("base"), nb::arg("keys"));
  // nb::class_<gtsam::RedirectCout>(m_, "RedirectCout")
  //     .def(nb::init<>())
  //     .def("str", [](gtsam::RedirectCout *self) { return self->str(); });
}
