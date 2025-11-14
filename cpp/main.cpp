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
#include "gtsam/inference/Key.h"
#include "gtsam/nonlinear/utilities.h"

#include <boost/serialization/export.hpp>

// Export classes for serialization
BOOST_CLASS_EXPORT(gtsam::KeyList)
BOOST_CLASS_EXPORT(gtsam::KeySet)


// Holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(TYPE_PLACEHOLDER_DONOTUSE, boost::shared_ptr<TYPE_PLACEHOLDER_DONOTUSE>);

// Preamble for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/preamble/gtsam.h"

using namespace std;

namespace py = pybind11;

void base(py::module_ &);
void inference(py::module_ &);
void discrete(py::module_ &);
void geometry(py::module_ &);
void linear(py::module_ &);
void nonlinear(py::module_ &);
void custom(py::module_ &);
void symbolic(py::module_ &);
void sam(py::module_ &);
void slam(py::module_ &);
void sfm(py::module_ &);
void navigation(py::module_ &);
void basis(py::module_ &);
void hybrid(py::module_ &);

PYBIND11_MODULE(gtsam, m_) {
    m_.doc() = "pybind11 wrapper of gtsam";

base(m_);
inference(m_);
discrete(m_);
geometry(m_);
linear(m_);
nonlinear(m_);
custom(m_);
symbolic(m_);
sam(m_);
slam(m_);
sfm(m_);
navigation(m_);
basis(m_);
hybrid(m_);


    m_.attr("DefaultKeyFormatter") = gtsam::DefaultKeyFormatter;
    py::class_<gtsam::KeyList, boost::shared_ptr<gtsam::KeyList>>(m_, "KeyList")
        .def(py::init<>())
        .def(py::init<const gtsam::KeyList&>(), py::arg("other"))
        .def("size",[](gtsam::KeyList* self){return self->size();})
        .def("empty",[](gtsam::KeyList* self){return self->empty();})
        .def("clear",[](gtsam::KeyList* self){ self->clear();})
        .def("front",[](gtsam::KeyList* self){return self->front();})
        .def("back",[](gtsam::KeyList* self){return self->back();})
        .def("push_back",[](gtsam::KeyList* self, size_t key){ self->push_back(key);}, py::arg("key"))
        .def("push_front",[](gtsam::KeyList* self, size_t key){ self->push_front(key);}, py::arg("key"))
        .def("pop_back",[](gtsam::KeyList* self){ self->pop_back();})
        .def("pop_front",[](gtsam::KeyList* self){ self->pop_front();})
        .def("sort",[](gtsam::KeyList* self){ self->sort();})
        .def("remove",[](gtsam::KeyList* self, size_t key){ self->remove(key);}, py::arg("key"))
        .def("serialize", [](gtsam::KeyList* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::KeyList* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::KeyList &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::KeyList obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::KeySet, boost::shared_ptr<gtsam::KeySet>>(m_, "KeySet")
        .def(py::init<>())
        .def(py::init<const gtsam::KeySet&>(), py::arg("set"))
        .def(py::init<const gtsam::KeyVector&>(), py::arg("vector"))
        .def(py::init<const gtsam::KeyList&>(), py::arg("list"))
        .def("print",[](gtsam::KeySet* self, string s){ py::scoped_ostream_redirect output; self->print(s);}, py::arg("s") = "")
        .def("__repr__",
                    [](const gtsam::KeySet& self, string s){
                        gtsam::RedirectCout redirect;
                        self.print(s);
                        return redirect.str();
                    }, py::arg("s") = "")
        .def("equals",[](gtsam::KeySet* self, const gtsam::KeySet& other){return self->equals(other);}, py::arg("other"))
        .def("size",[](gtsam::KeySet* self){return self->size();})
        .def("empty",[](gtsam::KeySet* self){return self->empty();})
        .def("clear",[](gtsam::KeySet* self){ self->clear();})
        .def("insert",[](gtsam::KeySet* self, size_t key){ self->insert(key);}, py::arg("key"))
        .def("merge",[](gtsam::KeySet* self, const gtsam::KeySet& other){ self->merge(other);}, py::arg("other"))
        .def("erase",[](gtsam::KeySet* self, size_t key){return self->erase(key);}, py::arg("key"))
        .def("count",[](gtsam::KeySet* self, size_t key){return self->count(key);}, py::arg("key"))
        .def("serialize", [](gtsam::KeySet* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::KeySet* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::KeySet &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::KeySet obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::KeyGroupMap, boost::shared_ptr<gtsam::KeyGroupMap>>(m_, "KeyGroupMap")
        .def(py::init<>())
        .def("size",[](gtsam::KeyGroupMap* self){return self->size();})
        .def("empty",[](gtsam::KeyGroupMap* self){return self->empty();})
        .def("clear",[](gtsam::KeyGroupMap* self){ self->clear();})
        .def("at",[](gtsam::KeyGroupMap* self, size_t key){return self->at(key);}, py::arg("key"))
        .def("erase",[](gtsam::KeyGroupMap* self, size_t key){return self->erase(key);}, py::arg("key"))
        .def("insert2",[](gtsam::KeyGroupMap* self, size_t key, int val){return self->insert2(key, val);}, py::arg("key"), py::arg("val"));
    pybind11::module m_utilities = m_.def_submodule("utilities", "utilities submodule");

    m_utilities.def("createKeyList",[](const gtsam::Vector& I){return gtsam::utilities::createKeyList(I);}, py::arg("I"));
    m_utilities.def("createKeyList",[](string s, const gtsam::Vector& I){return gtsam::utilities::createKeyList(s, I);}, py::arg("s"), py::arg("I"));
    m_utilities.def("createKeyVector",[](const gtsam::Vector& I){return gtsam::utilities::createKeyVector(I);}, py::arg("I"));
    m_utilities.def("createKeyVector",[](string s, const gtsam::Vector& I){return gtsam::utilities::createKeyVector(s, I);}, py::arg("s"), py::arg("I"));
    m_utilities.def("createKeySet",[](const gtsam::Vector& I){return gtsam::utilities::createKeySet(I);}, py::arg("I"));
    m_utilities.def("createKeySet",[](string s, const gtsam::Vector& I){return gtsam::utilities::createKeySet(s, I);}, py::arg("s"), py::arg("I"));
    m_utilities.def("extractPoint2",[](const gtsam::Values& values){return gtsam::utilities::extractPoint2(values);}, py::arg("values"));
    m_utilities.def("extractPoint3",[](const gtsam::Values& values){return gtsam::utilities::extractPoint3(values);}, py::arg("values"));
    m_utilities.def("allPose2s",[](gtsam::Values& values){return gtsam::utilities::allPose2s(values);}, py::arg("values"));
    m_utilities.def("extractPose2",[](const gtsam::Values& values){return gtsam::utilities::extractPose2(values);}, py::arg("values"));
    m_utilities.def("allPose3s",[](gtsam::Values& values){return gtsam::utilities::allPose3s(values);}, py::arg("values"));
    m_utilities.def("extractPose3",[](const gtsam::Values& values){return gtsam::utilities::extractPose3(values);}, py::arg("values"));
    m_utilities.def("extractVectors",[](const gtsam::Values& values, char c){return gtsam::utilities::extractVectors(values, c);}, py::arg("values"), py::arg("c"));
    m_utilities.def("perturbPoint2",[](gtsam::Values& values, double sigma, int seed){ gtsam::utilities::perturbPoint2(values, sigma, seed);}, py::arg("values"), py::arg("sigma"), py::arg("seed") = 42u);
    m_utilities.def("perturbPose2",[](gtsam::Values& values, double sigmaT, double sigmaR, int seed){ gtsam::utilities::perturbPose2(values, sigmaT, sigmaR, seed);}, py::arg("values"), py::arg("sigmaT"), py::arg("sigmaR"), py::arg("seed") = 42u);
    m_utilities.def("perturbPoint3",[](gtsam::Values& values, double sigma, int seed){ gtsam::utilities::perturbPoint3(values, sigma, seed);}, py::arg("values"), py::arg("sigma"), py::arg("seed") = 42u);
    m_utilities.def("insertBackprojections",[](gtsam::Values& values, const gtsam::PinholeCamera<gtsam::Cal3_S2>& c, const gtsam::Vector& J, const gtsam::Matrix& Z, double depth){ gtsam::utilities::insertBackprojections(values, c, J, Z, depth);}, py::arg("values"), py::arg("c"), py::arg("J"), py::arg("Z"), py::arg("depth"));
    m_utilities.def("insertProjectionFactors",[](gtsam::NonlinearFactorGraph& graph, size_t i, const gtsam::Vector& J, const gtsam::Matrix& Z, const boost::shared_ptr<gtsam::noiseModel::Base> model, const boost::shared_ptr<gtsam::Cal3_S2> K, const gtsam::Pose3& body_P_sensor){ gtsam::utilities::insertProjectionFactors(graph, i, J, Z, model, K, body_P_sensor);}, py::arg("graph"), py::arg("i"), py::arg("J"), py::arg("Z"), py::arg("model"), py::arg("K"), py::arg("body_P_sensor") = gtsam::Pose3());
    m_utilities.def("reprojectionErrors",[](const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values){return gtsam::utilities::reprojectionErrors(graph, values);}, py::arg("graph"), py::arg("values"));
    m_utilities.def("localToWorld",[](const gtsam::Values& local, const gtsam::Pose2& base){return gtsam::utilities::localToWorld(local, base);}, py::arg("local"), py::arg("base"));
    m_utilities.def("localToWorld",[](const gtsam::Values& local, const gtsam::Pose2& base, const gtsam::KeyVector& keys){return gtsam::utilities::localToWorld(local, base, keys);}, py::arg("local"), py::arg("base"), py::arg("keys"));
    py::class_<gtsam::RedirectCout, boost::shared_ptr<gtsam::RedirectCout>>(m_, "RedirectCout")
        .def(py::init<>())
        .def("str",[](gtsam::RedirectCout* self){return self->str();});


// Specializations for STL classes
// TODO(fan): make this automatic
#include "python/gtsam/specializations/gtsam.h"

}

