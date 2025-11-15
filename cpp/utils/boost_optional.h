/*
    This is a slightly modified version of nanobind's optional.h to support
    boost::optional in addition to std::optional.
*/
/*
    nanobind/stl/optional.h: type caster for std::optional<...>

    Copyright (c) 2022 Yoshiki Matsuda and Wenzel Jakob

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE file.
*/

#pragma once

#include <boost/optional.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/detail/nb_optional.h>

NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)

template <typename T>
struct remove_opt_mono<boost::optional<T>>
    : remove_opt_mono<T> {};

template <typename T>
struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};

template <>
struct type_caster<boost::none_t> : none_caster<boost::none_t> {};

NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)
