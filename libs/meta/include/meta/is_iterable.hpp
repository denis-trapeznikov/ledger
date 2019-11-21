#pragma once
//------------------------------------------------------------------------------
//
//   Copyright 2018-2019 Fetch.AI Limited
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
//------------------------------------------------------------------------------

#include "meta/type_util.hpp"

#include <iterator>
#include <type_traits>
#include <utility>

namespace fetch {
namespace meta {
namespace detail {

template <typename T>
auto IsIterableImplementation(int)
    -> decltype(std::begin(std::declval<T &>()) != std::end(std::declval<T &>()),
                ++std::declval<decltype(std::begin(std::declval<T &>())) &>(),
                *std::begin(std::declval<T &>()), std::true_type{});

template <typename T>
std::false_type IsIterableImplementation(...);
}  // namespace detail

template <typename T>
using IsIterable = decltype(detail::IsIterableImplementation<T>(0));

template <typename T>
constexpr auto IsIterableV = IsIterable<T>::value;

template <typename T, typename R>
using IfIterable = std::enable_if_t<IsIterableV<T>, R>;

template <typename T1, typename T2>
using AreBothIterable = type_util::All<IsIterable, T1, T2>;

template<typename T1, typename T2>
static constexpr auto AreBothIterableV = AreBothIterable<T1, T2>::value;

template <typename T1, typename T2, typename R>
using IfBothIterable = std::enable_if_t<AreBothIterableV<T1, T2>, R>;

}  // namespace meta
}  // namespace fetch
