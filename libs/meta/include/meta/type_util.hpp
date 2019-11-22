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

#include <algorithm>
#include <functional>
#include <type_traits>
#include <utility>

namespace fetch {
namespace type_util {

template<class T> struct Box {
	using type = T;
};

// Primary generic template introduced to compensate for fold-expressions missing from C++14.
template <template<class...> class F, class... Args> struct LeftAccumulate;
template <template<class...> class F, class... Args> using LeftAccumulateT = typename LeftAccumulate<F, Args...>::type;
template <template<class...> class F, class... Args> constexpr auto LeftAccumulateV = LeftAccumulate<F, Args...>::value;

template <template<class...> class F, class A0, class A1, class... As>
struct LeftAccumulate<F, A0, A1, As...>: LeftAccumulate<F, F<A0, A1>, As...> {};

template <template<class...> class F, class A0>
struct LeftAccumulate<F, A0>: Box<A0> {};

// Right fold.
template <template<class...> class F, class... Args> struct RightAccumulate;
template <template<class...> class F, class... Args> using RightAccumulateT = typename RightAccumulate<F, Args...>::type;
template <template<class...> class F, class... Args> constexpr auto RightAccumulateV = RightAccumulate<F, Args...>::value;

template <template<class...> class F, class A0, class A1, class... As>
struct RightAccumulate<F, A0, A1, As...>: F<A0, RightAccumulateT<F, A1, As...>> {};

template <template<class...> class F, class A0>
struct RightAccumulate<F, A0>: Box<A0> {};


// Simple static arithmetic definition.
template<class Operator> struct LiftArithmetic {
	template<class... Args> struct type {
		static constexpr auto value = Operator{}(Args::value...);
	};
};

// STL simple binary operators are very generic when specialized for void.
template<template<class...> class StlOperator> using LiftStlArithmetic = LiftArithmetic<StlOperator<void>>;


template<class... Ts>
using Conjunction = LeftAccumulate<LiftStlArithmetic<std::logical_and>::template type, std::true_type, Ts...>;

template <typename... Ts>
static constexpr auto ConjunctionV = Conjunction<Ts...>::value;

template <template <typename...> class F, typename... Ts>
using All = Conjunction<F<Ts>...>;


template <template <typename...> class F, typename... Ts>
static constexpr auto AllV = All<F, Ts...>::value;

template <typename... Ts>
using Disjunction = LeftAccumulate<LiftStlArithmetic<std::logical_or>::template type, std::false_type, Ts...>;

template <typename... Ts>
static constexpr auto DisjunctionV = Disjunction<Ts...>::value;

template <template <typename...> class F, typename... Ts>
using Any = Disjunction<F<Ts>...>;


template <template <typename...> class F, typename... Prefix>
struct Bind
{
  template <typename... Args>
  using type = F<Prefix..., Args...>;
};


template <typename T, typename... Ts>
using IsAnyOf = Disjunction<std::is_same<T, Ts>...>;

template <typename T, typename... Ts>
static constexpr auto IsAnyOfV = IsAnyOf<T, Ts...>::value;


template <typename T, template <typename...> class... Predicates>
using SatisfiesAll = Conjunction<Predicates<T>...>;

template <typename T, template <typename...> class... Predicates>
static constexpr bool SatisfiesAllV = SatisfiesAll<T, Predicates...>::value;


template <typename F, typename... Args>
struct InvokeResult
{
  using type = decltype(std::declval<F>()(std::declval<Args>()...));
};

template <typename F, typename... Args>
using InvokeResultT = typename InvokeResult<F, Args...>::type;

namespace seq {

template<class... Seqs> struct Concat;
template<class... Seqs> using ConcatT = typename Concat<Seqs...>::type;

template<class... Seqs> struct Concat: LeftAccumulate<ConcatT, Seqs...> {};

template<template<class T, T...> class IntegerSequence, class T, T... seq1, T... seq2>
struct Concat<IntegerSequence<T, seq1...>, IntegerSequence<T, seq2...>>: Box<IntegerSequence<T, seq1..., seq2...>> {};

}
}  // namespace type_util
}  // namespace fetch
