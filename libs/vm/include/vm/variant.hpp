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

#include "core/serializers/base_types.hpp"
#include "core/serializers/main_serializer.hpp"
#include "meta/slots.hpp"
#include "meta/type_util.hpp"
#include "vectorise/fixed_point/fixed_point.hpp"
#include "vm/address.hpp"
#include "vm/object.hpp"
#include "vm/string.hpp"

#include <bitset>
#include <limits>

namespace fetch {
namespace vm {

union Primitive
{
  int8_t   i8;
  uint8_t  ui8;
  int16_t  i16;
  uint16_t ui16;
  int32_t  i32;
  uint32_t ui32;
  int64_t  i64;
  uint64_t ui64;
  float    f32;
  double   f64;

  template <typename T>
  auto Get() const noexcept;

  template<TypeId>
  auto &Ref() noexcept;

  template<TypeId>
  auto const &CRef() const noexcept;

  void Set(bool value) noexcept
  {
    ui8 = uint8_t(value);
  }

  void Set(int8_t value) noexcept
  {
    i8 = value;
  }

  void Set(uint8_t value) noexcept
  {
    ui8 = value;
  }

  void Set(int16_t value) noexcept
  {
    i16 = value;
  }

  void Set(uint16_t value) noexcept
  {
    ui16 = value;
  }

  void Set(int32_t value) noexcept
  {
    i32 = value;
  }

  void Set(uint32_t value) noexcept
  {
    ui32 = value;
  }

  void Set(int64_t value) noexcept
  {
    i64 = value;
  }

  void Set(uint64_t value) noexcept
  {
    ui64 = value;
  }

  void Set(float value) noexcept
  {
    f32 = value;
  }

  void Set(double value) noexcept
  {
    f64 = value;
  }

  void Set(fixed_point::fp32_t const &value) noexcept
  {
    i32 = value.Data();
  }

  void Set(fixed_point::fp64_t const &value) noexcept
  {
    i64 = value.Data();
  }
};

template <>
inline auto Primitive::Get<bool>() const noexcept
{
  return bool(ui8);
}

template <>
inline auto &Primitive::Ref<bool>() noexcept
{
  return ui8;
}

template <>
inline auto const &Primitive::Ref<bool>() const noexcept
{
  return ui8;
}

template <>
inline auto Primitive::Get<int8_t>() const noexcept
{
  return i8;
}

template <>
inline auto &Primitive::Ref<int8_t>() noexcept
{
  return i8;
}

template <>
inline auto const &Primitive::Ref<int8_t>() const noexcept
{
  return i8;
}

template <>
inline auto Primitive::Get<uint8_t>() const noexcept
{
  return ui8;
}

template <>
inline auto &Primitive::Ref<uint8_t>() noexcept
{
  return ui8;
}

template <>
inline auto const &Primitive::Ref<uint8_t>() const noexcept
{
  return ui8;
}

template <>
inline auto Primitive::Get<int16_t>() const noexcept
{
  return i16;
}

template <>
inline auto &Primitive::Ref<int16_t>() noexcept
{
  return i16;
}

template <>
inline auto const &Primitive::Ref<int16_t>() const noexcept
{
  return i16;
}

template <>
inline auto Primitive::Get<uint16_t>() const noexcept
{
  return ui16;
}

template <>
inline auto &Primitive::Ref<uint16_t>() noexcept
{
  return ui16;
}

template <>
inline auto const &Primitive::Ref<uint16_t>() const noexcept
{
  return ui16;
}

template <>
inline auto Primitive::Get<int32_t>() const noexcept
{
  return i32;
}

template <>
inline auto &Primitive::Ref<int32_t>() noexcept
{
  return i32;
}

template <>
inline auto const &Primitive::Ref<int32_t>() const noexcept
{
  return i32;
}

template <>
inline auto Primitive::Get<uint32_t>() const noexcept
{
  return ui32;
}

template <>
inline auto &Primitive::Ref<uint32_t>() noexcept
{
  return ui32;
}

template <>
inline auto const &Primitive::Ref<uint32_t>() const noexcept
{
  return ui32;
}

template <>
inline auto Primitive::Get<int64_t>() const noexcept
{
  return i64;
}

template <>
inline auto &Primitive::Ref<int64_t>() noexcept
{
  return i64;
}

template <>
inline auto const &Primitive::Ref<int64_t>() const noexcept
{
  return i64;
}

template <>
inline auto Primitive::Get<uint64_t>() const noexcept
{
  return ui64;
}

template <>
inline auto &Primitive::Ref<uint64_t>() noexcept
{
  return ui64;
}

template <>
inline auto const &Primitive::Ref<uint64_t>() const noexcept
{
  return ui64;
}

template <>
inline auto Primitive::Get<float>() const noexcept
{
  return f32;
}

template <>
inline auto &Primitive::Ref<float>() noexcept
{
  return f32;
}

template <>
inline auto const &Primitive::Ref<float>() const noexcept
{
  return f32;
}

template <>
inline auto Primitive::Get<double>() const noexcept
{
  return f64;
}

template <>
inline auto &Primitive::Ref<double>() noexcept
{
  return f64;
}

template <>
inline auto const &Primitive::Ref<double>() const noexcept
{
  return f64;
}

template <>
inline auto Primitive::Get<fixed_point::fp32_t>() const noexcept
{
  return fixed_point::fp32_t::FromBase(i32);
}

template <>
inline auto &Primitive::Ref<fixed_point::fp32_t>() noexcept
{
  return i32;
}

template <>
inline auto const &Primitive::Ref<fixed_point::fp32_t>() const noexcept
{
  return i32;
}

template <>
inline auto Primitive::Get<fixed_point::fp64_t>() const noexcept
{
  return fixed_point::fp64_t::FromBase(i64);
}

template <>
inline auto &Primitive::Ref<fixed_point::fp64_t>() noexcept
{
  return i64;
}

template <>
inline auto const &Primitive::Ref<fixed_point::fp64_t>() const noexcept
{
  return i64;
}

struct Variant
{
  union
  {
    Primitive   primitive{};
    Ptr<Object> object;
  };
  TypeId type_id = TypeIds::Unknown;

  Variant() noexcept
  {
    Construct();
  }

  Variant(Variant const &other) noexcept
  {
    Construct(other);
  }

  Variant(Variant &&other) noexcept
  {
    Construct(std::move(other));
  }

  template <typename T, std::enable_if_t<IsPrimitive<T>::value> * = nullptr>
  Variant(T other, TypeId other_type_id) noexcept
  {
    Construct(other, other_type_id);
  }

  template <typename T, typename std::enable_if_t<IsPtr<T>::value> * = nullptr>
  Variant(T const &other, TypeId other_type_id) noexcept
  {
    Construct(other, other_type_id);
  }

  template <typename T, typename std::enable_if_t<IsPtr<T>::value> * = nullptr>
  Variant(T &&other, TypeId other_type_id) noexcept
  {
    Construct(std::forward<T>(other), other_type_id);
  }

  Variant(Primitive other, TypeId other_type_id) noexcept
  {
    Construct(other, other_type_id);
  }

  void Construct() noexcept
  {
    type_id = TypeIds::Unknown;
  }

  void Construct(Variant const &other) noexcept
  {
    type_id = other.type_id;
    if (IsPrimitive())
    {
      primitive = other.primitive;
    }
    else
    {
      new (&object) Ptr<Object>(other.object);
    }
  }

  void Construct(Variant &&other) noexcept
  {
    type_id = other.type_id;
    if (IsPrimitive())
    {
      primitive = other.primitive;
    }
    else
    {
      new (&object) Ptr<Object>(std::move(other.object));
    }
    other.type_id = TypeIds::Unknown;
  }

  template <typename T>
  std::enable_if_t<IsPrimitive<T>::value> Construct(T other, TypeId other_type_id) noexcept
  {
    primitive.Set(other);
    type_id = other_type_id;
  }

  template <typename T>
  std::enable_if_t<IsPtr<T>::value> Construct(T const &other, TypeId other_type_id) noexcept
  {
    new (&object) Ptr<Object>(other);
    type_id = other_type_id;
  }

  template <typename T>
  std::enable_if_t<IsPtr<T>::value> Construct(T &&other, TypeId other_type_id) noexcept
  {
    new (&object) Ptr<Object>(std::forward<T>(other));
    type_id = other_type_id;
  }

  void Construct(Primitive other, TypeId other_type_id) noexcept
  {
    primitive = other;
    type_id   = other_type_id;
  }

  Variant &operator=(Variant const &other) noexcept
  {
    if (this != &other)
    {
      bool const is_object       = !IsPrimitive();
      bool const other_is_object = !other.IsPrimitive();
      type_id                    = other.type_id;
      if (is_object)
      {
        if (other_is_object)
        {
          // Copy object to current object
          object = other.object;
        }
        else
        {
          // Copy primitive to current object
          object.Reset();
          primitive = other.primitive;
        }
      }
      else
      {
        if (other_is_object)
        {
          // Copy object to current primitive
          new (&object) Ptr<Object>(other.object);
        }
        else
        {
          // Copy primitive to current primitive
          primitive = other.primitive;
        }
      }
    }
    return *this;
  }

  Variant &operator=(Variant &&other) noexcept
  {
    if (this != &other)
    {
      bool const is_object       = !IsPrimitive();
      bool const other_is_object = !other.IsPrimitive();
      type_id                    = other.type_id;
      other.type_id              = TypeIds::Unknown;
      if (is_object)
      {
        if (other_is_object)
        {
          // Move object to current object
          object = std::move(other.object);
        }
        else
        {
          // Move primitive to current object
          object.Reset();
          primitive = other.primitive;
        }
      }
      else
      {
        if (other_is_object)
        {
          // Move object to current primitive
          new (&object) Ptr<Object>(std::move(other.object));
        }
        else
        {
          // Move primitive to current primitive
          primitive = other.primitive;
        }
      }
    }
    return *this;
  }

  template <typename T>
  std::enable_if_t<IsPrimitive<T>::value> Assign(T other, TypeId other_type_id) noexcept
  {
    if (!IsPrimitive())
    {
      object.Reset();
    }
    primitive.Set(other);
    type_id = other_type_id;
  }

  template <typename T>
  std::enable_if_t<IsPtr<T>::value> Assign(T const &other, TypeId other_type_id) noexcept
  {
    if (IsPrimitive())
    {
      Construct(other, other_type_id);
    }
    else
    {
      object  = other;
      type_id = other_type_id;
    }
  }

  template <typename T>
  std::enable_if_t<IsPtr<T>::value> Assign(T &&other, TypeId other_type_id) noexcept
  {
    if (IsPrimitive())
    {
      Construct(std::forward<T>(other), other_type_id);
    }
    else
    {
      object  = std::forward<T>(other);
      type_id = other_type_id;
    }
  }

  template <typename T>
  std::enable_if_t<IsVariant<T>::value> Assign(T const &other, TypeId /* other_type_id */) noexcept
  {
    operator=(other);
  }

  template <typename T>
  std::enable_if_t<IsVariant<T>::value> Assign(T &&other, TypeId /* other_type_id */) noexcept
  {
    operator=(std::forward<T>(other));
  }

  template <typename T>
  constexpr std::enable_if_t<IsPrimitive<T>::value, T> Get() const noexcept
  {
    return primitive.Get<T>();
  }

  template <typename T>
  constexpr std::enable_if_t<IsPtr<T>::value, T> Get() const noexcept
  {
    return object;
  }

  template <typename T>
  constexpr std::enable_if_t<IsPrimitive<T>::value, T &> Ref() noexcept
  {
    return primitive.Ref<T>();
  }

  template <typename T>
  constexpr std::enable_if_t<IsPtr<T>::value, T &> Ref() noexcept
  {
    return object;
  }

  template <typename T>
  constexpr std::enable_if_t<IsPrimitive<T>::value, T const &> CRef() const noexcept
  {
    return primitive.CRef<T>();
  }

  template <typename T>
  constexpr std::enable_if_t<IsPtr<T>::value, T const &> CRef() const noexcept
  {
    return object;
  }

  template <typename T>
  std::enable_if_t<IsVariant<T>::value, T> Get() const noexcept
  {
    T variant;
    variant.type_id = type_id;
    if (IsPrimitive())
    {
      variant.primitive = primitive;
    }
    else
    {
      new (&variant.object) Ptr<Object>(object);
    }
    return variant;
  }

  template <typename T>
  std::enable_if_t<IsPrimitive<T>::value, T> Move() noexcept
  {
    type_id = TypeIds::Unknown;
    return primitive.Get<T>();
  }

  template <typename T>
  std::enable_if_t<IsPtr<T>::value, T> Move() noexcept
  {
    type_id = TypeIds::Unknown;
    return {std::move(object)};
  }

  template <typename T>
  constexpr std::enable_if_t<IsVariant<T>::value, T> Move() noexcept
  {
    T variant;
    variant.type_id = type_id;
    if (IsPrimitive())
    {
      variant.primitive = primitive;
    }
    else
    {
      new (&variant.object) Ptr<Object>(std::move(object));
    }
    type_id = TypeIds::Unknown;
    return variant;
  }

  ~Variant()
  {
    Reset();
  }

  constexpr bool IsPrimitive() const noexcept
  {
    return type_id <= TypeIds::PrimitiveMaxId;
  }

  constexpr void Reset() noexcept
  {
    if (!IsPrimitive())
    {
      object.Reset();
    }
    type_id = TypeIds::Unknown;
  }
};
using VariantArray = std::vector<Variant>;

template<TypeId id, class VariantRef> struct VariantView
{
	using type = IdToTypeT<type_id>;

	constexpr VariantView(VariantRef &var) noexcept: var(var) {}

	constexpr auto Get() const {
		assert(var.type_id == type_id);
		return var.Get<type>();
	}

	static constexpr void Set(type T) {
		var.Assign(std::move(T), type_id);
	}

	static constexpr T &Ref() {
		// relaxed assertion: while type_id mismatch could indicate a flaw in logic,
		// the afforementioned flaw could be as much as a transient object inconsistency,
		// and nonetheless referring to a mistyped variant's object member would not
		// constitute an UB
		assert(type_id > TypeIds::PrimitiveMaxId || var.type_id == type_id);
		return var.Ref<type>();
	}

	static constexpr T &CRef() const {
		// relaxed assertion: while type_id mismatch could indicate a flaw in logic,
		// the afforementioned flaw could be as much as a transient object inconsistency,
		// and nonetheless referring to a mistyped variant's object member would not
		// constitute an UB
		assert(type_id > TypeIds::PrimitiveMaxId || var.type_id == type_id);
		return var.CRef<type>();
	}

	VariantRef &var;
};
template<TypeId id, class Ref> constexpr decltype(auto) View(Ref &var) noexcept { return VariantView<id, Ref>(var); }

template<TypeId type_id> using VarView = VariantView<type_id, Variant &>;
template<TypeId type_id> using CVarView = VariantView<type_id, Variant const &>;

namespace detail_
{

template<TypeId type_id> struct IdToType: type_util::Box<Object> {};
template<TypeId type_id> using IdToTypeT = typename IdToType<type_id>::type;

template<> struct IdToType<TypeIds::Void>: type_util::Box<void> {};
template<> struct IdToType<TypeIds::Bool>: type_util::Box<bool> {};
template<> struct IdToType<TypeIds::Int8>: type_util::Box<int8_t> {};
template<> struct IdToType<TypeIds::Int8>: type_util::Box<int8_t> {};
template<> struct IdToType<TypeIds::Int16>: type_util::Box<int16_t> {};
template<> struct IdToType<TypeIds::Int16>: type_util::Box<int16_t> {};
template<> struct IdToType<TypeIds::Int32>: type_util::Box<int32_t> {};
template<> struct IdToType<TypeIds::Int32>: type_util::Box<int32_t> {};
template<> struct IdToType<TypeIds::Int64>: type_util::Box<int64_t> {};
template<> struct IdToType<TypeIds::Int64>: type_util::Box<int64_t> {};
template<> struct IdToType<TypeIds::Float>: type_util::Box<float> {};
template<> struct IdToType<TypeIds::Float>: type_util::Box<double> {};
template<> struct IdToType<TypeIds::Fixed32>: type_util::Box<fixed_point::fp32_t> {};
template<> struct IdToType<TypeIds::Fixed64>: type_util::Box<fixed_point::fp64_t> {};
template<> struct IdToType<TypeIds::String>: type_util::Box<String> {};
template<> struct IdToType<TypeIds::Address>: type_util::Box<Address> {};

template<class T> struct TypeToId: std::integral_constant<TypeId, TypeIds::NumReserved> {};
template<class T> static constexpr auto TypeToIdV = TypeToId<T>::value;

template<> struct TypeToId<void>: std::integral_constant<TypeId, TypeIds::Void> {};
template<> struct TypeToId<bool>: std::integral_constant<TypeId, TypeIds::Bool> {};
template<> struct TypeToId<int8_t>: std::integral_constant<TypeId, TypeIds::Int8> {};
template<> struct TypeToId<int8_t>: std::integral_constant<TypeId, TypeIds::Int8> {};
template<> struct TypeToId<int16_t>: std::integral_constant<TypeId, TypeIds::Int16> {};
template<> struct TypeToId<int16_t>: std::integral_constant<TypeId, TypeIds::Int16> {};
template<> struct TypeToId<int32_t>: std::integral_constant<TypeId, TypeIds::Int32> {};
template<> struct TypeToId<int32_t>: std::integral_constant<TypeId, TypeIds::Int32> {};
template<> struct TypeToId<int64_t>: std::integral_constant<TypeId, TypeIds::Int64> {};
template<> struct TypeToId<int64_t>: std::integral_constant<TypeId, TypeIds::Int64> {};
template<> struct TypeToId<float>: std::integral_constant<TypeId, TypeIds::Float> {};
template<> struct TypeToId<double>: std::integral_constant<TypeId, TypeIds::Float> {};
template<> struct TypeToId<fixed_point::fp32_t>: std::integral_constant<TypeId, TypeIds::Fixed32> {};
template<> struct TypeToId<fixed_point::fp64_t>: std::integral_constant<TypeId, TypeIds::Fixed64> {};
template<> struct TypeToId<String>: std::integral_constant<TypeId, TypeIds::String> {};
template<> struct TypeToId<Address>: std::integral_constant<TypeId, TypeIds::Address> {};

template<TypeId... type_ids> using TypeIdSeq = std::integer_sequence<TypeId, type_ids...>;

template<class Seq> struct Car;
template<class Seq> static constexpr auto CarV = Car<Seq>::value;

template<TypeId car, TypeId... cdr> struct Car<TypeIdSeq<car, cdr...>>: std::integral_constant<TypeId, car> {};

template<class F, class RV, TypeId... type_ids> constexpr decltype(auto) SeqAccumulate(F &&f, RV init, TypeIdSeq<type_ids...>)
{
	return value_util::LeftAccumulate(std::forward<F>(f), init, type_ids...);
}

template<class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<>, TypeId /*unused*/, F &&f, Variants &&...variants)
{
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		case id16: return std::forward<F>(f)(View<id16>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		case id16: return std::forward<F>(f)(View<id16>(variants)...);
		case id17: return std::forward<F>(f)(View<id17>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		case id16: return std::forward<F>(f)(View<id16>(variants)...);
		case id17: return std::forward<F>(f)(View<id17>(variants)...);
		case id18: return std::forward<F>(f)(View<id18>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, TypeId id19, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		case id16: return std::forward<F>(f)(View<id16>(variants)...);
		case id17: return std::forward<F>(f)(View<id17>(variants)...);
		case id18: return std::forward<F>(f)(View<id18>(variants)...);
		case id19: return std::forward<F>(f)(View<id19>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, TypeId id19, TypeId id20, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19, id20>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		case id16: return std::forward<F>(f)(View<id16>(variants)...);
		case id17: return std::forward<F>(f)(View<id17>(variants)...);
		case id18: return std::forward<F>(f)(View<id18>(variants)...);
		case id19: return std::forward<F>(f)(View<id19>(variants)...);
		case id20: return std::forward<F>(f)(View<id20>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, TypeId id19, TypeId id20, TypeId id21, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19, id20, id21>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return std::forward<F>(f)(View<id1>(variants)...);
		case id2: return std::forward<F>(f)(View<id2>(variants)...);
		case id3: return std::forward<F>(f)(View<id3>(variants)...);
		case id4: return std::forward<F>(f)(View<id4>(variants)...);
		case id5: return std::forward<F>(f)(View<id5>(variants)...);
		case id6: return std::forward<F>(f)(View<id6>(variants)...);
		case id7: return std::forward<F>(f)(View<id7>(variants)...);
		case id8: return std::forward<F>(f)(View<id8>(variants)...);
		case id9: return std::forward<F>(f)(View<id9>(variants)...);
		case id10: return std::forward<F>(f)(View<id10>(variants)...);
		case id11: return std::forward<F>(f)(View<id11>(variants)...);
		case id12: return std::forward<F>(f)(View<id12>(variants)...);
		case id13: return std::forward<F>(f)(View<id13>(variants)...);
		case id14: return std::forward<F>(f)(View<id14>(variants)...);
		case id15: return std::forward<F>(f)(View<id15>(variants)...);
		case id16: return std::forward<F>(f)(View<id16>(variants)...);
		case id17: return std::forward<F>(f)(View<id17>(variants)...);
		case id18: return std::forward<F>(f)(View<id18>(variants)...);
		case id19: return std::forward<F>(f)(View<id19>(variants)...);
		case id20: return std::forward<F>(f)(View<id20>(variants)...);
		case id21: return std::forward<F>(f)(View<id21>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

// 21 distinct id should be enough for everyone.

template<class F, class VariantsPack, TypeId example_id>
struct InvokeResult;

template<class F, class VariantsPack, TypeId example_id> using InvokeResultT = typename InvokeResult<F, VariantsPack, example_id>::type;

template<class F, class... Variants, TypeId example_id>
struct InvokeResult<F, value_util::ArgSet<Variants...>, example_id>: type_util::InvokeResult<F, VariantView<example_id, Variants>...> {};

template<class F, class VariantsPack, TypeIdSeq for_example>
static constexpr decltype(auto) WithAddedDefault(F &&f)
{
	using Default = InvokeResultT<F, VariantsPack, CarV<for_example>>;
	return value_util::Slots(std::forward<F>(f),
				 value_util::Slot<VariantsPack>(value_util::NoOp<Default>{}));
}

template<class F, class... Variants>
class HasDefault
{
	template<class G> static constexpr Can(G &&/*unused*/)->decltype(G(std::declval<Variants>()...), std::true_type{});
	static constexpr std::false_type Can(...);
public:
	static constexpr bool value = decltype(Can(std::declval<F>()))::value;
};

template<class F, class... Variants> static constexpr auto HasDefaultV = HasDefault<F, Variants...>::value;

template<class F, class... Variants>
static constexpr std::enable_if_t<HasDefaultV<F, Variants...>, F &&> WithDefault(F &&f, Variants &&.../*unused*/)
{
	return std::forward<F>(f);
}

template<class F, class... Variants>
static constexpr std::enable_if_t<!HasDefaultV<F, Variants...>, F &&> WithDefault(F &&f, Variants &&.../*unused*/)
{
	return WithAddedDefault(std::forward<F>(f));
}

}

template<TypeId... type_ids> using TypeIdSeq = detail_::TypeIdSeq<type_ids...>;

template<class... IdSets, class F, class... Variants> constexpr ApplyFunctor(TypeId type_id, F &&f, Variants &&...variants)
{
	using IdSet = type_util::seq::ConcatT<IdSets...>;
	static constexpr auto MaxId = type_util::seq::MaxV<IdSet>;
	static constexpr auto elder_bit = sizeof(uint64_t) * 8;

	static_assert(MaxId < elder_bit, "TypeId is too large currently");

	static constexpr uint64_t mask = detail_::SeqAccumulate([](uint64_t accum, auto type_index) { return accum | 1ull << type_index; }, 0ull, IdSet{});
	static constexpr std::bitset<static_cast<std::size_t>(MaxId)> type_id_filter(mask);

	if(
}

struct TemplateParameter1 : Variant
{
  using Variant::Variant;
};

struct TemplateParameter2 : Variant
{
  using Variant::Variant;
};

struct Any : Variant
{
  using Variant::Variant;
};

struct AnyPrimitive : Variant
{
  using Variant::Variant;
};

struct AnyInteger : Variant
{
  using Variant::Variant;
};

struct AnyFloatingPoint : Variant
{
  using Variant::Variant;
};

}  // namespace vm

namespace serializers {

template <typename D>
struct MapSerializer<fetch::vm::Variant, D>
{
public:
  using Type       = fetch::vm::Variant;
  using DriverType = D;

  static uint8_t const TYPEID    = 1;
  static uint8_t const PRIMITIVE = 2;

  template <typename Constructor>
  static void Serialize(Constructor &map_constructor, Type const &variant)
  {
    auto map = map_constructor(2);
    // TODO(tfr): This is dangerous: Type Id should never be serialized
    // dt: Any alternatives? How is it more dangerous than the value itself?
    map.Append(TYPEID, variant.type_id);

    if (!ApplyFunctor<PrimitiveTypeIds>(variant.type_id,
					VariantSlot<>([](auto &&var) {
						// primitive type variant
						map.Append(PRIMITIVE, var.Get());
						return true;
					}),
					variant))
    {
      // true was never returned, variant is not primitive
      // not supported yet
      throw std::runtime_error{"Serialization of Variant of Object type is not supported"};
    }
  }

  template <typename MapDeserializer>
  static void Deserialize(MapDeserializer &map, Type &variant)
  {
    map.ExpectKeyGetValue(TYPEID, variant.type_id);

    if (!ApplyFunctor<PrimitiveTypeIds>(variant.type_id,
					VariantSlot<>([](auto &&var) {
						map.ExpectKeyGetValue(PRIMITIVE, var.Ref());
					}),
					variant))
    {
      // true was never returned, type_id is not primitive
      throw std::runtime_error{"Deserialization of objects not possible for variants."};
    }
  }
};

}  // namespace serializers
}  // namespace fetch
