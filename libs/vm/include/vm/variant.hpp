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

template<TypeId... type_ids> using TypeIdSeq = std::integer_sequence<TypeId, type_ids...>;

using SignedIntTypeIds = TypeIdSeq<TypeIds::Int8, TypeIds::Int16, TypeIds::Int32, TypeIds::Int64>;
using UnsignedIntTypeIds = TypeIdSeq<TypeIds::UInt8, TypeIds::UInt16, TypeIds::UInt32, TypeIds::UInt64>;
using IntTypeIds = type_util::seq::Concat<SignedIntTypeIds, UnsignedIntTypeIds>;

using FloatTypeIds = TypeIdSeq<TypeIds::Float32, TypeIds::Float64>;
using FixedTypeIds = TypeIdSeq<TypeIds::Fixed32, TypeIds::Fixed64>;

using NumericTypeIds = type_util::seq::Concat<IntTypeIds, FloatTypeIds, FixedTypeIds>;

using PrimitiveTypeIds = detail_::ConsT<TypeIds::Bool, NumericTypeIds>;

using BuiltinPtrTypeIds = TypeIdSeq<TypeIds::String, TypeIds::Address, TypeIds::Fixed128>;

using BuiltinTypeIds = type_util::seq::Concat<PrimitiveTypeIds, BuiltinPtrTypeIds>;


namespace detail_
{

template<class T, class ST = T> struct DualBox {
	using type = T;
	using storage_type = ST;
};

template<TypeId type_id> struct TypeIdTraits: DoubleBox<Object> {};
template<TypeId type_id> using TypeIdTraitsT = typename TypeIdTraits<type_id>::type;

template<> struct TypeIdTraits<TypeIds::Void>: DoubleBox<void> {};
template<> struct TypeIdTraits<TypeIds::Null>: DoubleBox<std::nullptr_t> {};
template<> struct TypeIdTraits<TypeIds::Bool>: DoubleBox<bool, uint8_t> {};
template<> struct TypeIdTraits<TypeIds::Int8>: DoubleBox<int8_t> {};
template<> struct TypeIdTraits<TypeIds::UInt8>: DoubleBox<uint8_t> {};
template<> struct TypeIdTraits<TypeIds::Int16>: DoubleBox<int16_t> {};
template<> struct TypeIdTraits<TypeIds::UInt16>: DoubleBox<uint16_t> {};
template<> struct TypeIdTraits<TypeIds::Int32>: DoubleBox<int32_t> {};
template<> struct TypeIdTraits<TypeIds::UInt32>: DoubleBox<uint32_t> {};
template<> struct TypeIdTraits<TypeIds::Int64>: DoubleBox<int64_t> {};
template<> struct TypeIdTraits<TypeIds::UInt64>: DoubleBox<uint64_t> {};
template<> struct TypeIdTraits<TypeIds::Float>: DoubleBox<float> {};
template<> struct TypeIdTraits<TypeIds::Float>: DoubleBox<double> {};
template<> struct TypeIdTraits<TypeIds::Fixed32>: DoubleBox<fixed_point::fp32_t> {};
template<> struct TypeIdTraits<TypeIds::Fixed64>: DoubleBox<fixed_point::fp64_t> {};
template<> struct TypeIdTraits<TypeIds::String>: DoubleBox<String, Ptr<String>> {};
template<> struct TypeIdTraits<TypeIds::Address>: DoubleBox<Address, Ptr<Address>> {};

}

template<TypeId id, class VariantRef, class = std::decay_t<VariantRef>> struct VariantView: detail_::TypeIdTraits<id>
{
	static constexpr TypeId type_id = id;
	using Traits = detail_::TypeIdTraits<type_id>;
	using Traits::type;
	using Traits::storage_type;

	constexpr VariantView(VariantRef &var) noexcept: var(var) {}

	constexpr auto Get() const {
		return var.Get<type>();
	}

	constexpr void Set(type T) {
		var.Set(T);
	}

	constexpr decltype(auto) Ref() {
		// relaxed assertion: while type_id mismatch could indicate a flaw in logic,
		// the aforementioned flaw could be as much as a transient object inconsistency,
		// and nonetheless referring to a mistyped variant's object member would not
		// constitute an UB
		assert(type_id > TypeIds::PrimitiveMaxId || var.type_id == type_id);
		return var.Ref<type>();
	}

	constexpr decltype(auto) CRef() const {
		// relaxed assertion: while type_id mismatch could indicate a flaw in logic,
		// the aforementioned flaw could be as much as a transient object inconsistency,
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

template<TypeId id, class PrimitiveRef> struct VariantView<id, PrimitiveRef, Primitive>
{
	static_assert(id <= TypeIds::PrimitiveMaxId, "type_id is too large for Primitive types");

	static constexpr TypeId type_id = id;
	using type = typename TypeIdTraits<type_id>::type;
	using storage_type = typename TypeIdTraits<type_id>::storage_type;

	constexpr PrimitiveView(PrimitiveRef &primitive) noexcept: primitive(primitive) {}

	constexpr auto Get() const {
		assert(primitive.type_id == type_id);
		return primitive.Get<type>();
	}

	constexpr void Set(type T) {
		primitive.Set(T);
	}

	constexpr decltype(auto) Ref() {
		return primitive.Ref<type>();
	}

	constexpr decltype(auto) CRef() const {
		return primitive.CRef<type>();
	}

	PrimitiveRef &primitive;
};

// Used to convey type_id information for nullary invocations.
template<TypeId id> struct VariantView<id, void, void>: detail_::TypeIdTraits<id>
{
	static constexpr TypeId type_id = id;
	using Traits = TypeIdTraits<type_id>;
	using Traits::type;
	using Traits::storage_type;
};

namespace detail_
{

template<class Seq> struct Car;
template<class Seq> static constexpr auto CarV = Car<Seq>::value;

template<TypeId car, TypeId... cdr> struct Car<TypeIdSeq<car, cdr...>>: std::integral_constant<TypeId, car> {};

template<TypeId car, class Cdr> struct Cons;
template<TypeId car, class Cdr> using ConsT = typename Cons<car, Cdr>::type;

template<TypeId car, TypeId... cdr> struct Cons<car, TypeIdSeq<cdr...>>: type_util::Box<TypeIdSeq<car, cdr...>> {};

template<class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<>, TypeId /*unused*/, F &&f, Variants &&...variants)
{
	return std::forward<F>(f)(variants...);
}

template<class F>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<>, TypeId /*unused*/, F &&f)
{
	return std::forward<F>(f)();
}

template<TypeId id, class F>
constexpr decltype(auto) InvokeWith(F &&f)
{
	return std::forward<F>(f)(VariantView<id, void>{});
}

template<TypeId id, class F, class... Variants>
constexpr decltype(auto) InvokeWith(F &&f, Variants &&...variants)
{
	return std::forward<F>(f)(View<id>(std::forward<Variants>(variants))...);
}

template<TypeId id1, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id16: return InvokeWith<id16>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id16: return InvokeWith<id16>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id17: return InvokeWith<id17>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id16: return InvokeWith<id16>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id17: return InvokeWith<id17>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id18: return InvokeWith<id18>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, TypeId id19, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id16: return InvokeWith<id16>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id17: return InvokeWith<id17>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id18: return InvokeWith<id18>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id19: return InvokeWith<id19>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, TypeId id19, TypeId id20, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19, id20>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id16: return InvokeWith<id16>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id17: return InvokeWith<id17>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id18: return InvokeWith<id18>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id19: return InvokeWith<id19>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id20: return InvokeWith<id20>(std::forward<F>(f), std::forward<Variants>(variants)...);
		default:;
	}
	return std::forward<F>(f)(variants...);
}

template<TypeId id1, TypeId id2, TypeId id3, TypeId id4, TypeId id5, TypeId id6, TypeId id7, TypeId id8, TypeId id9, TypeId id10, TypeId id11, TypeId id12, TypeId id13, TypeId id14, TypeId id15, TypeId id16, TypeId id17, TypeId id18, TypeId id19, TypeId id20, TypeId id21, class F, class... Variants>
constexpr decltype(auto) ApplyFunctor(TypeIdSeq<id1, id2, id3, id4, id5, id6, id7, id8, id9, id10, id11, id12, id13, id14, id15, id16, id17, id18, id19, id20, id21>, TypeId id, F &&f, Variants &&...variants)
{
	switch(id) {
		case id1: return InvokeWith<id1>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id2: return InvokeWith<id2>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id3: return InvokeWith<id3>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id4: return InvokeWith<id4>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id5: return InvokeWith<id5>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id6: return InvokeWith<id6>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id7: return InvokeWith<id7>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id8: return InvokeWith<id8>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id9: return InvokeWith<id9>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id10: return InvokeWith<id10>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id11: return InvokeWith<id11>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id12: return InvokeWith<id12>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id13: return InvokeWith<id13>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id14: return InvokeWith<id14>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id15: return InvokeWith<id15>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id16: return InvokeWith<id16>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id17: return InvokeWith<id17>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id18: return InvokeWith<id18>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id19: return InvokeWith<id19>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id20: return InvokeWith<id20>(std::forward<F>(f), std::forward<Variants>(variants)...);
		case id21: return InvokeWith<id21>(std::forward<F>(f), std::forward<Variants>(variants)...);
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

template<class F, TypeId example_id>
struct InvokeResult<F, value_util::ArgSet<>, example_id>: type_util::InvokeResult<F, VariantView<example_id, void>> {};

template<class VariantsPack, class F>
constexpr DefaultSlot(F &&f)
{
	return value_util::Slot<VariantsPack>(std::forward<F>(f));
}

template<class F, class VariantsPack, TypeIdSeq for_example>
static constexpr decltype(auto) WithAddedDefault(F &&f)
{
	using Default = InvokeResultT<F, VariantsPack, CarV<for_example>>;
	return value_util::Slots(std::forward<F>(f),
				 DefaultSlot<VariantsPack>(value_util::NoOp<Default>{}));
}

template<class F, class... Variants>
class HasDefault
{
	template<class G> static constexpr Can(G &&/*unused*/)->decltype(G(std::declval<Variants>()...), std::true_type{});
	static constexpr std::false_type Can(...);
public:
	static constexpr bool value = decltype(Can(std::declval<F>()))::value;
};

template<class F>
class HasDefault<F>
{
	template<class G> static constexpr Can(G &&/*unused*/)->decltype(G(), std::true_type{});
	static constexpr std::false_type Can(...);
public:
	static constexpr bool value = decltype(Can(std::declval<F>()))::value;
};

template<class F, class... Variants> static constexpr auto HasDefaultV = HasDefault<F, Variants...>::value;

template<class F, class... Variants, class TypeIds>
static constexpr std::enable_if_t<HasDefaultV<F, Variants...>, F &&> WithDefault(F &&f, Variants &&.../*unused*/, TypeIds)
{
	return std::forward<F>(f);
}

template<class F, class... Variants, class TypeIds>
static constexpr std::enable_if_t<!HasDefaultV<F, Variants...>, F &&> WithDefault(F &&f, Variants &&.../*unused*/, TypeIds)
{
	return WithAddedDefault<F, value_util::ArgSet<Variants...>, TypeIds>(std::forward<F>(f));
}

template<TypeId type_id, class... VariantRefs> using VariantArgSet = std::conditional_t<sizeof...(VariantRefs) == 0,
	value_util::ArgSet<VariantView<type_id, void>>,
	value_util::ArgSet<VariantView<type_id, VariantRefs>...>>;

template<class F, TypeId... type_ids>
class VariantSlotType {
	F &&f_;
public:
	VariantSlot(F &&f_): f_(std::forward<F>(f_)) {}

	template<class F, class VariantRefs...> static constexpr auto Impl(VariantRefs &&.../*unused*/)
	{
		return value_util::Slot<VariantArgSet<type_ids, VariantRefs...>...>(std::forward<F>(f_));
	}
};
template<TypeId... type_ids, class F> constexpr auto VariantSlot(TypeIdSeq<type_ids...>, F &&f)
{
	return VariantSlotType<F, type_ids...>(std::forward<F>(f));
}

template<class F>
class DefaultSlotType {
	F &&f_;
public:
	VariantSlot(F &&f_): f_(std::forward<F>(f_)) {}

	template<class VariantRefs....> static constexpr auto Impl(VariantRefs &&.../*unused*/)
	{
		return value_util::Slot<value_util::ArgSet<VariantRefs...>>(std::forward<F>(f_));
	}
};
template<class F> constexpr auto DefaultSlot(F &&f)
{
	return DefaultSlotType<F>(std::forward<F>(f));
}

}

template<TypeId... type_ids> using TypeIdSeq = detail_::TypeIdSeq<type_ids...>;

template<class... IdSets, class F, class... Variants> constexpr decltype(auto) ApplyFunctor(TypeId type_id, F &&f, Variants &&...variants)
{
	using Ids = type_util::seq::ConcatT<IdSets...>;

	return  detail_::ApplyFunctor(Ids{}, type_id,
				     detail_::WithDefault(f.Impl<Variants>(), std::forward<Variants>(variants)..., Ids{}),
				     std::forward<Variants>(variants)...);
}

template<TypeId... type_ids, class F>
constexpr auto VariantSlot(F &&f)
{
  using Ids = TypeIdSeq<IdSets...>;

  return detail_::VariantSlot(Ids{}, std::forward<F>(f));
}

template<class... IdSets, class F>
constexpr auto VariantSlot(F &&f)
{
  using Ids = type_util::seq::Concat<IdSets...>;
  return detail_::VariantSlot(Ids{}, std::forward<F>(f));
}

template<class F>
constexpr auto DefaultSlot(F &&f)
{
  return detail_::DefaultSlot(std::forward<F>(f)):
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
						return true;
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
