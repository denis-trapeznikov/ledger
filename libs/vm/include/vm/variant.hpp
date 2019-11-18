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

  template<TypeId>
  auto const &Ref() const noexcept;

  template<TypeId>
  auto &Ref() noexcept;

  template <typename T>
  auto Get() const noexcept;

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
inline auto Primitive::Get<int8_t>() const noexcept
{
  return i8;
}

template <>
inline auto Primitive::Get<uint8_t>() const noexcept
{
  return ui8;
}

template <>
inline auto Primitive::Get<int16_t>() const noexcept
{
  return i16;
}

template <>
inline auto Primitive::Get<uint16_t>() const noexcept
{
  return ui16;
}

template <>
inline auto Primitive::Get<int32_t>() const noexcept
{
  return i32;
}

template <>
inline auto Primitive::Get<uint32_t>() const noexcept
{
  return ui32;
}

template <>
inline auto Primitive::Get<int64_t>() const noexcept
{
  return i64;
}

template <>
inline auto Primitive::Get<uint64_t>() const noexcept
{
  return ui64;
}

template <>
inline auto Primitive::Get<float>() const noexcept
{
  return f32;
}

template <>
inline auto Primitive::Get<double>() const noexcept
{
  return f64;
}

template <>
inline auto Primitive::Get<fixed_point::fp32_t>() const noexcept
{
  return fixed_point::fp32_t::FromBase(i32);
}

template <>
inline auto Primitive::Get<fixed_point::fp64_t>() const noexcept
{
  return fixed_point::fp64_t::FromBase(i64);
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

template<class F, class RV, TypeId... type_ids> constexpr decltype(auto) SeqAccumulate(F &&f, RV init, TypeIdSeq<type_ids...>)
{
	return value_util::LeftAccumulate(std::forward<F>(f), init, type_ids...);
}

}

template<TypeId type_id> struct VariantView
{
	using type = IdToTypeT<type_id>;

	static constexpr type Get(Variant const &v) {
		assert(v.type_id == type_id);
		return v.Get<type>();
	}

	static constexpr void Set(Variant &v, type T) {
		v.Assign(std::move(T), type_id);
	}
};

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
    // Neg on that.
    map.Append(TYPEID, variant.type_id);

    // primitive type variant

    if (variant.IsPrimitive())
    {
      // Since primitive is a union it suffices
      // that we store one of the values.
      uint64_t val = variant.primitive.ui64;
      map.Append(PRIMITIVE, val);
    }
    else
    {
      // not supported yet
      throw std::runtime_error{"Serialization of Variant of Object type is not supported"};
    }
  }

  template <typename MapDeserializer>
  static void Deserialize(MapDeserializer &map, Type &variant)
  {
    map.ExpectKeyGetValue(TYPEID, variant.type_id);

    if (variant.IsPrimitive())
    {
      uint64_t val;
      map.ExpectKeyGetValue(PRIMITIVE, val);
      variant.primitive.ui64 = val;
    }
    else
    {
      throw std::runtime_error{"Deserialization of objects not possible for variants."};
    }
  }
};

}  // namespace serializers
}  // namespace fetch
