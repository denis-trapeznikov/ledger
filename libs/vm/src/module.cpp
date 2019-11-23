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

#include "vectorise/fixed_point/fixed_point.hpp"
#include "vm/address.hpp"
#include "vm/array.hpp"
#include "vm/common.hpp"
#include "vm/map.hpp"
#include "vm/matrix.hpp"
#include "vm/module.hpp"
#include "vm/sharded_state.hpp"
#include "vm/state.hpp"
#include "vm/string.hpp"
#include "vm/variant.hpp"
#include "vm/vm.hpp"

#include <cstdint>

namespace fetch {
namespace vm {

namespace {

template <typename To>
To Cast(Variant const &from)
{
  return ApplyFunctor<PrimitiveTypeIds>(from.type_id,
					value_util::Slots(
						VariantSlot<PrimitiveTypeIds>([](auto &&var) {
							return static_cast<To>(var.Get());
						}),
						DefaultSlot([](Variant const &/*unused*/) {
							// Not a primitive
							assert(false);
							return T{};
						})),
					from);
}

int8_t toInt8(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<int8_t>(from);
}

uint8_t toUInt8(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<uint8_t>(from);
}

int16_t toInt16(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<int16_t>(from);
}

uint16_t toUInt16(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<uint16_t>(from);
}

int32_t toInt32(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<int32_t>(from);
}

uint32_t toUInt32(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<uint32_t>(from);
}

int64_t toInt64(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<int64_t>(from);
}

uint64_t toUInt64(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<uint64_t>(from);
}

float toFloat32(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<float>(from);
}

double toFloat64(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<double>(from);
}

fixed_point::fp32_t toFixed32(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<fixed_point::fp32_t>(from);
}

fixed_point::fp64_t toFixed64(VM * /* vm */, AnyPrimitive const &from)
{
  return Cast<fixed_point::fp64_t>(from);
}

}  // namespace

Module::Module()
{
  CreateFreeFunction("toInt8", &toInt8);
  CreateFreeFunction("toUInt8", &toUInt8);
  CreateFreeFunction("toInt16", &toInt16);
  CreateFreeFunction("toUInt16", &toUInt16);
  CreateFreeFunction("toInt32", &toInt32);
  CreateFreeFunction("toUInt32", &toUInt32);
  CreateFreeFunction("toInt64", &toInt64);
  CreateFreeFunction("toUInt64", &toUInt64);
  CreateFreeFunction("toFloat32", &toFloat32);
  CreateFreeFunction("toFloat64", &toFloat64);
  CreateFreeFunction("toFixed32", &toFixed32);
  CreateFreeFunction("toFixed64", &toFixed64);

  CreateTemplateType<IMatrix, AnyFloatingPoint>("Matrix")
      .CreateConstructor(&IMatrix::Constructor)
      .EnableIndexOperator(&IMatrix::GetIndexedValue, &IMatrix::SetIndexedValue)
      .CreateInstantiationType<Matrix<double>>()
      .CreateInstantiationType<Matrix<float>>()
      .EnableOperator(Operator::Negate)
      .EnableOperator(Operator::Add)
      .EnableOperator(Operator::Subtract)
      .EnableOperator(Operator::Multiply)
      .EnableOperator(Operator::InplaceAdd)
      .EnableOperator(Operator::InplaceSubtract)
      .EnableLeftOperator(Operator::Multiply)
      .EnableRightOperator(Operator::Add)
      .EnableRightOperator(Operator::Subtract)
      .EnableRightOperator(Operator::Multiply)
      .EnableRightOperator(Operator::Divide)
      .EnableRightOperator(Operator::InplaceAdd)
      .EnableRightOperator(Operator::InplaceSubtract)
      .EnableRightOperator(Operator::InplaceMultiply)
      .EnableRightOperator(Operator::InplaceDivide);

  GetClassInterface<IArray>()
      .CreateConstructor(&IArray::Constructor)
      .CreateSerializeDefaultConstructor(
          [](VM *vm, TypeId type_id) { return IArray::Constructor(vm, type_id, 0u); })
      .CreateMemberFunction("append", &IArray::Append)
      .CreateMemberFunction("count", &IArray::Count)
      .CreateMemberFunction("erase", &IArray::Erase)
      .CreateMemberFunction("extend", &IArray::Extend)
      .CreateMemberFunction("popBack", &IArray::PopBackOne)
      .CreateMemberFunction("popBack", &IArray::PopBackMany)
      .CreateMemberFunction("popFront", &IArray::PopFrontOne)
      .CreateMemberFunction("popFront", &IArray::PopFrontMany)
      .CreateMemberFunction("reverse", &IArray::Reverse)
      .EnableIndexOperator(&IArray::GetIndexedValue, &IArray::SetIndexedValue)
      .CreateInstantiationType<Array<bool>>()
      .CreateInstantiationType<Array<int8_t>>()
      .CreateInstantiationType<Array<uint8_t>>()
      .CreateInstantiationType<Array<int16_t>>()
      .CreateInstantiationType<Array<uint16_t>>()
      .CreateInstantiationType<Array<int32_t>>()
      .CreateInstantiationType<Array<uint32_t>>()
      .CreateInstantiationType<Array<int64_t>>()
      .CreateInstantiationType<Array<uint64_t>>()
      .CreateInstantiationType<Array<float>>()
      .CreateInstantiationType<Array<double>>()
      .CreateCPPCopyConstructor<std::vector<double>>(
          [](VM *vm, TypeId, std::vector<double> const &arr) -> Ptr<IArray> {
            auto ret = Ptr<Array<double>>(
                new Array<double>(vm, vm->GetTypeId<Array<double>>(), vm->GetTypeId<double>(), 0));
            ret->elements = arr;
            return ret;
          })
      .CreateCPPCopyConstructor<std::vector<std::vector<double>>>(
          [](VM *vm, TypeId, std::vector<std::vector<double>> const &arr) -> Ptr<IArray> {
            auto outerid = vm->GetTypeId<Array<Ptr<Array<double>>>>();
            auto innerid = vm->GetTypeId<Array<double>>();
            std::cout << "ID: " << vm->GetTypeId<Array<Ptr<Array<double>>>>() << std::endl;
            auto ret = Ptr<Array<Ptr<Array<double>>>>(
                new Array<Ptr<Array<double>>>(vm, outerid, innerid, 0));

            for (auto &element : arr)
            {
              auto a = Ptr<Array<double>>(new Array<double>(vm, vm->GetTypeId<Array<double>>(),
                                                            vm->GetTypeId<double>(), 0));

              a->elements = element;
              ret->elements.emplace_back(a);
            }

            return ret;
          })
      .CreateInstantiationType<Array<fixed_point::fp32_t>>()
      .CreateInstantiationType<Array<fixed_point::fp64_t>>()
      .CreateInstantiationType<Array<Ptr<String>>>()
      .CreateInstantiationType<Array<Ptr<Address>>>();
  /*
              std::cout << " " << vm->GetTypeId<Array<Ptr<Object>>>()
                        << " " << vm->GetTypeId<Array<Ptr<IArray>>>()
                        << " " << vm->GetTypeId<IArray>()
                        << " " << vm->GetTypeId<Array<Ptr<Array<double>>>>() << " "
                        << vm->GetTypeId<Array<double>>() << std::endl;

  */
  GetClassInterface<String>()
      .CreateSerializeDefaultConstructor(
          [](VM *vm, TypeId) -> Ptr<String> { return Ptr<String>{new String(vm, "")}; })
      .CreateCPPCopyConstructor<std::string>(
          [](VM *vm, TypeId, std::string const &s) -> Ptr<String> {
            return Ptr<String>{new String(vm, s)};
          })
      .CreateMemberFunction("find", &String::Find)
      .CreateMemberFunction("length", &String::Length)
      .CreateMemberFunction("sizeInBytes", &String::SizeInBytes)
      .CreateMemberFunction("reverse", &String::Reverse)
      .CreateMemberFunction("split", &String::Split)
      .CreateMemberFunction("substr", &String::Substring)
      .CreateMemberFunction("trim", &String::Trim);

  GetClassInterface<IMap>()
      .CreateConstructor(&IMap::Constructor)
      .CreateMemberFunction("count", &IMap::Count)
      .EnableIndexOperator(&IMap::GetIndexedValue, &IMap::SetIndexedValue);

  GetClassInterface<Address>()
      .CreateSerializeDefaultConstructor(&Address::Constructor)
      .CreateConstructor(&Address::ConstructorFromString)
      .CreateMemberFunction("signedTx", &Address::HasSignedTx);

  CreateFreeFunction("toString", &Address::ToString);

  GetClassInterface<IState>()
      .CreateConstructor(&IState::ConstructorFromString)
      .CreateConstructor(&IState::ConstructorFromAddress)
      .CreateMemberFunction("get", &IState::Get)
      .CreateMemberFunction("get", &IState::GetWithDefault)
      .CreateMemberFunction("set", &IState::Set)
      .CreateMemberFunction("existed", &IState::Existed);

  GetClassInterface<IShardedState>()
      .CreateConstructor(&IShardedState::ConstructorFromString)
      .CreateConstructor(&IShardedState::ConstructorFromAddress)
      // TODO (issue 1172): This will be enabled once the issue is resolved
      //.EnableIndexOperator<Ptr<String>, TemplateParameter1>()
      //.EnableIndexOperator<Ptr<Address>, TemplateParameter1>();
      .CreateMemberFunction("get", &IShardedState::GetFromString)
      .CreateMemberFunction("get", &IShardedState::GetFromAddress)
      .CreateMemberFunction("get", &IShardedState::GetFromStringWithDefault)
      .CreateMemberFunction("get", &IShardedState::GetFromAddressWithDefault)
      .CreateMemberFunction("set", &IShardedState::SetFromString)
      .CreateMemberFunction("set", &IShardedState::SetFromAddress);
}

}  // namespace vm
}  // namespace fetch
