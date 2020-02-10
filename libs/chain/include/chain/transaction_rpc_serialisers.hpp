#pragma once
//------------------------------------------------------------------------------
//
//   Copyright 2018-2020 Fetch.AI Limited
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

#include "chain/transaction.hpp"
#include "chain/transaction_serialiser.hpp"
#include "core/byte_array/const_byte_array.hpp"
#include "core/byte_array/encoders.hpp"

namespace fetch {
namespace serialisers {

template <typename D>
struct ForwardSerialiser<chain::Transaction, D>
{
public:
  using Type       = chain::Transaction;
  using DriverType = D;

  template <typename Serialiser>
  static void Serialise(Serialiser &s, Type const &tx)
  {
    chain::TransactionSerialiser serialiser{};
    serialiser << tx;
    s << serialiser.data();
  }

  template <typename Serialiser>
  static void Deserialise(Serialiser &s, Type &tx)
  {
    // extract the data from the stream
    byte_array::ConstByteArray data;
    s >> data;

    // create and extract the serialiser
    chain::TransactionSerialiser serialiser{data};
    serialiser >> tx;
  }
};

}  // namespace serialisers
}  // namespace fetch