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

#include "core/byte_array/decoders.hpp"
#include "core/byte_array/encoders.hpp"
#include "core/serialisers/group_definitions.hpp"
#include "core/serialisers/main_serialiser.hpp"

#include "gtest/gtest.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

using namespace fetch::byte_array;

namespace fetch {

namespace serialisers {

struct HelloWorld
{
  bool    compact{false};
  int32_t schema{0};
};

template <typename D>
struct MapSerialiser<HelloWorld, D>
{
public:
  using Type       = HelloWorld;
  using DriverType = D;

  template <typename Constructor>
  static void Serialise(Constructor &map_constructor, Type const &input)
  {
    auto map = map_constructor(2);
    map.Append("compact", input.compact);
    map.Append("schema", input.schema);
  }

  template <typename MapDeserialiser>
  static void Deserialise(MapDeserialiser &map, Type &output)
  {
    for (uint64_t i = 0; i < map.size(); ++i)
    {
      std::string key;
      map.GetKey(key);

      if (key == "compact")
      {
        map.GetValue(output.compact);
      }
      else if (key == "schema")
      {
        map.GetValue(output.schema);
      }
      else
      {
        throw std::runtime_error("unrecognised key: " + key);
      }
    }
  }
};

TEST(MsgPacker, SimpleTypes)
{
  MsgPackSerialiser stream;
  HelloWorld        a1, b1;
  a1.compact = true;
  a1.schema  = 3;

  stream = MsgPackSerialiser();
  stream << a1;
  EXPECT_EQ(FromHex("82a7636f6d70616374c3a6736368656d6103"), stream.data());
  stream.seek(0);
  stream >> b1;
  EXPECT_EQ(a1.compact, b1.compact);
  EXPECT_EQ(a1.schema, b1.schema);

  // Nested types
  std::map<std::string, std::vector<int32_t>> a2, b2;
  a2["compact"] = {1, 2, 3};
  a2["empty"]   = {};
  a2["schema"]  = {256, 257, 258, 259};
  stream        = MsgPackSerialiser();
  stream << a2;
  EXPECT_EQ(
      FromHex("83a7636f6d7061637493010203a5656d70747990a6736368656d6194cd0100cd0101cd0102cd0103"),
      stream.data());
  stream.seek(0);
  stream >> b2;
  EXPECT_EQ(a2, b2);
}

}  // namespace serialisers
}  // namespace fetch