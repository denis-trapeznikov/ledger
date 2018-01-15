#ifndef SERIALIZER_REFERENCED_BYTE_ARRAY_HPP
#define SERIALIZER_REFERENCED_BYTE_ARRAY_HPP
#include "byte_array/referenced_byte_array.hpp"

#include <type_traits>

namespace fetch {
namespace serializers {

template <typename T>
void Serialize(T &serializer, byte_array::ReferencedByteArray const &s) {
  serializer.Allocate(sizeof(uint64_t) + s.size());
  uint64_t size = s.size();

  serializer.WriteBytes(reinterpret_cast<uint8_t const *>(&size),
                        sizeof(uint64_t));
  serializer.WriteBytes(reinterpret_cast<uint8_t const *>(s.pointer()),
                        s.size());
}

template <typename T>
void Deserialize(T &serializer, byte_array::ReferencedByteArray &s) {
  uint64_t size = 0;

  serializer.ReadBytes(reinterpret_cast<uint8_t *>(&size), sizeof(uint64_t));
  s.Resize(size);
  serializer.ReadBytes(reinterpret_cast<uint8_t *>(s.pointer()), s.size());
}
};
};

#endif
