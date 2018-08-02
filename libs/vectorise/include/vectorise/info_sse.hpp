#pragma once

#include <cstddef>
#include <cstdint>
#include <emmintrin.h>
#include <immintrin.h>
#include <smmintrin.h>

namespace fetch {
namespace vectorize {

template <>
struct VectorInfo<uint8_t, 128>
{
  typedef uint8_t naitve_type;
  typedef __m128i register_type;
};

template <>
struct VectorInfo<uint16_t, 128>
{
  typedef uint16_t naitve_type;
  typedef __m128i  register_type;
};

template <>
struct VectorInfo<uint32_t, 128>
{
  typedef uint32_t naitve_type;
  typedef __m128i  register_type;
};

template <>
struct VectorInfo<uint64_t, 128>
{
  typedef uint64_t naitve_type;
  typedef __m128i  register_type;
};

template <>
struct VectorInfo<int, 128>
{
  typedef int     naitve_type;
  typedef __m128i register_type;
};

template <>
struct VectorInfo<float, 128>
{
  typedef float  naitve_type;
  typedef __m128 register_type;
};

template <>
struct VectorInfo<double, 128>
{
  typedef double  naitve_type;
  typedef __m128d register_type;
};
}  // namespace vectorize
}  // namespace fetch