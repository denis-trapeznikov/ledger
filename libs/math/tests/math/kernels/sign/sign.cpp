#include <iomanip>
#include <iostream>

#include <gtest/gtest.h>
#include "math/kernels/sign.hpp"
#include "math/shape_less_array.hpp"


using namespace fetch::math;
using data_type = double;
using container_type = fetch::memory::SharedArray<data_type>;

ShapeLessArray<data_type, container_type> RandomArray(std::size_t n, data_type adj)
{
    static fetch::random::LinearCongruentialGenerator gen;
    ShapeLessArray<data_type, container_type> a1(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        a1.At(i) = data_type(gen.AsDouble()) + adj;
    }
    return a1;
}
ShapeLessArray<data_type, container_type> ConstantArray(std::size_t n, data_type adj)
{
  ShapeLessArray<data_type, container_type> a1(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    a1.At(i) = adj;
  }
  return a1;
}

TEST(ndarray, zeros_out) {
    std::size_t n = 1000;
    ShapeLessArray<data_type, container_type> test_array = ConstantArray(n, 0);
    ShapeLessArray<data_type, container_type> test_array_2 = RandomArray(n, -1.0);

    // sanity check that all values equal 0
    for (std::size_t i = 0; i < n; ++i)
    {
      ASSERT_TRUE(test_array[i] == 0);
    }

    // check that sign(0) = 0
    test_array_2.Sign(test_array);
    for (std::size_t i = 0; i < n; ++i)
    {
        ASSERT_TRUE(test_array_2[i] == 0);
    }
}

TEST(ndarray, negative_ones) {
    std::size_t n = 1000;
    ShapeLessArray<data_type, container_type> test_array = RandomArray(n, -1.0);
    ShapeLessArray<data_type, container_type> test_array_2 = RandomArray(n, 1.0);

    // sanity check that all values less than 0
    for ( std::size_t i = 0; i<n; ++i)
    {
        ASSERT_TRUE(test_array[i] <= 0);
    }

    // check that sign(-) = -1
    test_array_2.Sign(test_array);
    for (std::size_t i = 0; i<n; ++i)
    {
        ASSERT_TRUE(test_array_2[i] == -1);
    }
}

TEST(ndarray, positive_ones) {
    std::size_t n = 1000;
    ShapeLessArray<data_type, container_type> test_array = RandomArray(n, 1.0);
    ShapeLessArray<data_type, container_type> test_array_2 = RandomArray(n, -1.0);

    // sanity check that all values gt than 0
    for (std::size_t i = 0; i < n; ++i)
    {
        ASSERT_TRUE(test_array[i] >= 0);
    }

    test_array_2.Sign(test_array);
    for (std::size_t i = 0; i < n; ++i)
    {
        ASSERT_TRUE(test_array_2[i] == 1);
    }
}