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

#include "ml/charge_estimation/types.hpp"

namespace fetch {
namespace ml {
namespace charge_estimation {
namespace ops {

static constexpr OperationsCount DROPOUT_PER_ELEMENT             = 3;
static constexpr OperationsCount RELU_PER_ELEMENT                = 1;
static constexpr OperationsCount SIGMOID_PER_ELEMENT             = 2;
static constexpr OperationsCount ADDITION_PER_ELEMENT            = 1;
static constexpr OperationsCount SUBTRACTION_PER_ELEMENT         = ADDITION_PER_ELEMENT;
static constexpr OperationsCount FLATTEN_PER_ELEMENT             = 1;
static constexpr OperationsCount MULTIPLICATION_PER_ELEMENT      = 3;
static constexpr OperationsCount PLACEHOLDER_READING_PER_ELEMENT = 0;
static constexpr OperationsCount WEIGHTS_READING_PER_ELEMENT     = 0;

}  // namespace ops
}  // namespace charge_estimation
}  // namespace ml
}  // namespace fetch