/**
 * @file realvector_state.cpp
 * @date 16-01-2023
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "sampling/space/realvector_state.hpp"

namespace xmotion {
template <int32_t N>
std::atomic<std::size_t> RealVectorState<N>::count = {0};
}