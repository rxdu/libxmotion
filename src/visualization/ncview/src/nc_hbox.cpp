/**
 * @file nc_hbox.cpp
 * @date 1/29/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "ncview/nc_hbox.hpp"

#include <cassert>

namespace xmotion {
namespace swviz {
void NcHbox::AllocateSpace(int rows, int cols) {
  SpaceAllocator1D(cols, components_, allocated_sizes_);
}

void NcHbox::OnResize(int rows, int cols, int y, int x) {
  if (components_.empty()) return;

  AllocateSpace(rows, cols);

  int x_start = x;
  int x_allocated = 0;
  for (int i = 0; i < components_.size(); ++i) {
    components_[i].element->OnResize(
        rows, allocated_sizes_[components_[i].element], y, x_start);
    x_allocated += allocated_sizes_[components_[i].element];
    x_start = x + x_allocated;
  }
}
}  // namespace swviz
}  // namespace xmotion