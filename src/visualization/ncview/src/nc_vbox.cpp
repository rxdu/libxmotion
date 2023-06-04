/**
 * @file nc_vbox.cpp
 * @date 1/31/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "ncview/nc_vbox.hpp"

#include <cassert>

namespace xmotion {
namespace swviz {
void NcVbox::AllocateSpace(int rows, int cols) {
  SpaceAllocator1D(rows, components_, allocated_sizes_);
}

void NcVbox::OnResize(int rows, int cols, int y, int x) {
  if (components_.empty()) return;

  AllocateSpace(rows, cols);

  int y_start = y;
  int y_allocated = 0;
  for (int i = 0; i < components_.size(); ++i) {
    components_[i].element->OnResize(allocated_sizes_[components_[i].element],
                                     cols, y_start, x);
    y_allocated += allocated_sizes_[components_[i].element];
    y_start = y + y_allocated;
  }
}
}  // namespace swviz
}  // namespace xmotion