/**
* @file nc_vbox.cpp
* @date 1/31/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#include "ncview/nc_vbox.hpp"

#include <cassert>

namespace robosw {
namespace swviz {
void NcVbox::AllocateSpace(int rows, int cols) {
  float allocated_ratio = 0;
  int allocated_rows = 0;
  for (int i = 0; i < components_.size(); ++i) {
    if (components_[i].constraint.type == NcConstraint::Type::kFixed) {
      allocated_sizes_[components_[i].element] = components_[i].constraint.ratio * rows;
      allocated_ratio += components_[i].constraint.ratio;
      allocated_rows += allocated_sizes_[components_[i].element];
    }
  }
  for (int i = 0; i < components_.size(); ++i) {
    if (components_[i].constraint.type == NcConstraint::Type::kFlexible) {
      allocated_sizes_[components_[i].element] = (1 - allocated_ratio) / num_of_flex_components_ * rows;
      allocated_rows += allocated_sizes_[components_[i].element];
    }
  }
  // assign the remaining space to the last element
  if (allocated_rows < rows) { allocated_sizes_[components_.back().element] += rows - allocated_rows; }
}

void NcVbox::OnResize(int rows, int cols, int y, int x) {
  if (components_.empty()) return;

  AllocateSpace(rows, cols);

  int y_start = y;
  for (int i = 0; i < components_.size(); ++i) {
    components_[i].element->OnResize(allocated_sizes_[components_[i].element], cols,
                                     y_start, x);
    y_start = y + allocated_sizes_[components_[i].element];
  }
}
}
} // robosw