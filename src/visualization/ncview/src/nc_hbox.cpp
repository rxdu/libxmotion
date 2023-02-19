/**
* @file nc_hbox.cpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#include "ncview/nc_hbox.hpp"

#include <cassert>

namespace robosw {
namespace swviz {
void NcHbox::AllocateSpace(int rows, int cols) {
  float allocated_ratio = 0;
  int allocated_cols = 0;
  for (int i = 0; i < components_.size(); ++i) {
    if (components_[i].constraint.type == NcConstraint::Type::kFixed) {
      allocated_sizes_[components_[i].element] = components_[i].constraint.ratio * cols;
      allocated_ratio += components_[i].constraint.ratio;
      allocated_cols += allocated_sizes_[components_[i].element];
    }
  }
  for (int i = 0; i < components_.size(); ++i) {
    if (components_[i].constraint.type == NcConstraint::Type::kFlexible) {
      allocated_sizes_[components_[i].element] = (1 - allocated_ratio) / num_of_flex_components_ * cols;
      allocated_cols += allocated_sizes_[components_[i].element];
    }
  }
  // assign the remaining space to the last element
  if (allocated_cols < cols) { allocated_sizes_[components_.back().element] += cols - allocated_cols; }
}

void NcHbox::OnResize(int rows, int cols, int y, int x) {
  AllocateSpace(rows, cols);

  int x_start = x;
  for (int i = 0; i < components_.size(); ++i) {
    components_[i].element->OnResize(rows,
                                     allocated_sizes_[components_[i].element],
                                     y, x_start);
    x_start = x + allocated_sizes_[components_[i].element];
  }
}
}
} // robosw