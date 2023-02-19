/**
* @file nc_container.cpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#include "ncview/details/nc_container.hpp"

#include <iostream>

namespace robosw {
namespace swviz {
bool NcContainer::AddElement(std::shared_ptr<NcElement> element, NcConstraint constraint) {
  if (!ValidateNewConstraint(constraint)) {
    std::cerr << "[ERROR] Constraint for the new element cannot be satisfied!" << std::endl;
    return false;
  }
  if (constraint.type == NcConstraint::Type::kFlexible) { ++num_of_flex_components_; }
  components_.push_back({element, constraint});
  return true;
}

void NcContainer::OnResize(int rows, int cols, int y, int x) {
  for (auto &component : components_) {
    component.element->OnResize(rows, cols, y, x);
  }
}

void NcContainer::OnDraw() {
  for (auto &component : components_) {
    component.element->OnDraw();
  }
}

bool NcContainer::ValidateNewConstraint(const NcConstraint &constraint) {
  double allocated_ratio = 0;
  for (const auto &component : components_) {
    if (component.constraint.type == NcConstraint::Type::kFixed)
      allocated_ratio += component.constraint.ratio;
  }
  if (allocated_ratio + constraint.ratio > 1.0) return false;
  return true;
}
}
} // robosw