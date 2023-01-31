/**
* @file nc_container.cpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#include "ncview/details/nc_container.hpp"

namespace robosw {
namespace swviz {
void NcContainer::AddElement(std::shared_ptr<NcElement> element) {
  elements_.push_back(element);
}

void NcContainer::OnResize(int rows, int cols, int y, int x) {
  for (auto &element : elements_) {
    element->OnResize(rows, cols, y, x);
  }
};

void NcContainer::OnDraw() {
  for (auto &element : elements_) {
    element->OnDraw();
  }
};
}
} // robosw