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
void NcVbox::OnResize(int rows, int cols, int y, int x) {
  int div = elements_.size();
  assert(div != 0 && "Vbox divider cannot be zero!");
  int row_height = rows / div;
  for (int i = 0; i < elements_.size(); ++i) {
    elements_[i]->OnResize(row_height, cols, y + row_height * i, x);
  }
}
}
} // robosw