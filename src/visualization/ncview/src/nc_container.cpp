/**
* @file nc_container.cpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#include "ncview/nc_container.hpp"

namespace robosw {
namespace swviz {
void NcContainer::AddElement(std::shared_ptr<NcElement> element) {
  elements_.push_back(element);
}
}
} // robosw