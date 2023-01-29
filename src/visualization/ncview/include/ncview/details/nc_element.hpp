/**
* @file ncelement.hpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_ELEMENT_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_ELEMENT_HPP_

#include "ncview/nc_types.hpp"

namespace robosw {
namespace swviz {
struct NcElement {
  virtual ~NcElement() = default;

  // common interface
  virtual void OnResize() {};
  virtual void OnDraw() = 0;
};
}
} // robosw

#endif //ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_ELEMENT_HPP_
