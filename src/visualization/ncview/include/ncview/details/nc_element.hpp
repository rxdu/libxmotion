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

namespace xmotion {
namespace swviz {
struct NcElement {
  virtual ~NcElement() = default;

  // common interface
  virtual void OnResize(int rows, int cols, int y, int x){};
  virtual void OnDraw() = 0;
};
}  // namespace swviz
}  // namespace xmotion

#endif  // ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_ELEMENT_HPP_
