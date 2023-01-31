
/**
* @file nc_vbox.hpp
* @date 1/31/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_VBOX_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_VBOX_HPP_

#include "ncview/nc_container.hpp"

namespace robosw {
namespace swviz {
class NcVbox : public NcContainer {
 public:
  NcVbox() = default;

  void OnResize(int rows, int cols, int y, int x) final;
};
}
} // robosw

#endif //ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_VBOX_HPP_
