
/**
* @file nc_hbox.hpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_HBOX_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_HBOX_HPP_

#include "ncview/details/nc_container.hpp"

#include <unordered_map>

namespace xmotion {
namespace swviz {
class NcHbox : public NcContainer {
 public:
  NcHbox() = default;

  void OnResize(int rows, int cols, int y, int x) final;

 private:
  void AllocateSpace(int rows, int cols) override;
  std::unordered_map<std::shared_ptr<NcElement>, int> allocated_sizes_;
};
}
} // xmotion

#endif //ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_HBOX_HPP_
