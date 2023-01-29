
/**
* @file nc_layout.hpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_DETAILS_NC_LAYOUT_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_DETAILS_NC_LAYOUT_HPP_

#include <vector>
#include <memory>

namespace robosw {
namespace swviz {
class NcLayout {
 public:

  void Split();

 private:
  std::shared_ptr<NcLayout> parent_;
  std::vector<std::shared_ptr<NcLayout>> children_;
};
}
} // robosw

#endif //ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_DETAILS_NC_LAYOUT_HPP_
