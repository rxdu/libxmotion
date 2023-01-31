
/**
* @file nc_container.hpp
* @date 1/29/23
* @brief
* 
* @copyright Copyright (c) 2023 Ruixiang Du (rdu)
*/

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_CONTAINER_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_CONTAINER_HPP_

#include <vector>
#include <memory>

#include "ncview/details/nc_element.hpp"

namespace robosw {
namespace swviz {
class NcContainer : public NcElement {
 public:
  NcContainer() = default;
  virtual ~NcContainer() = default;

  void AddElement(std::shared_ptr<NcElement> element);

 protected:
  void OnResize(int rows, int cols, int y, int x) override;
  void OnDraw() override;

  std::vector<std::shared_ptr<NcElement>> elements_;
};
}
} // robosw

#endif //ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_CONTAINER_HPP_
