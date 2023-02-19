
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
#include "ncview/details/nc_constraint.hpp"

namespace robosw {
namespace swviz {
class NcContainer : public NcElement {
 protected:
  struct Component {
    std::shared_ptr<NcElement> element;
    NcConstraint constraint;
  };

 public:
  NcContainer() = default;
  virtual ~NcContainer() = default;

  bool AddElement(std::shared_ptr<NcElement> element, NcConstraint constraint = NcConstraint{});

 protected:
  // resize() must be implemented by derived containers
  virtual void OnResize(int rows, int cols, int y, int x) = 0;
  void OnDraw() override;

  virtual bool ValidateNewConstraint(const NcConstraint &constraint);

  std::vector<Component> components_;
  uint32_t num_of_flex_components_{0};
};
}
} // robosw

#endif //ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_CONTAINER_HPP_
