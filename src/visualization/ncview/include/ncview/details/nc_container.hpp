
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
#include <unordered_map>

#include "ncview/details/nc_element.hpp"
#include "ncview/details/nc_constraint.hpp"

namespace xmotion {
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

  // public methods
  bool AddElement(std::shared_ptr<NcElement> element,
                  NcConstraint constraint = NcConstraint{});

 protected:
  // functions that must be implemented by derived containers
  virtual void OnResize(int rows, int cols, int y, int x) = 0;
  virtual void AllocateSpace(int rows, int cols) = 0;

  void OnDraw() override;
  virtual bool ValidateNewConstraint(const NcConstraint &constraint);

  // helper functions
  void SpaceAllocator1D(
      int dim_size, const std::vector<Component> &components,
      std::unordered_map<std::shared_ptr<NcElement>, int> &allocated_sizes);

  std::vector<Component> components_;
  uint32_t num_of_flex_components_{0};
};
}  // namespace swviz
}  // namespace xmotion

#endif  // ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_NC_CONTAINER_HPP_
