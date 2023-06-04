
/**
 * @file nc_constraint.hpp
 * @date 2/19/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_DETAILS_NC_CONSTRAINT_HPP_
#define ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_DETAILS_NC_CONSTRAINT_HPP_

namespace xmotion {
namespace swviz {
/// @brief Constraint for NcElement.
/// If the type is kFixed, the ratio is used to allocate space for the element.
/// After space for all fixed elements is allocated, the rest of the space is
/// allocated evenly to elements of type kFlexible.
struct NcConstraint {
  enum class Type {
    kFlexible = 0,
    kFixed = 1,
  };

  NcConstraint(Type type = Type::kFlexible, float ratio = 0.0)
      : type(type), ratio(ratio) {}

  Type type = Type::kFlexible;
  float ratio = 0.0;
};
}  // namespace swviz
}  // namespace xmotion

#endif  // ROBOSW_SRC_VISUALIZATION_NCVIEW_INCLUDE_NCVIEW_DETAILS_NC_CONSTRAINT_HPP_
